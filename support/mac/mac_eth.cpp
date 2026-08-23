// MacLC PDS Ethernet service — Main side of the Apple Ethernet LC Twisted
// Pair card (820-0532-B). See mac_eth.h for the mailbox contract (v2,
// "McLCETH2") and MacLC_MiSTer docs/pds_ethernet_scope.md for the design.
//
// Lifecycle follows the a2065 pattern via the existing mac_poll() hook:
// self-gating exact-match core detection (★ NOT the prefix-matching
// is_core_named() — "maclc" would also fire for the separate MacLCII
// core, whose DDR3 window this service must never touch), lazy-arm,
// bounded work per pass, full teardown when the core goes away.
//
// The SONIC model (mac_sonic) reaches guest RAM through the FPGA's DMA-RPC
// engine: post {seq, dir, addr, count} in DMA_CMD, data moves through the
// XFER bounce window, completion echoes in DMA_STAT. The FPGA picks a
// command up within ~32 us while hot (~660 us cold) and moves words on
// idle SDRAM edges only, so a wait here is microseconds, not milliseconds.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

#include "../../user_io.h"
#include "../../menu.h"
#include "../../shmem.h"
#include "../../hardware.h"
#include "mac.h"
#include "mac_eth.h"
#include "mac_sonic.h"
#include "mac_eth_declrom.h"

static volatile uint8_t *win;   // the mapped DDR3 window
static int      card_up;
static uint32_t rptr;
static uint8_t  dma_seq;
static uint8_t  guest_mac[6];
static char     ifname[64] = "eth0";

static volatile uint64_t *w64(uint32_t off)
{
	return (volatile uint64_t *)(win + off);
}

// ── OSD-selected settings ───────────────────────────────────────────────────
// Interface and MAC suffix live in core status bits. The FPGA never reads
// them - Main owns its own status word (the a2065 pattern) - and they
// persist in the core's config slot like every other OSD option, so there
// is nothing to save here.
#define ETH_OPT_IFACE  "[37:36]"   // core: "o45,Net interface,eth0,tap0,macvlan,eth1"
#define ETH_OPT_MACSUF "[35:32]"   // core: "o03,MAC suffix,0..F" (core status is
                                   // [31:0]; these Main-only bits stay clear of it)

static const char *const iface_names[4] = { "eth0", "tap0", "macvlan", "eth1" };

// Fixed base: Apple's OUI + 'M' + a per-core byte, so a MacLC and a future
// NuBus Mac core can never collide on one LAN. Only the last octet's low
// nibble is user-selectable, and the first octet is always 0x08, so no OSD
// selection can produce a multicast or locally-administered address.
//
// Deliberately NOT derived from anything on the box. The DE10-Nano has no
// MAC EEPROM: u-boot gives every MiSTer the same eth0 address
// (02:03:04:05:06:07) unless the owner wrote linux/u-boot.txt, and the
// default hostname is "MiSTer" everywhere - both "unique" sources are
// identical across stock boxes. A visible, stable, user-picked nibble beats
// a derivation that silently collides.
#define MAC_OUI_0   0x08
#define MAC_OUI_1   0x00
#define MAC_OUI_2   0x07
#define MAC_FAMILY  0x4D           // 'M'
#define MAC_CORE    0x4C           // 'L' - MacLC; give each Mac core its own

// config: optional /media/fat/games/MacLC/eth.cfg, overriding the OSD for
// what the OSD cannot express (an exact MAC, an iface outside the four).
//   iface=tap0
//   mac=08:00:07:12:34:56
static void load_config(void)
{
	guest_mac[0] = MAC_OUI_0; guest_mac[1] = MAC_OUI_1; guest_mac[2] = MAC_OUI_2;
	guest_mac[3] = MAC_FAMILY; guest_mac[4] = MAC_CORE;
	guest_mac[5] = (uint8_t)(user_io_status_get(ETH_OPT_MACSUF) & 0xF);

	snprintf(ifname, sizeof ifname, "%s",
	         iface_names[user_io_status_get(ETH_OPT_IFACE) & 3]);

	FILE *f = fopen("/media/fat/games/MacLC/eth.cfg", "r");
	if (!f) return;
	char line[160];
	while (fgets(line, sizeof line, f)) {
		line[strcspn(line, "\r\n")] = 0;
		if (!strncasecmp(line, "iface=", 6) && line[6])
			snprintf(ifname, sizeof ifname, "%s", line + 6);
		else if (!strncasecmp(line, "mac=", 4)) {
			unsigned m[6];
			if (sscanf(line + 4, "%x:%x:%x:%x:%x:%x",
			           &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) == 6)
				for (int i = 0; i < 6; i++) guest_mac[i] = (uint8_t)m[i];
		}
	}
	fclose(f);
}

// ── DMA-RPC client (the SONIC model's guest-memory backend) ─────────────
static int dma_rpc(uint32_t gaddr, uint32_t len, int wr)
{
	if (!card_up) return -1;
	if (++dma_seq == 0) dma_seq = 1;   // 0 = the engine's reset state

	*w64(ETH_OFF_DMACMD) = ((uint64_t)len << 40) | ((uint64_t)gaddr << 16)
	                     | ((uint64_t)(wr ? 1 : 0) << 8) | dma_seq;
	__sync_synchronize();

	// typical completion is tens of microseconds; 50 ms = FPGA gone
	for (int spin = 0; spin < 1000; spin++) {
		uint64_t st = *w64(ETH_OFF_DMASTAT);
		if ((st & 0xff) == dma_seq)
			return (st & 0x100) ? -1 : 0;
		usleep(50);
	}
	printf("mac_eth: DMA-RPC timeout (seq %u addr 0x%x len %u)\n",
	       dma_seq, gaddr, len);
	return -1;
}

// The guest-RAM DMA engine only moves 16-bit WORDS at EVEN addresses, but the
// SONIC issues byte accesses at ARBITRARY addresses: an odd-length packet (e.g.
// 101 bytes) leaves the next RX buffer pointer ODD, and a descriptor link value
// carries the end-of-list flag in A0. On real hardware / MAME the 16-bit bus
// handles this with byte enables; here the impedance match lives in these two
// wrappers — word-align the transfer and copy the requested [ga, ga+len) slice
// in/out of the (up to one byte larger) XFER window. Rejecting odd addresses
// (the old behaviour) silently failed the RX buffer store, so PKTRX was never
// raised and the guest driver hung at open time waiting for a looped-back frame.
static int rpc_read(uint32_t ga, uint8_t *dst, uint32_t len)
{
	if (!len) return 0;
	uint32_t a0  = ga & ~1u;              // word-aligned start
	uint32_t off = ga - a0;               // 0 or 1
	uint32_t n   = (off + len + 1) & ~1u; // even byte count spanning [ga, ga+len)
	if (n > 0xFFFE) return -1;
	if (dma_rpc(a0, n, 0)) return -1;
	memcpy(dst, (const void *)(win + ETH_OFF_XFER + off), len);
	return 0;
}

static int rpc_write(uint32_t ga, const uint8_t *src, uint32_t len)
{
	if (!len) return 0;
	uint32_t a0  = ga & ~1u;
	uint32_t off = ga - a0;
	uint32_t n   = (off + len + 1) & ~1u;
	if (n > 0xFFFE) return -1;
	// Read-modify-write when either edge is unaligned, so the head byte (before
	// an odd start) and the tail byte (after an odd end) that share a 16-bit
	// word with the payload are preserved instead of being clobbered with pad.
	if (off || (n != off + len)) {
		if (dma_rpc(a0, n, 0)) return -1;   // prime XFER with the current words
	}
	memcpy((void *)(win + ETH_OFF_XFER + off), src, len);
	return dma_rpc(a0, n, 1);
}

// grouped word accessors for the model: 68k big-endian values.
// stride 2 = 16-bit bus mode (the LC's); stride 4 = DCR_DW dword mode,
// 16-bit value on the low (BE) half of each dword — never used by the LC
// driver but kept correct.
static int gw_read_words(uint32_t ga, uint16_t *v, int n, int stride)
{
	uint8_t b[64 * 4];
	if (n <= 0 || n > 64) return -1;
	if (rpc_read(ga, b, (uint32_t)n * stride)) return -1;
	for (int i = 0; i < n; i++) {
		const uint8_t *p = b + i * stride + (stride == 4 ? 2 : 0);
		v[i] = (uint16_t)((p[0] << 8) | p[1]);
	}
	return 0;
}

static int gw_write_words(uint32_t ga, const uint16_t *v, int n, int stride)
{
	uint8_t b[64 * 4];
	if (n <= 0 || n > 64) return -1;
	memset(b, 0, (size_t)n * stride);
	for (int i = 0; i < n; i++) {
		uint8_t *p = b + i * stride + (stride == 4 ? 2 : 0);
		p[0] = (uint8_t)(v[i] >> 8);
		p[1] = (uint8_t)v[i];
	}
	return rpc_write(ga, b, (uint32_t)n * stride);
}

static int gb_read(uint32_t ga, uint8_t *b, int n)  { return rpc_read(ga, b, (uint32_t)n); }
static int gb_write(uint32_t ga, const uint8_t *b, int n) { return rpc_write(ga, b, (uint32_t)n); }
static int wire_send(const uint8_t *f, int n) { return mac_eth_iface_send(f, n); }

static const sonic_host_ops host_ops = {
	gw_read_words, gw_write_words, gb_read, gb_write, wire_send
};

// ── window staging ──────────────────────────────────────────────────────
static void stage_declrom(void)
{
	// flat 32K image (byteLanes $0F), top half of the ROM window; spans
	// from the generated header, zeros elsewhere
	volatile uint8_t *rom = win + ETH_OFF_ROM;
	for (uint32_t i = 0; i < 0x10000; i++) rom[i] = 0;
	const uint8_t *d = mac_eth_declrom_data;
	for (unsigned s = 0; s < sizeof mac_eth_declrom_spans / sizeof mac_eth_declrom_spans[0]; s++) {
		uint32_t off = MAC_ETH_DECLROM_WINDOW_OFF + mac_eth_declrom_spans[s].off;
		for (uint16_t i = 0; i < mac_eth_declrom_spans[s].len; i++)
			rom[off + i] = *d++;
	}
}

// MAC PROM cooking: bytes 0-5 = per-byte nibble-reversed-bit-order swizzle
// of the MAC (MAME enetlc.cpp, "IEEE standard according to SuperMario
// source"), byte 6 = 0, byte 7 = XOR of the swizzled bytes, complemented.
// The guest driver un-swizzles to recover the real MAC and loads THAT into
// the CAM, so the model and the wire always see the real address.
static uint8_t swizzle(uint8_t x)
{
	uint8_t lo = (uint8_t)(((x & 1) << 3) | ((x & 2) << 1) | ((x & 4) >> 1) | ((x & 8) >> 3));
	uint8_t hi = (uint8_t)((((x >> 4) & 1) << 3) | (((x >> 4) & 2) << 1) |
	                       (((x >> 4) & 4) >> 1) | (((x >> 4) & 8) >> 3));
	return (uint8_t)((lo << 4) | hi);
}

static void stage_macprom(void)
{
	uint8_t prom[8];
	uint8_t x = 0;
	for (int i = 0; i < 6; i++) {
		prom[i] = swizzle(guest_mac[i]);
		x ^= prom[i];
	}
	prom[6] = 0;
	prom[7] = (uint8_t)(x ^ 0xff);
	uint64_t v = 0;
	for (int i = 0; i < 8; i++) v |= (uint64_t)prom[i] << (8 * i);
	*w64(ETH_OFF_MACPROM) = v;
}

static void push_state(void)
{
	uint16_t r[64];
	sonic_fill_shadows(r);
	for (int n = 0; n < 16; n++) {
		uint64_t v = 0;
		for (int k = 0; k < 4; k++)
			v |= (uint64_t)r[4 * n + k] << (16 * k);
		*w64(ETH_OFF_SHAD + 8 * n) = v;
	}
	__sync_synchronize();   // the ISR the guest handler reads is never stale
	*w64(ETH_OFF_INT) = sonic_int_line() ? 1 : 0;
}

// ── doorbell ring ───────────────────────────────────────────────────────
static void drain_ring(void)
{
	uint32_t wp = (uint32_t)*w64(ETH_OFF_WPTR);
	if (wp < rptr) {
		printf("mac_eth: wptr regressed (%u < %u) — FPGA reset, resync\n", wp, rptr);
		rptr = 0;
	}
	int guard = ETH_RING_ENTRIES;
	while (rptr != wp && guard--) {
		uint64_t e = *w64(ETH_OFF_RING + 8 * (rptr & (ETH_RING_ENTRIES - 1)));
		rptr++;
		if (!(e & 1)) continue;
		int tag  = (int)(e >> 1) & 7;
		int r    = (int)(e >> 4) & 0x3f;
		int data = (int)(e >> 16) & 0xffff;
		switch (tag) {
		case ETH_TAG_REG_WR: sonic_reg_write(r, (uint16_t)data); break;
		case ETH_TAG_RESET:  sonic_reset(); break;
		}
	}
	*w64(ETH_OFF_RPTR) = rptr;
}

// ── lifecycle ───────────────────────────────────────────────────────────
// The OSD selections the running card was started with, so a mid-session
// change can be detected and applied without a core reload.
#define SEL_NONE 0xFFFFFFFFu
static uint32_t sel_cur = SEL_NONE;
static uint32_t fail_announced = SEL_NONE;

static uint32_t sel_snapshot(void)
{
	return (user_io_status_get(ETH_OPT_IFACE) & 3) |
	       ((user_io_status_get(ETH_OPT_MACSUF) & 0xF) << 8);
}

static void announce_up(void)
{
	char msg[96];
	snprintf(msg, sizeof msg,
	         "Ethernet: %s\n%02X:%02X:%02X:%02X:%02X:%02X\n"
	         "restart the Mac to apply",
	         ifname, guest_mac[0], guest_mac[1], guest_mac[2],
	         guest_mac[3], guest_mac[4], guest_mac[5]);
	Info(msg, 3000);
}

static void card_start(void)
{
	load_config();
	sel_cur = sel_snapshot();
	sonic_init(&host_ops);

	rptr = (uint32_t)*w64(ETH_OFF_WPTR);   // skip anything stale
	*w64(ETH_OFF_RPTR) = rptr;             // release ring backpressure
	dma_seq = 0;
	*w64(ETH_OFF_DMACMD)  = 0;             // engine reset state: seq 0 done
	*w64(ETH_OFF_DMASTAT) = 0;

	stage_declrom();
	stage_macprom();
	if (!mac_eth_iface_open(ifname)) {
		// A selection this box cannot serve (no tun for tap0, no second NIC)
		// leaves the guest cardless; say so once, or it reads as "ethernet is
		// broken". card_start() retries each second, hence the latch.
		if (fail_announced != sel_cur) {
			fail_announced = sel_cur;
			char msg[64];
			snprintf(msg, sizeof msg, "Ethernet: %s unavailable", ifname);
			Info(msg, 4000);
			printf("mac_eth: iface %s unavailable - card stays down\n", ifname);
		}
		return;
	}
	fail_announced = SEL_NONE;
	push_state();
	*w64(ETH_OFF_GEO) = 2;                 // layout version
	__sync_synchronize();
	*w64(ETH_OFF_MAGIC) = ETH_MAGIC_V2;    // MAGIC last: presence gate
	card_up = 1;
	printf("mac_eth: card up (iface %s, MAC %02X:%02X:%02X:%02X:%02X:%02X)\n",
	       ifname, guest_mac[0], guest_mac[1], guest_mac[2],
	       guest_mac[3], guest_mac[4], guest_mac[5]);
}

static void card_stop(void)
{
	if (!card_up) return;
	*w64(ETH_OFF_MAGIC) = 0;
	mac_eth_iface_close();
	card_up = 0;
	printf("mac_eth: card down\n");
}

// exact match — the prefix-matching is_core_named() would also fire for
// MacLCII (the daemon's 0de9974 lesson, carried over)
static int core_is_maclc(void)
{
	return !strcasecmp(user_io_get_core_name(0), "maclc")
	    || !strcasecmp(user_io_get_core_name(1), "maclc");
}

void mac_eth_poll(void)
{
	static unsigned long pace_timer, name_timer;
	static int mapped;

	// ~1 ms service pace; core-name recheck each second
	if (!CheckTimer(pace_timer)) return;
	pace_timer = GetTimer(1);

	if (CheckTimer(name_timer)) {
		name_timer = GetTimer(1000);
		int want = core_is_maclc();
		if (want && !card_up) {
			if (!mapped) {
				win = (volatile uint8_t *)shmem_map(ETH_DDR_BASE, ETH_WIN_SIZE);
				mapped = (win != NULL);
			}
			if (mapped) card_start();
		}
		else if (!want && card_up) card_stop();
		else if (want && card_up && sel_snapshot() != sel_cur) {
			// OSD selection changed: restart onto the new iface/MAC. The FPGA
			// latched presence at guest reset, so the guest keeps its card - a
			// changed MAC only reaches its driver on the next guest restart,
			// which announce_up() says out loud.
			card_stop();
			card_start();
			if (card_up) announce_up();
		}
	}

	if (!card_up) return;

	drain_ring();

	// bounded receive pump: the model filters and DMAs into guest RAM
	uint8_t frame[2048];
	for (int i = 0; i < 4; i++) {
		int n = mac_eth_iface_recv(frame, sizeof frame);
		if (n <= 0) break;
		sonic_rx_frame(frame, n);
	}

	push_state();
}
