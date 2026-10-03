// Ethernet bridge for the SunSparcStation core -- see sun_enet.h.
//
// DDR3 mailbox (ARM physical 0x1FF00000, the window the A2065 and the NeXT
// use; one core runs at a time). 64-bit little-endian words; frame byte i
// is byte i % 8 of word 1 + i / 8 of its slot:
//   +0x0000  MAGIC    "SSETH001", written last by the FPGA at start-up
//   +0x0008  GEN      a new value at every FPGA start-up
//   +0x0010  TX_WPTR  FPGA: frames posted
//   +0x0018  TX_RPTR  ours: frames taken
//   +0x0020  RX_WPTR  ours: frames posted
//   +0x0028  RX_RPTR  FPGA: frames taken
//   +0x0030  MAC      FPGA: bit 63 valid, bits 47:40 the first byte
//   +0x1000  TX ring: 8 slots x 2048 bytes, header bits 10:0 = length
//   +0x5000  RX ring: 8 slots x 2048 bytes, header bits 10:0 = length with
//            the FCS, bits 21:16 = the LADRF index of the destination
// The FPGA side is rtl/mister/eth_hps.vhd in the core. It drops a frame
// the guest sends while the TX ring is full; we drop received frames while
// the RX ring is full (draining the socket every pass, as the NeXT bridge
// does, so the guest never gets a backlog of stale traffic).

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "../../user_io.h"
#include "../../shmem.h"
#include "sun_enet.h"

// the host network layer of the A2065 module (the NeXT bridge uses it too)
extern int  ethernet_open(const char *iface, int promiscuous);
extern void ethernet_close(void);
extern void ethernet_send(const uint8_t *frame, int len);
extern int  ethernet_recv_nb(uint8_t *buf, int maxlen);
extern int  ethernet_set_mac_filter(const uint8_t *mac);
extern int  ethernet_macvlan_create(const char *parent, const char *name, const uint8_t *mac);
extern void ethernet_macvlan_delete(const char *name);
extern int  a2065_mode_available(int mode);
extern void ethernet_offload_off(const char *iface);
extern void ethernet_offload_on(const char *iface);

#define SB_BASE       0x1FF00000UL
#define SB_SIZE       0x10000UL

#define SB_MAGIC_OFF  0x0000
#define SB_GEN_OFF    0x0008
#define SB_TXW_OFF    0x0010
#define SB_TXR_OFF    0x0018
#define SB_RXW_OFF    0x0020
#define SB_RXR_OFF    0x0028
#define SB_MAC_OFF    0x0030
#define SB_TX_OFF     0x1000
#define SB_RX_OFF     0x5000
#define SB_SLOT       0x800
#define SB_RING       8

#define SB_MAGIC      0x5353455448303031ULL

#define MODE_OFF      0
#define MODE_ETH0     1
#define MODE_ETH1     2
#define MODE_MACVLAN  3
#define MODE_TAP      4

#define MACVLAN_NAME  "sun0"
#define MAX_FRAME     (SB_SLOT - 8)

static volatile uint8_t *mb = 0;
static int      running = 0;
static int      link_open = 0;
static int      cur_mode = MODE_OFF;
static int      made_macvlan = 0;
static const char *offload_iface = 0;   // offloads turned off on it by us
static uint64_t gen = ~0ULL;
static uint8_t  guest_mac[6];
static int      mac_known = 0;
static uint32_t crc_tab[256];

static inline uint64_t rd64(uint32_t off) { return *(volatile uint64_t *)(mb + off); }
static inline void wr64(uint32_t off, uint64_t v) { *(volatile uint64_t *)(mb + off) = v; }

// IEEE 802.3 CRC-32, bit-reversed (as on the wire)
static uint32_t crc32_le(const uint8_t *p, int n)
{
	uint32_t c = 0xFFFFFFFF;
	while (n--) c = crc_tab[(c ^ *p++) & 0xFF] ^ (c >> 8);
	return c;
}

static int read_guest_mac(uint8_t *out)
{
	uint64_t v = rd64(SB_MAC_OFF);
	if (!(v >> 63)) return 0;
	for (int i = 0; i < 6; i++) out[i] = (v >> (40 - 8 * i)) & 0xFF;
	return 1;
}

static void close_link(void)
{
	if (link_open)
	{
		ethernet_close();
		link_open = 0;
	}
	if (made_macvlan)
	{
		ethernet_macvlan_delete(MACVLAN_NAME);
		made_macvlan = 0;
	}
	if (offload_iface)
	{
		ethernet_offload_on(offload_iface);
		offload_iface = 0;
	}
}

static void open_link(int mode)
{
	const char *iface;
	int promisc = 0;

	close_link();
	switch (mode)
	{
	case MODE_ETH0:
		iface = "eth0";
		promisc = 1;   // shared NIC: promiscuous, with the BPF filter on our MAC
		break;
	case MODE_ETH1:
		iface = "eth1";
		promisc = 1;   // the guest's MAC is not eth1's: filter on it
		break;
	case MODE_MACVLAN:
		if (!mac_known) return;   // the child needs the guest's MAC
		if (!ethernet_macvlan_create("eth0", MACVLAN_NAME, guest_mac)) return;
		made_macvlan = 1;
		iface = MACVLAN_NAME;
		break;
	case MODE_TAP:
		iface = "tap0";
		break;
	default:
		return;
	}

	// GRO and friends would hand the guest super-frames no LANCE can take
	// (the A2065 does the same)
	if (mode != MODE_TAP)
	{
		offload_iface = (mode == MODE_ETH1) ? "eth1" : "eth0";
		ethernet_offload_off(offload_iface);
	}

	if (!ethernet_open(iface, promisc)) return;
	link_open = 1;

	uint8_t scratch[MAX_FRAME];
	while (ethernet_recv_nb(scratch, MAX_FRAME) > 0) ;
	if ((mode == MODE_ETH0 || mode == MODE_ETH1) && mac_known) ethernet_set_mac_filter(guest_mac);
	printf("[sun-enet] bridge up on %s (mode %d)\n", iface, mode);
}

void sun_enet_start(void)
{
	if (!mb)
	{
		mb = (volatile uint8_t *)shmem_map(SB_BASE, SB_SIZE);
		if (!mb)
		{
			printf("[sun-enet] shmem_map failed\n");
			return;
		}
		// a stale magic from an earlier session must not look alive
		wr64(SB_MAGIC_OFF, 0);
	}
	for (uint32_t i = 0; i < 256; i++)
	{
		uint32_t c = i;
		for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320 & -(c & 1));
		crc_tab[i] = c;
	}
	running = 1;
	gen = ~0ULL;
	mac_known = 0;
	cur_mode = MODE_OFF;
	printf("[sun-enet] armed, waiting for the core's mailbox\n");
}

void sun_enet_stop(void)
{
	if (!running) return;
	running = 0;
	close_link();
	printf("[sun-enet] stopped\n");
}

static int mode_from_status(void)
{
	// The OSD lists eth0 first, so that it is the default (value 0), then
	// Off; the other values are the modes' own (sun_enet.h)
	int v = (int)user_io_status_get(SUN_ENET_STATUS_OPT);
	int mode = (v == 0) ? MODE_ETH0 : (v == 1) ? MODE_OFF : v;
	if (mode < 0 || mode > MODE_TAP) mode = MODE_OFF;
	if (!a2065_mode_available(mode)) mode = MODE_OFF;
	return mode;
}

void sun_enet_poll(void)
{
	static uint8_t frame[MAX_FRAME + 64];

	if (!running || !mb) return;
	if (rd64(SB_MAGIC_OFF) != SB_MAGIC) return;    // the core is not up

	// a new FPGA start-up: its pointers are zero again
	uint64_t g = rd64(SB_GEN_OFF);
	if (g != gen)
	{
		gen = g;
		mac_known = 0;
		printf("[sun-enet] core mailbox up (generation %08llX)\n", (unsigned long long)g);
	}

	uint8_t m[6];
	if (read_guest_mac(m) && (!mac_known || memcmp(m, guest_mac, 6)))
	{
		memcpy(guest_mac, m, 6);
		mac_known = 1;
		printf("[sun-enet] guest MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
		       m[0], m[1], m[2], m[3], m[4], m[5]);
		if (link_open && (cur_mode == MODE_ETH0 || cur_mode == MODE_ETH1))
			ethernet_set_mac_filter(guest_mac);
		if (cur_mode == MODE_MACVLAN) cur_mode = -1;   // recreate the child
	}

	int mode = mode_from_status();
	if (mode != cur_mode)
	{
		cur_mode = mode;
		if (mode == MODE_OFF) close_link();
		else open_link(mode);
	}
	else if (mode != MODE_OFF && !link_open)
	{
		open_link(mode);
	}

	// the guest's frames
	uint64_t txw = rd64(SB_TXW_OFF);
	uint64_t txr = rd64(SB_TXR_OFF);
	if (txw - txr > SB_RING) txr = txw;              // out of step: resync
	while (txr != txw)
	{
		uint32_t slot = SB_TX_OFF + SB_SLOT * (uint32_t)(txr % SB_RING);
		int len = (int)(rd64(slot) & 0x7FF);
		if (link_open && len >= 14 && len <= MAX_FRAME)
		{
			memcpy(frame, (const void *)(mb + slot + 8), len);
			ethernet_send(frame, len);
		}
		txr++;
	}
	__sync_synchronize();
	wr64(SB_TXR_OFF, txr);

	if (!link_open) return;

	// frames for the guest, while the RX ring has room
	for (;;)
	{
		int len = ethernet_recv_nb(frame, MAX_FRAME);
		if (len <= 0) break;
		if (len < 14) continue;

		uint64_t rxw = rd64(SB_RXW_OFF);
		uint64_t rxr = rd64(SB_RXR_OFF);
		if (rxw - rxr >= SB_RING) continue;          // full: drop, keep draining

		if (len < 60)
		{
			memset(frame + len, 0, 60 - len);
			len = 60;
		}
		if (len > MAX_FRAME - 4) continue;
		uint32_t fcs = ~crc32_le(frame, len);
		for (int k = 0; k < 4; k++) frame[len++] = fcs >> (8 * k);
		uint32_t hash = crc32_le(frame, 6) >> 26;

		uint32_t slot = SB_RX_OFF + SB_SLOT * (uint32_t)(rxw % SB_RING);
		memcpy((void *)(mb + slot + 8), frame, len);
		__sync_synchronize();
		wr64(slot, (uint64_t)len | ((uint64_t)hash << 16));
		__sync_synchronize();
		wr64(SB_RXW_OFF, rxw + 1);
	}
}
