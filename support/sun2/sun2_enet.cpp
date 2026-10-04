// Identity and Ethernet for the Sun-2 core -- see sun2_enet.h.
//
// The mailbox is the NeXT core's layout (support/next/next_enet.cpp), in the
// same window, with the Sun-2's own magic and a sixteen-slot receive ring: a
// host delivers in bursts -- an 8 KiB NFS read is six fragments at once -- and
// the core drains at 10 Mb/s.  All slots are 64-bit little-endian words;
// frame byte i sits at slot byte 8+i:
//   +0x0000  MAGIC     0x53554E3245544831 ("SUN2ETH1"), FPGA-written last
//   +0x0008  TX_WPTR   FPGA increments per transmitted frame
//   +0x0010  RX_WPTR   this daemon increments per delivered frame
//   +0x0018  RX_RPTR   FPGA increments per consumed frame
//   +0x0020  GUEST_MAC bit63 valid, bits[47:0] the ID PROM's address
//   +0x0800  TX slots: 4 x 2048 bytes (u64 len header, then frame)
//   +0x2800  RX slots: 16 x 2048 bytes
// Frames carry no FCS either way; the core strips it and makes it.

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "../../user_io.h"
#include "../../file_io.h"
#include "../../shmem.h"
#include "sun2_enet.h"

// shared host-network layer from the A2065 module
extern int  ethernet_open(const char *iface, int promiscuous);
extern void ethernet_close(void);
extern void ethernet_send(const uint8_t *frame, int len);
extern int  ethernet_recv_nb(uint8_t *buf, int maxlen);
extern int  ethernet_set_mac_filter(const uint8_t *mac);
extern int  ethernet_macvlan_create(const char *parent, const char *name, const uint8_t *mac);
extern void ethernet_macvlan_delete(const char *name);
extern int  ethernet_read_iface_mac(const char *iface, uint8_t *out);
extern void ethernet_offload_off(const char *iface);
extern void ethernet_offload_on(const char *iface);
extern int  a2065_mode_available(int mode);

#define NB_BASE        0x1FF00000UL
#define NB_SIZE        0x10000UL

#define NB_MAGIC_OFF   0x0000
#define NB_TXWPTR_OFF  0x0008
#define NB_RXWPTR_OFF  0x0010
#define NB_RXRPTR_OFF  0x0018
#define NB_MAC_OFF     0x0020
#define NB_TXSLOT_OFF  0x0800
#define NB_RXSLOT_OFF  0x2800
#define NB_SLOT_SIZE   0x800
#define NB_RING        4       // transmit slots
#define NB_RX_RING     16      // receive slots

#define NB_MAGIC       0x53554E3245544831ULL

#define NB_MODE_OFF     0
#define NB_MODE_ETH0    1
#define NB_MODE_ETH1    2
#define NB_MODE_MACVLAN 3
#define NB_MODE_TAP     4

#define MACVLAN_NAME   "sun2"
#define MAX_FRAME      1518    // the core's limit: 1514 and a VLAN tag
#define IDPROM_INDEX   64      // the ioctl index of boot1.rom

static volatile uint8_t *mb = 0;
static int      running = 0;      // start() called for this core
static int      link_open = 0;    // host interface open
static int      cur_mode = NB_MODE_OFF;
static int      made_macvlan = 0;
static char     offload_iface[16];  // the NIC whose offloads are off, to restore
static uint64_t tx_rd = 0;        // local TX ring read index
static uint8_t  guest_mac[6];
static int      mac_known = 0;

static inline uint64_t rd64(uint32_t off)
{
	return *(volatile uint64_t *)(mb + off);
}

static inline void wr64(uint32_t off, uint64_t v)
{
	*(volatile uint64_t *)(mb + off) = v;
}

// ---- the ID PROM ------------------------------------------------------------
//
// Byte 0 format (1), 1 machine type (2, a VME Sun-2: 2/50 or 2/160), 2..7 the
// Ethernet address, 8..11 the date of manufacture, 12..14 the serial number,
// 15 the checksum -- the XOR of bytes 0..15 is zero -- and 16..31 reserved.

static int make_idprom(uint8_t *p)
{
	uint8_t hw[6];
	if (!ethernet_read_iface_mac("eth0", hw) && !ethernet_read_iface_mac("wlan0", hw))
		return 0;

	memset(p, 0xFF, 32);
	p[0] = 0x01;
	p[1] = 0x02;
	p[2] = 0x08; p[3] = 0x00; p[4] = 0x20;              // Sun's OUI
	p[5] = hw[3]; p[6] = hw[4]; p[7] = hw[5];
	p[8] = 0x1A; p[9] = 0xE4; p[10] = 0x23; p[11] = 0x3B;  // the core's own date
	p[12] = hw[3]; p[13] = hw[4]; p[14] = hw[5];         // serial number, so hostid 02xxxxxx
	p[15] = 0;
	for (int i = 0; i < 15; i++) p[15] ^= p[i];
	return 1;
}

static void send_idprom(void)
{
	uint8_t p[32];
	const char *src;

	memset(p, 0xFF, sizeof(p));
	char *path = user_io_make_filepath(HomeDir(), "boot1.rom");
	int n = FileLoad(path, 0, 0);
	if (n >= 16)
	{
		FileLoad(path, p, sizeof(p));
		src = "boot1.rom";
	}
	else if (make_idprom(p)) src = "this MiSTer's Ethernet address";
	else
	{
		printf("[sun2-enet] no boot1.rom and no host address: the core keeps its own ID PROM\n");
		return;
	}

	user_io_set_index(IDPROM_INDEX);
	user_io_set_download(1);
	user_io_file_tx_data(p, sizeof(p));
	user_io_set_download(0);
	printf("[sun2-enet] ID PROM from %s: %02X:%02X:%02X:%02X:%02X:%02X, serial %u\n", src,
	       p[2], p[3], p[4], p[5], p[6], p[7], (p[12] << 16) | (p[13] << 8) | p[14]);
}

// ---- the wire -----------------------------------------------------------------

static int read_guest_mac(uint8_t *out)
{
	uint64_t v = rd64(NB_MAC_OFF);
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
	if (offload_iface[0])
	{
		ethernet_offload_on(offload_iface);
		offload_iface[0] = 0;
	}
}

// Receive offloads coalesce TCP segments into frames far larger than Ethernet
// allows, which the 82586 cannot take and the daemon drops; a shared or
// dedicated NIC runs without them while the Sun uses it (as the A2065's does).
static void offload_off(const char *iface)
{
	ethernet_offload_off(iface);
	snprintf(offload_iface, sizeof(offload_iface), "%s", iface);
}

static void open_link(int mode)
{
	const char *iface = "eth0";
	int promisc = 0;

	close_link();

	switch (mode)
	{
	case NB_MODE_ETH0:
		iface = "eth0";
		promisc = 1;   // shared NIC: promiscuous plus the BPF MAC filter
		offload_off(iface);
		break;
	case NB_MODE_ETH1:
		// The Sun's own NIC, still promiscuous: the frames for the Sun are
		// addressed to the ID PROM's address, not the NIC's.
		iface = "eth1";
		promisc = 1;
		offload_off(iface);
		break;
	case NB_MODE_MACVLAN:
		if (!mac_known) return;    // needs the guest MAC to create the child
		offload_off("eth0");
		if (!ethernet_macvlan_create("eth0", MACVLAN_NAME, guest_mac)) return;
		made_macvlan = 1;
		iface = MACVLAN_NAME;
		break;
	case NB_MODE_TAP:
		iface = "tap0";
		break;
	default:
		return;
	}

	if (!ethernet_open(iface, promisc)) return;
	link_open = 1;

	// flush anything the kernel queued before the link came up
	{
		uint8_t scratch[2048];
		while (ethernet_recv_nb(scratch, sizeof(scratch)) > 0) ;
	}

	if ((mode == NB_MODE_ETH0 || mode == NB_MODE_ETH1) && mac_known)
		ethernet_set_mac_filter(guest_mac);

	printf("[sun2-enet] bridge up on %s (mode %d)\n", iface, mode);
}

void sun2_enet_start(void)
{
	send_idprom();

	if (!mb)
	{
		mb = (volatile uint8_t *)shmem_map(NB_BASE, NB_SIZE);
		if (!mb)
		{
			printf("[sun2-enet] shmem_map failed\n");
			return;
		}
	}
	running = 1;
	tx_rd = 0;
	mac_known = 0;
	cur_mode = NB_MODE_OFF;
	printf("[sun2-enet] armed, waiting for the core mailbox\n");
}

void sun2_enet_stop(void)
{
	if (!running) return;
	running = 0;
	close_link();
	printf("[sun2-enet] stopped\n");
}

static int mode_from_status(void)
{
	int mode = (int)user_io_status_get(SUN2_ENET_STATUS_OPT);
	if (mode < 0 || mode > NB_MODE_TAP) mode = NB_MODE_OFF;
	if (!a2065_mode_available(mode)) mode = NB_MODE_OFF;
	return mode;
}

void sun2_enet_poll(void)
{
	static uint8_t frame[2048];

	if (!running || !mb) return;
	if (rd64(NB_MAGIC_OFF) != NB_MAGIC) return;   // core not up, or Network is Off there

	// track the guest MAC published by the FPGA
	uint8_t m[6];
	if (read_guest_mac(m) && (!mac_known || memcmp(m, guest_mac, 6)))
	{
		memcpy(guest_mac, m, 6);
		mac_known = 1;
		printf("[sun2-enet] guest MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
		       m[0], m[1], m[2], m[3], m[4], m[5]);
		if (link_open && (cur_mode == NB_MODE_ETH0 || cur_mode == NB_MODE_ETH1))
			ethernet_set_mac_filter(guest_mac);
	}

	// follow the OSD selection
	int mode = mode_from_status();
	if (mode != cur_mode)
	{
		cur_mode = mode;
		if (mode == NB_MODE_OFF) close_link();
		else open_link(mode);
	}
	else if (mode != NB_MODE_OFF && !link_open)
	{
		// retry (macvlan waits for the MAC to become known)
		open_link(mode);
	}

	// The core restarts its pointers from zero whenever it republishes the
	// mailbox; follow it rather than replaying a ring's worth of old frames.
	uint64_t wptr = rd64(NB_TXWPTR_OFF);
	if (wptr < tx_rd || wptr - tx_rd > NB_RING) tx_rd = (wptr > NB_RING) ? wptr - NB_RING : 0;

	if (!link_open)
	{
		// with no wire, still consume the guest's TX frames
		tx_rd = wptr;
		return;
	}

	// drain guest transmissions
	while (tx_rd != wptr)
	{
		uint32_t slot = NB_TXSLOT_OFF + NB_SLOT_SIZE * (uint32_t)(tx_rd & (NB_RING - 1));
		int len = (int)(rd64(slot) & 0x7FF);
		if (len >= 14 && len <= MAX_FRAME)
		{
			memcpy(frame, (const void *)(mb + slot + 8), len);
			ethernet_send(frame, len);
		}
		tx_rd++;
	}

	// Deliver received frames while the FPGA ring has room, and drain the
	// socket dry every pass: a NIC whose receiver is not keeping up drops
	// frames on the wire, it does not queue them for later.
	for (;;)
	{
		int len = ethernet_recv_nb(frame, sizeof(frame));
		if (len <= 0) break;
		if (len < 14 || len > MAX_FRAME) continue;

		uint64_t rxw = rd64(NB_RXWPTR_OFF);
		uint64_t rxr = rd64(NB_RXRPTR_OFF);
		if (rxw - rxr >= NB_RX_RING) continue;   // ring full: drop, keep draining

		uint32_t slot = NB_RXSLOT_OFF + NB_SLOT_SIZE * (uint32_t)(rxw & (NB_RX_RING - 1));
		memcpy((void *)(mb + slot + 8), frame, len);
		__sync_synchronize();
		wr64(slot, (uint64_t)len);
		__sync_synchronize();
		wr64(NB_RXWPTR_OFF, rxw + 1);
	}
}
