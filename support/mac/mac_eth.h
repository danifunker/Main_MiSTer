// Mac LC PDS Ethernet (Apple Ethernet LC Twisted Pair, 820-0532-B) — Main
// side of the DDR3 shared-memory mailbox served by rtl/pds/pds_enet.sv in
// MacLC_MiSTer (branch apple-pds-ethernet). The FPGA is a dumb front-end
// (register doorbell + read shadows + MAC PROM + declROM window + a
// guest-RAM DMA engine); the DP83934 SONIC-T model lives here (mac_sonic)
// and the network bridge in mac_eth_iface.
//
// ★ LAYOUT CONTRACT v2 — mirrors rtl/pds/pds_enet.sv and
// docs/pds_ethernet_scope.md in the core repo. Change nothing here without
// changing both.

#ifndef MAC_ETH_H
#define MAC_ETH_H

#include <stdint.h>

// ── DDR3 window (ARM physical) ──────────────────────────────────────────
#define ETH_DDR_BASE   0x1FF00000UL
#define ETH_WIN_SIZE   0x21000UL

#define ETH_OFF_XFER   0x00000UL   // 64K guest-RAM DMA bounce buffer
#define ETH_OFF_ROM    0x10000UL   // 64K declROM window: byte i = guest $FEFF0000+i
#define ETH_OFF_MAGIC  0x20000UL   // control block, 64-bit words:
#define ETH_OFF_WPTR   0x20008UL   //   FPGA->ARM doorbell write index (monotonic)
#define ETH_OFF_SHAD   0x20010UL   //   16 words: regs 4n..4n+3, reg 4n+k at bits [16k+15:16k]
#define ETH_OFF_INT    0x20090UL   //   bit0 = SONIC INT line
#define ETH_OFF_MACPROM 0x20098UL  //   8 cooked PROM bytes (byte k = PROM byte k)
#define ETH_OFF_GEO    0x200A0UL   //   layout version = 2
#define ETH_OFF_RPTR   0x200A8UL   //   ARM ring read index (doorbell backpressure)
#define ETH_OFF_DMACMD 0x200B0UL   //   [7:0] seq | [8] dir(1=XFER->guest) |
                                   //   [39:16] even guest byte addr | [55:40] even count
#define ETH_OFF_DMASTAT 0x200B8UL  //   [7:0] seq echo | [8] error
#define ETH_OFF_RING   0x20800UL   // 256 x u64: [0] valid | [3:1] tag |
                                   //   [9:4] reg | [31:16] data | [39:32] seq
#define ETH_RING_ENTRIES 256

#define ETH_MAGIC_V2   0x4D634C4345544832ULL   // "McLCETH2"

#define ETH_TAG_REG_WR 0
#define ETH_TAG_RESET  1

// ── public API (called from mac.cpp) ────────────────────────────────────
void mac_eth_poll(void);   // self-gating: exact-match MacLC core detection,
                           // lazy-arm/teardown, bounded work per pass

// ── network iface layer (mac_eth_iface.cpp) ─────────────────────────────
int  mac_eth_iface_open(const char *name);
void mac_eth_iface_close(void);
int  mac_eth_iface_send(const uint8_t *frame, int len);
int  mac_eth_iface_recv(uint8_t *buf, int maxlen);
int  mac_eth_iface_fd(void);

#endif
