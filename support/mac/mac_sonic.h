// DP83932/DP83934 SONIC model — a C port of the flows in MAME's
// src/devices/machine/dp83932c.cpp (BSD-3-Clause, Patrick Mackinlay),
// restructured for the MacLC mailbox: registers live here, all descriptor
// and packet memory lives in GUEST RAM reached through injected host ops
// (on the LC card that is the FPGA's DMA-RPC engine; a future NuBus card
// with on-card RAM supplies a local-buffer backend instead).
//
// Kept deliberately ANSI-C-ish and free of Main includes so it can be unit
// tested off-target.

#ifndef MAC_SONIC_H
#define MAC_SONIC_H

#include <stdint.h>

typedef struct {
	// grouped guest-memory access; 16-bit values are the 68k big-endian
	// words at even guest addresses; byte forms move raw bytes (packet
	// data). All return 0 on success, -1 on host failure (abort the op).
	int (*read_words)(uint32_t gaddr, uint16_t *w, int n, int stride);
	int (*write_words)(uint32_t gaddr, const uint16_t *w, int n, int stride);
	int (*read_bytes)(uint32_t gaddr, uint8_t *b, int n);
	int (*write_bytes)(uint32_t gaddr, const uint8_t *b, int n);
	// frame WITHOUT FCS (mirrors sonic_rx_frame): the transport appends its
	// own. Adding the model's software FCS made max-size frames 1518 bytes,
	// which a raw socket refuses (EMSGSIZE past MTU 1500 + 14 header).
	int (*wire_send)(const uint8_t *frame, int len);
} sonic_host_ops;

void     sonic_init(const sonic_host_ops *ops);
void     sonic_reset(void);                    // hardware/software reset state
void     sonic_reg_write(int reg, uint16_t data);
uint16_t sonic_reg(int reg);                   // current value (shadow source)
void     sonic_fill_shadows(uint16_t regs[64]);
int      sonic_int_line(void);                 // level: ISR & IMR & 0x7fff
// Deliver one frame (WITHOUT FCS) into the guest's RX structures.
// Returns 1 = delivered, 0 = filtered/dropped (address filter, runt, or a
// failure after descriptor state already advanced - NOT retryable), -1 =
// BUSY before any state was touched (RXEN clear, RDE/RBE latched, or no
// free descriptor): the caller may hold the frame and retry once the guest
// services its ring - dropping it instead costs a TCP retransmit timeout.
int      sonic_rx_frame(const uint8_t *frame, int len);
uint32_t sonic_ea_stripped(void);              // dirty-top-byte addrs masked (24-bit-mode witness)

#endif
