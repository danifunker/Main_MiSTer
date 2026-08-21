// Host-side unit test for mac_sonic.cpp (the DP83932 SONIC model).
// Builds and runs on any Linux box — no MiSTer, no FPGA:
//   g++ -O1 -Wall -o /tmp/mac_sonic_test mac_sonic_test.cpp ../mac_sonic.cpp
//   /tmp/mac_sonic_test
// Exercises the descriptor flows (CAM load, RRA, RX ring writeback, TX
// gather/chain, loopback, ISR semantics) against a flat fake guest RAM,
// with the same big-endian word convention the DMA-RPC backend provides.

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "../mac_sonic.h"

static uint8_t ram[1 << 20];   // 1MB of fake guest RAM
static uint8_t last_tx[2048];
static int     last_tx_len;
static int     tx_count;

static int rd_words(uint32_t ga, uint16_t *w, int n, int stride)
{
	for (int i = 0; i < n; i++) {
		const uint8_t *p = ram + ga + i * stride + (stride == 4 ? 2 : 0);
		w[i] = (uint16_t)((p[0] << 8) | p[1]);
	}
	return 0;
}
static int wr_words(uint32_t ga, const uint16_t *w, int n, int stride)
{
	for (int i = 0; i < n; i++) {
		uint8_t *p = ram + ga + i * stride + (stride == 4 ? 2 : 0);
		p[0] = (uint8_t)(w[i] >> 8);
		p[1] = (uint8_t)w[i];
	}
	return 0;
}
static int rd_bytes(uint32_t ga, uint8_t *b, int n)  { memcpy(b, ram + ga, n); return 0; }
static int wr_bytes(uint32_t ga, const uint8_t *b, int n) { memcpy(ram + ga, b, n); return 0; }
static int tx(const uint8_t *f, int n)
{
	memcpy(last_tx, f, n);
	last_tx_len = n;
	tx_count++;
	return n;
}

static const sonic_host_ops ops = { rd_words, wr_words, rd_bytes, wr_bytes, tx };

static uint16_t rdw(uint32_t ga) { uint16_t v; rd_words(ga, &v, 1, 2); return v; }
static void     wrw(uint32_t ga, uint16_t v) { wr_words(ga, &v, 1, 2); }

static int fails;
static void check(int cond, const char *name)
{
	printf("%s: %s\n", cond ? "pass" : "FAIL", name);
	if (!cond) fails++;
}

// register numbers used below
enum { CR = 0, DCR = 1, RCR = 2, TCR = 3, IMR = 4, ISR = 5, UTDA = 6, CTDA = 7,
       URDA = 0xd, CRDA = 0xe, CRBA0 = 0xf, CRBA1 = 0x10, RBWC0 = 0x11,
       RBWC1 = 0x12, EOBC = 0x13, URRA = 0x14, RSA = 0x15, REA = 0x16,
       RRP = 0x17, RWP = 0x18, CE = 0x25, CDP = 0x26, CDC = 0x27, SR = 0x28 };

static const uint8_t MAC[6] = { 0x08, 0x00, 0x07, 0x12, 0x34, 0x56 };

int main()
{
	sonic_init(&ops);

	// ── reset state ──────────────────────────────────────────────────
	check(sonic_reg(CR) == 0x0094, "reset: CR = RST|STP|RXDIS");
	check(sonic_reg(EOBC) == 0x02f8, "reset: EOBC = $2F8");
	check(sonic_reg(SR) == 6, "SR = 6 (DP83932CVF)");

	// leave software reset, 16-bit mode
	sonic_reg_write(CR, 0x0000);
	check(!(sonic_reg(CR) & 0x80), "CR RST clears on write of 0");
	sonic_reg_write(DCR, 0x0059);   // BMS|PO1-ish, DW=0 (NetBSD's LC config)

	// ── CAM load ─────────────────────────────────────────────────────
	// descriptor at URRA:CDP: {entry, CAP0, CAP1, CAP2} + CE word after.
	// stored word-swapped (MAME convention): CAP0 = MAC[1]:MAC[0] etc.
	sonic_reg_write(URRA, 0x0004);
	sonic_reg_write(CDP,  0x1000);   // -> $41000
	wrw(0x41000, 0);                                       // entry 0
	wrw(0x41002, (uint16_t)((MAC[1] << 8) | MAC[0]));      // CAP0
	wrw(0x41004, (uint16_t)((MAC[3] << 8) | MAC[2]));      // CAP1
	wrw(0x41006, (uint16_t)((MAC[5] << 8) | MAC[4]));      // CAP2
	wrw(0x41008, 0x0001);                                  // CE: entry 0 on
	sonic_reg_write(CDC, 1);
	sonic_reg_write(CR, 0x0200);     // LCAM
	check(sonic_reg(CDC) == 0, "LCAM consumed the descriptor count");
	check(sonic_reg(CE) == 1, "LCAM read the CAM enable word");
	check(sonic_reg(ISR) & 0x1000, "LCAM set ISR_LCD");
	sonic_reg_write(ISR, 0x1000);    // clear it

	// ── RRA ──────────────────────────────────────────────────────────
	// two resources at URRA:RSA (consuming the LAST one sets RBE and the
	// chip stops receiving — real drivers always keep the RRA stocked, so
	// the test does too). 4 KB buffer so no mid-test refill triggers.
	sonic_reg_write(RSA, 0x2000);    // -> $42000
	sonic_reg_write(REA, 0x2010);
	sonic_reg_write(RRP, 0x2000);
	sonic_reg_write(RWP, 0x2010);
	wrw(0x42000, 0x2000);            // CRBA0
	wrw(0x42002, 0x0006);            // CRBA1 -> $62000
	wrw(0x42004, 0x0800);            // RBWC0 (4 KB)
	wrw(0x42006, 0x0000);            // RBWC1
	wrw(0x42008, 0x4000);            // second resource (never consumed here)
	wrw(0x4200a, 0x0006);
	wrw(0x4200c, 0x0800);
	wrw(0x4200e, 0x0000);
	sonic_reg_write(CR, 0x0100);     // RRRA
	check(sonic_reg(CRBA0) == 0x2000 && sonic_reg(CRBA1) == 6,
	      "RRRA loaded CRBA from the resource area");
	check(sonic_reg(RBWC0) == 0x0800, "RRRA loaded RBWC");
	check(sonic_reg(RRP) == 0x2008, "RRP advanced 4 words");

	// ── receive ──────────────────────────────────────────────────────
	// RDA descriptors at URDA:$3000 (5 status words + link + in-use);
	// next descriptor at $3100, its link marks end-of-list
	sonic_reg_write(URDA, 0x0004);
	sonic_reg_write(CRDA, 0x3000);
	wrw(0x43000 + 5 * 2, 0x3100);    // link -> next descriptor
	wrw(0x43000 + 6 * 2, 0xffff);    // in-use (device clears)
	wrw(0x43100 + 5 * 2, 0x3101);    // next link: LSB set = end of list
	sonic_reg_write(RCR, 0x2000);    // BRD
	sonic_reg_write(CR, 0x0008);     // RXEN

	uint8_t frame[64];
	memset(frame, 0, sizeof frame);
	memcpy(frame, MAC, 6);           // dst = our CAM entry
	memset(frame + 6, 0x22, 6);      // src
	frame[12] = 0x08; frame[13] = 0x00;
	for (int i = 14; i < 60; i++) frame[i] = (uint8_t)i;

	sonic_rx_frame(frame, 60);
	check(sonic_reg(ISR) & 0x0400, "RX set ISR_PKTRX");
	check(memcmp(ram + 0x62000, frame, 60) == 0, "RX stored the frame at the RBA");
	check(rdw(0x43000) & 0x0001, "RDA status word has PRX");
	check(rdw(0x43002) == 64, "RDA byte count = frame + FCS");
	check(rdw(0x43004) == 0x2000 && rdw(0x43006) == 0x0006,
	      "RDA packet pointer = original CRBA");
	check(rdw(0x43000 + 6 * 2) == 0, "next in-use field cleared");
	check(sonic_reg(CRDA) == 0x3100, "CRDA advanced to the next descriptor");
	check(sonic_reg(CRBA0) == 0x2040, "CRBA advanced past the packet");
	check(sonic_reg(RBWC0) == 0x0800 - 32, "RBWC decremented by word count");

	// unknown unicast must be filtered out
	uint8_t other[60];
	memcpy(other, frame, 60);
	other[5] ^= 0xff;
	sonic_reg_write(ISR, 0x0400);
	sonic_rx_frame(other, 60);
	check(!(sonic_reg(ISR) & 0x0400), "unknown unicast filtered by CAM");

	// broadcast accepted via BRD
	memset(other, 0xff, 6);
	sonic_rx_frame(other, 60);
	check(sonic_reg(ISR) & 0x0400, "broadcast accepted (BRD)");
	check(rdw(0x43100) & 0x0080, "RDA status has BC for broadcast");
	check(sonic_reg(ISR) & 0x0040, "end-of-list link set ISR_RDE");
	sonic_reg_write(ISR, 0x0440);

	// ── transmit ─────────────────────────────────────────────────────
	// TDA at UTDA:$5000: status, config, TPS, TFC=1, {TSA0,TSA1,TFS}, link
	sonic_reg_write(UTDA, 0x0004);
	sonic_reg_write(CTDA, 0x5000);
	const int TLEN = 80;
	wrw(0x45000 + 1 * 2, 0x0000);    // config
	wrw(0x45000 + 2 * 2, TLEN);      // TPS
	wrw(0x45000 + 3 * 2, 1);         // TFC
	wrw(0x45000 + 4 * 2, 0x7000);    // TSA0
	wrw(0x45000 + 5 * 2, 0x0004);    // TSA1 -> $47000
	wrw(0x45000 + 6 * 2, TLEN);      // TFS
	wrw(0x45000 + 7 * 2, 0x5201);    // link: LSB = end of list
	for (int i = 0; i < TLEN; i++) ram[0x47000 + i] = (uint8_t)(0xA0 + i);

	tx_count = 0;
	sonic_reg_write(CR, 0x0002);     // TXP
	check(tx_count == 1, "TXP transmitted one frame");
	check(last_tx_len == TLEN + 4, "TX appended the FCS");
	check(memcmp(last_tx, ram + 0x47000, TLEN) == 0, "TX gathered the fragment");
	check(rdw(0x45000) & 0x0001, "TX status written back with PTX");
	check(sonic_reg(ISR) & 0x0200, "end-of-list set ISR_TXDN");
	check(!(sonic_reg(CR) & 0x0002), "CR TXP self-cleared");
	sonic_reg_write(ISR, 0x0200);

	// ── loopback short-circuits into the receive path ────────────────
	// fresh descriptor at $3200 (the previous list hit end-of-list)
	sonic_reg_write(CRDA, 0x3200);
	wrw(0x43200 + 5 * 2, 0x3201);    // link: end of list
	wrw(0x43200 + 6 * 2, 0xffff);
	sonic_reg_write(RCR, 0x0200);    // LB mode (MAC loopback), BRD off
	sonic_reg_write(CTDA, 0x5000);   // reuse the same TDA
	memcpy(ram + 0x47000, frame, 60);   // dst = our CAM MAC
	wrw(0x45000 + 2 * 2, 60);
	wrw(0x45000 + 6 * 2, 60);
	wrw(0x45000 + 7 * 2, 0x5201);    // link: end of list again
	tx_count = 0;
	sonic_reg_write(ISR, 0x0400);
	sonic_reg_write(CR, 0x0002);     // TXP
	check(tx_count == 0, "loopback frame did not reach the wire");
	check(sonic_reg(ISR) & 0x0400, "loopback frame received (PKTRX)");
	check(rdw(0x43200 + 0) & 0x0002, "loopback RX status has LBK");

	// ── ISR write-1-clear and IMR gating of the int line ─────────────
	check(sonic_reg(ISR) & 0x0400, "ISR bit set before clear");
	sonic_reg_write(IMR, 0x0400);
	check(sonic_int_line(), "int line high when ISR & IMR");
	sonic_reg_write(ISR, 0x0400);
	check(!(sonic_reg(ISR) & 0x0400), "ISR write-1-clear");
	check(!sonic_int_line(), "int line low after the clear");

	printf(fails ? "%d FAILURES (mac_sonic_test)\n" : "ALL PASS (mac_sonic_test)\n", fails);
	return fails != 0;
}
