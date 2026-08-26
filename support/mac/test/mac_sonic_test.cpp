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
// The real DMA-RPC backend (mac_eth.cpp rpc_read/rpc_write) REQUIRES even
// guest addresses and rejects odd ones. Model that here so the harness catches
// the model handing a link value (EOL bit in LSB) to the word accessors as an
// odd address — the boot-time wedge that a permissive `ram + ga` hid.
static int     backend_odd;
// Any address beyond the fake RAM is a model bug (e.g. a 24-bit-mode pointer
// whose Memory-Manager flag byte was not masked): count it and fail the op
// instead of stomping host memory.
static int     backend_oob;
static int oob(uint32_t ga, uint32_t n) { if (ga + n > sizeof ram) { backend_oob++; return 1; } return 0; }

// access log for descriptor-ordering assertions (RX publish-order test):
// records every word-level host op while log_on is set.
static struct { char kind; uint32_t ga; int n; } alog[64];
static int alog_n, log_on;
static void alog_push(char kind, uint32_t ga, int n)
{
	if (log_on && alog_n < 64) { alog[alog_n].kind = kind; alog[alog_n].ga = ga; alog[alog_n].n = n; alog_n++; }
}

static int rd_words(uint32_t ga, uint16_t *w, int n, int stride)
{
	if (ga & 1) { backend_odd++; return -1; }
	if (oob(ga, (uint32_t)n * stride)) return -1;
	alog_push('r', ga, n);
	for (int i = 0; i < n; i++) {
		const uint8_t *p = ram + ga + i * stride + (stride == 4 ? 2 : 0);
		w[i] = (uint16_t)((p[0] << 8) | p[1]);
	}
	return 0;
}
static int wr_words(uint32_t ga, const uint16_t *w, int n, int stride)
{
	if (ga & 1) { backend_odd++; return -1; }
	if (oob(ga, (uint32_t)n * stride)) return -1;
	alog_push('w', ga, n);
	for (int i = 0; i < n; i++) {
		uint8_t *p = ram + ga + i * stride + (stride == 4 ? 2 : 0);
		p[0] = (uint8_t)(w[i] >> 8);
		p[1] = (uint8_t)w[i];
	}
	return 0;
}
static int rd_bytes(uint32_t ga, uint8_t *b, int n)  { if (oob(ga, (uint32_t)n)) return -1; memcpy(b, ram + ga, n); return 0; }
static int wr_bytes(uint32_t ga, const uint8_t *b, int n) { if (oob(ga, (uint32_t)n)) return -1; memcpy(ram + ga, b, n); return 0; }
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
	check(last_tx_len == TLEN, "TX wire frame has NO software FCS (transport appends its own)");
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

	// ── dynamic TDA append: TXP with CTDA left at an ODD end-of-list link ──
	// After a completed transmit the chip leaves CTDA holding the previous
	// descriptor's link value WITH the EOL bit (LSB=1) set — an odd value.
	// Re-issuing TXP without rewriting CTDA (the SONIC dynamic-append flow)
	// must fetch the descriptor WORD-ALIGNED; the DMA-RPC backend rejects odd
	// addresses, so without the word-align the fetch aborted transmit_chain and
	// left CR.TXP set — the exact boot wedge seen on HW (the guest driver
	// spin-polls TXP forever, one extension icon, boot never completes).
	backend_odd = 0;
	sonic_reg_write(UTDA, 0x0004);
	sonic_reg_write(CTDA, 0x6001);         // odd EOL link -> descriptor at $6000 ($46000 on the bus)
	wrw(0x46000 + 1 * 2, 0x0000);          // config
	wrw(0x46000 + 2 * 2, TLEN);            // TPS
	wrw(0x46000 + 3 * 2, 1);               // TFC
	wrw(0x46000 + 4 * 2, 0x7000);          // TSA0
	wrw(0x46000 + 5 * 2, 0x0004);          // TSA1 -> $47000
	wrw(0x46000 + 6 * 2, TLEN);            // TFS
	wrw(0x46000 + 7 * 2, 0x6001);          // link: EOL again (odd)
	for (int i = 0; i < TLEN; i++) ram[0x47000 + i] = (uint8_t)(0xC0 + i);
	sonic_reg_write(RCR, 0x0000);          // wire mode (clears the loopback LB bits)
	tx_count = 0;
	sonic_reg_write(ISR, 0x0200);          // clear any stale TXDN
	sonic_reg_write(CR, 0x0002);           // TXP with CTDA odd
	check(tx_count == 1, "odd-EOL CTDA: TXP transmitted (word-aligned fetch, no wedge)");
	check(last_tx_len == TLEN, "odd-EOL CTDA: wire frame without software FCS");
	check(memcmp(last_tx, ram + 0x47000, TLEN) == 0, "odd-EOL CTDA: fragment gathered from $47000");
	check(!(sonic_reg(CR) & 0x0002), "odd-EOL CTDA: CR.TXP self-cleared (releases the guest TXP-poll)");
	check(sonic_reg(ISR) & 0x0200, "odd-EOL CTDA: ISR_TXDN set");
	check(backend_odd == 0, "odd-EOL CTDA: no odd descriptor address reached the backend");
	sonic_reg_write(ISR, 0x0200);

	// ── ISR write-1-clear and IMR gating of the int line ─────────────
	check(sonic_reg(ISR) & 0x0400, "ISR bit set before clear");
	sonic_reg_write(IMR, 0x0400);
	check(sonic_int_line(), "int line high when ISR & IMR");
	sonic_reg_write(ISR, 0x0400);
	check(!(sonic_reg(ISR) & 0x0400), "ISR write-1-clear");
	check(!sonic_int_line(), "int line low after the clear");

	// ── RX descriptor publish order ──────────────────────────────────
	// The status word at rda+0 is the driver publish flag: the Apple
	// driver's ISR consumes and RECYCLES a descriptor (rewriting its link)
	// the moment status goes nonzero, and each host op here is a ~100 us
	// DMA-RPC on real hardware. So the status write must be the LAST
	// descriptor access of the sequence - in particular AFTER the link
	// read. Publishing first let the driver rewrite the link under our
	// pending read: CRDA left the ring and the guest hard-froze in a
	// link=0 self-orbit (HW post-mortem 2026-08-23).
	sonic_reg_write(ISR, 0x0040);    // clear RDE left by the end-of-list test
	sonic_reg_write(RCR, 0x2000);    // BRD back on (loopback test turned it off)
	sonic_reg_write(CRDA, 0xA000);   // fresh descriptor at $4A000
	wrw(0x4A00a, 0xA011);            // link: odd = end of list after one packet
	wrw(0x4A00c, 0xffff);            // in-use
	alog_n = 0; log_on = 1;
	sonic_rx_frame(other, 60);       // broadcast frame from above
	log_on = 0;
	check(sonic_reg(ISR) & 0x0400, "publish-order: frame received");
	{
		int i_status = -1, i_link = -1, last_desc = -1;
		for (int i = 0; i < alog_n; i++) {
			if (alog[i].ga >= 0x4A000 && alog[i].ga < 0x4A00e) {
				last_desc = i;
				if (alog[i].kind == 'w' && alog[i].ga == 0x4A000 && alog[i].n == 1) i_status = i;
				if (alog[i].kind == 'r' && alog[i].ga == 0x4A00a) i_link = i;
			}
		}
		check(i_status >= 0, "publish-order: status written as its own 1-word op");
		check(i_link >= 0, "publish-order: link read seen");
		check(i_status > i_link, "publish-order: status write AFTER the link read");
		check(i_status == last_desc, "publish-order: status is the LAST descriptor access");
	}
	check(rdw(0x4A000) & 0x0001, "publish-order: status has PRX");
	check(sonic_reg(ISR) & 0x0040, "publish-order: odd link latched RDE");

	// ── 24-bit-mode dirty pointers: the top byte is MM flags, not address ──
	// A fresh System 7 runs 24-bit addressing, where a pointer's top byte
	// carries Memory Manager flags (bit 31 = locked — and a DMA buffer IS
	// locked). The LC PDS has 24 address lines, so the real card never sees
	// that byte; the model must mask it at the bus or every multi-fragment
	// (IP/TCP) transmit aborts silently while single-fragment ARP — sent from
	// clean driver-owned NewPtr buffers — passes. That asymmetry (arp tx 33,
	// ip tx 0) was the exact 2026-08-26 HW fingerprint on a fresh 7.5.5.
	uint32_t eas0 = sonic_ea_stripped();
	backend_oob = 0;
	sonic_reg_write(ISR, 0x0640);          // clear TXDN + PKTRX + RDE
	sonic_reg_write(UTDA, 0x8004);         // dirty upper half: locked flag set
	sonic_reg_write(CTDA, 0x8000);         // descriptor true address $48000
	wrw(0x48000 + 1 * 2, 0x0000);          // config
	wrw(0x48000 + 2 * 2, 114);             // TPS: 14 hdr + 100 payload
	wrw(0x48000 + 3 * 2, 2);               // TFC = 2: the IP shape
	wrw(0x48000 + 4 * 2, 0x8100);          // frag 1 TSA0 (driver header, clean)
	wrw(0x48000 + 5 * 2, 0x0004);          // frag 1 TSA1 -> $48100
	wrw(0x48000 + 6 * 2, 14);              // frag 1 TFS
	wrw(0x48000 + 7 * 2, 0x8200);          // frag 2 TSA0 (locked-handle payload)
	wrw(0x48000 + 8 * 2, 0x8004);          // frag 2 TSA1 DIRTY -> true $48200
	wrw(0x48000 + 9 * 2, 100);             // frag 2 TFS
	wrw(0x48000 + 10 * 2, 0x8015);         // link: odd = end of list
	for (int i = 0; i < 14; i++)  ram[0x48100 + i] = (uint8_t)(0x10 + i);
	for (int i = 0; i < 100; i++) ram[0x48200 + i] = (uint8_t)(0x50 + i);
	tx_count = 0;
	sonic_reg_write(CR, 0x0002);           // TXP
	check(tx_count == 1, "dirty-24bit TX: multi-fragment frame transmitted");
	check(last_tx_len == 114, "dirty-24bit TX: both fragments, no software FCS");
	check(memcmp(last_tx, ram + 0x48100, 14) == 0, "dirty-24bit TX: header fragment gathered (clean ptr)");
	check(memcmp(last_tx + 14, ram + 0x48200, 100) == 0, "dirty-24bit TX: payload gathered via MASKED pointer");
	check(rdw(0x48000) & 0x0001, "dirty-24bit TX: status written back at the masked TDA");
	check(!(sonic_reg(CR) & 0x0002), "dirty-24bit TX: CR.TXP self-cleared");
	check(sonic_reg(ISR) & 0x0200, "dirty-24bit TX: ISR_TXDN set");
	check(backend_oob == 0, "dirty-24bit TX: no out-of-RAM address reached the backend");
	check(sonic_ea_stripped() > eas0, "dirty-24bit TX: ea_stripped witness counted");

	// Max-size frame on the wire: 1514 bytes must reach wire_send as 1514.
	// The model used to append its software FCS here too (1518), and a raw
	// socket refuses anything past MTU 1500 + 14 header with EMSGSIZE - so
	// every full-size TX frame silently died while smaller ones passed:
	// bulk uploads stalled at MacTCP's RTO cadence (2026-08-26).
	sonic_reg_write(ISR, 0x0200);
	sonic_reg_write(CTDA, 0x8800);         // fresh TDA at true $48800
	wrw(0x48800 + 1 * 2, 0x0000);          // config
	wrw(0x48800 + 2 * 2, 1514);            // TPS
	wrw(0x48800 + 3 * 2, 1);               // TFC
	wrw(0x48800 + 4 * 2, 0x9000);          // TSA0 -> $49000
	wrw(0x48800 + 5 * 2, 0x0004);          // TSA1
	wrw(0x48800 + 6 * 2, 1514);            // TFS: a full-size ethernet frame
	wrw(0x48800 + 7 * 2, 0x8811);          // link: odd = end of list
	for (int i = 0; i < 1514; i++) ram[0x49000 + i] = (uint8_t)(i * 7);
	tx_count = 0;
	sonic_reg_write(CR, 0x0002);           // TXP
	check(tx_count == 1, "max-size TX: frame transmitted");
	check(last_tx_len == 1514, "max-size TX: exactly 1514 on the wire (no FCS overshoot)");
	check(memcmp(last_tx, ram + 0x49000, 1514) == 0, "max-size TX: payload intact");

	// ── chain budget: a long walk suspends and resumes ───────────────
	// The guest appends descriptors while the chain runs (dynamic append),
	// so an unbounded synchronous walk can run for seconds - during which
	// no other register write gets applied and the write stash overflows
	// (3,459 ISR acks lost, guest wedged - 2026-08-26 upload stall #3).
	// The chain must stop after TX_CHAIN_BUDGET packets with CR.TXP still
	// set and resume via sonic_tx_continue().
	sonic_reg_write(ISR, 0x0200);
	for (int i = 0; i < 10; i++) {
		uint32_t d = 0x4C000 + i * 0x20;
		wrw(d + 1 * 2, 0x0000);            // config
		wrw(d + 2 * 2, TLEN);              // TPS
		wrw(d + 3 * 2, 1);                 // TFC
		wrw(d + 4 * 2, 0x7000);            // TSA0 -> $47000
		wrw(d + 5 * 2, 0x0004);            // TSA1
		wrw(d + 6 * 2, TLEN);              // TFS
		wrw(d + 7 * 2, (uint16_t)(i == 9 ? 0xC001 : (0xC000 + (i + 1) * 0x20)));
	}
	sonic_reg_write(CTDA, 0xC000);
	tx_count = 0;
	sonic_reg_write(CR, 0x0002);           // TXP: walks 8 of the 10, then yields
	check(tx_count == 8, "chain budget: 8 packets per invocation");
	check(sonic_reg(CR) & 0x0002, "chain budget: CR.TXP still set while suspended");
	check(!(sonic_reg(ISR) & 0x0200), "chain budget: no TXDN while suspended");
	sonic_tx_continue();
	check(tx_count == 10, "chain budget: continue delivers the rest");
	check(!(sonic_reg(CR) & 0x0002), "chain budget: TXP clear at end of list");
	check(sonic_reg(ISR) & 0x0200, "chain budget: TXDN set at end of list");
	sonic_tx_continue();
	check(tx_count == 10, "chain budget: continue is a no-op when idle");
	sonic_reg_write(ISR, 0x0200);

	// RX with a dirty receive-buffer pointer: stores at the masked address,
	// but the REGISTER advance and the published packet pointer keep the
	// driver's top byte like real silicon (only the bus is 24-bit).
	sonic_reg_write(ISR, 0x0640);          // clear TXDN + PKTRX + RDE
	sonic_reg_write(CRDA, 0xB000);         // fresh descriptor at $4B000
	wrw(0x4B00a, 0xB011);                  // link: odd = end of list after one
	wrw(0x4B00c, 0xffff);                  // in-use
	sonic_reg_write(CRBA0, 0x3000);
	sonic_reg_write(CRBA1, 0x8006);        // dirty upper half -> true $63000
	sonic_reg_write(RBWC0, 0x0800);
	sonic_reg_write(RBWC1, 0x0000);
	sonic_rx_frame(other, 60);             // broadcast (BRD still on)
	check(sonic_reg(ISR) & 0x0400, "dirty-24bit RX: frame received (PKTRX)");
	check(memcmp(ram + 0x63000, other, 60) == 0, "dirty-24bit RX: stored at the MASKED RBA");
	check(rdw(0x4B000) & 0x0001, "dirty-24bit RX: RDA status has PRX");
	check(rdw(0x4B004) == 0x3000 && rdw(0x4B006) == 0x8006,
	      "dirty-24bit RX: published packet pointer keeps the driver's top byte");
	check(sonic_reg(CRBA0) == 0x3040 && sonic_reg(CRBA1) == 0x8006,
	      "dirty-24bit RX: CRBA register advance preserves the top byte");
	check(backend_oob == 0, "dirty-24bit RX: no out-of-RAM address reached the backend");

	// ── sonic_rx_frame return contract (drives the host's redelivery queue) ──
	// -1 must mean "busy before any state was touched" (safe to retry the
	// same frame); 0 = filtered/dropped; 1 = delivered. The host holds
	// refused unicasts on -1 and redelivers when the ring frees - dropping
	// them instead cost a Linux-TCP RTO backoff spiral (rto 111 s, 55%
	// retransmit = the 2-4 KB/s FTP crawl measured 2026-08-26).
	check(sonic_reg(ISR) & 0x0040, "rx-contract: RDE latched by the previous test");
	check(sonic_rx_frame(other, 60) == -1, "rx-contract: RDE latched -> -1 (retryable)");
	sonic_reg_write(ISR, 0x0440);          // clear PKTRX + RDE; CRDA still odd EOL
	check(sonic_rx_frame(other, 60) == -1, "rx-contract: no free descriptor -> -1 (retryable)");
	wrw(0x4B00a, 0xB100);                  // driver appends: link now points to a fresh RDA
	wrw(0x4B10a, 0xB111);                  // its link: odd = end of list
	wrw(0x4B10c, 0xffff);                  // in-use
	check(sonic_rx_frame(other, 60) == 1, "rx-contract: reload finds the appended RDA -> 1");
	check(rdw(0x4B100) & 0x0001, "rx-contract: delivered via the appended descriptor");
	// Leaving a descriptor via the RELOAD path must release it exactly like
	// the normal advance does: in_use = 0. The Apple driver re-appends a
	// freed descriptor only when in_use is 0 (.ENET $15be), else defers it
	// to a list drained only on the next PKTRX - so a reload that keeps
	// in_use = FFFF leaks the descriptor and, at upload cadence (every
	// descriptor leaves via reload), starves the ring into a permanent
	// deaf-mute wedge (2026-08-26). Rev>3 silicon releases it (the driver
	// runs its TC chip-reset workaround only when SR <= 3, and we report 6).
	check(rdw(0x4B00c) == 0, "rx-contract: reload RELEASED the parked descriptor (in_use = 0)");
	// the model checks descriptor availability BEFORE the address filter, so
	// give it a free descriptor or the busy -1 masks the filter's 0
	wrw(0x4B10a, 0xB200);
	wrw(0x4B20a, 0xB211);
	wrw(0x4B20c, 0xffff);
	sonic_reg_write(ISR, 0x0440);
	uint8_t wrongdst[60];
	memcpy(wrongdst, frame, 60);
	wrongdst[5] ^= 0xff;                   // unknown unicast: CAM rejects
	check(sonic_rx_frame(wrongdst, 60) == 0, "rx-contract: address-filtered -> 0 (not retryable)");

	// ── watchdog timer: the driver's deadman must actually fire ──────
	// The Apple driver re-arms {WT1,WT0} on nearly every interrupt and
	// unmasks ISR_TC; its timeout/recovery paths (ring repair above all)
	// only run when TC fires. A model that stores the writes but never
	// counts leaves every wedge permanent (2026-08-26 upload stall #5).
	enum { WT0r = 0x29, WT1r = 0x2A };
	sonic_reg_write(0, 0x0020);            // CR: ST (start timer)
	sonic_reg_write(WT0r, 0x9C40);         // 40,000 counts = 4 ms at 10/us
	sonic_reg_write(WT1r, 0x0000);
	sonic_reg_write(4, 0x0080);            // IMR: unmask TC
	sonic_time_tick(1000);                 // 1 ms
	check(!(sonic_reg(5) & 0x0080), "watchdog: not expired at 1 ms");
	// one count per two bus clocks, 20 MHz crystal on the LC card = 10/us
	// (confirmed by the driver arming 0x02FAF080 = 50M counts = a round 5 s)
	check(sonic_reg(WT0r) == 0x9C40 - 10000, "watchdog: counts down at 10/us");
	sonic_time_tick(3500);                 // total 4.5 ms
	check(sonic_reg(5) & 0x0080, "watchdog: TC fired on expiry");
	check(sonic_int_line(), "watchdog: TC raises the int line via IMR");
	sonic_reg_write(5, 0x0080);            // ack TC
	sonic_time_tick(10000);
	check(!(sonic_reg(5) & 0x0080), "watchdog: no refire until re-armed");
	sonic_reg_write(WT0r, 0x0010);         // re-arm tiny
	sonic_time_tick(100);
	check(sonic_reg(5) & 0x0080, "watchdog: fires again after re-arm");
	sonic_reg_write(5, 0x7fff);            // clean slate for the protocol test

	// ── the Apple driver's recycle protocol vs the parked ring ───────
	// Faithful port of the .ENET 1.0.7 descriptor management (disassembled
	// from the guest System's 'enet' 46 resource):
	//   consume gate ($17ce): a descriptor is taken when its STATUS word is
	//     nonzero, then status+count are cleared;
	//   free gate ($15be):    a returned descriptor re-enters the ring ONLY
	//     if its in_use word is 0 - else it parks on a deferred list that
	//     drains ONLY at the top of the next PKTRX pass;
	//   append ($1544):       new tail gets {status 0, link 0x0001 = bare
	//     EOL, in_use FFFF} and the old tail's link is rewritten to the new
	//     descriptor's (even) address.
	// A bulk upload's steady state - one ACK consumed at the tail, park,
	// one descriptor re-appended - makes EVERY descriptor leave the ring
	// via the CRDA reload path. If the reload does not release the
	// descriptor it leaves (in_use = 0), every free defers, nothing ever
	// re-appends, and the guest is permanently deaf with CRDA parked on a
	// bare 0001 link: the 2026-08-26 upload wedge, reproduced here.
	{
		enum { NDESC = 4, NFRAMES = 24 };
		const uint32_t DBASE = 0x4D000;
		uint32_t head = 0, tail = 0;
		uint32_t deferred[8]; int ndef = 0, ndef_max = 0;
		int consumed = 0;

		// .ENET $1544: append a descriptor to the ring tail
		auto drv_append = [&](uint32_t a) {
			wrw(a + 0x0, 0);            // status
			wrw(a + 0x2, 0);            // count
			wrw(a + 0xA, 0x0001);       // link: bare end-of-list
			wrw(a + 0xC, 0xFFFF);       // in_use: owned by the chip
			if (tail) wrw(tail + 0xA, (uint16_t)(a - 0x40000));
			tail = a;
			if (!head) head = a;
		};
		// .ENET $15be: protocol stack returned a buffer
		auto drv_free = [&](uint32_t a) {
			if (rdw(a + 0xC) == 0) drv_append(a);
			else { deferred[ndef++] = a; if (ndef > ndef_max) ndef_max = ndef; }
		};
		// the ISR loop ($15d2): PKTRX fast path consumes + recycles;
		// RDE/RBE/TC handlers are ack + count, NOTHING else ($1658)
		auto drv_isr = [&]() {
			int guard = 64;
			while (sonic_int_line() && guard--) {
				uint16_t isr = sonic_reg(ISR);
				if (isr & 0x0400) {
					sonic_reg_write(ISR, 0x0400);
					// deferred drain at the top of the RX pass ($17a4),
					// still gated on in_use == 0
					for (int i = 0; i < ndef; ) {
						if (rdw(deferred[i] + 0xC) == 0) {
							uint32_t d = deferred[i];
							deferred[i] = deferred[--ndef];
							drv_append(d);
						} else i++;
					}
					while (head && rdw(head + 0x0) != 0) {
						uint16_t link = rdw(head + 0xA);
						wrw(head + 0x0, 0); wrw(head + 0x2, 0);
						uint32_t freed = head;
						head = (link & 1) ? 0 : (0x40000 + (uint32_t)link);
						consumed++;
						drv_free(freed);
					}
					continue;
				}
				if (isr & 0x0200) { sonic_reg_write(ISR, 0x0200); continue; }
				uint16_t rest = (uint16_t)(isr & 0x66F1 & ~0x0600);
				if (!rest) break;
				for (int b = 15; b >= 0; b--)
					if (rest & (1u << b)) { sonic_reg_write(ISR, (uint16_t)(1u << b)); break; }
			}
		};

		sonic_reg_write(IMR, 0x66F1);      // the driver's real mask
		sonic_reg_write(RCR, 0x2000);      // BRD
		sonic_reg_write(CR, 0x0008);       // RXEN
		sonic_reg_write(CRBA0, 0x6000);    // fresh buffer at $66000
		sonic_reg_write(CRBA1, 0x0006);
		sonic_reg_write(RBWC0, 0x4000);    // 32 KB: no RRA refill mid-test
		sonic_reg_write(RBWC1, 0x0000);
		for (int i = 0; i < NDESC; i++) drv_append(DBASE + i * 0x10);
		sonic_reg_write(CRDA, 0xD000);     // driver-open's single CRDA write

		int delivered = 0, parks_seen = 0, wedged_at = -1;
		for (int f = 0; f < NFRAMES && wedged_at < 0; f++) {
			int tries = 0, park_this = 0;
			for (;;) {
				if (sonic_reg(CRDA) & 1) park_this = 1;
				int r = sonic_rx_frame(other, 60);
				if (r == 1) { delivered++; break; }
				// refused: the host's elasticity queue holds the frame and
				// retries; the guest gets its interrupt if one is pending
				if (sonic_int_line()) drv_isr();
				else if (++tries > 8) { wedged_at = f; break; }
			}
			parks_seen += park_this;
			// service only every 4th frame: the chip must reach the tail
			// and park, as it does between real upload-ACK interrupts
			if ((f & 3) == 3 && sonic_int_line()) drv_isr();
		}
		if (sonic_int_line()) drv_isr();

		check(wedged_at < 0, "driver-protocol: no permanent wedge at upload cadence");
		check(delivered == NFRAMES, "driver-protocol: all 24 frames delivered");
		check(parks_seen >= 3, "driver-protocol: the ring parked repeatedly (reload path exercised)");
		check(ndef_max >= 1, "driver-protocol: parked descriptor took the deferred path (in_use gate)");
		check(consumed == NFRAMES, "driver-protocol: guest consumed every frame");
	}

	printf(fails ? "%d FAILURES (mac_sonic_test)\n" : "ALL PASS (mac_sonic_test)\n", fails);
	return fails != 0;
}
