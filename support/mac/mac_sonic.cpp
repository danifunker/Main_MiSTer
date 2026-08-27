// DP83932/DP83934 SONIC model — see mac_sonic.h. Flow-for-flow port of
// MAME's dp83932c.cpp (BSD-3-Clause, Patrick Mackinlay) with these
// deliberate differences, each grounded in the MacLC port's needs:
//   * descriptor/packet memory is guest RAM via injected host ops, with
//     GROUPED transfers (one DMA-RPC per descriptor cluster / packet body)
//     instead of MAME's per-word bus calls — the grouping preserves the
//     driver-visible write ORDER (packet bytes before the RDA status that
//     publishes them, status before the link/in-use updates);
//   * received frames arrive WITHOUT FCS (tap/raw socket); the model
//     appends the computed FCS so buffer contents and byte counts match
//     real SONIC behaviour, and skips MAME's residue check (which assumed
//     FCS-carrying frames);
//   * loopback (RCR b9-8) short-circuits a transmitted frame back into the
//     receive path instead of MAME's network-layer echo — Mac drivers
//     self-test at open (the v1 DP8390 lesson, docs/pds_ethernet_scope.md);
//   * the TXP descriptor chain runs as a synchronous loop (wire_send is
//     synchronous here), where MAME re-arms a zero-delay timer.

#include <string.h>
#include <stdio.h>
#include "mac_sonic.h"

// ── registers ───────────────────────────────────────────────────────────
enum {
	CR = 0x00, DCR, RCR, TCR, IMR, ISR, UTDA, CTDA,
	TPS, TFC, TSA0, TSA1, TFS, URDA, CRDA, CRBA0,
	CRBA1, RBWC0, RBWC1, EOBC, URRA, RSA, REA, RRP,
	RWP, TRBA0, TRBA1, TBWC0, TBWC1, ADDR0, ADDR1, LLFA,
	TTDA, CEP, CAP2, CAP1, CAP0, CE, CDP, CDC,
	SR, WT0, WT1, RSC, CRCT, FAET, MPT, MDT,
	DCR2 = 0x3f
};

#define CR_HTX   0x0001
#define CR_TXP   0x0002
#define CR_RXDIS 0x0004
#define CR_RXEN  0x0008
#define CR_STP   0x0010
#define CR_ST    0x0020
#define CR_RST   0x0080
#define CR_RRRA  0x0100
#define CR_LCAM  0x0200

#define DCR_DW   0x0020
#define DCR_EXBUS 0x8000
#define DCR_LBR  0x2000

#define RCR_PRX  0x0001
#define RCR_LBK  0x0002
#define RCR_FAER 0x0004
#define RCR_CRCR 0x0008
#define RCR_LPKT 0x0040
#define RCR_BC   0x0080
#define RCR_MC   0x0100
#define RCR_LB   0x0600
#define RCR_AMC  0x0800
#define RCR_PRO  0x1000
#define RCR_BRD  0x2000
#define RCR_RNT  0x4000
#define RCR_ERR  0x8000

#define TCR_PTX  0x0001
#define TCR_NCRS 0x0100
#define TCR_CRCI 0x2000
#define TCR_PINT 0x8000
#define TCR_TPC  0xf000
#define TCR_TPS_ 0x07ff
#define TCR_BCM  0x0002

#define ISR_RBE  0x0020
#define ISR_RDE  0x0040
#define ISR_TC   0x0080
#define ISR_TXDN 0x0200
#define ISR_PKTRX 0x0400
#define ISR_PINT 0x0800
#define ISR_LCD  0x1000

// per-register write masks (MAME regmask[])
static const uint16_t regmask[64] = {
	0x03bf, 0xbfff, 0xfe00, 0xf000, 0x7fff, 0x7fff, 0xffff, 0xffff,
	0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
	0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xfffe, 0xfffe, 0xfffe,
	0xfffe, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
	0xffff, 0x000f, 0x0000, 0x0000, 0x0000, 0xffff, 0xfffe, 0x001f,
	0x0000, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x0000,
	0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
	0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xf017,
};

static const sonic_host_ops *host;
static uint16_t reg[64];
static uint64_t cam[16];
static uint16_t isr_seen;   // ack-clearable ISR bits; see sonic_fill_shadows

// Bus effective address. The LC PDS exposes 24 address lines, so the real
// card's DMA only ever drives A0-A23: whatever the driver leaves in the top
// byte of a pointer never reaches memory. That byte is NOT always zero — a
// 24-bit-mode System stores Memory Manager flags there (bit 31 = locked, and
// a DMA buffer is locked while pinned), so TX fragment pointers arrive as
// $80xxxxxx while ARP frames — sent from driver-owned NewPtr buffers with a
// clean top byte — pass. Reading those raw sent the RPC outside guest RAM
// (and the dirty byte into the mailbox COUNT field, bits 47:40 vs the 24-bit
// addr field 39:16), so every multi-fragment (IP) transmit aborted silently:
// the exact HW fingerprint "arp tx 33, ip tx 0". Mask to the slot's physical
// 24 bits, exactly like the hardware; ea_stripped counts the dirty ones as a
// witness (surfaced in /tmp/mac_eth_stats). Register ARITHMETIC (e.g. the
// CRBA advance) must use the raw 32-bit combine so guest-visible register
// values keep their top byte like real silicon — mask only at the bus.
static uint32_t ea_stripped_cnt;
static inline uint32_t ea24(uint32_t hi, uint32_t lo)
{
	uint32_t a = (hi << 16) | lo;
	if (a >> 24) ea_stripped_cnt++;
	return a & 0x00ffffffu;
}
#define EA(hi, lo) ea24((uint16_t)(hi), (uint16_t)(lo))
#define WIDTH()    ((reg[DCR] & DCR_DW) ? 4 : 2)

uint32_t sonic_ea_stripped(void) { return ea_stripped_cnt; }

// Descriptor (word) effective address. A link value's LSB is the END-OF-LIST
// flag, NOT an address bit: the 16-bit SONIC bus has no A0, so a descriptor
// pointer that still carries the EOL bit (e.g. CTDA left at an odd link after
// a completed transmit) must be fetched WORD-ALIGNED. MAME reaches this via
// address_space::read_word() (which drops A0); our DMA-RPC backend instead
// REQUIRES even addresses and rejects odd ones, so an odd descriptor fetch
// aborted transmit_chain mid-way and left CR.TXP set — a guest that spin-polls
// TXP (or waits for TXDN) then hung forever at driver-open time. Mask A0 here
// exactly where a link value is used as an address. (Byte/packet-buffer
// pointers — CRBA, TSA — are NOT descriptor pointers and keep EA().)
#define DA(hi, lo) (EA(hi, lo) & ~(uint32_t)1)

// TX-path witnesses: every silent exit gets a counter, so a "the guest
// stopped sending" stall can be split between the model and the guest with
// one look at /tmp/mac_eth_stats (2026-08-26: MacTCP waited forever on a
// send completion; nothing said whether the chain aborted, the kick was
// swallowed, or the driver never kicked).
sonic_tx_debug_t sonic_txd;

// Any chain exit that is not a completed send PARKS on the descriptor it was
// reading: CTDA is restored to that descriptor's address with the EOL bit,
// TXP clears and TXDN raises exactly as at end-of-list, and the guest's next
// kick RE-READS the descriptor from its start. The old TX_ABORT advanced
// past the header words it had consumed, so one bad walk left CTDA pointing
// MID-DESCRIPTOR and every later kick gathered garbage from a misaligned
// ring - the permanently dead pipeline behind the 2026-08-27 upload stall
// (ovs=1 then silence). Parking makes every stop recoverable: the driver
// keeps kicking (its own retry paths), and a re-read of a by-then-finished
// descriptor proceeds normally.
#define TX_PARK(ttda) do { \
	reg[CTDA] = (uint16_t)((ttda) | 1); \
	reg[CR]  &= (uint16_t)~CR_TXP; \
	reg[ISR] |= ISR_TXDN; \
	return; } while (0)
#define TX_ABORT() do { sonic_txd.aborts++; TX_PARK(reg[TTDA]); } while (0)

// ── ethernet CRC32 (reflected, poly 0xEDB88320), FCS byte order LE ──────
static uint32_t crc32_eth(const uint8_t *p, int n)
{
	uint32_t c = 0xFFFFFFFFu;
	for (int i = 0; i < n; i++) {
		c ^= p[i];
		for (int k = 0; k < 8; k++)
			c = (c >> 1) ^ (0xEDB88320u & (0-(c & 1)));
	}
	return ~c;
}

void sonic_init(const sonic_host_ops *ops)
{
	host = ops;
	memset(reg, 0, sizeof reg);
	reg[SR] = 6;   // DP83932CVF silicon revision
	sonic_reset();
}

void sonic_reset(void)
{
	// permissive until the first pushes cycle: before the host publishes
	// shadows the guest cannot be reacting to unseen bits anyway
	isr_seen  = 0xFFFF;
	reg[CR]   = CR_RST | CR_STP | CR_RXDIS;
	reg[DCR] &= (uint16_t)~(DCR_EXBUS | DCR_LBR);
	reg[RCR] &= (uint16_t)~(RCR_RNT | RCR_BRD | RCR_LB);
	reg[TCR] |= TCR_NCRS | TCR_PTX;
	reg[TCR] &= (uint16_t)~(TCR_TPC | TCR_BCM);
	reg[IMR]  = 0;
	reg[ISR]  = 0;
	reg[EOBC] = 0x02f8;
	reg[CE]   = 0;
	reg[RSC]  = 0;
	reg[DCR2] = 0;
}

uint16_t sonic_reg(int r) { return reg[r & 0x3f]; }

// ISR bits the guest can legitimately acknowledge: only what has been
// PUSHED to it (sonic_fill_shadows = the publish) since the bit last
// cleared. On real silicon an ISR write-1-clear lands at a bus instant, so
// a bit set AFTER it stays set. Here the guest composes its ack from a
// shadow read and the ack applies milliseconds later - clearing a bit the
// guest never saw. Watched on HW 2026-08-26: two transmit chains complete
// close together, the ack meant for the first TXDN applies after the model
// set the second, the completion interrupt is eaten, the driver's TXDN
// walker never re-runs, and its send queue jams with the kick gated on a
// walk that never comes - the bulk-upload 8KB freeze. Clamping the ack to
// pushed-and-still-set bits makes the second TXDN survive and re-interrupt.
// Blind clears of already-clear bits (driver-open writes ISR before
// enabling anything) are naturally unaffected: clearing a clear bit is a
// no-op whether or not it is "seen".
void sonic_fill_shadows(uint16_t out[64])
{
	isr_seen |= reg[ISR];
	memcpy(out, reg, sizeof reg);
}

int sonic_int_line(void) { return (reg[ISR] & reg[IMR] & 0x7fff) != 0; }

// ── RRA / CAM ───────────────────────────────────────────────────────────
static void read_rra(int command)
{
	const int w = WIDTH();
	uint16_t v[4];
	if (host->read_words(EA(reg[URRA], reg[RRP]), v, 4, w)) return;
	reg[CRBA0] = v[0];
	reg[CRBA1] = v[1];
	reg[RBWC0] = v[2];
	reg[RBWC1] = v[3];

	reg[RRP] += 4 * w;
	if (reg[RRP] == reg[REA]) reg[RRP] = reg[RSA];
	if (reg[RRP] == reg[RWP]) reg[ISR] |= ISR_RBE;

	if (command)
		reg[CR] &= (uint16_t)~CR_RRRA;
	else
		reg[RSC] = (uint16_t)((reg[RSC] & 0xff00) + 0x100);
}

static void load_cam(void)
{
	const int w = WIDTH();
	while (reg[CDC]) {
		uint16_t v[4];
		if (host->read_words(EA(reg[URRA], reg[CDP]), v, 4, w)) return;
		unsigned cep = v[0] & 0xf;
		// stored word-swapped (MAME swapendian_int16 per word)
		cam[cep] = ((uint64_t)(uint16_t)((v[1] >> 8) | (v[1] << 8)) << 32)
		         | ((uint64_t)(uint16_t)((v[2] >> 8) | (v[2] << 8)) << 16)
		         | ((uint64_t)(uint16_t)((v[3] >> 8) | (v[3] << 8)) << 0);
		reg[CDP] += 4 * w;
		reg[CDC]--;
	}
	uint16_t ce;
	if (host->read_words(EA(reg[URRA], reg[CDP]), &ce, 1, WIDTH())) return;
	reg[CE] = ce;
	reg[CR] &= (uint16_t)~CR_LCAM;
	reg[ISR] |= ISR_LCD;
}

// ── receive ─────────────────────────────────────────────────────────────
static int address_filter(const uint8_t *buf)
{
	if (reg[RCR] & RCR_PRO) return 1;

	uint64_t const address =
		((uint64_t)buf[0] << 40) | ((uint64_t)buf[1] << 32) |
		((uint64_t)buf[2] << 24) | ((uint64_t)buf[3] << 16) |
		((uint64_t)buf[4] << 8)  | buf[5];

	if (address == 0xffffffffffffULL && (reg[RCR] & (RCR_AMC | RCR_BRD))) {
		reg[RCR] |= RCR_BC;
		return 1;
	}
	if ((address & 0x010000000000ULL) && (reg[RCR] & RCR_AMC)) {
		reg[RCR] |= RCR_MC;
		return 1;
	}
	for (unsigned i = 0; i < 16; i++)
		if (address == cam[i] && ((reg[CE] >> i) & 1))
			return 1;
	return 0;
}

// frame WITHOUT FCS; model appends the computed FCS (see header note).
// Return contract (see mac_sonic.h): -1 only on the BUSY exits that happen
// before any guest-visible state is touched, so the caller can safely retry
// the same frame later; anything past the first mutation returns 0/1.
int sonic_rx_frame(const uint8_t *frame, int len)
{
	uint8_t buf[1524];

	if (!host) return -1;
	if (len <= 0 || len > 1518) return 0;
	if (!(reg[CR] & CR_RXEN) || (reg[ISR] & (ISR_RDE | ISR_RBE))) return -1;

	// reload receive descriptor address after end-of-list
	if (reg[CRDA] & 1) {
		uint16_t v;
		if (host->read_words(DA(reg[URDA], reg[LLFA]), &v, 1, WIDTH())) return -1;
		reg[CRDA] = v;
		if (reg[CRDA] & 1) return -1;   // still no free descriptor: ring empty
		// The ring rejoined: RELEASE the descriptor being left (in_use = 0,
		// the field after the link LLFA points at). The normal advance below
		// releases only descriptors left with an even link; one left via THIS
		// reload kept in_use = FFFF forever (MAME has the same gap - its TODO
		// list stops at the "watchdog timers" this papered over). The Apple
		// LC driver re-appends a freed descriptor to the ring ONLY when
		// in_use is 0 ($c(desc) gate at .ENET 0x15be), else defers it to a
		// list drained ONLY on the next PKTRX - and a bulk upload's steady
		// state (single ACK consumed at the tail, park, single re-append)
		// makes EVERY descriptor leave via reload: all frees deferred, no
		// appends, no PKTRX possible = the 2026-08-26 upload wedge (crda
		// parked on a bare 0001 tail link, guest deaf-mute, clock ticking).
		// The driver only runs its chip-reset TC workaround on silicon
		// revision <= 3 (.ENET 0x166e reads SR, rts if > 3): rev>3 silicon
		// self-recovers, i.e. does exactly this release - and we report SR 6.
		uint16_t zero = 0;
		host->write_words(DA(reg[URDA], reg[LLFA]) + WIDTH(), &zero, 1, WIDTH());
	}

	reg[RCR] &= (uint16_t)~(RCR_MC | RCR_BC | RCR_LPKT | RCR_CRCR | RCR_FAER | RCR_LBK | RCR_PRX);

	if (!address_filter(frame)) return 0;

	memcpy(buf, frame, len);
	uint32_t const fcs = crc32_eth(buf, len);
	buf[len + 0] = (uint8_t)(fcs >> 0);
	buf[len + 1] = (uint8_t)(fcs >> 8);
	buf[len + 2] = (uint8_t)(fcs >> 16);
	buf[len + 3] = (uint8_t)(fcs >> 24);
	int length = len + 4;

	if (length < 64 && !(reg[RCR] & RCR_RNT)) return 0;
	reg[RCR] |= RCR_PRX;
	if (reg[RCR] & RCR_LB) reg[RCR] |= RCR_LBK;

	// save rba pointers
	reg[TRBA0] = reg[CRBA0];
	reg[TRBA1] = reg[CRBA1];
	reg[TBWC0] = reg[RBWC0];
	reg[TBWC1] = reg[RBWC1];

	// store packet to the rba FIRST (write order is driver-visible)
	// (advance the CRBA register from the RAW 32-bit value — real silicon
	// keeps the top byte the driver wrote; only the bus address is 24-bit)
	uint32_t const rba_reg = ((uint32_t)reg[CRBA1] << 16) | reg[CRBA0];
	if (host->write_bytes(EA(reg[CRBA1], reg[CRBA0]), buf, length)) return 0;

	uint32_t const crba = rba_reg + length;
	reg[CRBA1] = (uint16_t)(crba >> 16);
	reg[CRBA0] = (uint16_t)crba;

	uint32_t const rbwc = (((uint32_t)reg[RBWC1] << 16) | reg[RBWC0]) - (length + 1) / 2;
	reg[RBWC1] = (uint16_t)(rbwc >> 16);
	reg[RBWC0] = (uint16_t)rbwc;
	if (rbwc < reg[EOBC]) reg[RCR] |= RCR_LPKT;

	// Write the RDA: body words FIRST, the status word LAST. The status word
	// is the driver's publish flag - the Apple driver's ISR consumes a
	// descriptor the instant status goes nonzero and RECYCLES it (rewriting
	// link/in-use) within microseconds. On real silicon the chip's whole
	// status/link/in-use sequence lands in ~us, long before driver code can
	// run; here every transfer is a ~100 us DMA-RPC. The old order published
	// status first and read the link LAST, so a spinning driver could rewrite
	// the link while our read was in flight: CRDA left the ring (seen parked
	// mid-buffer-area on HW), later 5-word writes clobbered driver RAM, and
	// the driver ended in a link=0 self-orbit that hard-froze the guest
	// (2026-08-23 post-mortem via the guest-RAM dump). Nothing may be written
	// or read at this descriptor after its status is published.
	const int w = WIDTH();
	uint32_t const rda = DA(reg[URDA], reg[CRDA]);
	uint16_t st[5] = { reg[RCR], (uint16_t)length, reg[TRBA0], reg[TRBA1], reg[RSC] };
	if (host->write_words(rda + 1 * w, st + 1, 4, w)) return 0;   // count/ptrs/seq
	reg[LLFA] = (uint16_t)(reg[CRDA] + 5 * w);
	uint16_t link;
	if (host->read_words(rda + 5 * w, &link, 1, w)) return 0;
	if (!(link & 1)) {
		uint16_t zero = 0;
		host->write_words(rda + 6 * w, &zero, 1, w);   // clear in-use
	}
	if (host->write_words(rda, st, 1, w)) return 0;     // status: the publish
	reg[CRDA] = link;

	if (reg[CRDA] & 1)
		reg[ISR] |= ISR_RDE;

	if (rbwc < reg[EOBC])
		read_rra(0);
	else
		reg[RSC] = (uint16_t)((reg[RSC] & 0xff00) | (uint8_t)(reg[RSC] + 1));

	reg[ISR] |= ISR_PKTRX;
	return 1;
}

// ── transmit ────────────────────────────────────────────────────────────
// Bounded per invocation: the guest driver APPENDS descriptors while the
// chain runs (SONIC dynamic append), so a busy sender can keep the walk
// alive indefinitely - and this port is synchronous, called from the apply
// path, so an unbounded walk starves every other register write. The write
// stash (1024) overflowed in seconds that way (3,459 ISR acks lost, model
// desynced, guest TCP wedged - 2026-08-26 upload stall #3). Real silicon
// transmits concurrently with register writes; approximate that at packet
// granularity: up to TX_CHAIN_BUDGET packets per call, then return with
// CR.TXP still set (the dynamic-append guard swallows redundant kicks) and
// let sonic_tx_continue() resume from the poll after the stash has been
// applied.
#define TX_CHAIN_BUDGET 8
// One kick never revisits a descriptor. The driver's TX descriptors form a
// RING (in-place rebuild, one EOL link at the software tail); real silicon
// walks it at wire speed inside windows the driver's interrupts-off
// critical sections make atomic, so it always meets the tail EOL. This
// port's walk is minutes-of-bus-time slow and ASYNC to the guest, so it can
// catch the ring in a transient zero-EOL state (mid-recycle) and LAP it:
// watched on the wire 2026-08-27 as six identical TCP segments inside 6 ms
// (a budget-of-8 chain re-sending stale descriptors) followed by an
// oversize abort from a descriptor read mid-rebuild - the bulk-upload
// stall. A revisit is protocol-impossible on real hardware, so treat it
// exactly like end-of-list: stop, TXDN, park CTDA odd on the revisited
// address. Reset per fresh TXP command; a budget suspend/resume keeps the
// set (same chain).
#define TX_SEEN_MAX 64
static uint32_t tx_seen[TX_SEEN_MAX];
static int      tx_nseen;
void sonic_tx_new_chain(void) { tx_nseen = 0; }
static int tx_revisited(uint32_t da)
{
	for (int i = 0; i < tx_nseen; i++)
		if (tx_seen[i] == da) return 1;
	if (tx_nseen < TX_SEEN_MAX) tx_seen[tx_nseen++] = da;
	return tx_nseen >= TX_SEEN_MAX;   // set full: stop too (witnessed)
}
static void transmit_chain(void)
{
	sonic_txd.chains++;
	sonic_txd.last_ctda = reg[CTDA];
	// synchronous port of MAME transmit() + send_complete_cb(), looping the
	// descriptor chain until end-of-list or HTX
	for (int pkts = 0; pkts < TX_CHAIN_BUDGET; pkts++) {
		const int w = WIDTH();
		reg[TTDA] = reg[CTDA];
		uint32_t const tda = DA(reg[UTDA], reg[CTDA]);
		unsigned word = 1;   // word 0 is the status slot

		if (tx_revisited(tda)) {
			sonic_txd.laps++;
			TX_PARK(reg[CTDA]);
		}

		// status slot included in the same RPC: the driver's own convention
		// is status == 0 <=> queued-and-unsent (its builder zeroes it, the
		// chip writes nonzero completion, its walker writes FFFF when
		// consumed). Nonzero here means this walk has run into a descriptor
		// that is NOT a fresh send - completed awaiting recycle, or the far
		// side of a transient zero-EOL window - which real silicon never
		// reads because the driver's interrupts-off critical sections are
		// atomic against its wire-speed walk. Park as if it were EOL; the
		// guest's next kick re-reads it.
		uint16_t const tcr_old = reg[TCR];
		uint16_t hdr[4];
		if (host->read_words(tda, hdr, 4, w)) TX_ABORT();
		if (hdr[0] != 0) {
			sonic_txd.busy_stop++;
			TX_PARK(reg[CTDA]);
		}
		reg[TCR] = hdr[1] & TCR_TPC;
		reg[TPS] = hdr[2];
		reg[TFC] = hdr[3];
		word += 3;

		if ((reg[TCR] & TCR_PINT) && !(tcr_old & TCR_PINT))
			reg[ISR] |= ISR_PINT;

		uint8_t buf[1520];
		unsigned length = 0;

		for (unsigned frag = 0; frag < reg[TFC]; frag++) {
			uint16_t fr[3];
			if (host->read_words(tda + word * w, fr, 3, w)) TX_ABORT();
			reg[TSA0] = fr[0];
			reg[TSA1] = fr[1];
			reg[TFS]  = fr[2];
			word += 3;

			if (length + reg[TFS] > sizeof buf - 4) { sonic_txd.oversize++; TX_ABORT(); }
			if (host->read_bytes(EA(reg[TSA1], reg[TSA0]), buf + length, reg[TFS])) TX_ABORT();
			length += reg[TFS];
		}

		if (!(reg[TCR] & TCR_CRCI)) {
			uint32_t const crc = crc32_eth(buf, length);
			buf[length + 0] = (uint8_t)(crc >> 0);
			buf[length + 1] = (uint8_t)(crc >> 8);
			buf[length + 2] = (uint8_t)(crc >> 16);
			buf[length + 3] = (uint8_t)(crc >> 24);
			length += 4;
		}

		// advance ctda to the link field
		reg[CTDA] = (uint16_t)(reg[CTDA] + word * w);

		// send: loopback modes short-circuit into the receive path (Mac
		// driver open-time self-test); otherwise out the wire. BOTH paths
		// strip the FCS the model just computed: the rx path re-appends its
		// own, and the AF_PACKET wire gets a real FCS from the NIC - our 4
		// software bytes would ride as payload and push a max-size segment
		// (1514) to 1518, which the kernel refuses outright (EMSGSIZE: raw
		// sends cap at MTU 1500 + 14 header). That silently killed every
		// full-size TX frame - bulk uploads stalled at MacTCP's RTO cadence
		// (tx_bytes +1518 every ~15 s, tx_fail counting) while ARP/ACKs/
		// small segments passed under the limit (2026-08-26 upload stall).
		{
			int plen = (int)length;
			if (!(reg[TCR] & TCR_CRCI)) plen -= 4;
			if (reg[RCR] & RCR_LB)
				sonic_rx_frame(buf, plen);
			else
				host->wire_send(buf, plen);
		}

		// completion (MAME send_complete_cb, success path)
		sonic_txd.pkts++;
		reg[TCR] |= TCR_PTX;
		uint16_t st = reg[TCR] & TCR_TPS_;
		if (host->write_words(DA(reg[UTDA], reg[TTDA]), &st, 1, w)) TX_ABORT();

		if (reg[CR] & CR_HTX) {
			// Halt-transmit ends the chain, so it completes the transmit just
			// as end-of-list does. Every other exit raises TXDN (normal
			// completion and TX_ABORT both do); leaving it out here strands a
			// driver that halts TX and then waits for the interrupt.
			reg[ISR] |= ISR_TXDN;
			reg[CR] &= (uint16_t)~CR_TXP;
			return;
		}

		uint16_t link;
		if (host->read_words(DA(reg[UTDA], reg[CTDA]), &link, 1, w)) TX_ABORT();
		reg[CTDA] = link;
		if (reg[CTDA] & 1) {
			sonic_txd.ends_eol++;
			reg[ISR] |= ISR_TXDN;
			reg[CR]  &= (uint16_t)~CR_TXP;
			return;
		}
		// else: chain to the next packet
	}
	// budget exhausted with the chain still live: CR.TXP stays set and
	// sonic_tx_continue() picks the walk up from CTDA on the next poll
}

// Resume a budget-suspended transmit chain (call from the service poll,
// AFTER pending register writes have been applied - never re-entrantly).
void sonic_tx_continue(void)
{
	if ((reg[CR] & (CR_TXP | CR_RST)) == CR_TXP) transmit_chain();
}

// ── watchdog timer ──────────────────────────────────────────────────────
// {WT1,WT0} is the chip's 32-bit general-purpose down-counter, one count
// per two bus clocks: the LC card's SONIC runs from a 20 MHz crystal, so
// 10 counts/us. The Apple LC driver arms 0x02FAF080 = 50,000,000 counts =
// a round 5.0 s deadman (which is how the rate was confirmed) and re-arms
// it at the end of every receive pass. Disassembly of the driver's TC
// handler (.ENET 0x166e) showed the timeout RECOVERY - the CRDA-parked
// check and chip reset - runs only on silicon revision <= 3; on the SR 6
// we report, TC is acknowledged and counted, nothing more. So the timer
// exists for fidelity (the driver arms it and acks TC), not as the upload
// -wedge fix: that is the reload-path in_use release in sonic_rx_frame.
// Tick from the service poll with elapsed wall time; fires once per
// expiry, quiet until the driver rewrites WT.
void sonic_time_tick(unsigned us)
{
	if (!(reg[CR] & CR_ST)) return;
	uint32_t wt = ((uint32_t)reg[WT1] << 16) | reg[WT0];
	if (!wt) return;                      // expired and not yet re-armed
	uint32_t dec = us * 10;               // 20 MHz bus clock / 2
	if (wt > dec) {
		wt -= dec;
	} else {
		wt = 0;
		reg[ISR] |= ISR_TC;               // the deadman the driver waits for
	}
	reg[WT0] = (uint16_t)wt;
	reg[WT1] = (uint16_t)(wt >> 16);
}

// ── command / register writes ───────────────────────────────────────────
static void command(uint16_t param)
{
	if (param & CR_HTX)   reg[CR] &= (uint16_t)~CR_TXP;
	if (param & CR_TXP) { reg[CR] &= (uint16_t)~CR_HTX; sonic_tx_new_chain(); transmit_chain(); }
	if (param & CR_RXDIS) reg[CR] &= (uint16_t)~CR_RXEN;
	if (param & CR_RXEN)  reg[CR] &= (uint16_t)~CR_RXDIS;
	if (param & CR_STP)   reg[CR] &= (uint16_t)~CR_ST;
	if (param & CR_ST)    reg[CR] &= (uint16_t)~CR_STP;
	if (param & CR_RRRA)  read_rra(1);
	if (param & CR_LCAM)  load_cam();
}

void sonic_reg_write(int r, uint16_t data)
{
	r &= 0x3f;

	switch (r) {
	case CR:
		if (reg[CR] & CR_RST) {
			if (!(data & CR_RST))
				reg[CR] &= (uint16_t)~CR_RST;
		} else if (data & CR_RST) {
			reg[CR] &= (uint16_t)~(CR_LCAM | CR_RRRA | CR_TXP | CR_HTX);
			reg[CR] |= CR_RST | CR_RXDIS;
		} else {
			uint16_t cmd = data & regmask[r];
			// dynamic TDA append (datasheet 3.5.4): a TXP while TXP is
			// already running must not restart the engine
			if (data & CR_TXP) {
				if (reg[CR] & CR_TXP) sonic_txd.kicks_swallowed++;
				else if (reg[CTDA] & 1) sonic_txd.kicks_odd++;
				else sonic_txd.kicks_even++;
			}
			if (reg[CR] & CR_TXP) cmd &= (uint16_t)~CR_TXP;
			reg[r] |= data & regmask[r];
			command(cmd);
		}
		break;

	case RCR:
		reg[r] = (uint16_t)((reg[r] & ~regmask[r]) | (data & regmask[r]));
		break;

	case IMR:
		reg[r] = (uint16_t)((reg[r] & ~regmask[r]) | (data & regmask[r]));
		break;

	case ISR:
		// the guest can only clear bits it has SEEN (see isr_seen above);
		// a bit set since the last shadow push survives this ack and
		// re-raises the interrupt line
		data &= isr_seen;
		isr_seen &= (uint16_t)~data;
		// reload rra when RBE is cleared (MAME quirk, drivers rely on it)
		if ((reg[r] & ISR_RBE) && (data & ISR_RBE))
			read_rra(0);
		reg[r] &= (uint16_t)~(data & regmask[r]);
		break;

	case CRCT:
	case FAET:
	case MPT:
		reg[r] = (uint16_t)~data;   // tally counters are written inverted
		break;

	default:
		if (regmask[r])
			reg[r] = (uint16_t)((reg[r] & ~regmask[r]) | (data & regmask[r]));
		break;
	}
}
