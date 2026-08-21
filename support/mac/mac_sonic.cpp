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

#define EA(hi, lo) ((uint32_t)((uint32_t)(hi) << 16 | (lo)))
#define WIDTH()    ((reg[DCR] & DCR_DW) ? 4 : 2)

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

void sonic_fill_shadows(uint16_t out[64]) { memcpy(out, reg, sizeof reg); }

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

// frame WITHOUT FCS; model appends the computed FCS (see header note)
void sonic_rx_frame(const uint8_t *frame, int len)
{
	uint8_t buf[1524];

	if (!host) return;
	if (len <= 0 || len > 1518) return;
	if (!(reg[CR] & CR_RXEN) || (reg[ISR] & (ISR_RDE | ISR_RBE))) return;

	// reload receive descriptor address after end-of-list
	if (reg[CRDA] & 1) {
		uint16_t v;
		if (host->read_words(EA(reg[URDA], reg[LLFA]), &v, 1, WIDTH())) return;
		reg[CRDA] = v;
		if (reg[CRDA] & 1) return;
	}

	reg[RCR] &= (uint16_t)~(RCR_MC | RCR_BC | RCR_LPKT | RCR_CRCR | RCR_FAER | RCR_LBK | RCR_PRX);

	if (!address_filter(frame)) return;

	memcpy(buf, frame, len);
	uint32_t const fcs = crc32_eth(buf, len);
	buf[len + 0] = (uint8_t)(fcs >> 0);
	buf[len + 1] = (uint8_t)(fcs >> 8);
	buf[len + 2] = (uint8_t)(fcs >> 16);
	buf[len + 3] = (uint8_t)(fcs >> 24);
	int length = len + 4;

	if (length < 64 && !(reg[RCR] & RCR_RNT)) return;
	reg[RCR] |= RCR_PRX;
	if (reg[RCR] & RCR_LB) reg[RCR] |= RCR_LBK;

	// save rba pointers
	reg[TRBA0] = reg[CRBA0];
	reg[TRBA1] = reg[CRBA1];
	reg[TBWC0] = reg[RBWC0];
	reg[TBWC1] = reg[RBWC1];

	// store packet to the rba FIRST (write order is driver-visible)
	uint32_t const rba = EA(reg[CRBA1], reg[CRBA0]);
	if (host->write_bytes(rba, buf, length)) return;

	uint32_t const crba = rba + length;
	reg[CRBA1] = (uint16_t)(crba >> 16);
	reg[CRBA0] = (uint16_t)crba;

	uint32_t const rbwc = (((uint32_t)reg[RBWC1] << 16) | reg[RBWC0]) - (length + 1) / 2;
	reg[RBWC1] = (uint16_t)(rbwc >> 16);
	reg[RBWC0] = (uint16_t)rbwc;
	if (rbwc < reg[EOBC]) reg[RCR] |= RCR_LPKT;

	// write the 5-word RDA status, then the link handling
	const int w = WIDTH();
	uint32_t const rda = EA(reg[URDA], reg[CRDA]);
	uint16_t st[5] = { reg[RCR], (uint16_t)length, reg[TRBA0], reg[TRBA1], reg[RSC] };
	if (host->write_words(rda, st, 5, w)) return;
	reg[LLFA] = (uint16_t)(reg[CRDA] + 5 * w);
	uint16_t link;
	if (host->read_words(rda + 5 * w, &link, 1, w)) return;
	reg[CRDA] = link;

	if (reg[CRDA] & 1)
		reg[ISR] |= ISR_RDE;
	else {
		uint16_t zero = 0;
		host->write_words(rda + 6 * w, &zero, 1, w);   // clear in-use
	}

	if (rbwc < reg[EOBC])
		read_rra(0);
	else
		reg[RSC] = (uint16_t)((reg[RSC] & 0xff00) | (uint8_t)(reg[RSC] + 1));

	reg[ISR] |= ISR_PKTRX;
}

// ── transmit ────────────────────────────────────────────────────────────
static void transmit_chain(void)
{
	// synchronous port of MAME transmit() + send_complete_cb(), looping the
	// descriptor chain until end-of-list or HTX
	for (;;) {
		const int w = WIDTH();
		reg[TTDA] = reg[CTDA];
		uint32_t const tda = EA(reg[UTDA], reg[CTDA]);
		unsigned word = 1;   // word 0 is the status slot

		uint16_t const tcr_old = reg[TCR];
		uint16_t hdr[3];
		if (host->read_words(tda + 1 * w, hdr, 3, w)) return;
		reg[TCR] = hdr[0] & TCR_TPC;
		reg[TPS] = hdr[1];
		reg[TFC] = hdr[2];
		word += 3;

		if ((reg[TCR] & TCR_PINT) && !(tcr_old & TCR_PINT))
			reg[ISR] |= ISR_PINT;

		uint8_t buf[1520];
		unsigned length = 0;

		for (unsigned frag = 0; frag < reg[TFC]; frag++) {
			uint16_t fr[3];
			if (host->read_words(tda + word * w, fr, 3, w)) return;
			reg[TSA0] = fr[0];
			reg[TSA1] = fr[1];
			reg[TFS]  = fr[2];
			word += 3;

			if (length + reg[TFS] > sizeof buf - 4) { reg[CR] &= (uint16_t)~CR_TXP; return; }
			if (host->read_bytes(EA(reg[TSA1], reg[TSA0]), buf + length, reg[TFS])) return;
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
		// driver open-time self-test); otherwise out the wire
		if (reg[RCR] & RCR_LB) {
			int plen = (int)length;
			if (!(reg[TCR] & TCR_CRCI)) plen -= 4;   // rx path re-appends FCS
			sonic_rx_frame(buf, plen);
		} else
			host->wire_send(buf, (int)length);

		// completion (MAME send_complete_cb, success path)
		reg[TCR] |= TCR_PTX;
		uint16_t st = reg[TCR] & TCR_TPS_;
		if (host->write_words(EA(reg[UTDA], reg[TTDA]), &st, 1, w)) return;

		if (reg[CR] & CR_HTX) {
			reg[CR] &= (uint16_t)~CR_TXP;
			return;
		}

		uint16_t link;
		if (host->read_words(EA(reg[UTDA], reg[CTDA]), &link, 1, w)) return;
		reg[CTDA] = link;
		if (reg[CTDA] & 1) {
			reg[ISR] |= ISR_TXDN;
			reg[CR]  &= (uint16_t)~CR_TXP;
			return;
		}
		// else: chain to the next packet
	}
}

// ── command / register writes ───────────────────────────────────────────
static void command(uint16_t param)
{
	if (param & CR_HTX)   reg[CR] &= (uint16_t)~CR_TXP;
	if (param & CR_TXP) { reg[CR] &= (uint16_t)~CR_HTX; transmit_chain(); }
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
