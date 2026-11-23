/*
 *  ██████╗ ██████╗ ██╗███╗   ██╗
 * ██╔═══██╗██╔══██╗██║████╗  ██║
 * ██║   ██║██║  ██║██║██╔██╗ ██║
 * ██║   ██║██║  ██║██║██║╚██╗██║
 * ╚██████╔╝██████╔╝██║██║ ╚████║
 *  ╚═════╝ ╚═════╝ ╚═╝╚═╝  ╚═══╝
 *
 * Copyright (C) 2026 Réda Chérif-Touil
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "odin/machine/cpu/tms320c82/c82_pp.hpp"

#include <bit>
#include <cstring>

#include "odin/machine/cpu/tms320c82/tms320c82.hpp"

namespace odin {

namespace {

/* The class, A and F of BASE1 that have their own handlers (FORM_F),
 * the game's most frequent when it runs; F the same for both signs */
struct C82PpF {
	unsigned cls;
	bool arith;
	uint8_t f;
};
constexpr C82PpF C82PP_F[] = {
	{ 1, false, 0xF0 },
	{ 3, true, 0x6A },
	{ 0, false, 0x88 },
	{ 0, true, 0x66 },
	{ 1, true, 0xA5 },
	{ 7, true, 0xA5 },
	{ 1, true, 0x5A },
	{ 7, true, 0x66 },
	{ 7, true, 0x5A },
	{ 7, true, 0x99 },
	{ 2, true, 0x5A },
	{ 7, false, 0x88 },
	{ 7, false, 0xEE },
	{ 3, true, 0xA6 },
};
static_assert(sizeof(C82PP_F) / sizeof(C82PP_F[0]) * 3 == C82PpInsn::FORM_LEA - C82PpInsn::FORM_F);

constexpr uint32_t
bits(uint64_t x, unsigned hi, unsigned lo)
{
	return static_cast<uint32_t>((x >> lo) & ((1ull << (hi - lo + 1)) - 1));
}

constexpr bool
bit(uint64_t x, unsigned n)
{
	return (x >> n) & 1;
}

/* The parallel transfer format, from bits 38-37 (Lmode: 00xx is no local
 * transfer), 16-15 (Gmode), 24, 14-13 and 2 (fig. A-1 b) */
unsigned
pt_format(uint64_t op)
{
	const unsigned l = bits(op, 38, 37), g = bits(op, 16, 15);

	if (l && g)
		return 1;
	if (l) {
		if (bit(op, 24))
			return 4;

		const unsigned t = bits(op, 14, 13);

		return t == 0 ? 2 : t == 1 ? 3 : 6;
	}
	if (g)
		return bit(op, 2) ? 9 : 5;

	const unsigned t = bits(op, 14, 13);

	return t == 0 ? 7 : t == 1 ? 8 : 10;
}

C82PpXfer
xfer(bool global, unsigned mode, unsigned reg, unsigned ar, unsigned index, unsigned le, unsigned size, bool scale,
     unsigned rel)
{
	C82PpXfer t;

	t.global = global;
	t.mode = static_cast<uint8_t>(mode);
	t.reg = static_cast<uint8_t>(reg);
	t.ar = static_cast<uint8_t>(ar);
	t.index = static_cast<uint16_t>(index);
	t.le = static_cast<uint8_t>(le);
	t.size = static_cast<uint8_t>(size);
	t.scale = scale;
	t.rel = static_cast<uint8_t>(rel);
	return t;
}

/* For a byte access the s bit is the offset's MSB (A.4.1 s; table 8-33) */
unsigned
offset(uint64_t op, unsigned hi, unsigned lo, unsigned size, bool s, unsigned top)
{
	return bits(op, hi, lo) | (size == 0 && s ? top : 0);
}

void prepare(C82PpInsn &d);
void prepare_du(C82PpInsn &d);
void f_masks(C82PpInsn &d);

constexpr bool
index_mode(unsigned mode)
{
	return mode == 4 || mode == 5 || mode == 8 || mode == 9 || mode == 12 || mode == 13;
}

/* Appendix A.4: the ten parallel transfer formats */
void
decode_transfers(uint64_t op, C82PpInsn &d)
{
	const unsigned f = pt_format(op);
	C82PpXfer t[2];
	unsigned n = 0;

	d.pt = static_cast<uint8_t>(f);
	if (f == 1) {
		/* A.4.1: local then global; the global register is a lower one */
		unsigned size = bits(op, 30, 29);
		bool s = bit(op, 28);

		t[n++] = xfer(false, bits(op, 38, 35), 0x20 | bits(op, 34, 32), bits(op, 27, 25), offset(op, 2, 0, size, s, 8),
		              bit(op, 21) << 1 | bit(op, 31), size, s, 0);
		size = bits(op, 8, 7);
		s = bit(op, 6);
		t[n++] = xfer(true, bits(op, 16, 13), bits(op, 20, 18) << 3 | bits(op, 12, 10), 8 | bits(op, 5, 3),
		              offset(op, 24, 22, size, s, 8), bit(op, 17) << 1 | bit(op, 9), size, s, 0);
	} else if (f == 2 || f == 3 || f == 4 || f == 6) {
		/* A.4.2-A.4.4, A.4.6: L is bit 17 */
		const unsigned size = bits(op, 30, 29), le = bit(op, 17) << 1 | bit(op, 31);
		const bool s = bit(op, 28);

		if (f == 4)
			t[n++] = xfer(false, bits(op, 38, 35), bits(op, 21, 18) << 3 | bits(op, 34, 32), bits(op, 27, 25),
			              offset(op, 14, 0, size, s, 0x8000), le, size, s, bits(op, 23, 22));
		else
			t[n++] = xfer(false, bits(op, 38, 35), 0x20 | bits(op, 34, 32), bits(op, 27, 25),
			              offset(op, 2, 0, size, s, 8), le, size, s, f == 3 ? 0 : bits(op, 23, 22));
		if (f == 2) {
			d.move = true;
			d.move_dst = static_cast<uint8_t>(bits(op, 21, 18) << 3 | bits(op, 5, 3));
			d.move_src = static_cast<uint8_t>(bits(op, 9, 6) << 3 | bits(op, 12, 10));
		} else if (f == 3) {
			d.fmove = true;
			d.move_dst = static_cast<uint8_t>(bits(op, 21, 18) << 3 | bits(op, 5, 3));
			d.move_src = static_cast<uint8_t>(0x20 | bits(op, 12, 10));
			d.itm = static_cast<uint8_t>(bits(op, 23, 22));
			d.fsign = bit(op, 9);
			d.fsize = static_cast<uint8_t>(bits(op, 8, 7));
			d.replicate = bit(op, 6);
		} else if (f == 6) {
			d.dstbank = static_cast<uint8_t>(bits(op, 21, 18));
			d.s1bank = static_cast<uint8_t>(bits(op, 9, 6));
		}
	} else if (f == 5 || f == 9) {
		/* A.4.5, A.4.9: a 4-bit bank; Grm in bits 1-0 */
		const unsigned size = bits(op, 8, 7);
		const bool s = bit(op, 6);
		const unsigned index = f == 5 ? offset(op, 36, 22, size, s, 0x8000) : offset(op, 24, 22, size, s, 8);

		t[n++] = xfer(true, bits(op, 16, 13), bits(op, 21, 18) << 3 | bits(op, 12, 10), 8 | bits(op, 5, 3), index,
		              bit(op, 17) << 1 | bit(op, 9), size, s, bits(op, 1, 0));
	}
	if (f >= 7) {
		/* A.4.7-A.4.10 */
		d.cond = static_cast<uint8_t>(bits(op, 35, 32));
		d.c = bit(op, 31);
		d.r = bit(op, 30);
		d.g = f != 10 && bit(op, 29);
		d.prot = static_cast<uint8_t>(bits(op, 28, 25));
		if (f == 7) {
			d.move = true;
			d.move_dst = static_cast<uint8_t>(bits(op, 21, 18) << 3 | bits(op, 5, 3));
			d.move_src = static_cast<uint8_t>(bits(op, 9, 6) << 3 | bits(op, 12, 10));
		} else if (f == 8) {
			d.fmove = true;
			d.move_dst = static_cast<uint8_t>(bits(op, 21, 18) << 3 | bits(op, 5, 3));
			d.move_src = static_cast<uint8_t>(0x20 | bits(op, 12, 10));
			d.itm = static_cast<uint8_t>(bits(op, 23, 22));
			d.fsign = bit(op, 9);
			d.fsize = static_cast<uint8_t>(bits(op, 8, 7));
			d.replicate = bit(op, 6);
		} else if (f == 10) {
			d.dstbank = static_cast<uint8_t>(bits(op, 21, 18));
			d.s1bank = static_cast<uint8_t>(bits(op, 9, 6));
		}
	}
	/* Addressing modes 00xx: no transfer (table A-7) */
	for (unsigned k = 0; k < n; k++)
		if (t[k].mode >> 2)
			d.xfer[d.xfers++] = t[k];
}

} /* namespace */

/* Appendix A.3 (fig. A-1 a): bits 63-61 011 six-operand; 1 with class 000,
 * A = 1, the odd ALU bits and bit 44 at 0 miscellaneous; else bits 44-43
 * pick the 5-bit immediate (0x), the register (10) or the 32-bit
 * immediate (11); 00x and 010 are reserved */
C82PpInsn
c82_pp_decode(uint64_t op)
{
	C82PpInsn d;
	const unsigned top = static_cast<unsigned>(op >> 61);

	if (top == 3) {
		d.du = C82PpInsn::A;
		d.oper = static_cast<uint8_t>(bits(op, 60, 57));
		d.src3 = static_cast<uint8_t>(bits(op, 56, 54));
		d.dst2 = static_cast<uint8_t>(bits(op, 53, 51));
		d.dst = static_cast<uint8_t>(bits(op, 50, 48));
		d.src1 = static_cast<uint8_t>(bits(op, 47, 45));
		d.src4 = static_cast<uint8_t>(bits(op, 44, 42));
		d.src2 = static_cast<uint8_t>(bits(op, 41, 39));
		decode_transfers(op, d);
	} else if (bit(op, 63)) {
		d.cls = static_cast<uint8_t>(bits(op, 62, 60));
		d.arith = bit(op, 59);
		d.alu = static_cast<uint8_t>(bits(op, 58, 51));
		if (d.cls == 0 && d.arith && (op & 0x02A8100000000000ull) == 0) {
			d.du = C82PpInsn::E;
			d.misc = static_cast<uint8_t>(bits(op, 43, 39));
			/* nop, eint, dint; the other codes are reserved (A.3.5) */
			d.illegal = d.misc != 0 && d.misc != 2 && d.misc != 3;
			decode_transfers(op, d);
		} else {
			d.dst = static_cast<uint8_t>(bits(op, 50, 48));
			d.src1 = static_cast<uint8_t>(bits(op, 47, 45));

			const unsigned k = bits(op, 44, 43);

			if (k < 2) {
				d.du = C82PpInsn::B;
				d.src2_imm = true;
				d.imm = bits(op, 43, 39);
				decode_transfers(op, d);
			} else if (k == 2) {
				d.du = C82PpInsn::C;
				d.src2 = static_cast<uint8_t>(bits(op, 41, 39));
				decode_transfers(op, d);
			} else {
				/* A.3.4: the destination bank, a lower register's bank for
				 * src1, a condition (no parallel transfer) */
				d.du = C82PpInsn::D;
				d.dstbank = static_cast<uint8_t>(bits(op, 42, 39));
				d.s1bank = static_cast<uint8_t>(bits(op, 38, 36));
				d.cond = static_cast<uint8_t>(bits(op, 35, 32));
				d.r = d.cond != 0;
				d.src2_imm = true;
				d.imm = static_cast<uint32_t>(op);
			}
		}
	} else {
		d.illegal = true;
	}
	prepare(d);
	return d;
}

C82PpOp
c82_pp_op(const C82PpInsn &in)
{
	C82PpOp op;
	const C82PpXfer &t = in.xfer[0];

	op.form = in.form;
	op.xfers = in.xfers;
	op.illegal = in.illegal;
	op.g_change = in.g_change;
	for (unsigned k = 0; k < in.xfers; k++) {
		const C82PpXfer &x = in.xfer[k];

		op.agen[k] = { x.base_slot, x.index_slot, x.wb, x.shift, static_cast<uint16_t>(x.offset), x.sub, x.pre };
	}
	op.reg = t.reg;
	op.msize = t.msize;
	op.cond = in.cond;
	op.s1_slot = in.s1_slot;
	op.src2 = in.src2;
	op.dst = in.dst_code();
	op.f[0] = in.f_by_sign[0];
	op.f[1] = in.f_by_sign[1];
	op.cin_by_sign[0] = in.cin_by_sign[0];
	op.cin_by_sign[1] = in.cin_by_sign[1];
	op.cin_sel = in.cin_sel;
	op.prot = in.prot;
	op.flags = static_cast<uint16_t>(((t.le & 1) ? C82PpOp::SIGN : 0) | (t.divert ? C82PpOp::DIVERT : 0) |
	                                 (in.a_keep == 0 ? C82PpOp::A_ZERO : 0) | (in.reads_cond ? C82PpOp::READS_COND : 0) |
	                                 (in.r ? C82PpOp::R : 0) | (in.src2_imm ? C82PpOp::SRC2_IMM : 0) |
	                                 (in.sets_status ? C82PpOp::SETS_STATUS : 0) | (in.bang ? C82PpOp::BANG : 0) |
	                                 (in.s1_pair ? C82PpOp::S1_PAIR : 0) | (in.cn9 ? C82PpOp::CN9 : 0));
	op.imm = in.imm;
	return op;
}

/* Source 1, or the odd or even register of a D pair when c says so (§ 8.9.3:
 * odd if the condition holds) */
uint32_t
C82Pp::source1(const C82PpInsn &in, bool cond_true)
{
	const unsigned code = in.src1_code();

	if (in.c && (code & 0x78) == 0x20)
		return m_r[0x20 | (cond_true ? code | 1 : code & 6)];
	return get(code);
}

/* The expander (§ 4.2.3; registers mf and sr, tables 7-3 and 7-5).
 * HEURISTIC: a reserved Msize expands nothing */
uint32_t
C82Pp::expand_mf() const
{
	const bool r = m_sr & 0x40;

	switch ((m_sr >> 3) & 7) {
	case 4: {
		const unsigned flags = r ? m_mf >> 28 : m_mf;
		uint32_t v = 0;

		for (unsigned k = 0; k < 4; k++)
			if ((flags >> k) & 1)
				v |= 0xFFu << (8 * k);
		return v;
	}
	case 5: {
		const unsigned flags = r ? m_mf >> 30 : m_mf;

		return ((flags & 2) ? 0xFFFF0000u : 0) | ((flags & 1) ? 0xFFFFu : 0);
	}
	case 6:
		return ((r ? m_mf >> 31 : m_mf) & 1) ? 0xFFFFFFFFu : 0;
	default:
		return 0;
	}
}

namespace {

/* The three-input ALU (SPRU110 ex. 13-1): propagate = the minterms of A,
 * B, C enabled by F7-F0; generate (arithmetic only) = A & ~propagate &
 * the even minterm of B, C; result = propagate ^ carry-in. Split in 1, 2
 * or 4 pieces (§ 4.3.6), `cins` the carry-in of each piece. As propagate
 * and generate never overlap, each piece is the sum (G | P) + G + cin. */
struct PpAlu {
	uint32_t result = 0;
	/* Per piece: carry out, carry into the MSB, propagate of the MSB */
	unsigned cout = 0, cin_msb = 0, p_msb = 0;
};

PpAlu
pp_alu(uint8_t f, bool arith, uint32_t a, uint32_t b, uint32_t c, unsigned pieces, unsigned cins)
{
	/* The minterms of B and C; F's bits as masks: the even ones (A = 0)
	 * and the odd ones (A = 1) */
	const uint32_t t0 = ~b & ~c, t1 = b & ~c, t2 = ~b & c, t3 = b & c;
	const auto m = [f](unsigned k) { return 0u - ((f >> k) & 1u); };
	const uint32_t p0 = (m(0) & t0) | (m(2) & t1) | (m(4) & t2) | (m(6) & t3);
	const uint32_t p1 = (m(1) & t0) | (m(3) & t1) | (m(5) & t2) | (m(7) & t3);
	const uint32_t p = (~a & p0) | (a & p1);
	/* Arithmetic: the even minterm of B, C is p0 */
	const uint32_t g = arith ? a & ~p & p0 : 0;

	PpAlu out;
	const uint32_t x = g | p, y = g;

	if (pieces == 1) {
		const uint64_t sum = static_cast<uint64_t>(x) + y + (cins & 1);

		out.result = static_cast<uint32_t>(sum);
		out.cout = static_cast<unsigned>(sum >> 32) & 1;
		out.cin_msb = ((out.result ^ p) >> 31) & 1;
		out.p_msb = p >> 31;
		return out;
	}

	const unsigned width = 32 / pieces;
	const uint64_t mask = (1ull << width) - 1;

	for (unsigned k = 0; k < pieces; k++) {
		const unsigned shift = k * width;
		const uint64_t sum = ((x >> shift) & mask) + ((y >> shift) & mask) + ((cins >> k) & 1);

		out.result |= static_cast<uint32_t>((sum & mask) << shift);
		out.cout |= static_cast<unsigned>((sum >> width) & 1) << k;
	}

	/* The carry into each bit */
	const uint32_t carries = out.result ^ p;

	for (unsigned k = 0; k < pieces; k++) {
		const unsigned msb = k * width + width - 1;

		out.cin_msb |= ((carries >> msb) & 1u) << k;
		out.p_msb |= ((p >> msb) & 1u) << k;
	}
	return out;
}

/* The mask generator (§ 8.2.8, table 8-7): n right-justified ones; in
 * multiple arithmetic from 4 or 3 LSBs, replicated in each piece; %! makes
 * all ones of 0 (§ 8.3.4.5) */
uint32_t
pp_mask(uint32_t n, bool bang, unsigned pieces)
{
	if (pieces == 1) {
		n &= 31;
		return n ? (1u << n) - 1 : bang ? 0xFFFFFFFFu : 0;
	}

	const unsigned width = 32 / pieces;
	const unsigned m = n & (width - 1);
	const uint32_t piece = m ? (1u << m) - 1 : bang ? static_cast<uint32_t>((1ull << width) - 1) : 0;
	uint32_t v = 0;

	for (unsigned k = 0; k < pieces; k++)
		v |= piece << (k * width);
	return v;
}

/* Bit detection (§ 4.2.4, § 8.3.4.2, table 8-19): 31 minus the position of
 * the leftmost one (FMOD 2), rightmost one (3), leftmost bit unlike bit 31
 * (6), rightmost bit unlike bit 0 (7); 32 when there is none */
uint32_t
pp_bit_detect(unsigned fmod, uint32_t v, bool &none)
{
	uint32_t x = v;

	if (fmod == 6 && (v >> 31))
		x = ~v;
	else if (fmod == 7 && (v & 1))
		x = ~v;
	if (x == 0) {
		none = true;
		return 32;
	}
	none = false;
	if (fmod == 2 || fmod == 6)
		return static_cast<uint32_t>(std::countl_zero(x));
	return 31u - static_cast<uint32_t>(std::countr_zero(x));
}

/* Bit k: the LSB of piece k */
unsigned
pp_piece_lsbs(uint32_t v, unsigned pieces)
{
	unsigned out = 0;

	for (unsigned k = 0; k < pieces; k++)
		out |= ((v >> (k * (32 / pieces))) & 1u) << k;
	return out;
}

/* Bit k: the MSB of piece k */
unsigned
pp_piece_msbs(uint32_t v, unsigned pieces)
{
	unsigned out = 0;

	for (unsigned k = 0; k < pieces; k++)
		out |= ((v >> (k * (32 / pieces) + 32 / pieces - 1)) & 1u) << k;
	return out;
}

/* Bit k: piece k is zero */
unsigned
pp_piece_zeros(uint32_t v, unsigned pieces)
{
	const unsigned width = 32 / pieces;
	const uint64_t mask = width == 32 ? 0xFFFFFFFFull : (1ull << width) - 1;
	unsigned out = 0;

	for (unsigned k = 0; k < pieces; k++)
		if (((v >> (k * width)) & mask) == 0)
			out |= 1u << k;
	return out;
}

/* The multiplier (§ 4.7, § 8.5): 16 x 16, signed or unsigned, of the 16
 * LSBs. With an EALU, d0 adds (fig. 8-14; SPRS048 fig. 24): M two 8 x 8
 * (src3 unsigned bytes, src4 signed or not), DMS a left shift of 0-3, R a
 * rounding into the upper halfword with src3's upper halfword below, U
 * (C82) a saturation of signed products (SPRS048 table 11). HEURISTIC: the
 * shift of a split product shifts the whole word; with R and U the rounded
 * upper halfword saturates 0x4000 to 0x3FFF (DMS 0) and 0x8000 to 0x7FFF
 * (DMS 1), the data sheet's text giving 0x7FFF to 0x8000, the reverse of
 * its table */
uint32_t
pp_multiply(uint32_t s3, uint32_t s4, bool is_unsigned, uint32_t d0, bool ealu)
{
	const bool split = ealu && ((d0 >> 7) & 1);
	uint32_t r;

	if (split) {
		const int32_t b0 = is_unsigned ? static_cast<int32_t>(s4 & 0xFF) : static_cast<int8_t>(s4 & 0xFF);
		const int32_t b1 = is_unsigned ? static_cast<int32_t>((s4 >> 8) & 0xFF) : static_cast<int8_t>((s4 >> 8) & 0xFF);
		const uint32_t lo = static_cast<uint32_t>(static_cast<int32_t>(s3 & 0xFF) * b0) & 0xFFFF;
		const uint32_t hi = static_cast<uint32_t>(static_cast<int32_t>((s3 >> 8) & 0xFF) * b1) & 0xFFFF;

		r = hi << 16 | lo;
	} else if (is_unsigned) {
		r = (s3 & 0xFFFF) * (s4 & 0xFFFF);
	} else {
		r = static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(s3)) *
		                          static_cast<int32_t>(static_cast<int16_t>(s4)));
	}
	if (!ealu)
		return r;

	const unsigned dms = (d0 >> 8) & 3;
	const bool saturate = ((d0 >> 5) & 1) && !is_unsigned && !split && dms < 2;

	r <<= dms;
	if ((d0 >> 6) & 1) {
		uint32_t hi = ((r >> 16) + ((r >> 15) & 1)) & 0xFFFF;

		if (saturate && hi == (dms ? 0x8000u : 0x4000u))
			hi -= 1;
		return hi << 16 | (s3 >> 16);
	}
	if (saturate && r == (dms ? 0x80000000u : 0x40000000u))
		r -= 1;
	return r;
}

/* The rotation of mf for an Asize or Msize code (tables 7-5, 7-6) */
unsigned
mf_rotation(unsigned code)
{
	return code == 4 ? 4 : code == 5 ? 2 : 1;
}

/* Base set arithmetics (table A-5): the ALU code from the odd bits 57, 55,
 * 53, 51 and its carry-in. HEURISTIC: code 0 outside format E takes the
 * table's function bits 0xA8 with a carry-in of 0 */
constexpr uint8_t BASE_F[16] = { 0xA8, 0xA9, 0xA6, 0xA5, 0x9A, 0x99, 0x96, 0x95,
	                         0x6A, 0x69, 0x66, 0x65, 0x5A, 0x59, 0x56, 0x60 };
constexpr bool BASE_CIN[16] = { 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0 };

/* Pieces of the split ALU and the mf rotation for an Asize or Msize code
 * (tables 7-5, 7-6). HEURISTIC: a reserved code acts as 32 bits */
unsigned
pieces_of(unsigned code)
{
	return code == 4 ? 4 : code == 5 ? 2 : 1;
}

/* Status bits N C V Z of sr (sr register: status bits) */
constexpr uint32_t SR_N = 1u << 31, SR_C = 1u << 30, SR_V = 1u << 29, SR_Z = 1u << 28;

/* What only depends on the instruction word, worked out once: the
 * transfers' addressing and order, the data unit's way, the execute
 * stage's shape */
void
prepare(C82PpInsn &d)
{
	/* The transfers: their address (table A-7) and their order in the
	 * execute stage */
	for (unsigned k = 0; k < d.xfers; k++) {
		C82PpXfer &t = d.xfer[k];

		t.xreg = static_cast<uint8_t>(index_mode(t.mode) ? (t.global ? 0x18u : 0x10u) | (t.index & 7u) : 0);
		t.shift = static_cast<uint8_t>(t.scale && (t.size == 1 || t.size == 2) ? t.size : 0);
		t.sub = t.mode & 1;
		t.pre = t.mode >= 8;
		t.writeback = t.mode < 8 || t.mode >= 12;
		/* The same as base and index for the address stage: HEURISTIC, the
		 * reserved Lrm/Grm 01 as absolute; sp written back by code 6 */
		t.base_slot = static_cast<uint8_t>(t.rel == 2   ? c82_pp_regs::DBA
		                                   : t.rel == 3 ? c82_pp_regs::PBA
		                                                : c82_pp_regs::READ_SLOT[t.ar]);
		t.index_slot = static_cast<uint8_t>(t.xreg ? t.xreg : c82_pp_regs::ZERO);
		t.offset = t.xreg ? 0 : t.index;
		t.neg = t.sub ? ~0u : 0;
		t.pre_mask = t.pre ? ~0u : 0;
		t.wb = static_cast<uint8_t>(!t.writeback ? 0xFF : t.ar == 0xE ? 6 : t.ar);
		t.msize = static_cast<uint8_t>(t.size == 3 ? 2 : t.size);
		t.pair = t.le == 0 && t.global && d.g && (t.reg & 0x78) == 0x20;
		t.pcload = t.global && (t.reg == 0x38 || t.reg == 0x39) && t.le >= 2 && !t.scale && !index_mode(t.mode);
		t.divert = !t.global && d.pt == 1;
	}
	for (int pass = 1; pass >= 0; pass--)
		for (unsigned k = 0; k < d.xfers; k++)
			if (d.xfer[k].global == (pass == 1) && d.xfer[k].le != 1)
				d.mem_order[d.mems++] = static_cast<uint8_t>(k);
	for (int pass = 0; pass < 2; pass++)
		for (unsigned k = 0; k < d.xfers; k++)
			if (d.xfer[k].global == (pass == 1) && d.xfer[k].le != 0)
				d.write_order[d.writes++] = static_cast<uint8_t>(k);
	d.protect = ((d.prot >> 3) & 1u) << 31 | ((d.prot >> 2) & 1u) << 30 | ((d.prot >> 1) & 1u) << 29 | (d.prot & 1u) << 28;
	prepare_du(d);

	/* The execute stage's shape: a single plain load or store */
	const C82PpXfer &t = d.xfer[0];
	const bool simple = !d.move && !d.fmove && !d.illegal &&
	                    (d.path == C82PpInsn::NO_DU || (d.path == C82PpInsn::BASE1 && !d.s1_special));
	const bool plain = d.xfers == 1 && t.le != 1 && !t.pcload && !t.pair && !(t.global && d.g);

	if (simple && d.path == C82PpInsn::NO_DU && d.xfers == 1 && t.le == 1 && !(t.global && d.g))
		d.shape = C82PpInsn::LEA;
	else if (!simple || (d.xfers && !plain))
		d.shape = C82PpInsn::GENERIC_SHAPE;
	else if (d.path == C82PpInsn::NO_DU)
		d.shape = !d.xfers ? C82PpInsn::NOP : t.le == 0 ? C82PpInsn::STORE : C82PpInsn::LOAD;
	else
		d.shape = !d.xfers ? C82PpInsn::DU : t.le == 0 ? C82PpInsn::DU_STORE : C82PpInsn::DU_LOAD;
	if (d.shape == C82PpInsn::LEA)
		d.form = C82PpInsn::FORM_LEA;
	else if (d.shape == C82PpInsn::GENERIC_SHAPE || d.shape == C82PpInsn::NOP || d.shape == C82PpInsn::LOAD ||
	         d.shape == C82PpInsn::STORE)
		d.form = d.shape;
	else
		d.form = static_cast<uint8_t>(C82PpInsn::FORM_DU + 3 * (d.cls << 1 | d.arith) + (d.shape - C82PpInsn::DU));
	if (d.form >= C82PpInsn::FORM_DU && d.f_by_sign[0] == d.f_by_sign[1])
		for (unsigned k = 0; k < sizeof(C82PP_F) / sizeof(C82PP_F[0]); k++)
			if (C82PP_F[k].cls == d.cls && C82PP_F[k].arith == d.arith && C82PP_F[k].f == d.f_by_sign[0])
				d.form = static_cast<uint8_t>(C82PpInsn::FORM_F + 3 * k + (d.shape - C82PpInsn::DU));
}

/* The data unit's way for an instruction, and for the base set with one
 * piece the choices data_unit makes by class and code (table 8-13, A-5,
 * A-6), made once */
void
prepare_du(C82PpInsn &d)
{
	if (d.du == C82PpInsn::E && (d.misc == 2 || d.misc == 3))
		d.g_change = d.misc == 2;
	if (d.du == C82PpInsn::E || d.du == C82PpInsn::RESERVED) {
		d.path = C82PpInsn::NO_DU;
		return;
	}
	if (d.du == C82PpInsn::A) {
		/* 0u0s MPY||ADD (A.3.1): data_unit's case, by mpy_add */
		d.path = d.oper == 0 || d.oper == 1 || d.oper == 4 || d.oper == 5 ? C82PpInsn::MPY_ADD : C82PpInsn::GENERIC;
		return;
	}

	const unsigned cls = d.cls;
	const bool maskgen = cls == 2 || cls == 3 || cls == 4, uses_mf = cls == 0 || cls == 5;
	const unsigned code = (d.alu >> 3 & 8) | (d.alu >> 2 & 4) | (d.alu >> 1 & 2) | (d.alu & 1);
	const unsigned fmod = (d.alu >> 4 & 8) | (d.alu >> 3 & 4) | (d.alu >> 2 & 2) | (d.alu >> 1 & 1);
	const bool special = d.arith && (fmod == 2 || fmod == 3 || fmod == 6 || fmod == 7);
	const bool multiple = d.arith && fmod >= 8 && fmod != 15;
	const bool bang = special && maskgen;

	if (multiple || (special && !maskgen)) {
		d.path = C82PpInsn::GENERIC;
		return;
	}
	d.path = C82PpInsn::BASE1;
	d.reads_cond = d.r || d.c;
	/* Source 1 as source1() reads it */
	if (const unsigned s1 = d.src1_code(); d.c && (s1 & 0x78) == 0x20) {
		d.s1_slot = static_cast<uint8_t>(0x20 | (s1 & 6));
		d.s1_pair = 1;
	} else if (s1 < 0x28) {
		d.s1_slot = c82_pp_regs::READ_SLOT[s1];
	} else {
		d.s1_special = true;
	}
	/* Ports A, B, C by class: base1 */
	if (d.arith && (fmod == 4 || fmod == 5 || (bang && (fmod == 6 || fmod == 7))))
		d.a_keep = 0;
	d.bang = bang;
	d.sets_status = d.dst_code() != 0x38 && d.dst_code() != 0x39 && d.dst_code() != 0x29;
	if (!d.arith) {
		d.f_by_sign[0] = d.f_by_sign[1] = d.alu;
		f_masks(d);
		return;
	}
	d.f_by_sign[0] = d.f_by_sign[1] = BASE_F[code];
	d.cin_by_sign[0] = d.cin_by_sign[1] = BASE_CIN[code];
	if (code == 4) {
		d.f_by_sign[0] = 0x95;
		d.f_by_sign[1] = 0x9A;
	} else if (code == 0xB) {
		d.f_by_sign[0] = 0x6A;
		d.f_by_sign[1] = 0x65;
	} else if ((code == 6 || code == 9) && !uses_mf) {
		d.f_by_sign[1] = static_cast<uint8_t>(BASE_F[code] ^ 0x0F);
		d.cin_by_sign[0] = code != 6;
		d.cin_by_sign[1] = code == 6;
	}
	if ((code == 6 || code == 9) && uses_mf) {
		d.cin_sel = 2;
		d.cn9 = code == 9;
	} else if (fmod == 1 || fmod == 5 || (bang && (fmod == 3 || fmod == 7))) {
		d.cin_sel = 1;
	}
	f_masks(d);
}

/* f_masks for every F code, for alu1 (C82PpOp keeps F only) */
constexpr std::array<std::array<uint32_t, 8>, 256> F_MASKS = [] {
	std::array<std::array<uint32_t, 8>, 256> t{};

	for (unsigned f = 0; f < 256; f++) {
		const auto m = [f](unsigned k) { return 0u - ((f >> k) & 1u); };

		t[f] = { m(0), m(0) ^ m(2), m(4), m(4) ^ m(6), m(1), m(1) ^ m(3), m(5), m(5) ^ m(7) };
	}
	return t;
}();

/* F's bits as masks of 32 bits, for each sign (C82PpInsn::f_masks) */
void
f_masks(C82PpInsn &d)
{
	for (unsigned sign = 0; sign < 2; sign++) {
		const auto m = [&d, sign](unsigned k) { return 0u - ((d.f_by_sign[sign] >> k) & 1u); };
		uint32_t *f = d.f_masks[sign];

		f[0] = m(0);
		f[1] = m(0) ^ m(2);
		f[2] = m(4);
		f[3] = m(4) ^ m(6);
		f[4] = m(1);
		f[5] = m(1) ^ m(3);
		f[6] = m(5);
		f[7] = m(5) ^ m(7);
	}
}

} /* namespace */

/* The data unit of one instruction (ch. 4, § 8.3-8.6): what it writes and
 * the status it sets, written by the caller in the order of § 8.11 */
void
C82Pp::data_unit(const C82PpInsn &in, bool cond_true, DuOut &out)
{
	const bool keep = !in.r || cond_true;
	const uint32_t d0 = m_r[0x20];
	const unsigned asize = m_sr & 7;
	uint8_t f = 0;
	bool arith = false;
	unsigned fmod = 0, pieces = 1, mask_pieces = 1;
	uint32_t a = 0, b = 0, rot_in = 0, c = 0, cins = 0;
	bool maskgen = false, uses_mf = false, bitdet = false, bang = false;
	bool saturate = false, divi = false, sign = false;
	uint32_t mask_n = 0;
	unsigned rot = 0;
	enum { C_DIRECT, C_MASK, C_MF } cport = C_DIRECT;
	const uint8_t dst = in.dst_code();

	if (in.du == C82PpInsn::E || in.du == C82PpInsn::RESERVED)
		return;

	const uint32_t s1 = source1(in, cond_true);

	if (in.du == C82PpInsn::A) {
		const uint32_t s2 = m_r[0x20 | in.src2], s3 = m_r[0x20 | in.src3], s4 = m_r[0x20 | in.src4];
		const unsigned oper = in.oper;
		bool ealu = false, ealuf = false;
		/* The second destination: multiply or rotate result (A.3.1) */
		bool multiply = false;

		a = s2;
		rot_in = s1;
		if (oper == 0 || oper == 1 || oper == 4 || oper == 5) {
			/* 0u0s MPY||ADD: src2 +/- src1 */
			multiply = true;
			out.second_value = pp_multiply(s3, s4, oper & 4, 0, false);
			b = s1;
			arith = true;
			f = (oper & 1) ? 0x99 : 0x66;
			cins = (oper & 1) ? 1 : 0;
		} else if (oper >= 12) {
			/* 11us MPY||SADD: src2 +/- src1 >> -d0, signed with the
			 * multiply or not; the shift as the class-specific arithmetics
			 * do it (table A-5 codes 4, 7, 8, B; § 8.2.10) */
			const bool u = oper & 2;

			multiply = true;
			out.second_value = pp_multiply(s3, s4, u, 0, false);
			rot = d0 & 31;
			b = std::rotl(s1, static_cast<int>(rot));
			sign = s1 >> 31;
			c = pp_mask(rot, true, 1);
			arith = true;
			if (oper & 1) {
				f = !u && sign ? 0x9A : 0x95;
				cins = 1;
			} else {
				f = !u && sign ? 0x65 : 0x6A;
			}
		} else if (oper == 0xA) {
			/* divi (§ 8.6.2): the rotator shifts src1 left with the MSB of
			 * mf in; the ALU adds the negated divisor (src2) */
			divi = true;
			b = (s1 << 1) | (m_mf >> 31);
			sign = s1 >> 31;
			arith = true;
			f = 0x66;
			out.second_value = b;
		} else {
			/* 0u1f MPY||EALU, 10fk EALU||ROTATE (A.3.1; § 8.4, ch. 13) */
			ealu = true;
			ealuf = oper & 1;
			if (oper & 8) {
				const bool k = oper & 1;

				ealuf = oper & 2;
				rot = s3 & 31;
				out.second_value = std::rotl(s1, static_cast<int>(rot));
				mask_n = s4;
				if (!k) {
					cport = C_DIRECT;
					c = s4;
				} else {
					cport = (d0 >> 13) & 1 ? C_MF : C_MASK;
				}
			} else {
				multiply = true;
				out.second_value = pp_multiply(s3, s4, oper & 4, d0, true);
				rot = d0 & 31;
				mask_n = d0;
				cport = (d0 >> 13) & 1 ? C_MF : C_MASK;
			}
			b = std::rotl(s1, static_cast<int>(rot));
			sign = s1 >> 31;
		}
		if (ealu) {
			/* d0 (fig. 13-1; SPRS048 fig. 24): FMOD, A, the function code,
			 * C I S N E F, T (C82) */
			const bool cbit = (d0 >> 18) & 1, ibit = (d0 >> 17) & 1, sbit = (d0 >> 16) & 1;
			const bool nbit = (d0 >> 15) & 1, ebit = (d0 >> 14) & 1, tbit = (d0 >> 12) & 1;

			fmod = d0 >> 28;
			arith = (d0 >> 27) & 1;
			f = static_cast<uint8_t>(d0 >> 19);
			if (ealuf)
				f = static_cast<uint8_t>(~f);
			/* HEURISTIC: FMOD 2, 3, 6, 7 are %! when the C port takes the
			 * mask generator, bit detection when it does not (as the base
			 * set does by class, table 8-22); F is then ignored (§ 13.2.5) */
			const bool special = fmod == 2 || fmod == 3 || fmod == 6 || fmod == 7;

			bitdet = special && cport != C_MASK;
			bang = special && cport == C_MASK;
			const bool cin_mod = fmod == 1 || fmod == 5 || (bang && (fmod == 3 || fmod == 7));
			const bool multiple = fmod >= 8 && fmod != 15;

			if (fmod == 4 || fmod == 5 || (bang && (fmod == 6 || fmod == 7)))
				a = 0;
			pieces = multiple ? pieces_of(asize) : bitdet ? 4 : 1;
			/* N: one 32-bit mask even in multiple arithmetic; N with T:
			 * saturate 32-bit signed results (SPRS048 table 10) */
			mask_pieces = multiple && !nbit ? pieces : 1;
			saturate = nbit && tbit && arith && pieces == 1;
			uses_mf = cport == C_MF && !bitdet;
			maskgen = cport == C_MASK;
			if (maskgen)
				c = pp_mask(mask_n, bang, mask_pieces);
			else if (uses_mf)
				c = expand_mf();
			/* The four LSBs inverted for a negative rotator input with S,
			 * unless E and A give the carries (§ 13.2.3, table 13-2) */
			if (sbit && sign && (cin_mod || !(ebit && arith)))
				f ^= 0x0F;
			if (bitdet)
				c = pp_bit_detect(fmod, rot_in, out.none);
			/* The carry-in (§ 13.2.6) */
			if (cin_mod) {
				cins = (m_sr & SR_C) ? 1 : 0;
			} else if (ebit && arith) {
				cins = pp_piece_lsbs(c, pieces) ^ ((f & 1) ? (1u << pieces) - 1 : 0);
			} else {
				bool cin = cbit ^ ealuf;

				if (sbit && ibit && sign)
					cin = !cin;
				cins = cin ? (1u << pieces) - 1 : 0;
			}
		}
		if (multiply && in.dst2 == (dst & 7) && (dst & 0x78) == 0x20) {
			/* A multiply and an ALU result to one register: the ALU
			 * operation is not done (§ 8.11) */
			out.second = keep;
			out.second_dst = static_cast<uint8_t>(0x20 | in.dst2);
			return;
		}
		out.second = keep;
		out.second_dst = static_cast<uint8_t>(0x20 | in.dst2);
	} else {
		/* Formats B, C, D: the operation classes (table 8-13) */
		const uint32_t s2 = in.src2_imm ? in.imm : m_r[0x20 | in.src2];
		const uint32_t dstc = m_r[0x20 | (dst & 7)];

		rot_in = s1;
		switch (in.cls) {
		case 0:
			a = s2, uses_mf = true;
			break;
		case 1:
			a = dstc, rot = d0, c = s2;
			break;
		case 2:
			a = dstc, maskgen = true, mask_n = s2;
			break;
		case 3:
			a = dstc, rot = s2, maskgen = true, mask_n = s2;
			break;
		case 4:
			a = s2, rot = d0, maskgen = true, mask_n = d0;
			break;
		case 5:
			a = s2, rot = d0, uses_mf = true;
			break;
		case 6:
			a = dstc, c = s2;
			break;
		default:
			a = s1, rot_in = 1, rot = s2, c = s2;
			break;
		}
		rot &= 31;
		b = std::rotl(rot_in, static_cast<int>(rot));
		sign = rot_in >> 31;
		arith = in.arith;
		if (!arith) {
			/* Booleans: the function code as is, no modifier (§ 8.3.2.1) */
			f = in.alu;
		} else {
			/* Arithmetics: the odd bits give the function (table A-5), the
			 * even bits the modifier (table A-6) */
			const unsigned code = (in.alu >> 3 & 8) | (in.alu >> 2 & 4) | (in.alu >> 1 & 2) | (in.alu & 1);

			fmod = (in.alu >> 4 & 8) | (in.alu >> 3 & 4) | (in.alu >> 2 & 2) | (in.alu >> 1 & 1);
			f = BASE_F[code];
			cins = BASE_CIN[code];

			const bool special = fmod == 2 || fmod == 3 || fmod == 6 || fmod == 7;
			const bool multiple = fmod >= 8 && fmod != 15;

			bitdet = special && !maskgen;
			bang = special && maskgen;
			if (fmod == 4 || fmod == 5 || (bang && (fmod == 6 || fmod == 7)))
				a = 0;
			pieces = multiple ? pieces_of(asize) : bitdet ? 4 : 1;
			mask_pieces = multiple ? pieces : 1;
			/* The sign of the rotator input picks the function (table
			 * A-5: codes 4 and B; 6 and 9 outside the @mf classes) */
			if (code == 4)
				f = sign ? 0x9A : 0x95;
			else if (code == 0xB)
				f = sign ? 0x65 : 0x6A;
			else if ((code == 6 || code == 9) && !uses_mf) {
				f = static_cast<uint8_t>(sign ? f ^ 0x0F : f);
				cins = (code == 6) == sign;
			}
			if (fmod == 1 || fmod == 5 || (bang && (fmod == 3 || fmod == 7)))
				cins = (m_sr & SR_C) ? 1 : 0;
		}
		if (maskgen)
			c = pp_mask(mask_n, bang, mask_pieces);
		else if (uses_mf)
			c = expand_mf();
		if (bitdet)
			c = pp_bit_detect(fmod, rot_in, out.none);
		if (arith) {
			const unsigned code = (in.alu >> 3 & 8) | (in.alu >> 2 & 4) | (in.alu >> 1 & 2) | (in.alu & 1);

			if ((code == 6 || code == 9) && uses_mf && !bitdet) {
				/* C(n): the LSB of each piece of the C port (table A-5
				 * note 1), inverted for code 9 */
				cins = pp_piece_lsbs(c, pieces) ^ (code == 9 ? (1u << pieces) - 1 : 0);
			} else if (cins && pieces > 1) {
				/* Each piece takes the carry-in of bit 0 (§ 13.2.6;
				 * bit detection: ex. 8-14) */
				cins = (1u << pieces) - 1;
			}
		}
	}

	/* The ALU (ex. 13-1) */
	const PpAlu r = pp_alu(f, arith, a, b, c, pieces, cins);
	uint32_t value = r.result;
	uint32_t sr = m_sr;
	const unsigned all = (1u << pieces) - 1;

	/* Saturate-clamp (SPRS048 table 10). HEURISTIC: N and Z then follow the
	 * clamped result, C and V the ALU */
	if (saturate && ((r.cin_msb ^ r.cout) & 1))
		value = (r.cout & 1) ? 0x80000000u : 0x7FFFFFFFu;
	sr &= ~(SR_N | SR_C | SR_V | SR_Z);
	if (divi) {
		/* N = carry | sign of the rotator input (sr: Divide; § 8.6.2) */
		const bool q = (r.cout & 1) || sign;

		sr |= (q ? SR_N : 0) | ((r.cout & 1) ? SR_C : 0) | (((r.cin_msb ^ r.cout) & 1) ? SR_V : 0) |
		      (value == 0 ? SR_Z : 0);
		out.mf = (m_mf << 1) | (q ? 1u : 0);
		out.mf_set = true;
	} else if (pieces == 1 && !bitdet) {
		sr |= ((value >> 31) ? SR_N : 0) | ((r.cout & 1) ? SR_C : 0) | (((r.cin_msb ^ r.cout) & 1) ? SR_V : 0) |
		      (value == 0 ? SR_Z : 0);
	} else {
		/* Multiple arithmetic and bit detection: N = AND of the pieces'
		 * zeros, C = OR of the carries, V = AND of the carries (bit
		 * detection: nothing found), Z = OR of the zeros */
		const unsigned zeros = pp_piece_zeros(value, pieces);

		sr |= (zeros == all ? SR_N : 0) | (r.cout ? SR_C : 0) | (zeros ? SR_Z : 0);
		if (bitdet ? out.none : r.cout == all)
			sr |= SR_V;
	}
	if (!bitdet && fmod >= 8 && fmod != 15) {
		/* mf (register mf: bit setting, rotation, clearing; table 7-4) */
		uint32_t mf = m_mf;
		const unsigned n = pieces;

		/* Rotation by Asize (mrc, mre), else by Msize when R and @mf */
		if (fmod == 10 || fmod == 11)
			mf = std::rotl(mf, static_cast<int>(mf_rotation(asize)));
		else if (uses_mf && (m_sr & 0x40))
			mf = std::rotl(mf, static_cast<int>(mf_rotation((m_sr >> 3) & 7)));
		if (fmod == 12 || fmod == 13)
			mf = 0;
		if (fmod != 14) {
			unsigned flags;

			if (fmod == 8 || fmod == 10 || fmod == 12) {
				flags = r.cout;
			} else {
				switch ((m_sr >> 7) & 3) {
				case 0:
					flags = pp_piece_zeros(value, n);
					break;
				case 1:
					flags = pp_piece_msbs(value, n);
					break;
				default:
					/* Extended sign: propagate ^ carry of each MSB;
					 * HEURISTIC: the reserved MSS 11 too */
					flags = r.p_msb ^ r.cout;
					break;
				}
			}
			mf = (mf & ~all) | (flags & all);
		}
		out.mf = mf;
		out.mf_set = true;
	} else if (uses_mf && !bitdet && (m_sr & 0x40)) {
		/* R: the expander's bits rotated away by Msize (table 7-5) */
		out.mf = std::rotl(m_mf, static_cast<int>(mf_rotation((m_sr >> 3) & 7)));
		out.mf_set = true;
	}
	out.alu = keep;
	out.alu_dst = dst;
	out.alu_value = value;
	out.sr = sr;
	/* No status for a write to pc or sr, nor when the condition fails
	 * (sr register: status setting) */
	out.status = keep && dst != 0x38 && dst != 0x39 && dst != 0x29;
	if (!keep)
		out.mf_set = false;
}

/* data_unit for BASE1 (prepare_du): the same operation, by its prepared
 * choices, the class's ports known */
template <unsigned CLS, bool ARITH>
void
C82Pp::base1(const C82PpInsn &in, bool cond_true, DuOut &out)
{
	const bool keep = !in.r || cond_true;
	const uint32_t s1 = source1(in, cond_true);
	const uint32_t s2 = in.src2_imm ? in.imm : m_r[0x20 | in.src2];
	/* Table 8-13: ports A, B (rotated), C by class */
	const uint32_t rot_in = CLS == 7 ? 1u : s1;
	const uint32_t rot = CLS == 1 || CLS == 4 || CLS == 5 ? m_r[0x20] : CLS == 3 || CLS == 7 ? s2 : 0;
	const uint32_t b = std::rotl(rot_in, static_cast<int>(rot & 31));
	const unsigned sign = rot_in >> 31;
	const uint32_t a =
		(CLS == 0 || CLS == 4 || CLS == 5 ? s2 : CLS == 7 ? s1 : m_r[0x20 | in.dst]) & in.a_keep;
	uint32_t c;

	if constexpr (CLS == 2 || CLS == 3 || CLS == 4) {
		const uint32_t n = (CLS == 4 ? m_r[0x20] : s2) & 31;

		c = n ? (1u << n) - 1 : in.bang ? 0xFFFFFFFFu : 0;
	} else if constexpr (CLS == 0 || CLS == 5) {
		c = expand_mf();
	} else {
		c = s2;
	}

	/* The ALU (ex. 13-1) of one piece: the minterms of B and C picked by
	 * F for A = 0 (p0, also the generate's) and A = 1 (p1) */
	const uint32_t *f = in.f_masks[sign];
	const uint32_t u0 = f[0] ^ (f[1] & b), v0 = f[2] ^ (f[3] & b);
	const uint32_t u1 = f[4] ^ (f[5] & b), v1 = f[6] ^ (f[7] & b);
	const uint32_t p0 = u0 ^ ((u0 ^ v0) & c), p1 = u1 ^ ((u1 ^ v1) & c);
	const uint32_t p = p0 ^ ((p0 ^ p1) & a);
	uint32_t value, cv = 0;

	if constexpr (ARITH) {
		const unsigned cins[3] = { in.cin_by_sign[sign], (m_sr >> 30) & 1, (c & 1) ^ in.cn9 };
		const uint32_t g = a & ~p & p0;
		const uint64_t sum = static_cast<uint64_t>(g | p) + g + cins[in.cin_sel];
		const uint32_t cout = static_cast<uint32_t>(sum >> 32) & 1;

		value = static_cast<uint32_t>(sum);
		/* C, and V = the carry into the MSB ^ the carry out */
		cv = (cout ? SR_C : 0) | ((((value ^ p) >> 31) ^ cout) ? SR_V : 0);
	} else {
		/* A Boolean has no carry: C and V are 0 */
		value = p;
	}
	out.sr = (m_sr & ~(SR_N | SR_C | SR_V | SR_Z)) | ((value >> 31) ? SR_N : 0) | cv | (value == 0 ? SR_Z : 0);
	if constexpr (CLS == 0 || CLS == 5) {
		if ((m_sr & 0x40) && keep) {
			/* R: the expander's bits rotated away by Msize (table 7-5) */
			out.mf = std::rotl(m_mf, static_cast<int>(mf_rotation((m_sr >> 3) & 7)));
			out.mf_set = true;
		}
	}
	out.alu = keep;
	out.alu_dst = in.dst_code();
	out.alu_value = value;
	out.status = keep && in.sets_status;
}

/* base1's operation for the handlers, from the instruction's C82PpOp: the
 * result, N C V Z in `ncvz` (bits 31-28) */
template <unsigned CLS, bool ARITH, int F>
[[gnu::always_inline]] inline uint32_t
C82Pp::alu1(const C82PpOp &op, bool cond_true, uint32_t &ncvz) const
{
	const uint32_t s1 = m_r[op.s1_slot | (op.has(C82PpOp::S1_PAIR) & cond_true)];
	const uint32_t r2 = m_r[0x20 | op.src2];
	const uint32_t s2 = op.has(C82PpOp::SRC2_IMM) ? op.imm : r2;
	const uint32_t rot_in = CLS == 7 ? 1u : s1;
	const uint32_t rot = CLS == 1 || CLS == 4 || CLS == 5 ? m_r[0x20] : CLS == 3 || CLS == 7 ? s2 : 0;
	const uint32_t b = std::rotl(rot_in, static_cast<int>(rot & 31));
	const unsigned sign = rot_in >> 31;
	const uint32_t a = (CLS == 0 || CLS == 4 || CLS == 5 ? s2 : CLS == 7 ? s1 : m_r[0x20 | (op.dst & 7)]) &
	                   (op.has(C82PpOp::A_ZERO) ? 0u : ~0u);
	uint32_t c;

	if constexpr (CLS == 2 || CLS == 3 || CLS == 4) {
		const uint32_t n = (CLS == 4 ? m_r[0x20] : s2) & 31;

		c = n ? (1u << n) - 1 : op.has(C82PpOp::BANG) ? 0xFFFFFFFFu : 0;
	} else if constexpr (CLS == 0 || CLS == 5) {
		c = expand_mf();
	} else {
		c = s2;
	}

	/* F known (FORM_F): its masks are constants */
	const std::array<uint32_t, 8> &f = F_MASKS[F >= 0 ? static_cast<unsigned>(F) : op.f[sign]];
	const uint32_t u0 = f[0] ^ (f[1] & b), v0 = f[2] ^ (f[3] & b);
	const uint32_t u1 = f[4] ^ (f[5] & b), v1 = f[6] ^ (f[7] & b);
	const uint32_t p0 = u0 ^ ((u0 ^ v0) & c), p1 = u1 ^ ((u1 ^ v1) & c);
	const uint32_t p = p0 ^ ((p0 ^ p1) & a);
	uint32_t value, cv = 0;

	if constexpr (ARITH) {
		const unsigned cins[3] = { op.cin_by_sign[sign], (m_sr >> 30) & 1, (c & 1) ^ op.has(C82PpOp::CN9) };
		const uint32_t g = a & ~p & p0;
		const uint64_t sum = static_cast<uint64_t>(g | p) + g + cins[op.cin_sel];
		const uint32_t cout = static_cast<uint32_t>(sum >> 32) & 1;

		value = static_cast<uint32_t>(sum);
		cv = (cout ? SR_C : 0) | ((((value ^ p) >> 31) ^ cout) ? SR_V : 0);
	} else {
		value = p;
	}
	ncvz = (value & SR_N) | cv | (value == 0 ? SR_Z : 0);
	return value;
}

/* data_unit for MPY||ADD (opers 0, 1, 4, 5): the 16 x 16 product of src3
 * and src4 (signed unless 4, 5), src2 + src1 (0, 4) or src2 - src1 (1,
 * 5) by the same ALU sum (f 0x66 or 0x99, port C 0, one piece) */
void
C82Pp::mpy_add(const C82PpInsn &in, bool cond_true, DuOut &out)
{
	const bool keep = !in.r || cond_true;
	const uint32_t s1 = source1(in, cond_true);
	const uint32_t s3 = m_r[0x20 | in.src3], s4 = m_r[0x20 | in.src4];
	const uint8_t dst = in.dst_code();

	out.second_value = in.oper & 4 ? (s3 & 0xFFFF) * (s4 & 0xFFFF)
	                               : static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(s3)) *
	                                                       static_cast<int32_t>(static_cast<int16_t>(s4)));
	out.second = keep;
	out.second_dst = static_cast<uint8_t>(0x20 | in.dst2);
	/* A multiply and an ALU result to one register: the ALU operation is
	 * not done (§ 8.11) */
	if (in.dst2 == (dst & 7) && (dst & 0x78) == 0x20)
		return;

	/* pp_alu with f 0x66 (propagate A ^ B, generate A & B) or 0x99 (A ^ ~B,
	 * A & ~B, carry-in 1) and C = 0 */
	const uint32_t a = m_r[0x20 | in.src2], b = in.oper & 1 ? ~s1 : s1;
	const uint32_t p = a ^ b, g = a & b;
	const uint64_t sum = static_cast<uint64_t>(g | p) + g + (in.oper & 1);
	const uint32_t value = static_cast<uint32_t>(sum), cout = static_cast<uint32_t>(sum >> 32) & 1;
	const uint32_t cin_msb = ((value ^ p) >> 31) & 1;

	out.sr = (m_sr & ~(SR_N | SR_C | SR_V | SR_Z)) | (value & SR_N) | (cout ? SR_C : 0) | ((cin_msb ^ cout) ? SR_V : 0) |
	         (value == 0 ? SR_Z : 0);
	out.alu = keep;
	out.alu_dst = dst;
	out.alu_value = value;
	out.status = keep && dst != 0x38 && dst != 0x39 && dst != 0x29;
}

void
C82Pp::data_unit_base1(const C82PpInsn &in, bool cond_true, DuOut &out)
{
	switch (in.cls << 1 | in.arith) {
	case 0:
		return base1<0, false>(in, cond_true, out);
	case 1:
		return base1<0, true>(in, cond_true, out);
	case 2:
		return base1<1, false>(in, cond_true, out);
	case 3:
		return base1<1, true>(in, cond_true, out);
	case 4:
		return base1<2, false>(in, cond_true, out);
	case 5:
		return base1<2, true>(in, cond_true, out);
	case 6:
		return base1<3, false>(in, cond_true, out);
	case 7:
		return base1<3, true>(in, cond_true, out);
	case 8:
		return base1<4, false>(in, cond_true, out);
	case 9:
		return base1<4, true>(in, cond_true, out);
	case 10:
		return base1<5, false>(in, cond_true, out);
	case 11:
		return base1<5, true>(in, cond_true, out);
	case 12:
		return base1<6, false>(in, cond_true, out);
	case 13:
		return base1<6, true>(in, cond_true, out);
	case 14:
		return base1<7, false>(in, cond_true, out);
	default:
		return base1<7, true>(in, cond_true, out);
	}
}

namespace {

/* A local access while the global unit transfers too: over the local
 * port, one more cycle when outside the PP's local RAMs (§ 6.3.2): its
 * data RAMs 0 and 1 and its parameter RAM (SPRU154 fig. 1-1, 1-2) */
bool
local_ram(unsigned pp, uint32_t addr)
{
	const uint32_t block = addr >> 12;

	return block == pp || block == 8 + pp || block == 0x1000 + pp;
}

} /* namespace */

/* A load of a transfer from the address the address stage made: aligned
 * (§ 8.8.1.3), zero- or sign-extended (L e 10, 11: A.4.1); false on a
 * fault */
bool
C82Pp::load_xfer(const C82PpXfer &t, uint32_t ea, uint32_t &value)
{
	const unsigned size = t.msize;
	const uint32_t addr = ea & ~((1u << size) - 1);
	uint32_t v = 0;

	/* HEURISTIC: format 1 only; § 6.3.2 says nothing of a move in
	 * parallel (formats 2, 3) */
	if (t.divert && !local_ram(m_number, addr))
		m_extra += 1;
	if (const uint8_t *p = addr < 0x02000000u && !m_park.measuring ? m_chip.pp_ram(addr) : nullptr)
		/* On-chip RAM, as load() reads it */
		v = static_cast<uint32_t>(c82_get(p, 2) >> (32 - (8u << size)));
	else if (!load(addr, size, v, m_extra))
		return false;
	if (size == 0 && (t.le & 1))
		v = static_cast<uint32_t>(static_cast<int8_t>(v));
	else if (size == 1 && (t.le & 1))
		v = static_cast<uint32_t>(static_cast<int16_t>(v));
	value = v;
	return true;
}

bool
C82Pp::store_xfer(const C82PpXfer &t, uint32_t ea, uint32_t value)
{
	const unsigned size = t.msize;
	const uint32_t addr = ea & ~((1u << size) - 1);

	if (t.divert && !local_ram(m_number, addr))
		m_extra += 1;
	return store(addr, size, value, m_extra);
}

/* load_xfer and store_xfer from the instruction's C82PpOp */
[[gnu::always_inline]] inline bool
C82Pp::load_op(const C82PpOp &op, uint32_t ea, uint32_t &value)
{
	const unsigned size = op.msize;
	const uint32_t addr = ea & ~((1u << size) - 1);
	uint32_t v = 0;

	if (op.has(C82PpOp::DIVERT) && !local_ram(m_number, addr))
		m_extra += 1;
	if (const uint8_t *p = addr < 0x02000000u && !m_park.measuring ? m_chip.pp_ram(addr) : nullptr)
		v = static_cast<uint32_t>(c82_get(p, 2) >> (32 - (8u << size)));
	else if (!load(addr, size, v, m_extra))
		return false;
	if (size == 0 && op.has(C82PpOp::SIGN))
		v = static_cast<uint32_t>(static_cast<int8_t>(v));
	else if (size == 1 && op.has(C82PpOp::SIGN))
		v = static_cast<uint32_t>(static_cast<int16_t>(v));
	value = v;
	return true;
}

bool
C82Pp::store_op(const C82PpOp &op, uint32_t ea, uint32_t value)
{
	const unsigned size = op.msize;
	const uint32_t addr = ea & ~((1u << size) - 1);

	if (op.has(C82PpOp::DIVERT) && !local_ram(m_number, addr))
		m_extra += 1;
	return store(addr, size, value, m_extra);
}

/* The data unit's results: status, then its results; the second result
 * (multiply or rotate) below the ALU's (§ 8.11) */
void
C82Pp::apply(const DuOut &du)
{
	if (du.status)
		m_sr = (m_sr & (m_protect | ~0xF0000000u)) | (du.sr & 0xF0000000u & ~m_protect);
	if (du.mf_set)
		m_mf = du.mf;
	if (du.second)
		put(du.second_dst, du.second_value);
	if (du.alu)
		put(du.alu_dst, du.alu_value);
}

/* Any instruction: sources read, data unit, memory, then the writes from
 * the lowest priority to the highest (§ 8.11) */
void
C82Pp::execute_generic(const C82PpInsn &in, const uint32_t *ea)
{
	const bool ct = condition(in.cond);
	std::array<uint32_t, 2> data{}, value{};

	for (unsigned k = 0; k < in.xfers; k++) {
		const C82PpXfer &t = in.xfer[k];

		/* A conditional global store of a D pair: the odd register when
		 * the condition holds, else the even one (§ 8.8.3.2) */
		if (t.le == 0)
			data[k] = get(t.pair ? (ct ? t.reg | 1u : t.reg & ~1u) : t.reg);
	}

	uint32_t moved = 0;

	if (in.move || in.fmove) {
		moved = get(in.move_src);
		if (in.fmove) {
			/* Field moves (§ 8.8.4.1-8.8.4.2). HEURISTIC: the reserved size
			 * 1x moves the word; a halfword item uses its bit 0 */
			if (in.replicate) {
				if (in.fsize == 0)
					moved = (moved & 0xFF) * 0x01010101u;
				else if (in.fsize == 1)
					moved = (moved & 0xFFFF) * 0x00010001u;
			} else if (in.fsize == 0) {
				const uint32_t v = (moved >> (8 * in.itm)) & 0xFF;

				moved = in.fsign ? static_cast<uint32_t>(static_cast<int8_t>(v)) : v;
			} else if (in.fsize == 1) {
				const uint32_t v = (moved >> (16 * (in.itm & 1))) & 0xFFFF;

				moved = in.fsign ? static_cast<uint32_t>(static_cast<int16_t>(v)) : v;
			}
		}
	}

	DuOut du;

	if (in.path == C82PpInsn::BASE1)
		data_unit_base1(in, ct, du);
	else if (in.path == C82PpInsn::MPY_ADD)
		mpy_add(in, ct, du);
	else if (in.path == C82PpInsn::GENERIC)
		data_unit(in, ct, du);

	/* Memory: the global access, then the local one (HEURISTIC order) */
	for (unsigned j = 0; j < in.mems; j++) {
		const unsigned k = in.mem_order[j];

		if (in.xfer[k].le == 0 ? !store_xfer(in.xfer[k], ea[k], data[k]) : !load_xfer(in.xfer[k], ea[k], value[k]))
			return;
	}
	for (unsigned k = 0; k < in.xfers; k++) {
		const C82PpXfer &t = in.xfer[k];

		if (t.le == 1) {
			/* Address unit arithmetic: the address, its 8 or 16 LSBs
			 * sign-extended for b and h (§ 8.8.5) */
			const uint32_t a = ea[k];

			value[k] = t.size == 0 ? static_cast<uint32_t>(static_cast<int8_t>(a))
			           : t.size == 1 ? static_cast<uint32_t>(static_cast<int16_t>(a))
			                         : a;
		}
	}

	apply(du);
	/* The local transfer, then the global transfer or move */
	for (unsigned j = 0; j < in.writes; j++) {
		const unsigned k = in.write_order[j];
		const C82PpXfer &t = in.xfer[k];

		/* A conditional global load or arithmetic writes only when
		 * the condition holds (§ 8.8.2.2, § 8.8.5.2) */
		if (t.global && in.g && !ct)
			continue;
		if (t.pcload) {
			/* A load to pc with an unscaled offset: offset bit 1 takes
			 * G, bit 0 takes L from the word read (pc register) */
			if (t.reg == 0x38)
				m_iprs = (m_pc & ~7u) + 8;
			branch(value[k], t.index & 3);
			continue;
		}
		put(t.reg, value[k]);
	}
	if ((in.move || in.fmove) && (!in.g || ct)) {
		/* A move to a15: an interprocessor command (§ 3.2.1) */
		if (in.move_dst == 0xF && in.move)
			send_command(moved);
		else
			put(in.move_dst, moved);
	}
}

/* The execute stage of an instruction of the form FORM (C82PpInsn::form):
 * the steps of execute_generic that it has, in its order (the data to
 * store read, the data unit, the memory, nothing written after a fault,
 * the data unit's results, then the transfer's) */
template <unsigned FORM>
[[gnu::always_inline]] inline void
C82Pp::exec([[maybe_unused]] C82Pp &p, [[maybe_unused]] const C82PpOp &op, [[maybe_unused]] const C82PpInsn *in,
            [[maybe_unused]] const uint32_t *ea)
{
	if constexpr (FORM == C82PpInsn::FORM_GENERIC) {
		p.execute_generic(*in, ea);
	} else if constexpr (FORM == C82PpInsn::FORM_LOAD) {
		uint32_t v = 0;

		if (p.load_op(op, ea[0], v))
			p.put(op.reg, v);
	} else if constexpr (FORM == C82PpInsn::FORM_STORE) {
		p.store_op(op, ea[0], p.get(op.reg));
	} else if constexpr (FORM == C82PpInsn::FORM_LEA) {
		/* execute_generic's address unit arithmetic: the address, its 8 or
		 * 16 LSBs sign-extended for b and h (§ 8.8.5); a reserved size as
		 * a word */
		const uint32_t a = ea[0];

		p.put(op.reg, op.msize == 0 ? static_cast<uint32_t>(static_cast<int8_t>(a))
		              : op.msize == 1 ? static_cast<uint32_t>(static_cast<int16_t>(a))
		                              : a);
	} else if constexpr (FORM >= C82PpInsn::FORM_DU) {
		/* base1 and apply, by alu1 */
		constexpr bool FIXED = FORM >= C82PpInsn::FORM_F;
		constexpr unsigned X = (FORM - (FIXED ? C82PpInsn::FORM_F : C82PpInsn::FORM_DU)) % 3;
		constexpr unsigned K = FIXED ? 2 * C82PP_F[(FORM - C82PpInsn::FORM_F) / 3].cls + C82PP_F[(FORM - C82PpInsn::FORM_F) / 3].arith
		                             : (FORM - C82PpInsn::FORM_DU) / 3;
		constexpr int F = FIXED ? C82PP_F[(FORM - C82PpInsn::FORM_F) / 3].f : -1;
		constexpr unsigned CLS = K / 2;
		const bool ct = !op.has(C82PpOp::READS_COND) || p.condition(op.cond);
		const bool keep = !op.has(C82PpOp::R) || ct;
		uint32_t v = 0, ncvz = 0, mf = p.m_mf;

		if constexpr (X == 2)
			v = p.get(op.reg);

		const uint32_t value = p.alu1<CLS, K % 2 != 0, F>(op, ct, ncvz);

		if constexpr (CLS == 0 || CLS == 5)
			if (p.m_sr & 0x40)
				mf = std::rotl(p.m_mf, static_cast<int>(mf_rotation((p.m_sr >> 3) & 7)));
		if constexpr (X == 1) {
			if (!p.load_op(op, ea[0], v))
				return;
		} else if constexpr (X == 2) {
			if (!p.store_op(op, ea[0], v))
				return;
		}
		if (keep) {
			if (op.has(C82PpOp::SETS_STATUS))
				p.m_sr = (p.m_sr & (op.protect() | ~0xF0000000u)) | (ncvz & ~op.protect());
			if constexpr (CLS == 0 || CLS == 5)
				p.m_mf = mf;
			p.put(op.dst, value);
		}
		if constexpr (X == 1)
			p.put(op.reg, v);
	}
}

template <size_t... K>
constexpr std::array<C82Pp::Exec, sizeof...(K)>
C82Pp::exec_table(std::index_sequence<K...>)
{
	return { { &C82Pp::exec<K>... } };
}

const std::array<C82Pp::Exec, C82PpInsn::FORMS> C82Pp::EXEC = exec_table(std::make_index_sequence<C82PpInsn::FORMS>());

/* EXEC[form], inlined (steps, run_block) */
[[gnu::always_inline]] inline void
C82Pp::execute(unsigned form, const C82PpOp &op, const C82PpInsn *in, const uint32_t *ea)
{
	static_assert(C82PpInsn::FORMS == 95);
	switch (form) {
	case 0:
		return exec<0>(*this, op, in, ea);
	case 1:
		return exec<1>(*this, op, in, ea);
	case 2:
		return exec<2>(*this, op, in, ea);
	case 3:
		return exec<3>(*this, op, in, ea);
	case 4:
		return exec<4>(*this, op, in, ea);
	case 5:
		return exec<5>(*this, op, in, ea);
	case 6:
		return exec<6>(*this, op, in, ea);
	case 7:
		return exec<7>(*this, op, in, ea);
	case 8:
		return exec<8>(*this, op, in, ea);
	case 9:
		return exec<9>(*this, op, in, ea);
	case 10:
		return exec<10>(*this, op, in, ea);
	case 11:
		return exec<11>(*this, op, in, ea);
	case 12:
		return exec<12>(*this, op, in, ea);
	case 13:
		return exec<13>(*this, op, in, ea);
	case 14:
		return exec<14>(*this, op, in, ea);
	case 15:
		return exec<15>(*this, op, in, ea);
	case 16:
		return exec<16>(*this, op, in, ea);
	case 17:
		return exec<17>(*this, op, in, ea);
	case 18:
		return exec<18>(*this, op, in, ea);
	case 19:
		return exec<19>(*this, op, in, ea);
	case 20:
		return exec<20>(*this, op, in, ea);
	case 21:
		return exec<21>(*this, op, in, ea);
	case 22:
		return exec<22>(*this, op, in, ea);
	case 23:
		return exec<23>(*this, op, in, ea);
	case 24:
		return exec<24>(*this, op, in, ea);
	case 25:
		return exec<25>(*this, op, in, ea);
	case 26:
		return exec<26>(*this, op, in, ea);
	case 27:
		return exec<27>(*this, op, in, ea);
	case 28:
		return exec<28>(*this, op, in, ea);
	case 29:
		return exec<29>(*this, op, in, ea);
	case 30:
		return exec<30>(*this, op, in, ea);
	case 31:
		return exec<31>(*this, op, in, ea);
	case 32:
		return exec<32>(*this, op, in, ea);
	case 33:
		return exec<33>(*this, op, in, ea);
	case 34:
		return exec<34>(*this, op, in, ea);
	case 35:
		return exec<35>(*this, op, in, ea);
	case 36:
		return exec<36>(*this, op, in, ea);
	case 37:
		return exec<37>(*this, op, in, ea);
	case 38:
		return exec<38>(*this, op, in, ea);
	case 39:
		return exec<39>(*this, op, in, ea);
	case 40:
		return exec<40>(*this, op, in, ea);
	case 41:
		return exec<41>(*this, op, in, ea);
	case 42:
		return exec<42>(*this, op, in, ea);
	case 43:
		return exec<43>(*this, op, in, ea);
	case 44:
		return exec<44>(*this, op, in, ea);
	case 45:
		return exec<45>(*this, op, in, ea);
	case 46:
		return exec<46>(*this, op, in, ea);
	case 47:
		return exec<47>(*this, op, in, ea);
	case 48:
		return exec<48>(*this, op, in, ea);
	case 49:
		return exec<49>(*this, op, in, ea);
	case 50:
		return exec<50>(*this, op, in, ea);
	case 51:
		return exec<51>(*this, op, in, ea);
	case 52:
		return exec<52>(*this, op, in, ea);
	case 53:
		return exec<53>(*this, op, in, ea);
	case 54:
		return exec<54>(*this, op, in, ea);
	case 55:
		return exec<55>(*this, op, in, ea);
	case 56:
		return exec<56>(*this, op, in, ea);
	case 57:
		return exec<57>(*this, op, in, ea);
	case 58:
		return exec<58>(*this, op, in, ea);
	case 59:
		return exec<59>(*this, op, in, ea);
	case 60:
		return exec<60>(*this, op, in, ea);
	case 61:
		return exec<61>(*this, op, in, ea);
	case 62:
		return exec<62>(*this, op, in, ea);
	case 63:
		return exec<63>(*this, op, in, ea);
	case 64:
		return exec<64>(*this, op, in, ea);
	case 65:
		return exec<65>(*this, op, in, ea);
	case 66:
		return exec<66>(*this, op, in, ea);
	case 67:
		return exec<67>(*this, op, in, ea);
	case 68:
		return exec<68>(*this, op, in, ea);
	case 69:
		return exec<69>(*this, op, in, ea);
	case 70:
		return exec<70>(*this, op, in, ea);
	case 71:
		return exec<71>(*this, op, in, ea);
	case 72:
		return exec<72>(*this, op, in, ea);
	case 73:
		return exec<73>(*this, op, in, ea);
	case 74:
		return exec<74>(*this, op, in, ea);
	case 75:
		return exec<75>(*this, op, in, ea);
	case 76:
		return exec<76>(*this, op, in, ea);
	case 77:
		return exec<77>(*this, op, in, ea);
	case 78:
		return exec<78>(*this, op, in, ea);
	case 79:
		return exec<79>(*this, op, in, ea);
	case 80:
		return exec<80>(*this, op, in, ea);
	case 81:
		return exec<81>(*this, op, in, ea);
	case 82:
		return exec<82>(*this, op, in, ea);
	case 83:
		return exec<83>(*this, op, in, ea);
	case 84:
		return exec<84>(*this, op, in, ea);
	case 85:
		return exec<85>(*this, op, in, ea);
	case 86:
		return exec<86>(*this, op, in, ea);
	case 87:
		return exec<87>(*this, op, in, ea);
	case 88:
		return exec<88>(*this, op, in, ea);
	case 89:
		return exec<89>(*this, op, in, ea);
	case 90:
		return exec<90>(*this, op, in, ea);
	case 91:
		return exec<91>(*this, op, in, ea);
	case 92:
		return exec<92>(*this, op, in, ea);
	case 93:
		return exec<93>(*this, op, in, ea);
	case 94:
		return exec<94>(*this, op, in, ea);
	default:
		return;
	}
}

bool
C82Pp::steps(uint64_t until)
{
	/* A pass that cannot park any more: what it watches does not matter */
	if (m_park.measuring && m_park.spoiled)
		stop_watching();
	if (m_seq != NONE || m_park.on || m_sa->kind != INSN || m_se->kind != INSN)
		return false;

	/* pc, the cycle count and the stages held here, stored where the
	 * handlers can read them (before each execute stage) and at the end */
	uint32_t pc = m_pc;
	uint64_t cycles = m_cycles;
	Stage *sa = m_sa, *se = m_se, *sf = m_sf;
	bool ran = false;

	while (cycles < until) {
		const uint32_t at = pc & ~7u;

		/* cycle() for the wait, an illegal instruction, an interrupt */
		if (at == m_wait_pc || sa->stop || ((pc & 2) && (m_intflg & m_inten & ~1u)))
			break;

		/* The word fetched: decoded already, still the same in memory */
		const unsigned slot = (pc >> 3) & 4095;
		const C82PpOp &fi = m_ops[slot];
		uint64_t raw;

		if (fi.addr != at || !fi.host)
			break;
		std::memcpy(&raw, fi.host, 8);
		if (raw != fi.raw)
			break;
		/* A pass being measured, as cycle() and fetch_word() note it */
		if (m_park.measuring) {
			if (m_park.cycles == Park::CYCLES)
				m_park.spoiled = true;
			else
				m_park.cycles++;
			note_fetch(at);
		}

		m_written = 0;
		m_lc_written = 0;
		m_extra = 0;
		m_pc_written = false;
		sf->kind = INSN;
		sf->addr = pc;
		sf->word = __builtin_bswap64(raw);
		sf->ea = {};
		sf->insn = &m_decoded[slot].insn;
		sf->op = &fi;
		sf->stop = fi.illegal;

		/* The loop logic only at an enabled loop end (loops_changed) */
		std::array<uint32_t, 3> lc;
		const bool loops = !(pc & 1) && (at == m_ends[0] || at == m_ends[1] || at == m_ends[2]);
		uint32_t next = (at + 8) | (pc & 3);

		if (loops) {
			lc = m_lc;
			next = next_fetch(pc, lc);
		}

		/* The address stage (address_stage), its write-backs kept */
		const C82PpOp &ao = *sa->op;
		uint32_t back[2];

		for (unsigned k = 0; k < ao.xfers; k++) {
			const C82PpOp::Agen &g = ao.agen[k];
			const uint32_t base = m_r[g.base_slot];
			const uint32_t index = (((m_r[g.index_slot] + g.offset) << g.shift) ^ (0u - g.neg)) + g.neg;

			sa->ea[k] = base + (index & (0u - g.pre));
			back[k] = base + index;
		}

		const C82PpOp &eo = *se->op;

		m_pc = pc;
		m_cycles = cycles;
		m_sa = sa;
		m_se = se;
		m_sf = sf;
		m_protect = eo.protect();
		execute(eo.form, eo, se->insn, se->ea.data());
		if (m_faulted)
			return true;
		for (unsigned k = 0; k < ao.xfers; k++) {
			const unsigned code = ao.agen[k].wb;

			if (code != 0xFF && !((m_written >> code) & 1))
				put(code, back[k]);
		}
		if (m_pc_written) {
			const uint32_t keep = 3u & ~m_new_pc_bits;

			pc = (m_new_pc & ~(4u | keep)) | (pc & keep);
		} else {
			pc = next;
			if (loops)
				for (unsigned n = 0; n < 3; n++)
					if (!((m_lc_written >> n) & 1))
						m_lc[n] = lc[n];
		}
		if (fi.g_change >= 0)
			pc = (pc & ~2u) | (static_cast<uint32_t>(fi.g_change) << 1);

		Stage *const done = se;

		se = sa;
		sa = sf;
		sf = done;
		cycles += 1 + m_extra;
		ran = true;
		if (m_halted)
			break;
		if (m_park.measuring && m_park.spoiled)
			stop_watching();
	}
	m_pc = pc;
	m_cycles = cycles;
	m_sa = sa;
	m_se = se;
	m_sf = sf;
	return ran;
}

/* A block's steps from step k (c82_pp_block.cpp): each one cycle of steps(),
 * in its order, with what the block knows already. Stops before a step at
 * the end of the slice, when the fetched word changed (CHANGED) or when its
 * execute stage would reach anything but on-chip RAM or plain memory; leaves
 * after a step that branched, took a loop end, fetched eint or dint, wrote a
 * register that ends blocks, or was the last. `f` is then the state there */
C82Pp::BlockEnd
C82Pp::run_block(Block &b, unsigned k, Flow &f, uint64_t until)
{
	const BlockStep *st = b.steps.data() + k;
	const uint8_t *host = b.host + 8 * k;
	const uint32_t base_pc = b.f0 | b.bits;
	uint32_t pc = f.pc;
	int64_t left = f.left;
	/* The addresses the execute stage reads, those the address stage makes */
	uint32_t ea[2] = { f.e_ea[0], f.e_ea[1] }, made[2] = { 0, 0 };
	uint32_t next = 0;
	/* The flow as the block leaves it: pc, the instruction of index i in
	 * the address stage, i - 1 in the execute stage with the addresses of
	 * its transfers (a stage is fetched with none) */
	const auto leave = [&](int i, uint32_t to, const uint32_t *addresses) {
		const unsigned xfers = b.ops[static_cast<unsigned>(i + 1)].xfers;

		f.pc = to;
		f.b = &b;
		f.i = i;
		f.e_ea[0] = xfers > 0 ? addresses[0] : 0;
		f.e_ea[1] = xfers > 1 ? addresses[1] : 0;
		f.left = left;
	};

	m_extra = 0;
	for (;;) {
		const unsigned flags = st->flags;
		uint64_t raw;

		std::memcpy(&raw, host, 8);
		if (left <= 0 || raw != st->raw) [[unlikely]] {
			leave(static_cast<int>((pc - base_pc) >> 3) - 1, pc, ea);
			return left <= 0 ? BlockEnd::STOP : BlockEnd::CHANGED;
		}
		if (flags & MEMORY) {
			for (unsigned m = 0; m < st->mems; m++) {
				const uint32_t addr = ea[st->mem_x[m]] & ~((1u << st->mem_size[m]) - 1);

				if (!(addr < 0x02000000u ? m_chip.pp_ram(addr) : m_chip.pp_code(addr))) [[unlikely]] {
					leave(static_cast<int>((pc - base_pc) >> 3) - 1, pc, ea);
					return BlockEnd::STOP;
				}
			}
		}
		/* A wait block counts the cycle (its limit: run_blocks) */
		if (b.wait)
			m_park.cycles++;

		/* The loop logic, before the execute stage writes anything */
		std::array<uint32_t, 3> lc;

		next = pc + 8;
		if (flags & LOOP_END) [[unlikely]] {
			lc = m_lc;
			next = next_fetch(pc, lc);
		}

		/* The address stage, its write-backs kept */
		uint32_t back[2];
		const unsigned axfers = st->axfers;

		for (unsigned x = 0; x < axfers; x++) {
			const C82PpOp::Agen &g = st->agen[x];
			const uint32_t base = m_r[g.base_slot];
			const uint32_t index = (((m_r[g.index_slot] + g.offset) << g.shift) ^ (0u - g.neg)) + g.neg;

			made[x] = base + (index & (0u - g.pre));
			back[x] = base + index;
		}

		/* The execute stage; what read_reg and write_reg read made up to
		 * date first when it touches a special register */
		const C82PpOp &eo = *st->eop;

		if (flags & (SPECIAL | CAREFUL | BRANCH | LOOP_END)) [[unlikely]] {
			if (flags & SPECIAL) {
				const int j = static_cast<int>((pc - base_pc) >> 3);

				m_pc = pc;
				m_cycles = until - static_cast<uint64_t>(left);
				m_sa->addr = j >= 1 ? pc - 8 : b.ipa;
				m_se->addr = j >= 2 ? pc - 16 : j == 1 ? b.ipa : b.ipe;
				m_protect = eo.protect();
			}
			m_written = 0;
			m_pc_written = false;
			m_lc_written = 0;
		}
		execute(st->form, eo, st->ein, ea);
		if (axfers) {
			if (!(flags & CAREFUL)) [[likely]] {
				/* put() without its mark: nothing reads m_written before
				 * a careful step clears it */
				if (st->wb_slot[0] != 0xFF)
					m_r[st->wb_slot[0]] = back[0] & st->wb_keep[0];
				if (axfers > 1 && st->wb_slot[1] != 0xFF)
					m_r[st->wb_slot[1]] = back[1] & st->wb_keep[1];
			} else {
				for (unsigned x = 0; x < axfers; x++) {
					const unsigned code = st->agen[x].wb;

					if (code != 0xFF && !((m_written >> code) & 1))
						put(code, back[x]);
				}
			}
		}
		if (flags & MEMORY) {
			left -= m_extra;
			m_extra = 0;
		}
		left -= 1;
		if (flags & (BRANCH | LOOP_END | LAST)) [[unlikely]] {
			if ((flags & BRANCH) && m_pc_written) {
				const uint32_t keep = 3u & ~m_new_pc_bits;

				next = (m_new_pc & ~(4u | keep)) | (pc & keep);
				break;
			}
			if (flags & LOOP_END) {
				for (unsigned n = 0; n < 3; n++)
					if (!((m_lc_written >> n) & 1))
						m_lc[n] = lc[n];
				if (next != pc + 8)
					break;
			}
			if (flags & LAST)
				break;
		}
		ea[0] = made[0];
		ea[1] = made[1];
		st++;
		host += 8;
		pc += 8;
	}
	if (st->g_change >= 0)
		next = (next & ~2u) | (static_cast<uint32_t>(st->g_change) << 1);
	leave(static_cast<int>((pc - base_pc) >> 3), next, made);
	return BlockEnd::NEXT;
}

} /* namespace odin */
