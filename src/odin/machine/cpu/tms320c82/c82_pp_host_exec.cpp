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

#include <array>
#include <bit>
#include <utility>

#include "odin/machine/cpu/tms320c82/c82_pp_host.hpp"
#include "odin/machine/cpu/tms320c82/tms320c82.hpp"

/* The execute stage of a block's step in host code (x86-64): for the forms
 * written here, the work of the handler (c82_pp_ops.cpp, exec<FORM>) in its
 * order, with what the block knows as constants. The rest, and any access
 * the fast path does not make (external memory, a line a waiting processor
 * watches), calls the handler on a slow path out of the line. Other hosts
 * keep the blocks in C++ (run_block): no AArch64 writer yet. */

namespace odin {

#if defined(ODIN_HOST_X86_64)

namespace {

using C = C82PpCode;
using M = C82PpCode::Mem;

/* The registers of the accesses: their offsets in the on-chip RAM */
constexpr uint8_t ACCESS[2] = { C::R14, C::R15 };

/* Status bits N C V Z of sr */
constexpr uint32_t SR_NCVZ = 0xF0000000u;

/* expand_mf() when R is 0: the word for each Msize (bits 5-3 of sr) and the
 * four LSBs of mf (tables 7-3, 7-5) */
constexpr std::array<uint32_t, 128> EXPAND = [] {
	std::array<uint32_t, 128> t{};

	for (unsigned msize = 0; msize < 8; msize++)
		for (unsigned flags = 0; flags < 16; flags++) {
			uint32_t v = 0;

			if (msize == 4) {
				for (unsigned k = 0; k < 4; k++)
					if ((flags >> k) & 1)
						v |= 0xFFu << (8 * k);
			} else if (msize == 5) {
				v = ((flags & 2) ? 0xFFFF0000u : 0) | ((flags & 1) ? 0xFFFFu : 0);
			} else if (msize == 6) {
				v = (flags & 1) ? 0xFFFFFFFFu : 0;
			}
			t[msize << 4 | flags] = v;
		}
	return t;
}();

/* A 32-bit value of the ALU being written: a constant, or a register (inv:
 * its complement); temp: a register of the pool */
struct Val {
	bool k = false;
	uint32_t v = 0;
	uint8_t reg = 0;
	bool inv = false, temp = false;
};

Val
konst(uint32_t v)
{
	Val x;

	x.k = true;
	x.v = v;
	return x;
}

Val
in_reg(uint8_t r)
{
	Val x;

	x.reg = r;
	return x;
}

Val
inverse(Val x)
{
	if (x.k)
		x.v = ~x.v;
	else
		x.inv = !x.inv;
	return x;
}

/* The ALU's Boolean steps on values, folded when an operand is a constant
 * or the same register, written in registers of the pool otherwise. An
 * operation never changes its operands; its result may be one of them */
struct Alu {
	C82PpCode &c;
	std::vector<uint8_t> pool;

	Val fresh()
	{
		Val t;

		t.reg = pool.back();
		t.temp = true;
		pool.pop_back();
		return t;
	}
	/* x's register back to the pool, unless a live value holds it */
	void release(const Val &x, std::initializer_list<Val> live)
	{
		if (x.k || !x.temp)
			return;
		for (const Val &l : live)
			if (!l.k && l.reg == x.reg)
				return;
		pool.push_back(x.reg);
	}
	void into(uint8_t dst, const Val &x)
	{
		if (x.k) {
			c.mov_ri(dst, x.v);
			return;
		}
		if (x.reg != dst)
			c.op_rr(0x89, dst, x.reg);					/* mov dst, reg */
		if (x.inv)
			c.not_r(dst);
	}
	Val copy(const Val &x)
	{
		Val t = fresh();

		into(t.reg, x);
		return t;
	}
	Val and_(Val x, Val y)
	{
		if (x.k && y.k)
			return konst(x.v & y.v);
		if (y.k)
			std::swap(x, y);
		if (x.k) {
			if (!x.v)
				return konst(0);
			if (x.v == ~0u)
				return y;
			Val t = copy(y);
			c.op_ri(4, t.reg, x.v);						/* and t, value */
			return t;
		}
		if (x.reg == y.reg)
			return x.inv == y.inv ? x : konst(0);

		Val t = fresh();

		if (x.inv && y.inv) {
			/* ~x & ~y = ~(x | y) */
			c.op_rr(0x89, t.reg, x.reg);
			c.op_rr(0x09, t.reg, y.reg);
			t.inv = true;
			return t;
		}
		if (x.inv)
			std::swap(x, y);
		c.op_rr(0x89, t.reg, y.reg);						/* mov t, y */
		if (y.inv)
			c.not_r(t.reg);
		c.op_rr(0x21, t.reg, x.reg);						/* and t, x */
		return t;
	}
	Val xor_(Val x, Val y)
	{
		if (x.k && y.k)
			return konst(x.v ^ y.v);
		if (y.k)
			std::swap(x, y);
		if (x.k) {
			if (!x.v)
				return y;
			if (x.v == ~0u)
				return inverse(y);
			Val t = copy(y);
			c.op_ri(6, t.reg, x.v);						/* xor t, value */
			return t;
		}
		if (x.reg == y.reg)
			return konst(x.inv == y.inv ? 0 : ~0u);

		Val t = fresh();

		c.op_rr(0x89, t.reg, x.reg);						/* mov t, x */
		c.op_rr(0x31, t.reg, y.reg);						/* xor t, y */
		t.inv = x.inv != y.inv;
		return t;
	}
};

/* What the steps write with, the places in the PP (rbx) */
struct Writer {
	C82PpCode &c;
	int32_t r, written, spoiled, ram, watched, sr, mf, iprs, new_pc, new_pc_bits, pc_written, ncvz;
	unsigned pp;
	/* The address after the step's fetch (a call's iprs) */
	uint32_t next;
	/* The special registers read and written here: pc, ipa and ipe
	 * as the step's frame makes them; the members; comm as comm_read()
	 * found it; source 1's code when special (0: none); the helpers */
	uint32_t pc = 0, ipa = 0, ipe = 0;
	int32_t inten = 0, intflg = 0, lctl = 0, lc = 0, lr = 0, ls = 0, le = 0, lc_written = 0, comm = 0;
	unsigned s1_special = 0;
	uintptr_t comm_written = 0, loops_written = 0;

	M reg_at(unsigned slot) const { return { C::PP, C::NO, 0, r + 4 * static_cast<int32_t>(slot) }; }
	/* The on-chip RAM at the offset in `access` */
	M ram_at(uint8_t access) const { return { C::PP, access, 0, ram }; }

	/* dst = get(code), code below 0x28 */
	void get(uint8_t dst, unsigned code) { c.op_m({ 0x8B }, dst, reg_at(c82_pp_regs::READ_SLOT[code])); }

	/* put(code, src), code below 0x28: src masked when the register keeps
	 * bits (src changed then), the mark of m_written when careful */
	void put(unsigned code, uint8_t src, bool careful)
	{
		if (c82_pp_regs::WRITE_KEEP[code] != ~0u)
			c.op_ri(4, src, c82_pp_regs::WRITE_KEEP[code]);			/* and src, kept */
		c.op_m({ 0x89 }, src, reg_at(c82_pp_regs::WRITE_SLOT[code]));		/* mov [register], src */
		if (!careful)
			return;
		const uint64_t mark = c82_pp_regs::WRITE_MARK[code];
		const M at{ C::PP, C::NO, 0, written };

		if (mark && mark < (1ull << 31)) {
			c.op_m({ 0x81 }, 1, at, true);					/* or qword [m_written], mark */
			c.word(static_cast<uint32_t>(mark));
		} else if (mark) {
			c.op_m({ 0x0F, 0xBA }, 5, at, true);				/* bts qword [m_written], bit */
			c.byte(static_cast<uint8_t>(__builtin_ctzll(mark)));
		}
	}

	/* read_reg() for the codes read here: below 0x28, pc, ipa, ipe,
	 * comm, and the plain members */
	static bool readable(unsigned code)
	{
		return code < 0x28 || code == 0x29 || code == 0x2A || (code >= 0x38 && code <= 0x3F) ||
		       (code >= 0x60 && code <= 0x76 && (code & 3) != 3 && (code < 0x68 || code >= 0x70));
	}
	void get_any(uint8_t dst, unsigned code)
	{
		int32_t at = 0;

		switch (code) {
		case 0x38:
			c.mov_ri(dst, pc);
			return;
		case 0x39:
			c.mov_ri(dst, ipa);
			return;
		case 0x3A:
			c.mov_ri(dst, ipe);
			return;
		case 0x29:
			at = sr;
			break;
		case 0x2A:
			at = mf;
			break;
		case 0x3B:
			at = iprs;
			break;
		case 0x3C:
			at = inten;
			break;
		case 0x3D:
			at = intflg;
			break;
		case 0x3E:
			at = comm;
			break;
		case 0x3F:
			at = lctl;
			break;
		default:
			if (code < 0x28) {
				get(dst, code);
				return;
			}
			at = (code < 0x64 ? lc : code < 0x70 ? lr : code < 0x74 ? ls : le) + 4 * static_cast<int32_t>(code & 3);
			break;
		}
		c.op_m({ 0x8B }, dst, { C::PP, C::NO, 0, at });				/* mov dst, [register] */
	}

	/* write_reg() for the codes written here; `lone`: nothing after
	 * the write needs a register (comm, lctl, lrs, lrse, le call a helper) */
	static bool writable(unsigned code, bool lone)
	{
		if (code < 0x28 || code == 0x2A || code == 0x38 || code == 0x39 || code == 0x3B || code == 0x3C ||
		    (code >= 0x60 && code <= 0x72 && (code & 3) != 3 && (code < 0x68 || code >= 0x70)))
			return true;
		return lone && (code == 0x3E || code == 0x3F || (code >= 0x68 && code <= 0x6E && (code & 3) != 3) ||
		                (code >= 0x74 && code <= 0x76));
	}
	/* A helper with the PP and src */
	void call(uintptr_t helper, uint8_t src)
	{
		c.op_r({ 0x89 }, src, C::ARG[1]);					/* mov the second argument, src */
		c.pp_first();
		c.call_to(helper);
	}

	/* write_reg() for the codes written here (writable): put() below 0x28;
	 * pc, a branch (0x39 br; 0x38 call: iprs = the address after the fetch,
	 * § 10.4); the others (src changed) */
	void write(unsigned code, uint8_t src, bool careful)
	{
		if (code < 0x28) {
			put(code, src, careful);
			return;
		}

		const auto in = [&](int32_t place) { return M{ C::PP, C::NO, 0, place }; };
		const int32_t n4 = 4 * static_cast<int32_t>(code & 3);
		const uint32_t bit = 1u << (code & 3);

		switch (code) {
		case 0x38:
			c.op_m({ 0xC7 }, 0, in(iprs));						/* mov dword [iprs], next */
			c.word(next);
			[[fallthrough]];
		case 0x39:
			c.op_m({ 0x89 }, src, in(new_pc));					/* mov [m_new_pc], src */
			c.op_m({ 0xC7 }, 0, in(new_pc_bits));					/* mov dword [m_new_pc_bits], 0 */
			c.word(0);
			c.op_m({ 0xC6 }, 0, in(pc_written));					/* mov byte [m_pc_written], 1 */
			c.byte(1);
			return;
		case 0x2A:
			c.op_m({ 0x89 }, src, in(mf));						/* mov [mf], src */
			return;
		case 0x3B:
			c.op_m({ 0x89 }, src, in(iprs));					/* mov [iprs], src */
			return;
		case 0x3C:
			c.op_m({ 0x89 }, src, in(inten));					/* mov [inten], src */
			return;
		case 0x3E:
			call(comm_written, src);
			return;
		case 0x3F:
			c.op_ri(4, src, 0xFFF);							/* and src, 0xFFF */
			c.op_m({ 0x89 }, src, in(lctl));					/* mov [lctl], src */
			call(loops_written, src);
			return;
		case 0x60:
		case 0x61:
		case 0x62:
			c.op_m({ 0x89 }, src, in(lc + n4));					/* mov [lc n], src */
			c.op_m({ 0x83 }, 1, in(lc_written));					/* or dword [m_lc_written], bit */
			c.byte(static_cast<uint8_t>(bit));
			return;
		case 0x70:
		case 0x71:
		case 0x72:
			c.op_m({ 0x89 }, src, in(ls + n4));					/* mov [ls n], src */
			return;
		case 0x74:
		case 0x75:
		case 0x76:
			c.op_m({ 0x89 }, src, in(le + n4));					/* mov [le n], src */
			call(loops_written, src);
			return;
		default:
			/* lr (lc too); lrs, lrse: ls = the address after the
			 * fetch, le too for lrse, the loop end enabled in lctl */
			c.op_m({ 0x89 }, src, in(lr + n4));					/* mov [lr n], src */
			c.op_m({ 0x89 }, src, in(lc + n4));					/* mov [lc n], src */
			c.op_m({ 0x83 }, 1, in(lc_written));					/* or dword [m_lc_written], bit */
			c.byte(static_cast<uint8_t>(bit));
			if (code < 0x68)
				return;
			c.op_m({ 0xC7 }, 0, in(ls + n4));					/* mov dword [ls n], next */
			c.word(next);
			if (code < 0x6C) {
				c.op_m({ 0xC7 }, 0, in(le + n4));				/* mov dword [le n], next */
				c.word(next);
			}
			c.op_m({ 0x81 }, 4, in(lctl));						/* and dword [lctl], ~(15 << 4n) */
			c.word(~(0xFu << (4 * (code & 3))));
			c.op_m({ 0x81 }, 1, in(lctl));						/* or dword [lctl], (8 | n + 1) << 4n */
			c.word((8u | ((code & 3) + 1)) << (4 * (code & 3)));
			call(loops_written, src);
			return;
		}
	}

	/* One more cycle for a local access over the global port outside the
	 * PP's local RAMs (load_op, store_op: local_ram), from the address in
	 * `address` */
	void divert(int32_t address)
	{
		c.op_m({ 0x8B }, C::RDX, { C::PP, C::NO, 0, address });		/* mov edx, address */
		c.shift_ri(5, C::RDX, 12);						/* shr edx, 12 */
		std::vector<HostCode::Jump> local;

		for (uint32_t block : { pp, 8 + pp, 0x1000 + pp }) {
			c.op_ri(7, C::RDX, block);					/* cmp edx, block */
			local.push_back(c.jump_if(C::JZ));
		}
		c.bytes({ 0x48, 0x83, 0xED, 0x01 });					/* sub rbp, 1 */
		for (const HostCode::Jump &j : local)
			c.bind(j);
	}

	/* dst = the bytes of `size` at the access, big-endian, zero- or
	 * sign-extended (load_op) */
	void load(uint8_t dst, uint8_t access, unsigned size, bool sign)
	{
		if (size == 0) {
			c.op_m({ 0x0F, static_cast<uint8_t>(sign ? 0xBE : 0xB6) }, dst, ram_at(access));	/* movzx/movsx dst, byte */
		} else if (size == 1) {
			c.op_m({ 0x0F, 0xB7 }, dst, ram_at(access));			/* movzx dst, word */
			c.op_r({ 0xC1 }, 0, dst, false, true);				/* rol dst16, 8 */
			c.byte(8);
			if (sign)
				c.op_r({ 0x0F, 0xBF }, dst, dst);			/* movsx dst, dst16 */
		} else {
			c.op_m({ 0x8B }, dst, ram_at(access));				/* mov dst, dword */
			c.swap32(dst);
		}
	}

	/* The bytes of `size` of src at the access, big-endian (src changed) */
	void store(uint8_t src, uint8_t access, unsigned size)
	{
		c.op_m({ 0xC6 }, 0, { C::PP, C::NO, 0, spoiled });			/* mov byte [m_park.spoiled], 1 */
		c.byte(1);
		if (size == 0) {
			c.op_m({ 0x88 }, src, ram_at(access), false, false, true);	/* mov byte, src8 */
		} else if (size == 1) {
			c.op_r({ 0xC1 }, 0, src, false, true);				/* rol src16, 8 */
			c.byte(8);
			c.op_m({ 0x89 }, src, ram_at(access), false, true);		/* mov word, src16 */
		} else {
			c.swap32(src);
			c.op_m({ 0x89 }, src, ram_at(access));				/* mov dword, src */
		}
	}

	/* The carry-in of an addition: 0 or 1, the C bit of sr, the LSB of c
	 * (in r10) or its inverse */
	enum Cin : unsigned { CIN_0, CIN_1, CIN_SR, CIN_C, CIN_NOT_C };

	/* eax = the ALU's result for F on a, b, c in one piece (alu1, ex.
	 * 13-1), r9d = its N C V Z in bits 31-28. rax, rcx, r9, r11 are free,
	 * rdx, r8, r10 hold b, a, c when they are registers */
	void alu(uint8_t f, bool arith, Val a, Val b, Val cv, unsigned cin)
	{
		Alu u{ c, { C::RCX, C::R11, C::R9, C::RAX } };
		const auto m = [f](unsigned k) { return 0u - ((f >> k) & 1u); };
		const uint32_t f0 = m(0), f1 = m(0) ^ m(2), f2 = m(4), f3 = m(4) ^ m(6);
		const uint32_t f4 = m(1), f5 = m(1) ^ m(3), f6 = m(5), f7 = m(5) ^ m(7);
		/* The minterms of B for A = 0 and C = 0 (u0), C = 1 (v0), then A = 1:
		 * constants, b or ~b, never written */
		const Val u0 = u.xor_(konst(f0), u.and_(konst(f1), b)), v0 = u.xor_(konst(f2), u.and_(konst(f3), b));
		const Val u1 = u.xor_(konst(f4), u.and_(konst(f5), b)), v1 = u.xor_(konst(f6), u.and_(konst(f7), b));
		/* p0 = u0 ^ ((u0 ^ v0) & c) */
		const Val x0 = u.and_(u.xor_(u0, v0), cv);
		const Val p0 = u.xor_(u0, x0);

		u.release(x0, { p0 });
		if (arith && f4 == ~f0 && f5 == f1 && f6 == ~f2 && f7 == f3) {
			/* p1 = ~p0: p = a ^ p0, g = a & p0, (g | p) + g = a + p0 with
			 * the same carries */
			sum(u, a, p0, cin);
			return;
		}

		const Val x1 = u.and_(u.xor_(u1, v1), cv);
		const Val p1 = u.xor_(u1, x1);

		u.release(x1, { p0, p1 });
		/* p = p0 ^ ((p0 ^ p1) & a) */
		const Val q = u.xor_(p0, p1);

		u.release(p1, { p0, q });

		const Val y = u.and_(q, a);

		u.release(q, { p0, y });

		const Val p = u.xor_(p0, y);

		u.release(y, { p0, p });
		if (!arith) {
			u.release(p0, { p });
			u.into(C::RAX, p);
			c.op_rr(0x85, C::RAX, C::RAX);				/* test eax, eax: C, V 0 */
			c.byte(0x9C);						/* pushfq */
			flags();
			return;
		}
		/* g = a & ~p & p0; the sum (g | p) + g + cin, g | p = g ^ p */
		const Val g1 = u.and_(a, p0);

		u.release(p0, { p, g1 });

		const Val g = u.and_(g1, inverse(p));

		u.release(g1, { p, g });

		const Val x = u.xor_(g, p);

		u.release(p, { g, x });
		sum(u, x, g, cin);
	}

	/* eax = x + y + cin with N C V Z in r9d (C: the carry out, V: the carry
	 * into the MSB ^ the carry out, as the host's OF) */
	void sum(Alu &u, const Val &x, const Val &y, unsigned cin)
	{
		Val t = u.copy(x);

		u.release(x, { t, y });

		Val v = y;

		if (!v.k && v.inv) {
			v = u.copy(y);
			u.release(y, { t, v });
		}
		switch (cin) {
		case CIN_1:
			c.byte(0xF9);						/* stc */
			break;
		case CIN_SR:
			c.op_m({ 0x0F, 0xBA }, 4, { C::PP, C::NO, 0, sr });	/* bt dword [sr], 30 */
			c.byte(30);
			break;
		case CIN_C:
		case CIN_NOT_C:
			c.op_r({ 0x0F, 0xBA }, 4, C::R10);			/* bt r10d, 0 */
			c.byte(0);
			if (cin == CIN_NOT_C)
				c.byte(0xF5);					/* cmc */
			break;
		default:
			break;
		}
		if (v.k)
			c.op_ri(cin == CIN_0 ? 0 : 2, t.reg, v.v);		/* add/adc t, value */
		else
			c.op_rr(cin == CIN_0 ? 0x01 : 0x11, t.reg, v.reg);	/* add/adc t, v */

		/* The host's flags kept (C: CF, V: OF, N: SF, Z: ZF) */
		c.byte(0x9C);							/* pushfq */
		if (t.reg != C::RAX)
			c.op_rr(0x89, C::RAX, t.reg);				/* mov eax, t */
		flags();
	}

	/* r9d = N C V Z in bits 31-28 from the rflags pushed (m_ncvz) */
	void flags()
	{
		c.bytes({ 0x41, 0x59 });					/* pop r9 */
		c.op_ri(4, C::R9, 0x8C1);					/* and r9d, SF ZF CF OF */
		c.op_m({ 0x8B }, C::R9, { C::PP, C::R9, 2, ncvz });		/* mov r9d, [m_ncvz + 4 r9] */
	}

	/* The forms DU and F (exec<FORM> for FORM_DU up: alu1, then the access
	 * `x` (0 none, 1 a load, 2 a store), then the writes), the slow path
	 * for an access out of the on-chip RAM, a write on a watched line, or
	 * the classes reading mf while R is set */
	void du(const C82PpOp &op, unsigned cls, bool arith, unsigned x, bool careful, C82PpSlow &s)
	{
		if (x)
			slow_unless_ram(s, ACCESS[0], x == 2);
		if (op.has(C82PpOp::READS_COND))
			condition(op.cond);
		base1(op, cls, arith, s);
		writes(op, x, careful);
	}

	/* r12d = the condition `code` holds on N C V Z (table 8-36) */
	void condition(unsigned code)
	{
		uint32_t mask = 0;

		for (unsigned n = 0; n < 16; n++)
			if ((c82_pp_regs::CONDITIONS[n] >> (code & 15)) & 1)
				mask |= 1u << n;
		c.op_m({ 0x8B }, C::RAX, { C::PP, C::NO, 0, sr });			/* mov eax, [sr] */
		c.shift_ri(5, C::RAX, 28);						/* shr eax, 28 */
		c.mov_ri(C::RCX, mask);
		c.op_rr(0x31, C::R12, C::R12);						/* xor r12d, r12d */
		c.op_r({ 0x0F, 0xA3 }, C::RAX, C::RCX);					/* bt ecx, eax */
		c.op_r({ 0x0F, 0x92 }, 0, C::R12, false, false, true);			/* setc r12b */
	}

	/* eax = base1's result (alu1: ports by class, rotation, mask or mf, the
	 * ALU), r9d = its N C V Z; the condition in r12d already when it reads
	 * it. The slow path for the classes reading mf while R is set */
	void base1(const C82PpOp &op, unsigned cls, bool arith, C82PpSlow &s)
	{
		const bool imm = op.has(C82PpOp::SRC2_IMM);
		const M s2_at = reg_at(0x20 | op.src2), d0_at = reg_at(0x20), sr_at{ C::PP, C::NO, 0, sr };

		if (cls == 0 || cls == 5) {
			c.op_m({ 0xF6 }, 0, sr_at);					/* test byte [sr], 0x40 */
			c.byte(0x40);
			s.from.push_back(c.jump_if(C::JNZ));
		}
		const auto s1_into = [&](uint8_t dst) {
			/* A special register (read_reg) */
			if (s1_special) {
				get_any(dst, s1_special);
				return;
			}

			M at = reg_at(op.s1_slot);

			if (op.has(C82PpOp::S1_PAIR)) {
				at.index = C::R12;
				at.scale = 2;
			}
			c.op_m({ 0x8B }, dst, at);					/* mov dst, source 1 */
		};

		/* Port B: the rotator's input in edx, the rotation by cl or a
		 * constant (table 8-13) */
		Val b = in_reg(C::RDX);
		bool by_cl = false;
		unsigned by = 0;

		if (cls == 7) {
			if (imm) {
				b = konst(std::rotl(1u, static_cast<int>(op.imm & 31)));
			} else {
				c.mov_ri(C::RDX, 1);
				c.op_m({ 0x8B }, C::RCX, s2_at);
				by_cl = true;
			}
		} else {
			s1_into(C::RDX);
			if (cls == 1 || cls == 4 || cls == 5) {
				c.op_m({ 0x8B }, C::RCX, d0_at);
				by_cl = true;
			} else if (cls == 3) {
				if (imm) {
					by = op.imm & 31;
				} else {
					c.op_m({ 0x8B }, C::RCX, s2_at);
					by_cl = true;
				}
			}
		}

		/* Port C in r10d (the count of a mask from ecx: s2 or d0, the
		 * rotation's own) */
		Val cv;

		if (cls == 2 || cls == 3 || cls == 4) {
			if (cls != 4 && imm) {
				const uint32_t n = op.imm & 31;

				cv = konst(n ? (1u << n) - 1 : op.has(C82PpOp::BANG) ? ~0u : 0);
			} else {
				if (cls == 2)
					c.op_m({ 0x8B }, C::RCX, s2_at);
				c.op_ri(4, C::RCX, 31);					/* and ecx, 31 */
				c.mov_ri(C::R10, 1);
				c.op_r({ 0xD3 }, 4, C::R10);				/* shl r10d, cl */
				c.op_ri(5, C::R10, 1);					/* sub r10d, 1 */
				if (op.has(C82PpOp::BANG)) {
					c.op_ri(7, C::RCX, 1);				/* cmp ecx, 1 */
					c.op_rr(0x19, C::RAX, C::RAX);			/* sbb eax, eax */
					c.op_rr(0x09, C::R10, C::RAX);			/* or r10d, eax */
				}
				cv = in_reg(C::R10);
			}
		} else if (cls == 0 || cls == 5) {
			const bool needed = (op.f[0] & 15) != (op.f[0] >> 4) || (op.f[1] & 15) != (op.f[1] >> 4) ||
			                    (arith && op.cin_sel == 2);

			cv = konst(0);
			if (needed) {
				c.op_m({ 0x8B }, C::RAX, sr_at);			/* mov eax, [sr] */
				c.shift_ri(5, C::RAX, 3);				/* shr eax, 3 */
				c.op_ri(4, C::RAX, 7);					/* and eax, 7 */
				c.shift_ri(4, C::RAX, 4);				/* shl eax, 4 */
				c.op_m({ 0x8B }, C::R10, { C::PP, C::NO, 0, mf });	/* mov r10d, [mf] */
				c.op_ri(4, C::R10, 15);					/* and r10d, 15 */
				c.op_rr(0x09, C::RAX, C::R10);				/* or eax, r10d */
				c.move_quad(C::R11, reinterpret_cast<uintptr_t>(EXPAND.data()));
				c.op_m({ 0x8B }, C::R10, { C::R11, C::RAX, 2, 0 });	/* mov r10d, [r11 + 4 rax] */
				cv = in_reg(C::R10);
			}
		} else if (imm) {
			cv = konst(op.imm);
		} else {
			c.op_m({ 0x8B }, C::R10, s2_at);
			cv = in_reg(C::R10);
		}

		/* Port A in r8d */
		Val av = in_reg(C::R8);

		if (op.has(C82PpOp::A_ZERO))
			av = konst(0);
		else if (cls == 0 || cls == 4 || cls == 5)
			av = imm ? konst(op.imm) : (c.op_m({ 0x8B }, C::R8, s2_at), in_reg(C::R8));
		else if (cls == 7)
			s1_into(C::R8);
		else
			c.op_m({ 0x8B }, C::R8, reg_at(0x20 | (op.dst & 7)));

		/* F and the carry-in by the sign of the rotator's input */
		const auto cin = [&](unsigned sign) -> unsigned {
			if (!arith)
				return CIN_0;
			if (op.cin_sel == 0)
				return op.cin_by_sign[sign] ? CIN_1 : CIN_0;
			if (op.cin_sel == 1)
				return CIN_SR;
			if (cv.k)
				return ((cv.v & 1) ^ op.has(C82PpOp::CN9)) ? CIN_1 : CIN_0;
			return op.has(C82PpOp::CN9) ? CIN_NOT_C : CIN_C;
		};
		const auto body = [&](unsigned sign) {
			if (by_cl)
				c.op_r({ 0xD3 }, 0, C::RDX);				/* rol edx, cl */
			else if (by)
				c.shift_ri(0, C::RDX, static_cast<uint8_t>(by));	/* rol edx, by */
			alu(op.f[sign], arith, av, b, cv, cin(sign));
		};

		if (cls != 7 && (op.f[0] != op.f[1] || cin(0) != cin(1))) {
			c.op_rr(0x85, C::RDX, C::RDX);					/* test edx, edx */
			const HostCode::Jump negative = c.jump_if(C::JS);
			body(0);
			const HostCode::Jump done = c.jump_ahead();
			c.bind(negative);
			body(1);
			c.bind(done);
		} else {
			body(0);
		}
	}

	/* The access `x` of a DU form, then its writes (keep = !r || the
	 * condition) */
	void writes(const C82PpOp &op, unsigned x, bool careful)
	{
		if (x == 1) {
			load(C::R11, ACCESS[0], op.msize, op.has(C82PpOp::SIGN));
		} else if (x == 2) {
			get(C::R11, op.reg);
			store(C::R11, ACCESS[0], op.msize);
		}

		HostCode::Jump skip{ 0 };

		if (op.has(C82PpOp::R)) {
			c.op_rr(0x85, C::R12, C::R12);					/* test r12d, r12d */
			skip = c.jump_if(C::JZ);
		}
		if (op.has(C82PpOp::SETS_STATUS))
			status(op.protect());
		write(op.dst, C::RAX, careful);
		if (op.has(C82PpOp::R))
			c.bind(skip);
		if (x == 1)
			put(op.reg, C::R11, careful);
	}

	/* execute_generic for the instructions written here (write_exec): the
	 * condition, the data unit (base1, or MPY||ADD: mpy_add), the accesses
	 * in their order (global, then local), then apply() (status, the
	 * product, the ALU's result when keep) and the writes of the transfers
	 * (local, then global; a conditional global one only when the
	 * condition holds) and of the move. `ea`: the addresses of the
	 * transfers */
	void generic(const C82PpInsn &in, const C82PpOp &op, int32_t ea, bool careful, C82PpSlow &s)
	{
		static constexpr uint8_t VALUE[2] = { C::R10, C::R11 };
		bool pair = false;

		for (unsigned m = 0; m < in.mems; m++)
			slow_unless_ram(s, ACCESS[m], in.xfer[in.mem_order[m]].le == 0);
		for (unsigned k = 0; k < in.xfers; k++)
			pair |= in.xfer[k].pair;
		if (in.r || in.c || in.g || pair)
			condition(in.cond);

		/* The data unit: eax, r9d; the product in ecx */
		const uint8_t dst = in.dst_code();
		const bool mpy = in.path == C82PpInsn::MPY_ADD;
		const bool same = mpy && in.dst2 == (dst & 7) && (dst & 0x78) == 0x20;

		if (in.path == C82PpInsn::BASE1) {
			base1(op, in.cls, in.arith, s);
		} else if (mpy && !same) {
			/* src2 + src1, or src2 + ~src1 + 1 (opers 1, 5) */
			const unsigned code = in.src1_code();
			M s1_at = reg_at(c82_pp_regs::READ_SLOT[code]);

			if (in.c && (code & 0x78) == 0x20) {
				s1_at = reg_at(0x20 | (code & 6));
				s1_at.index = C::R12;
				s1_at.scale = 2;
			}
			c.op_m({ 0x8B }, C::R8, reg_at(0x20 | in.src2));		/* mov r8d, src2 */
			c.op_m({ 0x8B }, C::RDX, s1_at);				/* mov edx, source 1 */

			Alu u{ c, { C::RCX, C::R11, C::R9, C::RAX } };

			sum(u, in_reg(C::R8), (in.oper & 1) ? inverse(in_reg(C::RDX)) : in_reg(C::RDX),
			    (in.oper & 1) ? CIN_1 : CIN_0);
		}
		if (mpy) {
			/* 16 x 16, signed unless opers 4, 5 (pp_multiply) */
			const uint8_t extend = (in.oper & 4) ? 0xB7 : 0xBF;

			c.op_m({ 0x0F, extend }, C::RCX, reg_at(0x20 | in.src3));	/* movsx/movzx ecx, src3 */
			c.op_m({ 0x0F, extend }, C::RDX, reg_at(0x20 | in.src4));	/* movsx/movzx edx, src4 */
			c.op_r({ 0x0F, 0xAF }, C::RCX, C::RDX);				/* imul ecx, edx */
		}
		if (in.move)
			get(C::R8, in.move_src);

		/* The accesses */
		for (unsigned m = 0; m < in.mems; m++) {
			const unsigned k = in.mem_order[m];
			const C82PpXfer &t = in.xfer[k];

			if (t.divert)
				divert(ea + 4 * static_cast<int32_t>(k));
			if (t.le == 0) {
				/* A conditional global store of a D pair: the odd one
				 * when the condition holds (§ 8.8.3.2) */
				M at = reg_at(c82_pp_regs::READ_SLOT[t.pair ? (t.reg & ~1u) : t.reg]);

				if (t.pair) {
					at.index = C::R12;
					at.scale = 2;
				}
				c.op_m({ 0x8B }, C::RDX, at);				/* mov edx, the data */
				store(C::RDX, ACCESS[m], t.msize);
			} else {
				load(VALUE[k], ACCESS[m], t.msize, t.le & 1);
			}
		}

		/* apply() */
		HostCode::Jump skip{ 0 };

		if (in.path != C82PpInsn::NO_DU && in.r) {
			c.op_rr(0x85, C::R12, C::R12);					/* test r12d, r12d */
			skip = c.jump_if(C::JZ);
		}
		if (in.path == C82PpInsn::BASE1) {
			if (in.sets_status)
				status(in.protect);
			write(dst, C::RAX, careful);
		} else if (mpy) {
			if (!same)
				status(in.protect);
			put(0x20 | in.dst2, C::RCX, careful);
			if (!same)
				put(dst, C::RAX, careful);
		}
		if (in.path != C82PpInsn::NO_DU && in.r)
			c.bind(skip);

		/* The transfers, then the move */
		for (unsigned j = 0; j < in.writes; j++) {
			const unsigned k = in.write_order[j];
			const C82PpXfer &t = in.xfer[k];
			const bool when = t.global && in.g;
			HostCode::Jump no{ 0 };
			uint8_t value = VALUE[k];

			if (when) {
				c.op_rr(0x85, C::R12, C::R12);				/* test r12d, r12d */
				no = c.jump_if(C::JZ);
			}
			if (t.le == 1) {
				/* Address unit arithmetic: the address, its 8 or 16
				 * LSBs sign-extended for b and h (§ 8.8.5) */
				c.op_m({ 0x8B }, C::RDX, { C::PP, C::NO, 0, ea + 4 * static_cast<int32_t>(k) });
				if (t.size == 0)
					c.op_r({ 0x0F, 0xBE }, C::RDX, C::RDX, false, false, true);	/* movsx edx, dl */
				else if (t.size == 1)
					c.op_r({ 0x0F, 0xBF }, C::RDX, C::RDX);		/* movsx edx, dx */
				value = C::RDX;
			}
			put(t.reg, value, careful);
			if (when)
				c.bind(no);
		}
		if (in.move) {
			HostCode::Jump no{ 0 };

			if (in.g) {
				c.op_rr(0x85, C::R12, C::R12);				/* test r12d, r12d */
				no = c.jump_if(C::JZ);
			}
			put(in.move_dst, C::R8, careful);
			if (in.g)
				c.bind(no);
		}
	}

	/* sr's N C V Z from r9d, but the bits protected (apply) */
	void status(uint32_t protect)
	{
		const uint32_t set = SR_NCVZ & ~protect;
		const M sr_at{ C::PP, C::NO, 0, sr };

		if (!set)
			return;
		c.op_m({ 0x81 }, 4, sr_at);						/* and dword [sr], ~set */
		c.word(~set);
		if (set != SR_NCVZ)
			c.op_ri(4, C::R9, set);						/* and r9d, set */
		c.op_m({ 0x09 }, C::R9, sr_at);						/* or [sr], r9d */
	}

	/* The slow path when the access is not in the on-chip RAM (-1), or is
	 * a write on a watched line (written_ram) */
	void slow_unless_ram(C82PpSlow &s, uint8_t access, bool write)
	{
		c.op_r({ 0x85 }, access, access, true);					/* test access, access */
		s.from.push_back(c.jump_if(C::JS));
		if (!write)
			return;
		c.op_r({ 0x8B }, C::RCX, access);					/* mov ecx, access */
		c.shift_ri(5, C::RCX, 12);						/* shr ecx, 12 */
		c.op_m({ 0x8B }, C::RDX, { C::PP, C::RCX, 3, watched }, true);		/* mov rdx, [m_watched_lines + 8 rcx] */
		c.op_r({ 0x8B }, C::RCX, access);					/* mov ecx, access */
		c.shift_ri(5, C::RCX, 6);						/* shr ecx, 6 */
		c.op_r({ 0x0F, 0xA3 }, C::RCX, C::RDX, true);				/* bt rdx, rcx */
		s.watched.push_back(c.jump_if(C::JC));
	}
};

} /* namespace */

/* The accesses of the step's execute stage, before the step (run_block's
 * test, plain()): an address below 0x02000000 in a block of on-chip RAM the
 * PPs reach gives its offset in m_ram (pp_ram); one in plain external memory
 * -1 (on a slow path, plain() called); anything else ends the block */
void
C82Pp::write_memory(C82PpCode &c, const BlockStep &st, unsigned k, bool handler, std::vector<C82PpSlow> &slow,
                    std::vector<uint32_t> &stops)
{
	const unsigned se = (k & 1) ^ 1;
	const int32_t internal = HostCode::place_in(this, m_nat_internal.data());

	for (unsigned m = 0; m < st.mems; m++) {
		const uint8_t access = ACCESS[m];
		C82PpSlow s;

		c.op_m({ 0x8B }, C::RAX, { C::PP, C::NO, 0, HostCode::place_in(this, &m_nat_ea[se][st.mem_x[m]]) });	/* mov eax, address */
		if (st.mem_size[m])
			c.op_ri(4, C::RAX, ~((1u << st.mem_size[m]) - 1));		/* and eax, the alignment */
		c.op_ri(7, C::RAX, 0x02000000u);					/* cmp eax, 0x02000000 */
		s.from.push_back(c.jump_if(C::JAE));
		c.op_r({ 0x8B }, C::RCX, C::RAX);					/* mov ecx, eax */
		c.shift_ri(5, C::RCX, 12);						/* shr ecx, 12 */
		c.op_m({ 0x0F, 0xBF }, C::RCX, { C::PP, C::RCX, 1, internal }, true);	/* movsx rcx, [m_internal + 2 rcx] */
		c.op_r({ 0x85 }, C::RCX, C::RCX);					/* test ecx, ecx: no RAM, or the data cache */
		stops.push_back(c.jump_if(C::JS).from);
		c.op_ri(4, C::RAX, 0xFFF);						/* and eax, 0xFFF */
		c.op_m({ 0x8D }, access, { C::RAX, C::RCX, 0, 0 }, true);		/* lea access, [rax + rcx] */

		/* A store alone made by the handler: its line looked at out
		 * of the way (on a watched one the block ends before the step;
		 * written here, slow_unless_ram's slow path looks) */
		const bool store = (st.mem_store >> m) & 1;
		C82PpSlow w;

		if (store && handler) {
			c.op_m({ 0x80 }, 7, { C::PP, C::NO, 0, HostCode::place_in(this, &m_alone) });	/* cmp byte [m_alone], 0 */
			c.byte(0);
			w.from.push_back(c.jump_if(C::JNZ));
		}
		s.back = c.here();
		s.k = k;
		s.se = se;
		s.reg = access;
		s.store = store;
		slow.push_back(s);
		if (store && handler) {
			w.back = s.back;
			w.k = k;
			w.reg = access;
			w.watch = true;
			slow.push_back(w);
		}
	}
}

/* Whether write_exec writes the step's execute stage in host code
 * (PLAN_WRITTEN; its tests: see there), and whether that code reads comm
 * or writes it (comm_read, comm_written: they read m_cycles, the step's
 * frame stays) */
unsigned
C82Pp::exec_plan(const BlockStep &st) const
{
	const C82PpOp &op = *st.eop;
	unsigned plan = PLAN_WRITTEN;

	switch (st.form) {
	case C82PpInsn::FORM_GENERIC: {
		/* The data unit's base set in one piece or MPY||ADD, or none;
		 * transfers, a move (not a field move, not a command) and every
		 * register below 0x28, no load to pc. Its step stays SPECIAL,
		 * BRANCH and CAREFUL (traits()): the frame around is the prepared
		 * blocks'. The base set's source 1 and destination may be special
		 * (readable, writable: a helper only when nothing follows) */
		const C82PpInsn &in = *st.ein;
		const bool lone = !in.xfers && !in.move;
		bool simple = !in.illegal && !in.fmove && in.mems == st.mems &&
		              (in.path == C82PpInsn::NO_DU || in.path == C82PpInsn::MPY_ADD || in.path == C82PpInsn::BASE1);

		if (in.path == C82PpInsn::MPY_ADD && (in.dst_code() >= 0x28 || in.src1_code() >= 0x28))
			simple = false;
		if (in.path == C82PpInsn::BASE1 &&
		    (!Writer::writable(in.dst_code(), lone) || (in.s1_special && !Writer::readable(in.src1_code()))))
			simple = false;
		if (in.path == C82PpInsn::BASE1 && in.s1_special && in.src1_code() == 0x3E)
			plan |= PLAN_COMM_READ;
		if (in.path == C82PpInsn::BASE1 && in.dst_code() == 0x3E)
			plan |= PLAN_COMM_WRITE;
		if (in.move && (in.move_src >= 0x28 || in.move_dst >= 0x28 || in.move_dst == 0xF))
			simple = false;
		for (unsigned x = 0; x < in.xfers; x++)
			if (in.xfer[x].pcload || in.xfer[x].reg >= 0x28)
				simple = false;
		if (!simple)
			return 0;
		break;
	}
	case C82PpInsn::FORM_LOAD:
	case C82PpInsn::FORM_LEA:
		/* A single transfer is never format 1's local one (DIVERT): both
		 * of its transfers always exist (A.4.1). The register may
		 * be special (writable; nothing follows its write) */
		if (!Writer::writable(op.reg, true) || op.has(C82PpOp::DIVERT))
			return 0;
		if (op.reg == 0x3E)
			plan |= PLAN_COMM_WRITE;
		break;
	case C82PpInsn::FORM_STORE:
		if (!Writer::readable(op.reg) || op.has(C82PpOp::DIVERT))
			return 0;
		if (op.reg == 0x3E)
			plan |= PLAN_COMM_READ;
		break;
	default:
		/* The base set in one piece (BASE1), registers below 0x28. Its
		 * destination may be special (writable; a helper alone) */
		if (st.form < C82PpInsn::FORM_DU || st.form >= C82PpInsn::FORM_LEA ||
		    !Writer::writable(op.dst, st.ein->shape == C82PpInsn::DU) ||
		    (st.ein->shape != C82PpInsn::DU && (op.reg >= 0x28 || op.has(C82PpOp::DIVERT))))
			return 0;
		if (op.dst == 0x3E)
			plan |= PLAN_COMM_WRITE;
		break;
	}
	return plan;
}

/* The execute stage of the step (fetching at pc) in host code, for the
 * forms written here: false for the rest (the caller calls the handler).
 * Every register code below 0x28; the special registers of
 * Writer::readable and writable too (ipa and ipe as the step's frame
 * makes them) */
bool
C82Pp::write_exec(C82PpCode &c, const BlockStep &st, unsigned k, uint32_t pc, uint32_t ipa, uint32_t ipe, bool frame,
                  std::vector<C82PpSlow> &slow)
{
	const C82PpOp &op = *st.eop;
	const unsigned se = (k & 1) ^ 1, flags = st.flags;
	const bool careful = flags & CAREFUL;
	const Tms320c82::RamMap map = m_chip.ram_map();
	Writer w{ c,
		  HostCode::place_in(this, m_r.data()),
		  HostCode::place_in(this, &m_written),
		  HostCode::place_in(this, &m_park.spoiled),
		  HostCode::place_in(this, map.ram),
		  HostCode::place_in(this, map.watched),
		  HostCode::place_in(this, &m_sr),
		  HostCode::place_in(this, &m_mf),
		  HostCode::place_in(this, &m_iprs),
		  HostCode::place_in(this, &m_new_pc),
		  HostCode::place_in(this, &m_new_pc_bits),
		  HostCode::place_in(this, &m_pc_written),
		  HostCode::place_in(this, m_ncvz.data()),
		  m_number,
		  (pc & ~7u) + 8 };
	const auto ea = [this, se](unsigned x) { return HostCode::place_in(this, &m_nat_ea[se][x]); };
	const unsigned plan = exec_plan(st);

	if (!(plan & PLAN_WRITTEN))
		return false;
	w.pc = pc;
	w.ipa = ipa;
	w.ipe = ipe;
	w.inten = HostCode::place_in(this, &m_inten);
	w.intflg = HostCode::place_in(this, &m_intflg);
	w.lctl = HostCode::place_in(this, &m_lctl);
	w.lc = HostCode::place_in(this, m_lc.data());
	w.lr = HostCode::place_in(this, m_lr.data());
	w.ls = HostCode::place_in(this, m_ls.data());
	w.le = HostCode::place_in(this, m_le.data());
	w.lc_written = HostCode::place_in(this, &m_lc_written);
	w.comm = HostCode::place_in(this, &m_nat_comm);
	w.comm_written = reinterpret_cast<uintptr_t>(&C82Pp::comm_written);
	w.loops_written = reinterpret_cast<uintptr_t>(&C82Pp::loops_written);
	if (st.form == C82PpInsn::FORM_GENERIC && st.ein->path == C82PpInsn::BASE1 && st.ein->s1_special)
		w.s1_special = st.ein->src1_code();
	/* comm read first (comm_read), when the step reads it */
	if (plan & PLAN_COMM_READ) {
		c.pp_first();
		c.call_to(reinterpret_cast<uintptr_t>(&C82Pp::comm_read));
	}

	C82PpSlow s;

	s.handler = true;
	/* The frame before the handler when the step has none */
	s.frame = frame;
	s.pc = pc;
	s.ipa = ipa;
	s.ipe = ipe;
	s.protect = op.protect();
	s.k = k;
	s.se = se;
	s.memory = flags & MEMORY;
	s.form = st.form;
	s.op = st.eop;
	s.in = st.ein;
	if (st.form == C82PpInsn::FORM_LOAD || st.form == C82PpInsn::FORM_STORE)
		w.slow_unless_ram(s, ACCESS[0], st.form == C82PpInsn::FORM_STORE);

	switch (st.form) {
	case C82PpInsn::FORM_LOAD:
		w.load(C::RAX, ACCESS[0], op.msize, op.has(C82PpOp::SIGN));
		w.write(op.reg, C::RAX, careful);
		break;
	case C82PpInsn::FORM_STORE:
		w.get_any(C::RAX, op.reg);
		w.store(C::RAX, ACCESS[0], op.msize);
		break;
	case C82PpInsn::FORM_LEA:
		/* The address, its 8 or 16 LSBs sign-extended for b and h */
		c.op_m({ 0x8B }, C::RAX, { C::PP, C::NO, 0, ea(0) });			/* mov eax, address */
		if (op.msize == 0)
			c.op_r({ 0x0F, 0xBE }, C::RAX, C::RAX, false, false, true);	/* movsx eax, al */
		else if (op.msize == 1)
			c.op_r({ 0x0F, 0xBF }, C::RAX, C::RAX);				/* movsx eax, ax */
		w.write(op.reg, C::RAX, careful);
		break;
	case C82PpInsn::FORM_GENERIC:
		w.generic(*st.ein, op, ea(0), careful, s);
		break;
	default:
		w.du(op, st.ein->cls, st.ein->arith, st.ein->shape - C82PpInsn::DU, careful, s);
		break;
	}
	s.back = c.here();
	if (!s.from.empty() || !s.watched.empty())
		slow.push_back(s);
	return true;
}

/* A slow path: the handler called as the prepared blocks call it, the
 * cycles it adds counted; or plain() for an external address (0: the block
 * ends before the step), the access register then -1 */
void
C82Pp::write_slow(C82PpCode &c, const C82PpSlow &s, std::vector<uint32_t> &stops)
{
	if (!s.watched.empty()) {
		/* Alone, a store on a watched line: the block ends before
		 * the step (nothing written yet but its scratch: the address
		 * stage's, the frame, which run_blocks makes again) */
		for (const HostCode::Jump &j : s.watched)
			c.bind(j);
		c.op_m({ 0x80 }, 7, { C::PP, C::NO, 0, HostCode::place_in(this, &m_alone) });	/* cmp byte [m_alone], 0 */
		c.byte(0);
		stops.push_back(c.jump_if(C::JNZ).from);
	}
	for (const HostCode::Jump &j : s.from)
		c.bind(j);
	if (s.handler) {
		if (s.frame)
			write_frame(c, s.pc, s.ipa, s.ipe, s.protect);
		c.pp_first();
		c.move_quad(C::ARG[1], reinterpret_cast<uintptr_t>(s.op));
		c.move_quad(C::ARG[2], reinterpret_cast<uintptr_t>(s.in));
		c.op_mem(0x8D, C::ARG[3], C::PP, HostCode::place_in(this, &m_nat_ea[s.se][0]), true);	/* lea the fourth argument, the addresses */
		c.call_to(reinterpret_cast<uintptr_t>(EXEC[s.form]));
		if (s.memory) {
			const int32_t extra = HostCode::place_in(this, &m_extra);

			c.op_mem(0x8B, C::RAX, C::PP, extra);				/* mov eax, [m_extra] */
			c.bytes({ 0x48, 0x29, 0xC5 });					/* sub rbp, rax */
			c.op_mem(0xC7, 0, C::PP, extra);				/* mov dword [m_extra], 0 */
			c.word(0);
		}
	} else if (s.watch) {
		/* Alone, a store on a watched line (as slow_unless_ram
		 * tests it): the block ends before the step */
		const int32_t watched = HostCode::place_in(this, m_chip.ram_map().watched);

		c.op_r({ 0x8B }, C::RCX, s.reg);					/* mov ecx, access */
		c.shift_ri(5, C::RCX, 12);						/* shr ecx, 12 */
		c.op_m({ 0x8B }, C::RDX, { C::PP, C::RCX, 3, watched }, true);		/* mov rdx, [m_watched_lines + 8 rcx] */
		c.op_r({ 0x8B }, C::RCX, s.reg);					/* mov ecx, access */
		c.shift_ri(5, C::RCX, 6);						/* shr ecx, 6 */
		c.op_r({ 0x0F, 0xA3 }, C::RCX, C::RDX, true);				/* bt rdx, rcx */
		stops.push_back(c.jump_if(C::JC).from);
	} else {
		c.op_r({ 0x89 }, C::RAX, C::ARG[1]);					/* mov the second argument, eax */
		c.pp_first();
		/* A store alone ends the block (plain_store) */
		c.call_to(reinterpret_cast<uintptr_t>(s.store ? &C82Pp::plain_store : &C82Pp::plain));
		c.bytes({ 0x85, 0xC0 });						/* test eax, eax */
		stops.push_back(c.jump_if(C::JZ).from);
		c.op_r({ 0xC7 }, 0, s.reg, true);					/* mov access, -1 */
		c.word(0xFFFFFFFFu);
	}
	c.jump_back(s.back);
}

#else

void
C82Pp::write_memory(C82PpCode &, const BlockStep &, unsigned, bool, std::vector<C82PpSlow> &, std::vector<uint32_t> &)
{
}

unsigned
C82Pp::exec_plan(const BlockStep &) const
{
	return 0;
}

bool
C82Pp::write_exec(C82PpCode &, const BlockStep &, unsigned, uint32_t, uint32_t, uint32_t, bool, std::vector<C82PpSlow> &)
{
	return false;
}

void
C82Pp::write_slow(C82PpCode &, const C82PpSlow &, std::vector<uint32_t> &)
{
}

#endif

} /* namespace odin */
