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


/* The bodies of the MP's steps written in x86-64 (the frame around them:
 * c82_mp_host.cpp). Each does what its handler (c82_mp_ops.cpp) does, in
 * the same order, for the cases it takes; any other case jumps out of the
 * line to the handler itself, before anything is written. AArch64 and the
 * other hosts have no blocks: the interpreter runs there. */

#include "odin/machine/cpu/tms320c82/c82_mp_host.hpp"

#include <array>
#include <bit>
#include <vector>

#include "odin/machine/cpu/tms320c82/c82_mp.hpp"
#include "odin/machine/cpu/tms320c82/tms320c82.hpp"

namespace odin {

#if defined(ODIN_HOST_X86_64)

/* bcnd, bbz, bbo, bsr, jsr (C82MpOps::bcnd, bit_branch, call): r12d = the
 * next instruction after the delay slot (or after the branch, annulled):
 * the target when taken; ecx the target */
void
C82Mp::write_branch(C82MpCode &c, const BlockStep &st)
{
	using C = C82MpCode;
	using Mem = C82MpCode::Mem;
	const auto reg = [this](unsigned n) { return Mem{ C::MP, C::NO, 0, HostCode::place_in(this, &m_r[n & 31]) }; };
	const C82MpOp &op = st.op;
	const C82Insn &in = st.in;
	const bool from_reg = in.form == C82Insn::REGISTER;

	if (in.op == C82Insn::JSR) {
		if (from_reg)
			c.op_m(0, { 0x8B }, C::RCX, reg(op.src1));		/* mov ecx, [r(src1)] */
		else
			c.move32(C::RCX, op.imm);
		c.op_m(0, { 0x03 }, C::RCX, reg(op.src2));			/* add ecx, [r(src2)] */
		c.op_ri(4, C::RCX, ~3u);					/* and ecx, ~3 */
	} else if (from_reg) {
		c.op_m(0, { 0x8B }, C::RCX, reg(op.src1));			/* mov ecx, [r(src1)] */
		c.shift_ri(4, C::RCX, 2);					/* shl ecx, 2 */
		c.op_ri(0, C::RCX, st.ip);					/* add ecx, ip */
	} else {
		c.move32(C::RCX, st.ip + 4 * op.imm);
	}
	if (in.op == C82Insn::BSR || in.op == C82Insn::JSR) {
		/* The link after the operands are read */
		c.op_r(0, { 0x89 }, C::RCX, C::NEXT);				/* mov r12d, ecx */
		if (op.dest) {
			c.op_m(0, { 0xC7 }, 0, reg(op.dest));			/* mov dword [r(dest)], link */
			c.word(st.ip + st.length + (in.annul ? 0 : 4));
		}
		return;
	}
	c.move32(C::NEXT, st.ip + st.length + (in.annul ? 0 : 4));
	C82MpCode::Cc cc = C::E;
	if (in.op == C82Insn::BCND) {
		/* The mask of the cases negative (bit 0), zero (1), positive (2) */
		switch (op.a) {
		case 0:
			return;
		case 7:
			c.op_r(0, { 0x89 }, C::RCX, C::NEXT);			/* mov r12d, ecx */
			return;
		case 4:
			cc = C::G;
			break;
		case 2:
			cc = C::E;
			break;
		case 6:
			cc = C::GE;
			break;
		case 1:
			cc = C::L;
			break;
		case 5:
			cc = C::NE;
			break;
		default:
			cc = C::LE;
			break;
		}
		c.op_m(0, { 0x8B }, C::RAX, reg(op.src2));			/* mov eax, [r(src2)] */
		if (op.b == 24)
			c.op_r(0, { 0x0F, 0xBE }, C::RAX, C::RAX);		/* movsx eax, al */
		else if (op.b == 16)
			c.op_r(0, { 0x0F, 0xBF }, C::RAX, C::RAX);		/* movsx eax, ax */
		c.op_r(0, { 0x85 }, C::RAX, C::RAX);				/* test eax, eax */
	} else {
		c.op_m(0, { 0x0F, 0xBA }, 4, reg(op.src2));			/* bt dword [r(src2)], bit */
		c.byte(op.a);
		cc = in.op == C82Insn::BBO ? C::B : C::AE;
	}
	c.op_r(0, { 0x0F, static_cast<uint8_t>(0x40 | cc) }, C::NEXT, C::RCX);	/* cmov<cc> r12d, ecx */
}

/* The body of step k when it is written out: integers (add, sub, the
 * Boolean ops, short shifts), swcr of the vector pointers, loads and stores
 * (the on-chip RAM and plain pages straight, anything else by the handler),
 * the floating point (c82_fast's cases, the others by the handler) */
bool
C82Mp::write_body(C82MpCode &c, const std::vector<BlockStep> &steps, unsigned k, std::vector<C82MpSlow> &slow)
{
	using C = C82MpCode;
	using Mem = C82MpCode::Mem;
	const BlockStep &st = steps[k];
	const C82MpOp &op = st.op;
	const C82Insn &in = st.in;
	const auto at = [this](const void *member) { return HostCode::place_in(this, member); };
	const auto reg = [&](unsigned n) { return Mem{ C::MP, C::NO, 0, at(&m_r[n & 31]) }; };
	const bool from_reg = in.form == C82Insn::REGISTER;
	/* eax = source1: a register or the immediate */
	const auto source1 = [&](uint8_t to) {
		if (from_reg)
			c.op_m(0, { 0x8B }, to, reg(op.src1));			/* mov to, [r(src1)] */
		else
			c.move32(to, op.imm);
	};
	const auto put = [&](uint8_t from, unsigned dest) {
		if (dest & 31)
			c.op_m(0, { 0x89 }, from, reg(dest));			/* mov [r(dest)], from */
	};

	switch (in.op) {
	case C82Insn::ADD:
	case C82Insn::SUB: {
		/* add: source1 + source2, sub: source1 - source2; a signed
		 * overflow posts io (C82MpOps::add) */
		const bool sub = in.op == C82Insn::SUB;

		if (in.is_unsigned && !op.dest)
			return true;
		if (sub || from_reg) {
			source1(C::RAX);
			c.op_m(0, { static_cast<uint8_t>(sub ? 0x2B : 0x03) }, C::RAX, reg(op.src2));	/* add/sub eax, [r(src2)] */
		} else {
			c.op_m(0, { 0x8B }, C::RAX, reg(op.src2));		/* mov eax, [r(src2)] */
			c.op_ri(0, C::RAX, op.imm);				/* add eax, imm */
		}
		if (!in.is_unsigned) {
			C82MpSlow s;

			s.kind = C82MpSlow::OVERFLOW;
			s.k = k;
			s.dest = op.dest;
			s.from.push_back(c.jump_if(C::OV));
			slow.push_back(std::move(s));
		}
		put(C::RAX, op.dest);
		return true;
	}
	case C82Insn::LOGIC: {
		/* The truth table: bit 0 a&b, 1 a&~b, 2 ~a&b, 3 ~a&~b */
		if (!op.dest)
			return true;
		source1(C::RAX);
		c.op_m(0, { 0x8B }, C::RCX, reg(op.src2));			/* mov ecx, [r(src2)] */
		switch (in.logic) {
		case 0:
			c.move32(C::RAX, 0);
			break;
		case 1:
			c.op_r(0, { 0x21 }, C::RCX, C::RAX);			/* and eax, ecx */
			break;
		case 2:
			c.not_r(C::RCX);
			c.op_r(0, { 0x21 }, C::RCX, C::RAX);			/* and eax, ~ecx */
			break;
		case 3:
			break;
		case 4:
			c.not_r(C::RAX);
			c.op_r(0, { 0x21 }, C::RCX, C::RAX);			/* and ~eax, ecx */
			break;
		case 5:
			c.op_r(0, { 0x89 }, C::RCX, C::RAX);			/* mov eax, ecx */
			break;
		case 6:
			c.op_r(0, { 0x31 }, C::RCX, C::RAX);			/* xor eax, ecx */
			break;
		case 7:
			c.op_r(0, { 0x09 }, C::RCX, C::RAX);			/* or eax, ecx */
			break;
		case 8:
			c.op_r(0, { 0x09 }, C::RCX, C::RAX);
			c.not_r(C::RAX);
			break;
		case 9:
			c.op_r(0, { 0x31 }, C::RCX, C::RAX);
			c.not_r(C::RAX);
			break;
		case 10:
			c.op_r(0, { 0x89 }, C::RCX, C::RAX);
			c.not_r(C::RAX);
			break;
		case 11:
			c.not_r(C::RCX);
			c.op_r(0, { 0x09 }, C::RCX, C::RAX);
			break;
		case 12:
			c.not_r(C::RAX);
			break;
		case 13:
			c.not_r(C::RAX);
			c.op_r(0, { 0x09 }, C::RCX, C::RAX);
			break;
		case 14:
			c.op_r(0, { 0x21 }, C::RCX, C::RAX);
			c.not_r(C::RAX);
			break;
		default:
			c.move32(C::RAX, ~0u);
			break;
		}
		put(C::RAX, op.dest);
		return true;
	}
	case C82Insn::SHIFT: {
		/* The short forms C82MpOps::shift takes (choose): rotated left by
		 * a, CM in aux; outside CM 0, the old bits, or the top bit of a
		 * low CM (bit b) */
		if (in.form != C82Insn::SHORT)
			return false;

		static constexpr uint8_t MERGE[8] = { 0, 1, 2, 0, 1, 2, 0, 1 };
		const uint32_t cm = op.aux;
		unsigned kind = MERGE[in.merge];

		if (kind == 2) {
			if ((cm & (cm + 1)) != 0)
				return false;
			if (!cm || cm == ~0u)
				kind = 0;
		}
		if (!op.dest)
			return true;
		c.op_m(0, { 0x8B }, C::RAX, reg(op.src2));			/* mov eax, [r(src2)] */
		if (op.a & 31)
			c.shift_ri(0, C::RAX, op.a & 31);			/* rol eax, a */
		if (cm != ~0u)
			c.op_ri(4, C::RAX, cm);					/* and eax, cm */
		if (kind == 1 && cm != ~0u) {
			c.op_m(0, { 0x8B }, C::RCX, reg(op.dest));		/* mov ecx, [r(dest)] */
			c.op_ri(4, C::RCX, ~cm);				/* and ecx, ~cm */
			c.op_r(0, { 0x09 }, C::RCX, C::RAX);			/* or eax, ecx */
		} else if (kind == 2) {
			c.op_r(0, { 0x89 }, C::RAX, C::RCX);			/* mov ecx, eax */
			if (31 - op.b)
				c.shift_ri(4, C::RCX, static_cast<uint8_t>(31 - op.b));	/* shl ecx, 31 - b */
			c.shift_ri(7, C::RCX, 31);				/* sar ecx, 31 */
			c.op_ri(4, C::RCX, ~cm);				/* and ecx, ~cm */
			c.op_r(0, { 0x09 }, C::RCX, C::RAX);			/* or eax, ecx */
		}
		put(C::RAX, op.dest);
		return true;
	}
	case C82Insn::SWCR: {
		/* wrcr, swcr of IN0P, IN1P, OUTP (C82MpOps::swcr_pointer) */
		if (from_reg || in.imm < IN0P || in.imm > OUTP)
			return false;

		const int32_t pointer = in.imm == IN0P ? at(&m_in0p) : in.imm == IN1P ? at(&m_in1p) : at(&m_outp);

		c.op_m(0, { 0x8B }, C::RAX, Mem{ C::MP, C::NO, 0, pointer });	/* mov eax, [pointer] */
		c.op_m(0, { 0xC6 }, 0, Mem{ C::MP, C::NO, 0, at(&m_park.spoiled) });	/* mov byte [spoiled], 1 */
		c.byte(1);
		c.op_m(0, { 0x8B }, C::RCX, reg(op.src2));			/* mov ecx, [r(src2)] */
		c.op_m(0, { 0x89 }, C::RCX, Mem{ C::MP, C::NO, 0, pointer });	/* mov [pointer], ecx */
		put(C::RAX, op.dest);
		return true;
	}
	case C82Insn::LOAD:
	case C82Insn::STORE:
		if (in.op == C82Insn::LOAD && in.is_unsigned && in.size >= 2)
			return false;
		return write_memory(c, steps, k, slow);
	case C82Insn::CMP:
		/* C82MpOps::cmp: the status word by compare_all, called */
		if (!op.dest)
			return true;
		if (from_reg)
			c.op_m(0, { 0x8B }, C::ARG[0], reg(op.src1));		/* mov the first argument, [r(src1)] */
		else
			c.move32(C::ARG[0], op.imm);
		c.op_m(0, { 0x8B }, C::ARG[1], reg(op.src2));			/* mov the second argument, [r(src2)] */
		c.call_to(reinterpret_cast<uintptr_t>(&C82Mp::compare_all));
		put(C::RAX, op.dest);
		return true;
	case C82Insn::FADD:
	case C82Insn::FSUB:
	case C82Insn::FMPY:
	case C82Insn::FDIV:
	case C82Insn::FCMP:
	case C82Insn::FRND:
	case C82Insn::VECTOR:
		return write_float(c, steps, k, slow);
	default:
		return false;
	}
}

namespace {

/* fcmp's status by (x < y, =, >), (x < 0, = 0, > 0), y < 0: from
 * c82_fast::compare itself, on singles giving each case */
const std::array<uint32_t, 18> &
compare_table()
{
	static const std::array<uint32_t, 18> table = [] {
		std::array<uint32_t, 18> t{};
		static constexpr float VALUES[] = { -2.0f, -1.0f, -0.5f, 0.0f, 0.5f, 1.0f, 2.0f };

		for (const float x : VALUES)
			for (const float y : VALUES) {
				uint32_t status = 0;

				c82_fast::compare(std::bit_cast<uint32_t>(x), FP_SINGLE, std::bit_cast<uint32_t>(y), FP_SINGLE, status);
				t[static_cast<size_t>(((x > y) + (x >= y)) * 6 + ((x > 0) + (x >= 0)) * 2 + (y < 0))] = status;
			}
		return t;
	}();

	return table;
}

} /* namespace */

/* The routines the steps call, written once after the way in: the memory
 * reached straight (Tms320c82::mp_load, mp_store), the floating point of
 * c82_fast (C82MpOps::float_sss, fcmp, vector) in the same operations of the
 * host on doubles (SSE2, rounding to nearest: m_float_mode says the host
 * does), with fast_done. They keep rbx, rbp, r12-r15 (the memory ones r8
 * too) and use only xmm0-xmm5 (the others are kept under Windows); they come
 * back with the carry set, having changed nothing, for a case that is not
 * theirs: the handler then does the step. Memory: eax the address (within
 * one line of 64 bytes), ecx or rcx the data of a store; eax or rax the
 * value loaded. Floating point: eax source 1, ecx source 2, rdx the
 * accumulator (vmac), r10d FPST's fixed bits (aux, the rounding); eax or rax
 * the result */
void
C82Mp::write_helpers()
{
	C82MpCode &c = *m_code;
	using C = C82MpCode;
	using Mem = C82MpCode::Mem;
	const Tms320c82::MpMap map = m_chip.mp_map();
	const auto at = [this](const void *member) { return HostCode::place_in(this, member); };
	const Mem FPST_AT{ C::MP, C::NO, 0, at(&m_fpst) };
	const auto sse = [&](uint8_t prefix, uint8_t opcode, uint8_t dst, uint8_t src, bool wide = false) {
		c.op_r(prefix, { 0x0F, opcode }, dst, src, wide);
	};
	/* xorps x, x before a conversion into x from another register: the
	 * conversion keeps x's high part, a false dependency on its last use */
	const auto fresh = [&](uint8_t x) { sse(0, 0x57, x, x); };

	c.writable();
	/* The way back of a case not taken */
	const uint32_t fail = c.here();
	c.bytes({ 0xF9, 0xC3 });						/* stc / ret */
	/* The way back of a case taken: ret alone when the carry is clear
	 * already (the last instruction that writes it, named at each ok(),
	 * leaves it clear), clc first otherwise */
	const auto ok = [&](bool carry_clear) {
		if (!carry_clear)
			c.byte(0xF8);						/* clc */
		c.byte(0xC3);							/* ret */
	};

	/* Memory */
	for (unsigned h = H_LOAD8S; h <= H_STORE64; h++) {
		const bool store = h >= H_STORE8;
		const unsigned size = store ? h - H_STORE8 : h <= H_LOAD8U ? 0 : h <= H_LOAD16U ? 1 : h - H_LOAD32 + 2;

		m_helper[h] = c.here();
		c.op_ri(7, C::RAX, 0x02000000);					/* cmp eax, 0x02000000 */
		const HostCode::Jump internal = c.jump_if(C::B);
		HostCode::Jump code{};
		uint32_t checked = 0;
		c.op_r(0, { 0x89 }, C::RAX, C::R10);				/* mov r10d, eax */
		c.shift_ri(5, C::R10, 20);					/* shr r10d, 20 */
		c.shift_ri(4, C::R10, 4);					/* shl r10d, 4 */
		c.op_m(0, { 0x8B }, C::RDX, Mem{ C::PAGES, C::R10, 0, 0 }, true);	/* mov rdx, [r14 + r10] (bytes) */
		c.op_r(0, { 0x85 }, C::RDX, C::RDX, true);			/* test rdx, rdx */
		c.jump_if_to(C::E, fail);
		if (store) {
			c.op_m(0, { 0x80 }, 7, Mem{ C::PAGES, C::R10, 0, map.writable });	/* cmp byte [writable], 0 */
			c.byte(0);
			c.jump_if_to(C::E, fail);
			c.op_m(0, { 0x80 }, 7, Mem{ C::PAGES, C::R10, 0, map.watched });	/* cmp byte [watched], 0 */
			c.byte(0);
			c.jump_if_to(C::NE, fail);
			/* A page where blocks read code: its line looked at below */
			c.op_m(0, { 0x80 }, 7, Mem{ C::PAGES, C::R10, 0, map.code });	/* cmp byte [code], 0 */
			c.byte(0);
			code = c.jump_if(C::NE);
			checked = c.here();
		}
		c.op_r(0, { 0x89 }, C::RAX, C::R10);				/* mov r10d, eax */
		c.op_ri(4, C::R10, 0xFFFFF);					/* and r10d, 0xFFFFF */
		c.op_r(0, { 0x01 }, C::R10, C::RDX, true);			/* add rdx, r10 */
		const HostCode::Jump plain = c.jump_ahead();
		c.bind(internal);
		/* eax is below 0x02000000 here, eax >> 12 below 0x2000 (no
		 * and) */
		c.op_r(0, { 0x89 }, C::RAX, C::R10);				/* mov r10d, eax */
		c.shift_ri(5, C::R10, 12);					/* shr r10d, 12 */
		c.op_m(0, { 0x0F, 0xBF }, C::R10, Mem{ C::PAGES, C::R10, 1, map.internal });	/* movsx r10d, word [internal] */
		c.op_r(0, { 0x85 }, C::R10, C::R10);				/* test r10d, r10d */
		c.jump_if_to(C::S, fail);
		if (store) {
			/* Not in user mode, its line not watched */
			c.op_m(0, { 0x80 }, 7, Mem{ C::MP, C::NO, 0, at(&m_user) });	/* cmp byte [user], 0 */
			c.byte(0);
			c.jump_if_to(C::NE, fail);
			c.op_r(0, { 0x89 }, C::R10, C::RDX);			/* mov edx, r10d */
			c.shift_ri(5, C::RDX, 12);				/* shr edx, 12 */
			c.op_m(0, { 0x8B }, C::RDX, Mem{ C::PAGES, C::RDX, 3, map.lines }, true);	/* mov rdx, [lines] */
			c.op_r(0, { 0x89 }, C::RAX, C::R11);			/* mov r11d, eax */
			c.shift_ri(5, C::R11, 6);				/* shr r11d, 6 */
			c.op_r(0, { 0x0F, 0xA3 }, C::R11, C::RDX, true);	/* bt rdx, r11 (its offset taken mod 64) */
			c.jump_if_to(C::B, fail);
		}
		c.op_r(0, { 0x89 }, C::RAX, C::RDX);				/* mov edx, eax */
		c.op_ri(4, C::RDX, 0xFFF);					/* and edx, 0xFFF */
		c.op_r(0, { 0x01 }, C::R10, C::RDX, true);			/* add rdx, r10 */
		c.op_m(0, { 0x8D }, C::RDX, Mem{ C::PAGES, C::RDX, 0, map.ram }, true);	/* lea rdx, [r14 + rdx + ram] */
		c.bind(plain);
		if (store) {
			write_store(c, size);
			/* The carry: from add rdx, r10 (no carry out: a host pointer, or
			 * both below 2^31, plus less than 2^20), but for rol cx, 8 */
			ok(size != 1);
			/* The line of the store read by a block: the handler (C82Mp::
			 * code_written drops the blocks) */
			c.bind(code);
			c.op_r(0, { 0x89 }, C::RAX, C::R11);			/* mov r11d, eax */
			c.shift_ri(5, C::R11, 20);				/* shr r11d, 20 */
			c.op_m(0, { 0x8B }, C::R11, Mem{ C::MP, C::R11, 3, at(m_code_lines.data()) }, true);	/* mov r11, [code_lines + 8 r11] */
			c.op_r(0, { 0x89 }, C::RAX, C::R9);			/* mov r9d, eax */
			c.shift_ri(5, C::R9, 12);				/* shr r9d, 12 */
			c.op_ri(4, C::R9, 0xFF);				/* and r9d, 0xFF */
			c.op_m(0, { 0x8B }, C::R11, Mem{ C::R11, C::R9, 3, 0 }, true);	/* mov r11, [r11 + 8 r9] */
			c.op_r(0, { 0x89 }, C::RAX, C::R9);			/* mov r9d, eax */
			c.shift_ri(5, C::R9, 6);				/* shr r9d, 6: the line, its bit */
			c.op_r(0, { 0x0F, 0xA3 }, C::R9, C::R11, true);		/* bt r11, r9 */
			c.jump_if_to(C::B, fail);
			c.jump_back(checked);
			continue;
		} else {
			switch (size) {
			case 0:
				c.op_m(0, { 0x0F, static_cast<uint8_t>(h == H_LOAD8U ? 0xB6 : 0xBE) }, C::RAX, Mem{ C::RDX });	/* movzx/movsx eax, byte [rdx] */
				break;
			case 1:
				c.op_m(0, { 0x0F, 0xB7 }, C::RAX, Mem{ C::RDX });	/* movzx eax, word [rdx] */
				c.op_r(0x66, { 0xC1 }, 0, C::RAX);		/* rol ax, 8 */
				c.byte(8);
				if (h == H_LOAD16S)
					c.op_r(0, { 0x0F, 0xBF }, C::RAX, C::RAX);	/* movsx eax, ax */
				break;
			case 2:
				c.op_m(0, { 0x8B }, C::RAX, Mem{ C::RDX });	/* mov eax, [rdx] */
				c.swap(C::RAX, false);				/* bswap eax */
				break;
			default:
				c.op_m(0, { 0x8B }, C::RAX, Mem{ C::RDX }, true);	/* mov rax, [rdx] */
				c.swap(C::RAX, true);				/* bswap rax */
				break;
			}
		}
		/* The carry: from add rdx, r10 as above, but for rol ax, 8 */
		ok(size != 1);
	}

	/* m_float_mode's bit: run, round by drm to nearest */
	const auto mode = [&](uint8_t bit) {
		c.op_m(0, { 0xF6 }, 0, Mem{ C::MP, C::NO, 0, at(&m_float_mode) });	/* test byte [float_mode], bit */
		c.byte(bit);
		c.jump_if_to(C::E, fail);
	};
	/* c82_fast::single_in(v): normal or zero */
	const auto single_in = [&](uint8_t v) {
		c.op_m(0, { 0x8D }, C::R11, Mem{ v, v, 0, 0 });			/* lea r11d, [v + v] */
		c.op_ri(0, C::R11, 0xFF000000u);					/* add r11d, -0x01000000 */
		c.op_ri(7, C::R11, 0xFE000000u);					/* cmp r11d, 0xFE000000 */
		const HostCode::Jump in = c.jump_if(C::B);
		c.op_r(0, { 0xF7 }, 0, v);						/* test v, 0x7FFFFFFF */
		c.word(0x7FFFFFFFu);
		c.jump_if_to(C::NE, fail);
		c.bind(in);
	};
	/* into = 1 when xmm a != xmm b (never NaN here), else 0 */
	const auto differ = [&](uint8_t into, uint8_t a, uint8_t b) {
		c.move32(into, 0);
		sse(0x66, 0x2E, a, b);							/* ucomisd a, b */
		c.op_r(0, { 0x0F, 0x95 }, 0, into, false, true);			/* setne into */
	};
	/* single_out(f): f's biased exponent e (eax) within 2..253, as
	 * (e << 23) - (2 << 23) below 252 << 23 (the field kept in place,
	 * the same test) */
	const auto single_out = [&]() {
		c.op_r(0, { 0x89 }, C::RAX, C::R11);					/* mov r11d, eax */
		c.op_ri(4, C::R11, 0x7F800000u);					/* and r11d, 0x7F800000 */
		c.op_ri(5, C::R11, 2u << 23);						/* sub r11d, 2 << 23 */
		c.op_ri(7, C::R11, 252u << 23);						/* cmp r11d, 252 << 23 */
	};
	/* single_out(f) | (z == 0): f's bits in eax, z a double in xmm zx */
	const auto single_out_or_zero = [&](uint8_t zx) {
		single_out();
		const HostCode::Jump out = c.jump_if(C::B);
		sse(0x66, 0x7E, zx, C::R11, true);					/* movq r11, zx */
		c.op_r(0, { 0x01 }, C::R11, C::R11, true);				/* add r11, r11 */
		c.jump_if_to(C::NE, fail);
		c.bind(out);
	};
	/* fast_done: FPST = (FPST & 0x07FE0000) | flags (ecx) | r10d | flags << 22;
	 * flags | flags << 22 as flags * 0x00400001 (the flags are below
	 * 2^5: the two copies do not meet) */
	const auto done = [&]() {
		c.op_m(0, { 0x8B }, C::RDX, FPST_AT);					/* mov edx, [fpst] */
		c.op_ri(4, C::RDX, 0x07FE0000u);					/* and edx, 0x07FE0000 */
		c.op_r(0, { 0x09 }, C::R10, C::RDX);					/* or edx, r10d */
		c.op_r(0, { 0x69 }, C::RCX, C::RCX);					/* imul ecx, ecx, 0x00400001 */
		c.word(0x00400001u);
		c.op_r(0, { 0x09 }, C::RCX, C::RDX);					/* or edx, ecx */
		c.op_m(0, { 0x89 }, C::RDX, FPST_AT);					/* mov [fpst], edx */
	};
	/* The same with no flag and drm as the rounding */
	const auto done_drm = [&]() {
		c.op_m(0, { 0x8B }, C::RDX, FPST_AT);					/* mov edx, [fpst] */
		c.op_r(0, { 0x89 }, C::RDX, C::R11);					/* mov r11d, edx */
		c.shift_ri(5, C::R11, 10);						/* shr r11d, 10 */
		c.op_ri(4, C::R11, 0x180);						/* and r11d, 0x180: drm << 7 */
		c.op_ri(4, C::RDX, 0x07FE0000u);					/* and edx, 0x07FE0000 */
		c.op_r(0, { 0x09 }, C::R11, C::RDX);					/* or edx, r11d */
		c.op_r(0, { 0x09 }, C::R10, C::RDX);					/* or edx, r10d */
		c.op_m(0, { 0x89 }, C::RDX, FPST_AT);					/* mov [fpst], edx */
	};
	/* xmm0 = eax, xmm1 = ecx as doubles */
	const auto widen = [&]() {
		sse(0x66, 0x6E, 0, C::RAX);						/* movd xmm0, eax */
		sse(0xF3, 0x5A, 0, 0);							/* cvtss2sd xmm0, xmm0 */
		sse(0x66, 0x6E, 1, C::RCX);						/* movd xmm1, ecx */
		sse(0xF3, 0x5A, 1, 1);							/* cvtss2sd xmm1, xmm1 */
	};

	/* fmpy.sss, vmpy.sss: mul_sss */
	m_helper[H_FMPY] = c.here();
	mode(2);
	single_in(C::RAX);
	single_in(C::RCX);
	widen();
	sse(0xF2, 0x59, 0, 1);								/* mulsd xmm0, xmm1: p */
	fresh(2);
	sse(0xF2, 0x5A, 2, 0);								/* cvtsd2ss xmm2, xmm0: f */
	sse(0x66, 0x7E, 2, C::RAX);							/* movd eax, xmm2 */
	single_out_or_zero(0);
	fresh(3);
	sse(0xF3, 0x5A, 3, 2);								/* cvtss2sd xmm3, xmm2 */
	differ(C::RCX, 3, 0);
	done();
	ok(true);								/* the carry: from or in done */

	/* fadd.sss (fsub: source 2 negated by the step): add_sss */
	m_helper[H_FADD] = c.here();
	mode(2);
	single_in(C::RAX);
	single_in(C::RCX);
	widen();
	sse(0x66, 0x28, 2, 0);								/* movapd xmm2, xmm0 */
	sse(0xF2, 0x58, 2, 1);								/* addsd xmm2, xmm1: s */
	fresh(3);
	sse(0xF2, 0x5A, 3, 2);								/* cvtsd2ss xmm3, xmm2: f */
	sse(0x66, 0x7E, 3, C::RAX);							/* movd eax, xmm3 */
	single_out_or_zero(2);
	/* sum_error(x, y, s): bb = s - x, (x - (s - bb)) + (y - bb) */
	sse(0x66, 0x28, 4, 2);								/* movapd xmm4, xmm2 */
	sse(0xF2, 0x5C, 4, 0);								/* subsd xmm4, xmm0: bb */
	sse(0x66, 0x28, 5, 2);								/* movapd xmm5, xmm2 */
	sse(0xF2, 0x5C, 5, 4);								/* subsd xmm5, xmm4: s - bb */
	sse(0xF2, 0x5C, 0, 5);								/* subsd xmm0, xmm5 */
	sse(0xF2, 0x5C, 1, 4);								/* subsd xmm1, xmm4 */
	sse(0xF2, 0x58, 0, 1);								/* addsd xmm0, xmm1: the error */
	sse(0x66, 0x57, 4, 4);								/* xorpd xmm4, xmm4 */
	differ(C::RCX, 0, 4);
	fresh(5);
	sse(0xF3, 0x5A, 5, 3);								/* cvtss2sd xmm5, xmm3 */
	differ(C::R9, 5, 2);
	c.op_r(0, { 0x09 }, C::R9, C::RCX);						/* or ecx, r9d */
	done();
	ok(true);								/* the carry: from or in done */

	/* fdiv.sss: div_sss */
	m_helper[H_FDIV] = c.here();
	mode(2);
	single_in(C::RAX);
	single_in(C::RCX);
	c.op_r(0, { 0xF7 }, 0, C::RCX);							/* test ecx, 0x7FFFFFFF */
	c.word(0x7FFFFFFFu);
	c.jump_if_to(C::E, fail);
	c.op_r(0, { 0x89 }, C::RAX, C::R9);						/* mov r9d, eax: a */
	widen();
	sse(0x66, 0x28, 2, 0);								/* movapd xmm2, xmm0 */
	sse(0xF2, 0x5E, 2, 1);								/* divsd xmm2, xmm1 */
	fresh(3);
	sse(0xF2, 0x5A, 3, 2);								/* cvtsd2ss xmm3, xmm2: f */
	sse(0x66, 0x7E, 3, C::RAX);							/* movd eax, xmm3 */
	{
		/* x != 0 and not single_out(f): not taken */
		c.op_r(0, { 0x01 }, C::R9, C::R9);						/* add r9d, r9d */
		const HostCode::Jump zero = c.jump_if(C::E);
		single_out();
		c.jump_if_to(C::AE, fail);
		c.bind(zero);
	}
	/* inexact when f x y != x */
	fresh(4);
	sse(0xF3, 0x5A, 4, 3);								/* cvtss2sd xmm4, xmm3 */
	sse(0xF2, 0x59, 4, 1);								/* mulsd xmm4, xmm1 */
	differ(C::RCX, 4, 0);
	done();
	ok(true);								/* the carry: from or in done */

	/* fcmp of two singles: the status from the table, drm */
	m_helper[H_FCMP] = c.here();
	mode(1);
	single_in(C::RAX);
	single_in(C::RCX);
	sse(0x66, 0x6E, 0, C::RAX);							/* movd xmm0, eax */
	sse(0x66, 0x6E, 1, C::RCX);							/* movd xmm1, ecx */
	sse(0, 0x57, 2, 2);								/* xorps xmm2, xmm2 */
	for (const uint8_t r : { C::R8, C::R9, C::R11, C::RCX, C::RDX })
		c.move32(r, 0);
	/* r8d = (x > y) + (x >= y), r9d = (x > 0) + (x >= 0), r11d = y < 0 */
	sse(0, 0x2E, 0, 1);								/* ucomiss xmm0, xmm1 */
	c.op_r(0, { 0x0F, 0x97 }, 0, C::R8, false, true);				/* seta r8b */
	c.op_r(0, { 0x0F, 0x93 }, 0, C::RCX, false, true);				/* setae cl */
	c.op_r(0, { 0x01 }, C::RCX, C::R8);						/* add r8d, ecx */
	sse(0, 0x2E, 0, 2);								/* ucomiss xmm0, xmm2 */
	c.op_r(0, { 0x0F, 0x97 }, 0, C::R9, false, true);				/* seta r9b */
	c.op_r(0, { 0x0F, 0x93 }, 0, C::RDX, false, true);				/* setae dl */
	c.op_r(0, { 0x01 }, C::RDX, C::R9);						/* add r9d, edx */
	sse(0, 0x2E, 2, 1);								/* ucomiss xmm2, xmm1 */
	c.op_r(0, { 0x0F, 0x97 }, 0, C::R11, false, true);				/* seta r11b */
	c.op_m(0, { 0x8D }, C::R8, Mem{ C::R8, C::R8, 1, 0 });			/* lea r8d, [r8 + r8 * 2] */
	c.op_r(0, { 0x01 }, C::R9, C::R8);						/* add r8d, r9d */
	c.op_r(0, { 0x01 }, C::R8, C::R8);						/* add r8d, r8d */
	c.op_r(0, { 0x01 }, C::R11, C::R8);						/* add r8d, r11d */
	c.move64(C::RCX, reinterpret_cast<uintptr_t>(compare_table().data()));
	c.op_m(0, { 0x8B }, C::RAX, Mem{ C::RCX, C::R8, 2, 0 });			/* mov eax, [rcx + r8 * 4] */
	done_drm();
	ok(true);								/* the carry: from or in done */

	/* vrnd of a single to a double: exact, no flag, drm */
	m_helper[H_VRND_SD] = c.here();
	mode(1);
	single_in(C::RAX);
	sse(0x66, 0x6E, 0, C::RAX);							/* movd xmm0, eax */
	sse(0xF3, 0x5A, 0, 0);								/* cvtss2sd xmm0, xmm0 */
	sse(0x66, 0x7E, 0, C::RAX, true);						/* movq rax, xmm0 */
	done_drm();
	ok(true);								/* the carry: from or in done */

	/* frnd (vrnd) of an integer to a single (only those 2^24 or less:
	 * exact) or a double, of a single to a double: exact, no flag; r10d holds
	 * the rounding */
	for (unsigned h = H_CVT_IS; h <= H_CVT_SD; h++) {
		m_helper[h] = c.here();
		mode(1);
		if (h == H_CVT_IS) {
			c.op_m(0, { 0x8D }, C::R11, Mem{ C::RAX, C::NO, 0, 0x01000000 });	/* lea r11d, [rax + 2^24] */
			c.op_ri(7, C::R11, 0x02000000);					/* cmp r11d, 2^25 */
			c.jump_if_to(C::A, fail);
		} else if (h == H_CVT_US) {
			c.op_ri(7, C::RAX, 0x01000000);					/* cmp eax, 2^24 */
			c.jump_if_to(C::A, fail);
		} else if (h == H_CVT_SD) {
			single_in(C::RAX);
		}
		fresh(0);
		switch (h) {
		case H_CVT_IS:
		case H_CVT_US:
			sse(0xF3, 0x2A, 0, C::RAX);					/* cvtsi2ss xmm0, eax */
			sse(0x66, 0x7E, 0, C::RAX);					/* movd eax, xmm0 */
			break;
		case H_CVT_ID:
			sse(0xF2, 0x2A, 0, C::RAX);					/* cvtsi2sd xmm0, eax */
			sse(0x66, 0x7E, 0, C::RAX, true);				/* movq rax, xmm0 */
			break;
		case H_CVT_UD:
			sse(0xF2, 0x2A, 0, C::RAX, true);				/* cvtsi2sd xmm0, rax (eax zero-extended) */
			sse(0x66, 0x7E, 0, C::RAX, true);				/* movq rax, xmm0 */
			break;
		default:
			sse(0x66, 0x6E, 0, C::RAX);					/* movd xmm0, eax */
			sse(0xF3, 0x5A, 0, 0);						/* cvtss2sd xmm0, xmm0 */
			sse(0x66, 0x7E, 0, C::RAX, true);				/* movq rax, xmm0 */
			break;
		}
		c.move32(C::RCX, 0);
		done();
		ok(true);								/* the carry: from or in done */
	}

	/* vmac, vmsc (source 1 negated by the step): c82_fast::mac, a single
	 * result toward zero, or a double */
	for (const bool single : { true, false }) {
		m_helper[single ? H_VMAC_S : H_VMAC_D] = c.here();
		mode(single ? 1 : 2);
		single_in(C::RAX);
		single_in(C::RCX);
		{
			/* double_in(rdx): 2^-969 up, finite, or zero. The biased
			 * exponent within 54..2046, else the value zero (the same test
			 * as rdx * 2 = 0, or rdx * 2 - (54 << 53) below 1993 << 53) */
			c.op_r(0, { 0x89 }, C::RDX, C::R9, true);				/* mov r9, rdx */
			c.op_r(0, { 0xC1 }, 5, C::R9, true);					/* shr r9, 52 */
			c.byte(52);
			c.op_ri(4, C::R9, 0x7FF);						/* and r9d, 0x7FF */
			c.op_ri(5, C::R9, 54);							/* sub r9d, 54 */
			c.op_ri(7, C::R9, 2047 - 54);						/* cmp r9d, 1993 */
			const HostCode::Jump in = c.jump_if(C::B);
			c.op_r(0, { 0x89 }, C::RDX, C::R9, true);				/* mov r9, rdx */
			c.op_r(0, { 0x01 }, C::R9, C::R9, true);				/* add r9, r9 */
			c.jump_if_to(C::NE, fail);
			c.bind(in);
		}
		widen();
		sse(0xF2, 0x59, 0, 1);							/* mulsd xmm0, xmm1: p */
		sse(0x66, 0x6E, 1, C::RDX, true);					/* movq xmm1, rdx: a */
		sse(0x66, 0x28, 2, 1);							/* movapd xmm2, xmm1 */
		sse(0xF2, 0x58, 2, 0);							/* addsd xmm2, xmm0: s */
		/* sum_error(a, p, s): bb = s - a, (a - (s - bb)) + (p - bb) */
		sse(0x66, 0x28, 3, 2);							/* movapd xmm3, xmm2 */
		sse(0xF2, 0x5C, 3, 1);							/* subsd xmm3, xmm1: bb */
		sse(0x66, 0x28, 4, 2);							/* movapd xmm4, xmm2 */
		sse(0xF2, 0x5C, 4, 3);							/* subsd xmm4, xmm3: s - bb */
		sse(0xF2, 0x5C, 1, 4);							/* subsd xmm1, xmm4 */
		sse(0xF2, 0x5C, 0, 3);							/* subsd xmm0, xmm3 */
		sse(0xF2, 0x58, 1, 0);							/* addsd xmm1, xmm0: e */
		sse(0x66, 0x7E, 2, C::RAX, true);					/* movq rax, xmm2: s */
		sse(0x66, 0x7E, 1, C::RDX, true);					/* movq rdx, xmm1: e */
		{
			/* in range, or zero */
			c.op_r(0, { 0x89 }, C::RAX, C::RCX, true);			/* mov rcx, rax */
			c.op_r(0, { 0xC1 }, 5, C::RCX, true);				/* shr rcx, 52 */
			c.byte(52);
			c.op_ri(4, C::RCX, 0x7FF);					/* and ecx, 0x7FF */
			c.op_ri(5, C::RCX, single ? 1023 - 125 : 2);			/* sub ecx, the lowest */
			c.op_ri(7, C::RCX, single ? 252 : 2044);			/* cmp ecx, the span */
			const HostCode::Jump in = c.jump_if(C::B);
			c.op_r(0, { 0x89 }, C::RAX, C::RCX, true);			/* mov rcx, rax */
			c.op_r(0, { 0x01 }, C::RCX, C::RCX, true);			/* add rcx, rcx */
			c.jump_if_to(C::NE, fail);
			c.bind(in);
		}
		for (const uint8_t r : { C::RCX, C::R8, C::R11 })
			c.move32(r, 0);
		/* r8d = e != 0 */
		c.op_r(0, { 0x89 }, C::RDX, C::R9, true);					/* mov r9, rdx */
		c.op_r(0, { 0x01 }, C::R9, C::R9, true);					/* add r9, r9 */
		c.op_r(0, { 0x0F, 0x95 }, 0, C::R8, false, true);				/* setne r8b */
		c.op_r(0, { 0x89 }, C::R8, C::RCX);						/* mov ecx, r8d: flags */
		if (single) {
			/* toward zero: t = s cut to a single, one step lower when t = s
			 * and the exact sum lies below it */
			c.op_r(0, { 0x89 }, C::RAX, C::R9, true);				/* mov r9, rax */
			c.op_r(0, { 0x81 }, 4, C::R9, true);					/* and r9, ~(2^29 - 1) */
			c.word(0xE0000000u);
			c.op_r(0, { 0x39 }, C::RAX, C::R9, true);				/* cmp r9, rax */
			c.op_r(0, { 0x0F, 0x95 }, 0, C::R11, false, true);			/* setne r11b: t != s */
			c.op_r(0, { 0x09 }, C::R11, C::RCX);					/* or ecx, r11d: flags */
			c.op_r(0, { 0x31 }, C::RAX, C::RDX, true);				/* xor rdx, rax */
			c.op_r(0, { 0xC1 }, 5, C::RDX, true);					/* shr rdx, 63: signs differ */
			c.byte(63);
			c.op_r(0, { 0x21 }, C::R8, C::RDX);					/* and edx, r8d */
			c.op_ri(6, C::R11, 1);							/* xor r11d, 1: t = s */
			c.op_r(0, { 0x21 }, C::R11, C::RDX);					/* and edx, r11d: below */
			c.op_r(0, { 0xC1 }, 4, C::RDX, true);					/* shl rdx, 29 */
			c.byte(29);
			c.op_r(0, { 0x29 }, C::RDX, C::R9, true);				/* sub r9, rdx */
			sse(0x66, 0x6E, 0, C::R9, true);					/* movq xmm0, r9 */
			sse(0xF2, 0x5A, 0, 0);							/* cvtsd2ss xmm0, xmm0 */
			sse(0x66, 0x7E, 0, C::RAX);						/* movd eax, xmm0 */
		}
		done();
		ok(true);								/* the carry: from or in done */
	}
	c.ready();
}

/* The store of ecx (rcx for a doubleword) at rdx, in the C82's byte order;
 * spoiled as C82Mp::store sets it */
void
C82Mp::write_store(C82MpCode &c, unsigned size)
{
	using C = C82MpCode;
	using Mem = C82MpCode::Mem;

	c.op_m(0, { 0xC6 }, 0, Mem{ C::MP, C::NO, 0, HostCode::place_in(this, &m_park.spoiled) });	/* mov byte [spoiled], 1 */
	c.byte(1);
	switch (size) {
	case 0:
		c.op_m(0, { 0x88 }, C::RCX, Mem{ C::RDX }, false, true);	/* mov [rdx], cl */
		break;
	case 1:
		c.op_r(0x66, { 0xC1 }, 0, C::RCX);				/* rol cx, 8 */
		c.byte(8);
		c.op_m(0x66, { 0x89 }, C::RCX, Mem{ C::RDX });			/* mov [rdx], cx */
		break;
	case 2:
		c.swap(C::RCX, false);						/* bswap ecx */
		c.op_m(0, { 0x89 }, C::RCX, Mem{ C::RDX });			/* mov [rdx], ecx */
		break;
	default:
		c.swap(C::RCX, true);						/* bswap rcx */
		c.op_m(0, { 0x89 }, C::RCX, Mem{ C::RDX }, true);		/* mov [rdx], rcx */
		break;
	}
}

/* A step's call of a routine, out to the handler when it does not take the
 * case */
void
C82Mp::call_helper(C82MpCode &c, unsigned h, std::vector<HostCode::Jump> &slow)
{
	c.byte(0xE8);								/* call the routine */
	c.jump_to(m_helper[h]);
	slow.push_back(c.jump_if(C82MpCode::B));
}

/* ld, st (C82MpOps::load, store): the address and :m here, the access by the
 * routines; the board, a register of the TC, a fault, a store the user mode
 * forbids or that a waiting processor watches: the handler does the step */
bool
C82Mp::write_memory(C82MpCode &c, const std::vector<BlockStep> &steps, unsigned k, std::vector<C82MpSlow> &slow)
{
	using C = C82MpCode;
	using Mem = C82MpCode::Mem;
	const BlockStep &st = steps[k];
	const C82MpOp &op = st.op;
	const C82Insn &in = st.in;
	const auto reg = [&](unsigned n) { return Mem{ C::MP, C::NO, 0, HostCode::place_in(this, &m_r[n & 31]) }; };
	const unsigned size = in.size;
	const bool store = in.op == C82Insn::STORE;
	C82MpSlow s;

	if (m_chip.mp_map().page != 16)
		return false;
	s.k = k;
	/* eax = the address, r8d the same for :m; then aligned */
	c.op_m(0, { 0x8B }, C::RAX, reg(op.src2));				/* mov eax, [r(src2)] */
	if (in.form == C82Insn::REGISTER) {
		c.op_m(0, { 0x8B }, C::RCX, reg(op.src1));			/* mov ecx, [r(src1)] */
		if (in.scale && size)
			c.shift_ri(4, C::RCX, static_cast<uint8_t>(size));	/* shl ecx, size */
		c.op_r(0, { 0x01 }, C::RCX, C::RAX);				/* add eax, ecx */
	} else {
		const uint32_t offset = in.scale ? op.imm << size : op.imm;

		if (offset)
			c.op_ri(0, C::RAX, offset);				/* add eax, offset */
	}
	if (in.modify)
		c.op_r(0, { 0x89 }, C::RAX, C::R8);				/* mov r8d, eax */
	if (size)
		c.op_ri(4, C::RAX, ~((1u << size) - 1));			/* and eax, ~(bytes - 1) */
	/* A store's data before :m (the old base when it is the source) */
	if (store)
		write_data(c, op.dest, size);
	unsigned helper = H_LOAD32 + size - 2;

	if (store)
		helper = H_STORE8 + size;
	else if (size == 0)
		helper = in.is_unsigned ? H_LOAD8U : H_LOAD8S;
	else if (size == 1)
		helper = in.is_unsigned ? H_LOAD16U : H_LOAD16S;
	call_helper(c, helper, s.from);
	if (in.modify && op.src2)
		c.op_m(0, { 0x89 }, C::R8, reg(op.src2));			/* mov [r(src2)], r8d */
	if (!store)
		write_loaded(c, size, op.dest);
	s.back = c.here();
	slow.push_back(std::move(s));
	return true;
}

/* ecx = the data of a store from reg (rcx = its pair for a doubleword: the
 * odd register high) */
void
C82Mp::write_data(C82MpCode &c, unsigned reg_number, unsigned size)
{
	using C = C82MpCode;
	using Mem = C82MpCode::Mem;
	const auto reg = [&](unsigned n) { return Mem{ C::MP, C::NO, 0, HostCode::place_in(this, &m_r[n & 31]) }; };

	if (size == 3) {
		c.op_m(0, { 0x8B }, C::RCX, reg(reg_number | 1u));		/* mov ecx, [r(odd)] */
		c.op_r(0, { 0xC1 }, 4, C::RCX, true);				/* shl rcx, 32 */
		c.byte(32);
		c.op_m(0, { 0x8B }, C::R9, reg(reg_number & ~1u));		/* mov r9d, [r(even)] */
		c.op_r(0, { 0x09 }, C::R9, C::RCX, true);			/* or rcx, r9 */
	} else {
		c.op_m(0, { 0x8B }, C::RCX, reg(reg_number));			/* mov ecx, [r(reg)] */
	}
}

/* eax (rax for a doubleword) into dest: the pair for a doubleword, none
 * into r0 */
void
C82Mp::write_loaded(C82MpCode &c, unsigned size, unsigned dest)
{
	using C = C82MpCode;
	using Mem = C82MpCode::Mem;
	const auto reg = [this](unsigned n) { return Mem{ C::MP, C::NO, 0, HostCode::place_in(this, &m_r[n & 31]) }; };

	if (size < 3) {
		if (dest)
			c.op_m(0, { 0x89 }, C::RAX, reg(dest));			/* mov [r(dest)], eax */
	} else if (dest) {
		if (dest & ~1u)
			c.op_m(0, { 0x89 }, C::RAX, reg(dest & ~1u));		/* mov [r(even)], eax */
		c.op_r(0, { 0xC1 }, 5, C::RAX, true);				/* shr rax, 32 */
		c.byte(32);
		c.op_m(0, { 0x89 }, C::RAX, reg(dest | 1u));			/* mov [r(odd)], eax */
	}
}

/* The floating point (C82MpOps::float_sss, fcmp, vector): the operands to
 * the routine, its result to its place; the parallel vld or vst */
bool
C82Mp::write_float(C82MpCode &c, const std::vector<BlockStep> &steps, unsigned k, std::vector<C82MpSlow> &slow)
{
	using C = C82MpCode;
	using Mem = C82MpCode::Mem;
	const BlockStep &st = steps[k];
	const C82MpOp &op = st.op;
	const C82Insn &in = st.in;
	const auto at = [this](const void *member) { return HostCode::place_in(this, member); };
	const auto reg = [&](unsigned n) { return Mem{ C::MP, C::NO, 0, at(&m_r[n & 31]) }; };
	const bool is_long = in.form == C82Insn::LONG;
	C82MpSlow s;

	s.k = k;
	/* eax = source 1 (the immediate, or r(src1)); ecx = r(src2) */
	const auto sources = [&](bool two) {
		if (is_long)
			c.move32(C::RAX, op.imm);
		else
			c.op_m(0, { 0x8B }, C::RAX, reg(op.src1));			/* mov eax, [r(src1)] */
		if (two)
			c.op_m(0, { 0x8B }, C::RCX, reg(op.src2));			/* mov ecx, [r(src2)] */
	};
	const auto fixed = [&](uint32_t round) { c.move32(C::R10, op.aux | round << 7); };

	switch (in.op) {
	case C82Insn::FADD:
	case C82Insn::FSUB:
	case C82Insn::FMPY:
	case C82Insn::FDIV: {
		/* C82MpOps::float_sss: fadd, fsub, fmpy, fdiv .sss */
		const uint8_t t1 = is_long && in.p1 == FP_DOUBLE ? uint8_t{ FP_SINGLE } : in.p1;

		if (t1 != FP_SINGLE || in.p2 != FP_SINGLE || in.pd != FP_SINGLE)
			return false;
		sources(true);
		if (in.op == C82Insn::FSUB)
			c.op_ri(6, C::RCX, 0x80000000u);				/* xor ecx, 0x80000000: -b */
		fixed(0);
		call_helper(c, in.op == C82Insn::FMPY ? H_FMPY : in.op == C82Insn::FDIV ? H_FDIV : H_FADD, s.from);
		if (op.dest)
			c.op_m(0, { 0x89 }, C::RAX, reg(op.dest));			/* mov [r(dest)], eax */
		break;
	}
	case C82Insn::FRND: {
		/* C82MpOps::frnd: from an integer to a single or a double, from a
		 * single to a double (op.a: P1 as read, op.b: PD, op.c: RM) */
		const uint8_t t1 = op.a, td = op.b;
		unsigned helper = HELPERS;

		if (td == FP_SINGLE)
			helper = t1 == FP_INT ? H_CVT_IS : t1 == FP_UINT ? H_CVT_US : HELPERS;
		else if (td == FP_DOUBLE)
			helper = t1 == FP_INT ? H_CVT_ID : t1 == FP_UINT ? H_CVT_UD : t1 == FP_SINGLE ? H_CVT_SD : HELPERS;
		if (helper == HELPERS)
			return false;
		sources(false);
		fixed(op.c);
		call_helper(c, helper, s.from);
		if (td == FP_SINGLE) {
			if (op.dest)
				c.op_m(0, { 0x89 }, C::RAX, reg(op.dest));		/* mov [r(dest)], eax */
		} else if (op.dest & ~1u) {
			c.op_m(0, { 0x89 }, C::RAX, reg(op.dest & ~1u));		/* mov [r(even)], eax */
			c.op_r(0, { 0xC1 }, 5, C::RAX, true);				/* shr rax, 32 */
			c.byte(32);
			c.op_m(0, { 0x89 }, C::RAX, reg(op.dest | 1u));		/* mov [r(odd)], eax */
		}
		break;
	}
	case C82Insn::FCMP:
		/* C82MpOps::fcmp of two singles */
		if (op.a != FP_SINGLE || op.b != FP_SINGLE)
			return false;
		sources(true);
		fixed(0);
		call_helper(c, H_FCMP, s.from);
		if (op.dest)
			c.op_m(0, { 0x89 }, C::RAX, reg(op.dest));			/* mov [r(dest)], eax */
		break;
	case C82Insn::VECTOR: {
		/* C82MpOps::vector: the kinds choose() gives it */
		unsigned kind = 0;

		if (in.vop == 2 && in.p1 == FP_SINGLE && in.pd == FP_SINGLE)
			kind = 0;
		else if (in.vop == 4 || in.vop == 5)
			kind = 1;
		else if ((in.vop == 6 || in.vop == 7) && in.p1 == FP_SINGLE)
			kind = in.vop == 6 ? 2 : 3;
		else
			return false;

		const bool to_accumulator = op.a & 1, zero = op.a & 4;
		const uint8_t p1 = (op.a >> 3) & 3, td = (op.a >> 5) & 3;
		const bool double_result = kind >= 2 && td == FP_DOUBLE;
		const Mem ACC{ C::MP, C::NO, 0, at(&m_acc[op.b & 3]) };

		if (kind == 1 && (p1 != FP_SINGLE || td != FP_DOUBLE))
			return false;
		sources(kind != 1);
		if (kind == 0) {
			fixed(0);
			call_helper(c, H_FMPY, s.from);
		} else if (kind == 1) {
			fixed(0);
			call_helper(c, H_VRND_SD, s.from);
		} else {
			if (zero)
				c.move32(C::RDX, 0);
			else
				c.op_m(0, { 0x8B }, C::RDX, ACC, true);			/* mov rdx, [acc] */
			if (kind == 3)
				c.op_ri(6, C::RAX, 0x80000000u);			/* xor eax, 0x80000000: -p */
			fixed(double_result ? 0 : ROUND_ZERO);
			call_helper(c, double_result ? H_VMAC_D : H_VMAC_S, s.from);
		}
		/* The result: a register, an accumulator, a pair */
		if (kind == 0 || (kind >= 2 && !double_result)) {
			if (op.dest)
				c.op_m(0, { 0x89 }, C::RAX, reg(op.dest));		/* mov [r(dest)], eax */
		} else if (to_accumulator) {
			c.op_m(0, { 0x89 }, C::RAX, ACC, true);				/* mov [acc], rax */
		} else if (op.dest & ~1u) {
			c.op_m(0, { 0x89 }, C::RAX, reg(op.dest & ~1u));		/* mov [r(even)], eax */
			c.op_r(0, { 0xC1 }, 5, C::RAX, true);				/* shr rax, 32 */
			c.byte(32);
			c.op_m(0, { 0x89 }, C::RAX, reg(op.dest | 1u));		/* mov [r(odd)], eax */
		}
		if (in.mem) {
			/* The parallel vld or vst (C82MpOps::vector_memory): aligned on
			 * its size, else the handler of that part */
			const unsigned size = op.d ? 3 : 2;
			const Mem POINTER{ C::MP, C::NO, 0, in.mem == 1 ? at(&m_outp) : in.mem == 2 ? at(&m_in0p) : at(&m_in1p) };
			C82MpSlow v;

			v.kind = C82MpSlow::VECTOR_MEMORY;
			v.k = k;
			c.op_m(0, { 0x8B }, C::RAX, POINTER);				/* mov eax, [pointer] */
			c.op_r(0, { 0xF7 }, 0, C::RAX);					/* test eax, bytes - 1 */
			c.word((1u << size) - 1);
			v.from.push_back(c.jump_if(C::NE));
			if (in.mem == 1)
				write_data(c, op.c, size);
			call_helper(c, in.mem == 1 ? H_STORE8 + size : H_LOAD32 + size - 2, v.from);
			if (in.mem != 1)
				write_loaded(c, size, op.c);
			c.op_m(0, { 0x83 }, 0, POINTER);				/* add dword [pointer], bytes */
			c.byte(static_cast<uint8_t>(1u << size));
			v.back = c.here();
			slow.push_back(std::move(v));
		}
		break;
	}
	default:
		return false;
	}
	s.back = c.here();
	slow.push_back(std::move(s));
	return true;
}

#else

void
C82Mp::write_branch(C82MpCode &, const BlockStep &)
{
}

bool
C82Mp::write_body(C82MpCode &, const std::vector<BlockStep> &, unsigned, std::vector<C82MpSlow> &)
{
	return false;
}

void
C82Mp::write_helpers()
{
}

#endif

} /* namespace odin */
