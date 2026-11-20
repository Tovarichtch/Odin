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

/* The instructions of the master processor (TI SPRU109 § 10.12, each
 * instruction's page cited where its rule is) */

#include "odin/machine/cpu/tms320c82/c82_mp.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdio>
#include <utility>

#include "odin/common/repeats.hpp"
#include "odin/machine/cpu/tms320c82/tms320c82.hpp"

namespace odin {

namespace {

/* FPST bits (fig. 3-3) */
enum : uint32_t { FPST_FM = 1u << 19, FPST_FS = 1u << 20, FPST_SM = 1u << 21 };

/* 2^code - 1, all ones for 32 (tables 10-3, 10-24) */
uint32_t
mask_of(unsigned code)
{
	return code >= 32 ? ~0u : (1u << code) - 1;
}

/* The ten conditions of cmp on a byte, halfword or word (fig. 10-2, table
 * 10-14): eq ne gt le lt ge hi ls lo hs from bit 0 */
uint32_t
compare_bits(uint32_t a, uint32_t b, unsigned bits)
{
	const uint32_t m = bits == 32 ? ~0u : (1u << bits) - 1, sign = 1u << (bits - 1);
	const uint32_t x = a & m, y = b & m, d = (x - y) & m;
	const bool z = d == 0, n = d & sign, c = x >= y, v = ((x ^ y) & (x ^ d)) & sign;
	const bool lt = n != v;

	return (z ? 1u : 0u) | (!z ? 2u : 0u) | (!z && !lt ? 4u : 0u) | (lt || z ? 8u : 0u) | (lt ? 16u : 0u) |
	       (!lt ? 32u : 0u) | (c && !z ? 64u : 0u) | (!c || z ? 128u : 0u) | (!c ? 256u : 0u) | (c ? 512u : 0u);
}

} /* namespace */

void
C82Mp::execute(const C82Insn &in, uint32_t word, uint32_t ip, uint32_t next)
{
	/* source1: a register, or the immediate (short or long) */
	const uint32_t source1 = in.form == C82Insn::REGISTER ? m_r[in.src1] : in.imm;

	switch (in.op) {
	case C82Insn::RESERVED:
		/* HEURISTIC: a reserved opcode is taken for an illegal one */
		repeats::print(stderr, "mp: reserved instruction 0x%08X at 0x%08X: illegal instruction trap\n", word, ip);
		exception(TRAP_ER, ip, next);
		return;
	case C82Insn::ILLOP:
		/* trap 38 (p. 10-106) */
		exception(TRAP_ER, ip, next);
		return;
	case C82Insn::TRAP:
		/* No delay slot; EIP the trap, EPC the next instruction
		 * (§ 9.2.2, p. 10-155). etrap: MIP and MPC (§ 3.1.3); HEURISTIC:
		 * otherwise as a trap */
		if (in.emulation) {
			m_mip = ip;
			m_mpc = next;
		}
		exception(source1, ip, next);
		return;
	case C82Insn::CMND:
		m_park.spoiled = true;
		/* MP bits: the message comes after one delay slot (p. 10-71),
		 * ICR and DCR reset the tags and LRU (note 2) */
		if (source1 & (1u << 8)) {
			if (source1 & (1u << 13)) {
				m_self_message = 2;
				m_check = true;
			}
			if (source1 & (1u << 28)) {
				m_itag.fill(0);
				m_ilru = 0;
			}
			if (source1 & (1u << 27)) {
				m_dtag.fill(0);
				m_dlru = 0;
			}
			if (source1 & (1u << 31))
				repeats::print(stderr, "mp: cmnd 0x%08X: the MP's own reset is not emulated yet\n", source1);
		}
		m_chip.command(source1);
		return;
	case C82Insn::RDCR:
		set(in.dest, read_control(source1));
		return;
	case C82Insn::SWCR: {
		/* wrcr is swcr into r0; user mode, below 0x4000: a nop (p. 10-153) */
		if (m_user && source1 < 0x4000)
			return;

		const uint32_t old = read_control(source1);

		write_control(source1, m_r[in.src2]);
		set(in.dest, old);
		return;
	}
	case C82Insn::BRCR: {
		/* bit 0 to ie, bit 1 to the user mode, the rest to PC, with a
		 * delay slot (p. 10-66) */
		if (m_user && source1 < 0x4000)
			return;

		const uint32_t v = read_control(source1);

		m_ie = (m_ie & ~INT_IE) | (v & 1);
		m_check = true;
		m_user = (v >> 1) & 1;
		exec_branch(in, true, v & ~3u);
		return;
	}
	case C82Insn::SHIFT:
		exec_shift(in);
		return;
	case C82Insn::LOGIC: {
		/* The truth table of bits 18-15: bit 0 a&b, 1 a&~b, 2 ~a&b, 3 ~a&~b
		 * (table 10-8; and.tf p. 10-54, or.tf p. 10-125) */
		const uint32_t a = source1, b = m_r[in.src2];
		uint32_t r = 0;

		if (in.logic & 1)
			r |= a & b;
		if (in.logic & 2)
			r |= a & ~b;
		if (in.logic & 4)
			r |= ~a & b;
		if (in.logic & 8)
			r |= ~a & ~b;
		set(in.dest, r);
		return;
	}
	case C82Insn::LOAD:
	case C82Insn::STORE:
	case C82Insn::DCACHE:
		exec_memory(in);
		return;
	case C82Insn::BSR:
	case C82Insn::JSR: {
		/* The link: the second instruction after the branch, or the first
		 * when annulled; the operands read before it is written
		 * (p. 10-68, 10-111) */
		const uint32_t link = ip + in.length + (in.annul ? 0 : 4);
		const uint32_t target = in.op == C82Insn::BSR ? ip + 4 * source1 : (source1 + m_r[in.src2]) & ~3u;

		set(in.dest, link);
		exec_branch(in, true, target);
		return;
	}
	case C82Insn::BBZ:
	case C82Insn::BBO: {
		/* BITNUM is the ones complement of the bit (p. 10-55) */
		const bool bit = (m_r[in.src2] >> (~in.dest & 31)) & 1;

		exec_branch(in, in.op == C82Insn::BBO ? bit : !bit, ip + 4 * source1);
		return;
	}
	case C82Insn::BCND: {
		/* Condition in bits 29-27, size in bits 31-30 (p. 10-62).
		 * HEURISTIC: the reserved size .d tests the word */
		const uint32_t v = m_r[in.src2];
		const unsigned size = in.dest >> 3;
		const int32_t x = size == 0 ? static_cast<int8_t>(v) : size == 1 ? static_cast<int16_t>(v) : static_cast<int32_t>(v);
		bool taken = false;

		switch (in.dest & 7) {
		case 0:
			taken = false;
			break;
		case 1:
			taken = x > 0;
			break;
		case 2:
			taken = x == 0;
			break;
		case 3:
			taken = x >= 0;
			break;
		case 4:
			taken = x < 0;
			break;
		case 5:
			taken = x != 0;
			break;
		case 6:
			taken = x <= 0;
			break;
		default:
			taken = true;
			break;
		}
		exec_branch(in, taken, ip + 4 * source1);
		return;
	}
	case C82Insn::CMP: {
		/* status(source1 - source2), never an overflow (p. 10-72) */
		const uint32_t a = source1, b = m_r[in.src2];

		set(in.dest, compare_bits(a, b, 8) | compare_bits(a, b, 16) << 10 | compare_bits(a, b, 32) << 20);
		return;
	}
	case C82Insn::ADD:
	case C82Insn::SUB: {
		/* sub: source1 - source2 (p. 10-151); the signed forms post an
		 * integer overflow, io (p. 10-49) */
		const uint32_t a = source1, b = m_r[in.src2];
		const uint32_t r = in.op == C82Insn::ADD ? a + b : a - b;
		const bool overflow = in.op == C82Insn::ADD ? (~(a ^ b) & (a ^ r)) >> 31 : ((a ^ b) & (a ^ r)) >> 31;

		if (overflow && !in.is_unsigned) {
			m_intpen |= INT_IO;
			m_check = true;
		}
		set(in.dest, r);
		return;
	}
	case C82Insn::LMO: {
		/* 31 - the bit of the leftmost one, 32 for none (p. 10-119) */
		const uint32_t v = m_r[in.src2];

		set(in.dest, v ? static_cast<uint32_t>(std::countl_zero(v)) : 32);
		return;
	}
	case C82Insn::RMO: {
		const uint32_t v = m_r[in.src2];

		set(in.dest, v ? 31 - static_cast<uint32_t>(std::countr_zero(v)) : 32);
		return;
	}
	case C82Insn::ESTOP:
		/* Reserved for emulation (p. 10-85). HEURISTIC: a nop */
		repeats::print(stderr, "mp: estop at 0x%08X: no emulator\n", ip);
		return;
	case C82Insn::VECTOR:
		exec_vector(in, word);
		return;
	case C82Insn::FADD:
	case C82Insn::FSUB:
	case C82Insn::FMPY:
	case C82Insn::FDIV:
	case C82Insn::FRND:
	case C82Insn::FCMP:
	case C82Insn::FSQRT:
		exec_float(in, word);
		return;
	}
}

/* Not annulled: the next instruction (the delay slot) runs, then the
 * target. Annulled: a nop in its place, one cycle, taken or not; then the
 * target, or the instruction after (§ 4.1.2.1, 10.4.4) */
void
C82Mp::exec_branch(const C82Insn &in, bool taken, uint32_t target)
{
	if (in.annul) {
		m_cycles++;
		if (taken) {
			m_ip = target;
			m_pc = target + 4;
		}
		return;
	}
	if (taken)
		m_pc = target;
	m_slot_next = true;
}

bool
C82Mp::load(uint32_t address, unsigned size, uint64_t &value)
{
	if (m_park.measuring)
		note_read(address, size);
	return m_chip.mp_load(address, size, value);
}

bool
C82Mp::store(uint32_t address, unsigned size, uint64_t value)
{
	m_park.spoiled = true;
	return m_chip.mp_store(address, size, value, m_user);
}

/* FLTOP: the destination, kind, size, x (an access the TC refused), R in
 * a delay slot or with a long immediate; FLTADR; the data of a store,
 * repeated over 64 bits (§ 3.8). The fault is taken after the instruction
 * (§ 5.6.2) */
void
C82Mp::memory_fault(uint32_t address, unsigned size, uint8_t kind, uint8_t dest, uint64_t data)
{
	const bool modified_return = m_in_slot || m_now_long;

	m_fltop = (static_cast<uint32_t>(dest) << 27) | (static_cast<uint32_t>(kind) << 16) | (size << 13) | (1u << 10) |
	          (modified_return ? 1u << 9 : 0);
	m_fltadr = address;
	if (size < 3)
		for (unsigned i = 1u << size; i < 8; i <<= 1)
			data |= data << (8 * i);
	m_fltdth = static_cast<uint32_t>(data >> 32);
	m_fltdtl = static_cast<uint32_t>(data);
	m_fault.on = true;
	m_check = true;
}

/* ld, ld.u, st, dld, dst, dcache (p. 10-74 to 10-84, 10-113 to 10-118,
 * 10-148). The address: base + offset, the offset scaled by the size with
 * :s; :m writes it back to the base as the access starts; the low bits
 * ignored for the size. A store whose source is the base stores the old
 * value; a load into the base wins over :m. dld and dst bypass the data
 * cache, which is not emulated: the same as ld and st here. */
void
C82Mp::exec_memory(const C82Insn &in)
{
	uint32_t offset = in.form == C82Insn::REGISTER ? m_r[in.src1] : in.imm;

	if (in.scale)
		offset <<= in.size;

	const uint32_t address = m_r[in.src2] + offset;
	const uint32_t aligned = address & ~((1u << in.size) - 1);
	/* A doubleword: the even register is the low word, the odd one the
	 * high word, whatever the endianness (p. 10-114 note 2) */
	const unsigned even = in.dest & ~1u, odd = in.dest | 1u;
	uint64_t data = 0;

	if (in.op == C82Insn::STORE)
		data = in.size == 3 ? (static_cast<uint64_t>(m_r[odd]) << 32) | m_r[even] : m_r[in.dest];
	if (in.modify)
		set(in.src2, address);
	if (in.op == C82Insn::DCACHE)
		return;
	if (in.op == C82Insn::STORE) {
		if (!store(aligned, in.size, data))
			memory_fault(aligned, in.size, 2, in.dest, data);
		return;
	}
	if (!load(aligned, in.size, data)) {
		memory_fault(aligned, in.size, in.is_unsigned ? 1 : 0, in.dest, 0);
		return;
	}
	switch (in.size) {
	case 0:
		set(in.dest, in.is_unsigned ? static_cast<uint8_t>(data) : static_cast<uint32_t>(static_cast<int8_t>(data)));
		break;
	case 1:
		set(in.dest,
		    in.is_unsigned ? static_cast<uint16_t>(data) : static_cast<uint32_t>(static_cast<int16_t>(data)));
		break;
	case 2:
		set(in.dest, static_cast<uint32_t>(data));
		break;
	default:
		/* Into r0: discarded (§ 10.9.2). HEURISTIC: an odd register
		 * (illegal) loads its pair */
		if (in.dest) {
			set(even, static_cast<uint32_t>(data));
			set(odd, static_cast<uint32_t>(data >> 32));
		}
		break;
	}
}

/* sl, sli, sr, sri (p. 10-135 to 10-146; § 10.8.1, tables 10-24, 10-25):
 * a rotate, then a merge through CM = shiftmask & endmask */
void
C82Mp::exec_shift(const C82Insn &in)
{
	const uint32_t source = m_r[in.src2];
	const bool short_form = in.form == C82Insn::SHORT;
	const unsigned rotate = short_form ? in.rotate : (m_r[in.src1] & 31);
	/* The register form takes the endmask code from the odd register
	 * after the rotate's, unless the field says 0 (32): all ones.
	 * HEURISTIC: an odd rotate register uses itself */
	const unsigned end_code = short_form ? in.endmask : in.endmask ? (m_r[in.src1 | 1u] & 31) : 0;
	uint32_t endmask = end_code ? mask_of(end_code) : ~0u;

	if (in.invert)
		endmask = ~endmask;

	const uint32_t rotated = in.right ? std::rotr(source, static_cast<int>(rotate)) : std::rotl(source, static_cast<int>(rotate));
	/* A right shift makes its shiftmask from the negated amount */
	const unsigned code = in.right ? (32 - rotate) & 31 : rotate;
	uint32_t shiftmask = ~0u;

	if (in.merge >= 6)
		shiftmask = code ? ~mask_of(code) : ~0u;
	else if (in.merge >= 3)
		shiftmask = code ? mask_of(code) : in.merge == 4 ? 0 : ~0u;

	const uint32_t cm = shiftmask & endmask;
	/* Outside CM: the old bit (merge), the bit below (sign), 0 (zero) */
	static constexpr uint8_t MERGE[8] = { 0, 1, 2, 0, 1, 2, 0, 1 };
	uint32_t result = rotated & cm;

	switch (MERGE[in.merge]) {
	case 1:
		result |= m_r[in.dest] & ~cm;
		break;
	case 2:
		if ((cm & (cm + 1)) == 0) {
			/* CM is a low mask: extend its top bit */
			if (cm && cm != ~0u && ((result >> (31 - std::countl_zero(cm))) & 1))
				result |= ~cm;
		} else {
			uint32_t below = 0;

			for (unsigned b = 0; b < 32; b++) {
				if (!((cm >> b) & 1))
					result = (result & ~(1u << b)) | (below << b);
				below = (result >> b) & 1;
			}
		}
		break;
	default:
		break;
	}
	set(in.dest, result);
}

/* A floating-point or vector instruction needs ie = 1 and fs = 0, else
 * trap 36 (§ 8.4.3) */
bool
C82Mp::float_allowed()
{
	if ((m_ie & INT_IE) && !(m_fpst & FPST_FS))
		return true;
	exception(TRAP_FE, m_now_ip, m_now_next);
	return false;
}

C82FpEnv
C82Mp::float_env(uint8_t rounding) const
{
	C82FpEnv env;

	env.round = static_cast<C82FpRound>(rounding & 3);
	/* fm = 0: fast mode, denormals are zero (§ 3.3.3.2, 8.5.1). The C82
	 * names bit 19 "vm, vector fast mode" (SPRS048 fig. 13) without more:
	 * HEURISTIC, SPRU109's rule for every instruction is kept */
	env.fast = !(m_fpst & FPST_FM);
	env.trap_o = m_ie & INT_FO;
	env.trap_u = m_ie & INT_FU;
	env.trap_x = m_ie & INT_FX;
	return env;
}

/* FPST written once by each instruction (§ 3.3.3.1): the status, the
 * accumulated status, mo, the rounding, the destination precision, e0
 * and e1, the opcode (bits 15-13 and bit 17 inverted), the destination.
 * An exception whose interrupt is on posts it in INTPEN and stalls the
 * unit (fs). In sequential mode its interrupt is taken now, EIP pointing
 * at the instruction, and an input exception does not write its
 * register (§ 8.5.2, C.1.1). Returns whether the result is written. */
bool
C82Mp::float_done(const C82FpEnv &env, uint32_t word, uint8_t dest, uint8_t pd)
{
	const uint32_t flags = env.flags & 31;
	const uint32_t opcode = ((word >> 13) & 7) | ((~word >> 17) & 1) << 3;

	m_fpst = (m_fpst & 0x07FE0000u) | flags | (env.mo ? 1u << 5 : 0) | (static_cast<uint32_t>(env.round) << 7) |
	         (static_cast<uint32_t>(pd & 3) << 9) | (static_cast<uint32_t>(env.e01) << 11) | (opcode << 13) |
	         (static_cast<uint32_t>(dest) << 27) | (flags << 22);

	static constexpr uint32_t TO_INTERRUPT[5] = { INT_FX, INT_FU, INT_FO, INT_FZ, INT_FI };
	uint32_t raised = 0;

	for (unsigned i = 0; i < 5; i++)
		if (flags & (1u << i))
			raised |= TO_INTERRUPT[i];
	raised &= m_ie;
	if (!raised)
		return true;
	m_intpen |= raised;
	m_check = true;
	m_fpst |= FPST_FS;

	const bool sequential = m_fpst & FPST_SM;
	const bool input = (raised & INT_FZ) || ((raised & INT_FI) && !env.output_invalid);

	if (sequential) {
		exception(static_cast<unsigned>(std::countr_zero(raised)), m_now_ip, m_now_next);
		return !input;
	}
	return true;
}

/* A register as an operand: r0 reads 0, a double from r0 reads r1 above 0
 * (§ 10.9.2) */
uint64_t
C82Mp::read_float(unsigned reg, uint8_t type) const
{
	if (type == FP_DOUBLE)
		return (static_cast<uint64_t>(m_r[reg | 1u]) << 32) | m_r[reg & ~1u];
	return m_r[reg];
}

/* A double into r0 changes nothing, r1 included (§ 10.9.2). HEURISTIC:
 * the illegal r1 destination is written as any register */
void
C82Mp::write_float(unsigned reg, uint8_t type, uint64_t value)
{
	if (type != FP_DOUBLE) {
		set(reg, static_cast<uint32_t>(value));
		return;
	}
	if (reg & ~1u) {
		m_r[reg & ~1u] = static_cast<uint32_t>(value);
		m_r[reg | 1u] = static_cast<uint32_t>(value >> 32);
	}
}

/* fadd, fsub, fmpy, fdiv, frnd, fcmp, fsqrt (p. 10-91 to 10-105). A long
 * immediate is a single, or an integer for an integer P1 */
void
C82Mp::exec_float(const C82Insn &in, uint32_t word)
{
	if (!float_allowed())
		return;

	const bool immediate = in.form == C82Insn::LONG;
	const C82FpType t1 = static_cast<C82FpType>(immediate && in.p1 == FP_DOUBLE ? uint8_t{ FP_SINGLE } : in.p1);
	const uint64_t a = immediate ? in.imm : read_float(in.src1, t1);
	const C82FpType t2 = static_cast<C82FpType>(in.p2), td = static_cast<C82FpType>(in.pd);
	const uint64_t b = read_float(in.src2, t2);
	/* drm, or frnd's own mode (p. 10-99) */
	C82FpEnv env = float_env(in.op == C82Insn::FRND ? in.rm : static_cast<uint8_t>((m_fpst >> 17) & 3));
	uint64_t result = 0;
	C82FpType written = td;

	switch (in.op) {
	case C82Insn::FADD:
	case C82Insn::FSUB:
		result = c82_fp_add(a, t1, b, t2, in.op == C82Insn::FSUB, td, env);
		break;
	case C82Insn::FMPY:
		result = c82_fp_mul(a, t1, b, t2, td, env);
		break;
	case C82Insn::FDIV:
		result = c82_fp_div(a, t1, b, t2, td, env);
		break;
	case C82Insn::FRND:
		result = c82_fp_convert(a, t1, td, env);
		break;
	case C82Insn::FCMP:
		result = c82_fp_compare(a, t1, b, t2, env);
		written = FP_INT;
		break;
	default:
		result = c82_fp_sqrt(a, t1, td, env);
		break;
	}
	if (float_done(env, word, in.dest, in.pd))
		write_float(in.dest, written, result);
}

/* The vector instructions and their parallel vld or vst (p. 10-157 to
 * 10-178; § 10.3, 10.3.4-10.3.6) */
void
C82Mp::exec_vector(const C82Insn &in, uint32_t word)
{
	if (!float_allowed())
		return;

	const bool immediate = in.form == C82Insn::LONG;
	const uint64_t drm = (m_fpst >> 17) & 3;
	C82FpEnv env = float_env(static_cast<uint8_t>(drm));
	const C82FpType t1 = static_cast<C82FpType>(in.p1);
	const uint64_t source = immediate ? in.imm : read_float(in.src1, t1);
	uint64_t result = 0;
	/* Where it goes: a register and its type, or an accumulator */
	unsigned dest = in.src2;
	C82FpType td = static_cast<C82FpType>(in.pd);
	bool to_accumulator = false;
	/* An accumulator destination: dest r0 with a double precision, or a
	 * vmac/vmsc with a parallel load or store (§ 10.3, 10.3.3) */
	const auto accumulator_if = [&](unsigned reg, bool is_double) {
		if (reg == 0 && is_double) {
			to_accumulator = true;
			td = FP_DOUBLE;
		}
	};

	switch (in.vop) {
	case 0:
		/* vadd: source + dest */
		result = c82_fp_add(source, t1, read_float(dest, td), td, false, td, env);
		break;
	case 1:
		/* vsub: dest - source */
		result = c82_fp_add(read_float(dest, td), td, source, t1, true, td, env);
		break;
	case 2:
		/* vmpy: source x dest */
		result = c82_fp_mul(source, t1, read_float(dest, td), td, td, env);
		break;
	case 3: {
		/* vmsub: source x 1.0 - acc (or 0); dest at bits 26-22, its
		 * precision at bit 7 */
		accumulator_if(dest, in.pd == 1);
		result = c82_fp_mac(static_cast<uint32_t>(source), 0, true, in.zero ? 0 : m_acc[in.acc], true, true, td,
		                    env);
		break;
	}
	case 4:
		/* vrnd from a float: dest r0 with PD double is the accumulator */
		accumulator_if(dest, in.pd == 1);
		result = c82_fp_convert(source, t1, td, env);
		break;
	case 5:
		result = c82_fp_convert(source, t1, td, env);
		break;
	default: {
		/* vmac, vmsc: source1 x source2 +/- acc (or 0). With a parallel
		 * memory operation the destination is the accumulator; without,
		 * the register at bits 31-27, its precision at bit 9 */
		const uint32_t source2 = m_r[in.src2];

		if (in.mem) {
			to_accumulator = true;
			td = FP_DOUBLE;
		} else {
			dest = in.memreg;
			td = in.mem_double ? FP_DOUBLE : FP_SINGLE;
			accumulator_if(dest, in.mem_double);
		}
		result = c82_fp_mac(static_cast<uint32_t>(source), source2, false, in.zero ? 0 : m_acc[in.acc],
		                    in.vop == 7, false, td, env);
		break;
	}
	}

	/* FPST names r0 for an accumulator (§ 10.3.3) */
	if (float_done(env, word, to_accumulator ? 0 : static_cast<uint8_t>(dest), td)) {
		if (to_accumulator)
			m_acc[in.acc] = result;
		else
			write_float(dest, td, result);
	}

	/* The parallel load or store, from IN0P, IN1P or to OUTP, then 4 or 8
	 * more (p. 10-159, 10-175) */
	if (!in.mem)
		return;

	const unsigned size = in.mem_double ? 3 : 2;
	const unsigned even = in.memreg & ~1u, odd = in.memreg | 1u;

	if (in.mem == 1) {
		const uint64_t data = size == 3 ? (static_cast<uint64_t>(m_r[odd]) << 32) | m_r[even] : m_r[in.memreg];

		if (!store(m_outp, size, data))
			memory_fault(m_outp, size, 2, in.memreg, data);
		m_outp += 1u << size;
		return;
	}

	uint32_t &pointer = in.mem == 2 ? m_in0p : m_in1p;
	uint64_t data = 0;

	if (!load(pointer, size, data)) {
		memory_fault(pointer, size, 0, in.memreg, 0);
	} else if (size == 2) {
		set(in.memreg, static_cast<uint32_t>(data));
	} else if (in.memreg) {
		set(even, static_cast<uint32_t>(data));
		set(odd, static_cast<uint32_t>(data >> 32));
	}
	pointer += 1u << size;
}

/* The handlers of prepared instructions (C82MpOp): each does what execute()
 * does for one kind and form of instruction, with the fields prepare() read
 * once; any other instruction, or a case a handler does not take, goes to
 * execute() itself (generic). */
struct C82MpOps {
	/* execute() may change anything: what attention() reads, the end of
	 * the run (end_run), the timer's zero (both set m_check) */
	static void generic(C82Mp &mp, const C82MpOp &op, uint32_t ip)
	{
		mp.m_now_ip = ip;
		mp.m_now_next = mp.m_ip;
		mp.m_now_long = op.length == 8;
		mp.execute(mp.decoded(op), static_cast<uint32_t>(op.key), ip, mp.m_ip);
		if (mp.attention())
			mp.m_check = true;
	}

	template <bool SUB, bool REG, bool UNSIGNED>
	static void add(C82Mp &mp, const C82MpOp &op, uint32_t)
	{
		const uint32_t a = REG ? mp.m_r[op.src1] : op.imm, b = mp.m_r[op.src2];
		const uint32_t r = SUB ? a - b : a + b;

		if (!UNSIGNED && ((SUB ? (a ^ b) & (a ^ r) : ~(a ^ b) & (a ^ r)) >> 31)) {
			mp.m_intpen |= C82Mp::INT_IO;
			mp.m_check = true;
		}
		mp.set(op.dest, r);
	}

	template <unsigned LOGIC, bool REG>
	static void logic(C82Mp &mp, const C82MpOp &op, uint32_t)
	{
		const uint32_t a = REG ? mp.m_r[op.src1] : op.imm, b = mp.m_r[op.src2];
		uint32_t r = 0;

		if (LOGIC & 1)
			r |= a & b;
		if (LOGIC & 2)
			r |= a & ~b;
		if (LOGIC & 4)
			r |= ~a & b;
		if (LOGIC & 8)
			r |= ~a & ~b;
		mp.set(op.dest, r);
	}

	/* A short-form shift: the left rotation (a), CM (aux) known; outside
	 * CM: 0, the old bits, or the top bit of a low CM (b) */
	template <unsigned KIND>
	static void shift(C82Mp &mp, const C82MpOp &op, uint32_t)
	{
		const uint32_t cm = op.aux;
		uint32_t result = std::rotl(mp.m_r[op.src2], op.a) & cm;

		if (KIND == 1)
			result |= mp.m_r[op.dest] & ~cm;
		if (KIND == 2 && ((result >> op.b) & 1))
			result |= ~cm;
		mp.set(op.dest, result);
	}

	template <bool REG>
	static void cmp(C82Mp &mp, const C82MpOp &op, uint32_t)
	{
		const uint32_t a = REG ? mp.m_r[op.src1] : op.imm, b = mp.m_r[op.src2];

		mp.set(op.dest, C82Mp::compare_all(a, b));
	}

	/* Loads and stores, K: size (bits 1-0), register offset (2), :s (3),
	 * :m (4), ld.u (5) */
	template <unsigned K>
	static uint32_t address(C82Mp &mp, const C82MpOp &op)
	{
		uint32_t offset = (K & 4) ? mp.m_r[op.src1] : op.imm;

		if (K & 8)
			offset <<= K & 3;
		return mp.m_r[op.src2] + offset;
	}

	template <unsigned K>
	static void load(C82Mp &mp, const C82MpOp &op, uint32_t)
	{
		constexpr unsigned SIZE = K & 3;
		constexpr bool UNSIGNED = K & 32;
		const uint32_t at = address<K>(mp, op), aligned = at & ~((1u << SIZE) - 1);
		uint64_t data = 0;

		if (K & 16)
			mp.set(op.src2, at);
		if (!mp.load(aligned, SIZE, data)) {
			mp.m_now_long = op.length == 8;
			mp.memory_fault(aligned, SIZE, UNSIGNED ? 1 : 0, op.dest, 0);
			return;
		}
		if (SIZE == 0)
			mp.set(op.dest, UNSIGNED ? static_cast<uint8_t>(data) : static_cast<uint32_t>(static_cast<int8_t>(data)));
		else if (SIZE == 1)
			mp.set(op.dest, UNSIGNED ? static_cast<uint16_t>(data) : static_cast<uint32_t>(static_cast<int16_t>(data)));
		else if (SIZE == 2)
			mp.set(op.dest, static_cast<uint32_t>(data));
		else if (op.dest) {
			mp.set(op.dest & ~1u, static_cast<uint32_t>(data));
			mp.set(op.dest | 1u, static_cast<uint32_t>(data >> 32));
		}
	}

	template <unsigned K>
	static void store(C82Mp &mp, const C82MpOp &op, uint32_t)
	{
		constexpr unsigned SIZE = K & 3;
		const uint32_t at = address<K>(mp, op), aligned = at & ~((1u << SIZE) - 1);
		const uint64_t data = SIZE == 3 ? static_cast<uint64_t>(mp.m_r[op.dest | 1u]) << 32 | mp.m_r[op.dest & ~1u]
		                                : mp.m_r[op.dest];

		if (K & 16)
			mp.set(op.src2, at);
		if (!mp.store(aligned, SIZE, data)) {
			mp.m_now_long = op.length == 8;
			mp.memory_fault(aligned, SIZE, 2, op.dest, data);
		}
	}

	/* exec_branch, selecting rather than jumping on the emulated
	 * condition */
	template <bool ANNUL>
	static void branch(C82Mp &mp, bool taken, uint32_t target)
	{
		if (ANNUL) {
			mp.m_cycles++;
			mp.m_ip = taken ? target : mp.m_ip;
			mp.m_pc = taken ? target + 4 : mp.m_pc;
			return;
		}
		mp.m_pc = taken ? target : mp.m_pc;
		mp.m_slot_next = true;
	}

	/* bcnd: the byte, halfword or word (sign extended by the shift b) is
	 * negative, zero or positive; the condition is the mask a of these
	 * three cases (bit 0, 1, 2) */
	template <bool ANNUL, bool REG>
	static void bcnd(C82Mp &mp, const C82MpOp &op, uint32_t ip)
	{
		const int32_t x = static_cast<int32_t>(mp.m_r[op.src2] << op.b) >> op.b;
		const unsigned sign = static_cast<unsigned>(x == 0) | static_cast<unsigned>(x > 0) << 1;

		branch<ANNUL>(mp, (op.a >> sign) & 1, ip + 4 * (REG ? mp.m_r[op.src1] : op.imm));
	}

	/* bbz, bbo: the bit (a) */
	template <bool ONE, bool ANNUL, bool REG>
	static void bit_branch(C82Mp &mp, const C82MpOp &op, uint32_t ip)
	{
		const bool bit = (mp.m_r[op.src2] >> op.a) & 1;

		branch<ANNUL>(mp, ONE ? bit : !bit, ip + 4 * (REG ? mp.m_r[op.src1] : op.imm));
	}

	/* bsr, jsr: the link written after the operands are read */
	template <bool JSR, bool ANNUL, bool REG>
	static void call(C82Mp &mp, const C82MpOp &op, uint32_t ip)
	{
		const uint32_t source1 = REG ? mp.m_r[op.src1] : op.imm;
		const uint32_t target = JSR ? (source1 + mp.m_r[op.src2]) & ~3u : ip + 4 * source1;

		mp.set(op.dest, ip + op.length + (ANNUL ? 0 : 4));
		branch<ANNUL>(mp, true, target);
	}

	/* The fast floating point (c82_fast) runs when the instruction is
	 * allowed (ie, not fs: else trap 36), no floating-point interrupt is on
	 * (its flags then post nothing, change no result) and the host rounds
	 * to nearest; and, when it rounds by drm, drm is to nearest */
	static bool fast_float(const C82Mp &mp, bool rounds)
	{
		return mp.m_float_mode & (rounds ? 2 : 1);
	}

	/* float_done with inexact or no flag (or mo, bit 5), nothing raised:
	 * aux holds the precision, opcode and destination fields */
	static void fast_done(C82Mp &mp, const C82MpOp &op, uint32_t flags, uint32_t round)
	{
		mp.m_fpst = (mp.m_fpst & 0x07FE0000u) | flags | round << 7 | op.aux | (flags & 31) << 22;
	}

	/* fadd, fsub, fmpy, fdiv .sss; fcmp .ss. K: the op (bits 1-0: add,
	 * sub, mpy, div), a long immediate (bit 2) */
	template <unsigned K>
	static void float_sss(C82Mp &mp, const C82MpOp &op, uint32_t ip)
	{
		const uint32_t a = (K & 4) ? op.imm : mp.m_r[op.src1], b = mp.m_r[op.src2];
		uint32_t r = 0, flags = 0;
		bool done = false;

		if (fast_float(mp, true)) {
			if ((K & 3) <= 1)
				done = c82_fast::add_sss(a, b, K & 1, r, flags);
			else if ((K & 3) == 2)
				done = c82_fast::mul_sss(a, b, r, flags);
			else
				done = c82_fast::div_sss(a, b, r, flags);
		}
		if (!done)
			return generic(mp, op, ip);
		fast_done(mp, op, flags, 0);
		mp.set(op.dest, r);
	}

	/* exec_float's first operand: a long immediate is a single, or an
	 * integer for an integer P1 */
	template <bool LONG>
	static uint64_t source1(const C82Mp &mp, const C82MpOp &op, C82FpType t1)
	{
		return LONG ? op.imm : mp.read_float(op.src1, t1);
	}

	/* fcmp of singles or doubles (a: P1, b: P2) */
	template <bool LONG>
	static void fcmp(C82Mp &mp, const C82MpOp &op, uint32_t ip)
	{
		const C82FpType t1 = static_cast<C82FpType>(op.a), t2 = static_cast<C82FpType>(op.b);
		uint32_t status = 0;

		if (!fast_float(mp, false) || !c82_fast::compare(source1<LONG>(mp, op, t1), t1, mp.read_float(op.src2, t2), t2, status))
			return generic(mp, op, ip);
		fast_done(mp, op, 0, (mp.m_fpst >> 17) & 3);
		mp.set(op.dest, status);
	}

	/* fadd, fsub to a double (a: P1, b: P2; c: fsub) */
	template <bool LONG>
	static void fadd_double(C82Mp &mp, const C82MpOp &op, uint32_t ip)
	{
		const C82FpType t1 = static_cast<C82FpType>(op.a), t2 = static_cast<C82FpType>(op.b);
		uint64_t r = 0;
		uint32_t flags = 0;

		if (!fast_float(mp, true) ||
		    !c82_fast::add_double(source1<LONG>(mp, op, t1), t1, mp.read_float(op.src2, t2), t2, op.c, r, flags))
			return generic(mp, op, ip);
		fast_done(mp, op, flags, 0);
		mp.write_float(op.dest, FP_DOUBLE, r);
	}

	/* fmpy.ssd: exact */
	template <bool LONG>
	static void fmpy_ssd(C82Mp &mp, const C82MpOp &op, uint32_t ip)
	{
		uint64_t r = 0;

		if (!fast_float(mp, false) || !c82_fast::mul_ssd(LONG ? op.imm : mp.m_r[op.src1], mp.m_r[op.src2], r))
			return generic(mp, op, ip);
		fast_done(mp, op, 0, (mp.m_fpst >> 17) & 3);
		mp.write_float(op.dest, FP_DOUBLE, r);
	}

	/* fmpy.iii, fmpy.uuu (c: unsigned): the low 32 bits of the product, mo
	 * when it does not fit, no flag */
	template <bool LONG, bool UNSIGNED>
	static void fmpy_integer(C82Mp &mp, const C82MpOp &op, uint32_t ip)
	{
		const uint32_t a = LONG ? op.imm : mp.m_r[op.src1], b = mp.m_r[op.src2];
		uint32_t r = 0;
		bool mo = false;

		if (!fast_float(mp, false))
			return generic(mp, op, ip);
		if (UNSIGNED) {
			const uint64_t p = static_cast<uint64_t>(a) * b;

			mo = p > 0xFFFFFFFFull;
			r = static_cast<uint32_t>(p);
		} else {
			const int64_t p = static_cast<int64_t>(static_cast<int32_t>(a)) * static_cast<int32_t>(b);

			mo = p < INT32_MIN || p > INT32_MAX;
			r = static_cast<uint32_t>(p);
		}
		fast_done(mp, op, mo ? 1u << 5 : 0u, (mp.m_fpst >> 17) & 3);
		mp.set(op.dest, r);
	}

	/* frnd (a: P1 as read, b: PD, c: RM) */
	template <bool LONG>
	static void frnd(C82Mp &mp, const C82MpOp &op, uint32_t ip)
	{
		const C82FpType t1 = static_cast<C82FpType>(op.a), td = static_cast<C82FpType>(op.b);
		uint64_t r = 0;
		uint32_t flags = 0;

		if (!fast_float(mp, false) ||
		    !c82_fast::convert(source1<LONG>(mp, op, t1), t1, td, static_cast<C82FpRound>(op.c), r, flags))
			return generic(mp, op, ip);
		fast_done(mp, op, flags, op.c);
		mp.write_float(op.dest, td, r);
	}

	/* wrcr, swcr of IN0P, IN1P or OUTP: read_control and write_control do
	 * nothing more for these */
	template <unsigned N>
	static void swcr_pointer(C82Mp &mp, const C82MpOp &op, uint32_t)
	{
		uint32_t &pointer = N == 0 ? mp.m_in0p : N == 1 ? mp.m_in1p : mp.m_outp;
		const uint32_t old = pointer;

		mp.m_park.spoiled = true;
		pointer = mp.m_r[op.src2];
		mp.set(op.dest, old);
	}

	/* The parallel vld or vst of a vector instruction (MEM: 1 vst, 2 vld0,
	 * 3 vld1): its register (c), double (d) */
	template <unsigned MEM>
	static void vector_memory(C82Mp &mp, const C82MpOp &op)
	{
		const unsigned size = op.d ? 3 : 2, reg = op.c;

		if (MEM == 1) {
			const uint64_t data = size == 3 ? static_cast<uint64_t>(mp.m_r[reg | 1u]) << 32 | mp.m_r[reg & ~1u]
			                                : mp.m_r[reg];

			if (!mp.store(mp.m_outp, size, data)) {
				mp.m_now_long = op.length == 8;
				mp.memory_fault(mp.m_outp, size, 2, static_cast<uint8_t>(reg), data);
			}
			mp.m_outp += 1u << size;
			return;
		}

		uint32_t &pointer = MEM == 2 ? mp.m_in0p : mp.m_in1p;
		uint64_t data = 0;

		if (!mp.load(pointer, size, data)) {
			mp.m_now_long = op.length == 8;
			mp.memory_fault(pointer, size, 0, static_cast<uint8_t>(reg), 0);
		} else if (size == 2) {
			mp.set(reg, static_cast<uint32_t>(data));
		} else if (reg) {
			mp.set(reg & ~1u, static_cast<uint32_t>(data));
			mp.set(reg | 1u, static_cast<uint32_t>(data >> 32));
		}
		pointer += 1u << size;
	}

	/* The result of a vector instruction: to the accumulator b (a bit 0),
	 * or to dest as its type (a bits 6-5) */
	static void vector_write(C82Mp &mp, const C82MpOp &op, uint64_t result)
	{
		if (op.a & 1)
			mp.m_acc[op.b] = result;
		else
			mp.write_float(op.dest, static_cast<uint8_t>((op.a >> 5) & 3), result);
	}

	/* Vector, K: the memory operation (bits 1-0), a long immediate (2), the
	 * operation (bits 4-3: vmpy.sss, vrnd, vmac, vmsc), a vmac's double
	 * result (5) */
	template <unsigned K>
	static void vector(C82Mp &mp, const C82MpOp &op, uint32_t ip)
	{
		constexpr unsigned MEM = K & 3, KIND = (K >> 3) & 3;
		/* P1 single but for a vrnd */
		const uint64_t source = (K & 4) ? op.imm : KIND == 1 ? mp.read_float(op.src1, static_cast<C82FpType>((op.a >> 3) & 3)) : mp.m_r[op.src1];
		uint64_t result = 0;
		uint32_t flags = 0, round = 0;

		if (KIND == 0) {
			uint32_t r = 0;

			if (!fast_float(mp, true) || !c82_fast::mul_sss(static_cast<uint32_t>(source), mp.m_r[op.src2], r, flags))
				return generic(mp, op, ip);
			result = r;
		} else if (KIND == 1) {
			round = (mp.m_fpst >> 17) & 3;
			if (!fast_float(mp, false) || !c82_fast::convert(source, static_cast<C82FpType>((op.a >> 3) & 3),
			                                                   static_cast<C82FpType>((op.a >> 5) & 3),
			                                                   static_cast<C82FpRound>(round), result, flags))
				return generic(mp, op, ip);
		} else {
			/* A single result rounds toward zero, whatever drm */
			constexpr bool single = !(K & 32);

			if (!fast_float(mp, !single) ||
			    !c82_fast::mac(static_cast<uint32_t>(source), mp.m_r[op.src2], (op.a & 4) ? 0 : mp.m_acc[op.b], KIND == 3, single, result,
			                   flags))
				return generic(mp, op, ip);
			round = single ? ROUND_ZERO : 0;
		}
		fast_done(mp, op, flags, round);
		/* vmpy.sss and a single vmac write a register */
		if (KIND == 0 || (KIND >= 2 && !(K & 32)))
			mp.set(op.dest, static_cast<uint32_t>(result));
		else
			vector_write(mp, op, result);
		if (MEM)
			vector_memory<MEM>(mp, op);
	}

	template <size_t... K>
	static constexpr std::array<C82MpExec, sizeof...(K)> loads(std::index_sequence<K...>)
	{
		return { { &load<K>... } };
	}
	template <size_t... K>
	static constexpr std::array<C82MpExec, sizeof...(K)> stores(std::index_sequence<K...>)
	{
		return { { &store<K>... } };
	}
	template <size_t... K>
	static constexpr std::array<C82MpExec, sizeof...(K)> vectors(std::index_sequence<K...>)
	{
		return { { &vector<K>... } };
	}
	template <size_t... K>
	static constexpr std::array<C82MpExec, sizeof...(K)> floats(std::index_sequence<K...>)
	{
		return { { &float_sss<K>... } };
	}
	template <size_t... K>
	static constexpr std::array<C82MpExec, sizeof...(K)> logics(std::index_sequence<K...>)
	{
		return { { &logic<K / 2, K % 2 == 1>... } };
	}

	/* The handler of an instruction, its fields set in op */
	static C82MpExec choose(const C82Insn &in, uint32_t word, C82MpOp &op)
	{
		static constexpr auto LOADS = loads(std::make_index_sequence<64>{});
		static constexpr auto STORES = stores(std::make_index_sequence<32>{});
		static constexpr auto VECTORS = vectors(std::make_index_sequence<64>{});
		static constexpr auto FLOATS = floats(std::make_index_sequence<8>{});
		static constexpr auto LOGICS = logics(std::make_index_sequence<32>{});
		const bool reg = in.form == C82Insn::REGISTER, long_form = in.form == C82Insn::LONG;
		/* FPST's opcode field: bits 15-13 and bit 17 inverted (float_done) */
		const uint32_t opcode = ((word >> 13) & 7) | ((~word >> 17) & 1) << 3;

		op.dest = in.dest;
		op.src1 = in.src1;
		op.src2 = in.src2;
		op.length = in.length;
		op.imm = in.imm;
		switch (in.op) {
		case C82Insn::ADD:
			return reg ? (in.is_unsigned ? &add<false, true, true> : &add<false, true, false>)
			           : (in.is_unsigned ? &add<false, false, true> : &add<false, false, false>);
		case C82Insn::SUB:
			return reg ? (in.is_unsigned ? &add<true, true, true> : &add<true, true, false>)
			           : (in.is_unsigned ? &add<true, false, true> : &add<true, false, false>);
		case C82Insn::LOGIC:
			return LOGICS[in.logic * 2 + (reg ? 1 : 0)];
		case C82Insn::CMP:
			return reg ? &cmp<true> : &cmp<false>;
		case C82Insn::SHIFT: {
			if (in.form != C82Insn::SHORT)
				break;

			/* exec_shift with everything known */
			const uint32_t endmask = (in.endmask ? mask_of(in.endmask) : ~0u) ^ (in.invert ? ~0u : 0u);
			const unsigned code = in.right ? (32 - in.rotate) & 31 : in.rotate;
			uint32_t shiftmask = ~0u;

			if (in.merge >= 6)
				shiftmask = code ? ~mask_of(code) : ~0u;
			else if (in.merge >= 3)
				shiftmask = code ? mask_of(code) : in.merge == 4 ? 0 : ~0u;

			const uint32_t cm = shiftmask & endmask;
			static constexpr uint8_t MERGE[8] = { 0, 1, 2, 0, 1, 2, 0, 1 };

			op.aux = cm;
			op.a = static_cast<uint8_t>(code);
			switch (MERGE[in.merge]) {
			case 1:
				return &shift<1>;
			case 2:
				/* Only a low CM; the sign of nothing or of all is nothing */
				if ((cm & (cm + 1)) != 0)
					break;
				if (!cm || cm == ~0u)
					return &shift<0>;
				op.b = static_cast<uint8_t>(31 - std::countl_zero(cm));
				return &shift<2>;
			default:
				return &shift<0>;
			}
			break;
		}
		case C82Insn::LOAD:
			if (in.is_unsigned && in.size >= 2)
				break;
			return LOADS[in.size | (reg ? 4 : 0) | (in.scale ? 8 : 0) | (in.modify ? 16 : 0) | (in.is_unsigned ? 32 : 0)];
		case C82Insn::STORE:
			return STORES[in.size | (reg ? 4 : 0) | (in.scale ? 8 : 0) | (in.modify ? 16 : 0)];
		case C82Insn::BCND: {
			/* The condition (bits 29-27): never, gt, eq, ge, lt, ne, le,
			 * always; the size (bits 31-30): byte, halfword, word, word
			 * (.d, HEURISTIC as execute) */
			static constexpr uint8_t MASKS[8] = { 0, 4, 2, 6, 1, 5, 3, 7 }, SHIFTS[4] = { 24, 16, 0, 0 };

			op.a = MASKS[in.dest & 7];
			op.b = SHIFTS[in.dest >> 3];
			return in.annul ? (reg ? &bcnd<true, true> : &bcnd<true, false>) : (reg ? &bcnd<false, true> : &bcnd<false, false>);
		}
		case C82Insn::BBZ:
		case C82Insn::BBO:
			op.a = ~in.dest & 31;
			if (in.op == C82Insn::BBO)
				return in.annul ? (reg ? &bit_branch<true, true, true> : &bit_branch<true, true, false>)
				                : (reg ? &bit_branch<true, false, true> : &bit_branch<true, false, false>);
			return in.annul ? (reg ? &bit_branch<false, true, true> : &bit_branch<false, true, false>)
			                : (reg ? &bit_branch<false, false, true> : &bit_branch<false, false, false>);
		case C82Insn::BSR:
			return in.annul ? (reg ? &call<false, true, true> : &call<false, true, false>)
			                : (reg ? &call<false, false, true> : &call<false, false, false>);
		case C82Insn::JSR:
			return in.annul ? (reg ? &call<true, true, true> : &call<true, true, false>)
			                : (reg ? &call<true, false, true> : &call<true, false, false>);
		case C82Insn::SWCR:
			/* wrcr, swcr of the vector pointers */
			if (reg || in.imm < C82Mp::IN0P || in.imm > C82Mp::OUTP)
				break;
			return in.imm == C82Mp::IN0P ? &swcr_pointer<0> : in.imm == C82Mp::IN1P ? &swcr_pointer<1> : &swcr_pointer<2>;
		case C82Insn::FADD:
		case C82Insn::FSUB:
		case C82Insn::FMPY:
		case C82Insn::FDIV:
		case C82Insn::FRND:
		case C82Insn::FCMP: {
			/* exec_float's types: a long immediate is a single, or an
			 * integer for an integer P1 */
			const uint8_t t1 = long_form && in.p1 == FP_DOUBLE ? uint8_t{ FP_SINGLE } : in.p1;
			const bool sss = t1 == FP_SINGLE && in.p2 == FP_SINGLE && in.pd == FP_SINGLE;

			op.aux = static_cast<uint32_t>(in.pd & 3) << 9 | opcode << 13 | static_cast<uint32_t>(in.dest) << 27;
			op.a = t1;
			op.b = in.op == C82Insn::FRND ? in.pd : in.p2;
			op.c = in.op == C82Insn::FRND ? in.rm : in.op == C82Insn::FSUB;
			if (in.op == C82Insn::FCMP)
				return t1 <= FP_DOUBLE && in.p2 <= FP_DOUBLE ? (long_form ? &fcmp<true> : &fcmp<false>) : &generic;
			if (in.op == C82Insn::FRND)
				return long_form ? &frnd<true> : &frnd<false>;
			if (sss)
				return FLOATS[(in.op - C82Insn::FADD) | (long_form ? 4 : 0)];
			if ((in.op == C82Insn::FADD || in.op == C82Insn::FSUB) && t1 <= FP_DOUBLE && in.p2 <= FP_DOUBLE &&
			    in.pd == FP_DOUBLE)
				return long_form ? &fadd_double<true> : &fadd_double<false>;
			if (in.op == C82Insn::FMPY && t1 == FP_SINGLE && in.p2 == FP_SINGLE && in.pd == FP_DOUBLE)
				return long_form ? &fmpy_ssd<true> : &fmpy_ssd<false>;
			if (in.op == C82Insn::FMPY && t1 >= FP_INT && in.p2 == t1 && in.pd == t1)
				return t1 == FP_INT ? (long_form ? &fmpy_integer<true, false> : &fmpy_integer<false, false>)
				                    : (long_form ? &fmpy_integer<true, true> : &fmpy_integer<false, true>);
			break;
		}
		case C82Insn::VECTOR: {
			/* exec_vector's destination: a register and its type, or an
			 * accumulator */
			unsigned kind = 0, dest = in.src2;
			uint8_t td = in.pd;
			bool to_accumulator = false;

			if (in.vop == 2 && in.p1 == FP_SINGLE && in.pd == FP_SINGLE) {
				kind = 0;
			} else if (in.vop == 4 || in.vop == 5) {
				/* vrnd: dest r0 with a double is the accumulator */
				kind = 1;
				to_accumulator = in.vop == 4 && dest == 0 && in.pd == 1;
			} else if ((in.vop == 6 || in.vop == 7) && in.p1 == FP_SINGLE) {
				kind = in.vop == 6 ? 2 : 3;
				if (in.mem) {
					to_accumulator = true;
					td = FP_DOUBLE;
				} else {
					dest = in.memreg;
					td = in.mem_double ? FP_DOUBLE : FP_SINGLE;
					to_accumulator = dest == 0 && in.mem_double;
				}
			} else {
				break;
			}
			op.dest = static_cast<uint8_t>(dest);
			op.a = static_cast<uint8_t>((to_accumulator ? 1 : 0) | (td == FP_DOUBLE ? 2 : 0) | (in.zero ? 4 : 0) |
			                            (in.p1 & 3) << 3 | (td & 3) << 5);
			op.b = in.acc;
			op.c = in.memreg;
			op.d = in.mem_double;
			op.aux = static_cast<uint32_t>(td & 3) << 9 | opcode << 13 | static_cast<uint32_t>(to_accumulator ? 0 : dest) << 27;
			return VECTORS[in.mem | (long_form ? 4 : 0) | kind << 3 | (kind >= 2 && td == FP_DOUBLE ? 32 : 0)];
		}
		default:
			break;
		}
		return &generic;
	}
};

uint32_t
C82Mp::compare_all(uint32_t a, uint32_t b)
{
	return compare_bits(a, b, 8) | compare_bits(a, b, 16) << 10 | compare_bits(a, b, 32) << 20;
}

/* For the blocks' host code: the parallel load or store of a vector
 * instruction whose operation it did, as vector() does it */
void
C82Mp::block_vector_memory(C82Mp &mp, uint32_t ip, uint64_t key)
{
	const C82MpOp &op = mp.prepared(ip, static_cast<uint32_t>(key), static_cast<uint32_t>(key >> 32));

	switch (mp.decoded(op).mem) {
	case 1:
		C82MpOps::vector_memory<1>(mp, op);
		break;
	case 2:
		C82MpOps::vector_memory<2>(mp, op);
		break;
	case 3:
		C82MpOps::vector_memory<3>(mp, op);
		break;
	default:
		break;
	}
}

void
C82Mp::prepare(C82MpOp &op, uint32_t word, uint32_t immediate)
{
	Decoded &d = m_decoded[&op - m_ops.data()];

	d.word = word;
	d.immediate = immediate;
	d.insn = c82_decode(word, immediate);
	op = C82MpOp{};
	op.key = word | static_cast<uint64_t>(immediate) << 32;
	op.exec = C82MpOps::choose(d.insn, word, op);
}

} /* namespace odin */
