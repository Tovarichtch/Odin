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


#ifndef ODIN_C82_MP_HOST_HPP
#define ODIN_C82_MP_HOST_HPP

#include <cstdint>
#include <initializer_list>
#include <vector>

#include "odin/machine/core/host_code.hpp"

namespace odin {

/* Host code for the MP's blocks (C82Mp::Block): the memory it is written in
 * and the x86-64 instructions the blocks are made of (c82_mp_host.cpp writes
 * the steps and their ends, c82_mp_host_exec.cpp the forms written out). A
 * block works on the MP itself (rbx), keeps the cycles left in the slice in
 * rbp and the slice's end in r15, finds the pages of the external bus from
 * r14 (the chip's other tables at fixed distances from them), and the next
 * instruction after a branch in r12. On another host
 * (AArch64 included: not written yet), or one that refuses the memory,
 * usable() is false and the interpreter runs alone. */
class C82MpCode : public HostCode {
public:
	C82MpCode();
	bool usable() const;
	/* Forgets every block and writes the ways in and out again */
	void clear();
	/* Room left for the longest block there is? */
	bool room() const;
	/* Where the code goes on: the steps (hot), or the ways out of their
	 * line and the ends (cold), apart so that the steps run close */
	void hot();
	void cold();
	/* Runs the code at `place` for the MP, with the pages of the external
	 * bus at `pages`, until the cycle `limit` (the blocks write where they
	 * stopped in the MP) */
	void run(void *mp, const uint8_t *code, const void *pages, uint64_t limit) const
	{
		m_enter(mp, code, pages, limit);
	}

	/* The jump whose displacement is at `at` sent to `place` (written
	 * already: writable() before, ready() after) */
	void relink(uint32_t at, uint32_t place);

#if defined(ODIN_HOST_X86_64)
	static constexpr uint8_t MP = RBX, LEFT = RBP, LIMIT = R15, PAGES = R14, NEXT = R12;
	using HostCode::RAX;
	using HostCode::RCX;
	using HostCode::RDX;
	using HostCode::R8;
	using HostCode::R9;
	using HostCode::R10;
	using HostCode::R11;
	using HostCode::ARG;
	using HostCode::byte;
	using HostCode::bytes;
	using HostCode::word;
	using HostCode::quad;
	using HostCode::bind;
	using HostCode::jump_back;
	using HostCode::call_to;
	/* Condition codes (the low nibble of 0F 8x, 0F 9x, 0F 4x) */
	enum Cc : uint8_t { OV, NOV, B, AE, E, NE, BE, A, S, NS, P, NP, L, GE, LE, G };
	/* [base + index * 2^scale + displacement]; NO: no index */
	static constexpr uint8_t NO = 0xFF;
	struct Mem {
		uint8_t base, index = NO, scale = 0;
		int32_t disp = 0;
	};
	/* `opcode` after the legacy prefix `prefix` (0: none, 0x66, 0xF2,
	 * 0xF3), with ModRM naming `reg` (or a group's step) and the memory
	 * `m`, or the register `rm`; wide: REX.W; low: a byte register */
	void op_m(uint8_t prefix, std::initializer_list<uint8_t> opcode, uint8_t reg, Mem m, bool wide = false, bool low = false);
	void op_r(uint8_t prefix, std::initializer_list<uint8_t> opcode, uint8_t reg, uint8_t rm, bool wide = false, bool low = false);
	/* reg = value, on 32 bits and on 64 */
	void move32(uint8_t reg, uint32_t value);
	/* bswap reg, on 32 or 64 bits */
	void swap(uint8_t reg, bool wide);
	void move64(uint8_t reg, uint64_t value);
	/* int3 up to the next multiple of 8 (a table follows) */
	void align8();
	/* A jump on a condition, or always, forward (bound later) or to a
	 * place written already */
	Jump jump_if(Cc cc);
	Jump jump_ahead();
	void jump_if_to(Cc cc, uint32_t place);
	/* The ways back to C++: with the cycles from rbp, or as they are in
	 * the MP (a handler wrote them) */
	uint32_t leave_left() const { return m_leave_left; }
	uint32_t leave_kept() const { return m_leave; }
#endif

private:
	uint64_t (*m_enter)(void *, const uint8_t *, const void *, uint64_t);
	uint32_t m_leave_left = 0;
	/* The place in the other part of the memory; whether cold is writing */
	uint32_t m_other = 0;
	bool m_cold = false;
	/* Where the MP keeps its cycle count, from rbx */
	friend class C82Mp;
	int32_t m_cycles_at = 0;
};

/* A way out of the line of a step (c82_mp_host_exec.cpp): the jumps that
 * lead to it, where the step goes on; the step's handler called for the
 * whole step, or a vector instruction's load or store alone, or a signed
 * overflow (the result in eax, for `dest`) */
struct C82MpSlow {
	enum Kind : uint8_t { HANDLER, VECTOR_MEMORY, OVERFLOW };
	std::vector<HostCode::Jump> from;
	uint32_t back = 0;
	unsigned k = 0;
	Kind kind = HANDLER;
	uint8_t dest = 0;
};

} /* namespace odin */

#endif /* ODIN_C82_MP_HOST_HPP */
