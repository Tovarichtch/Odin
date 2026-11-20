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


#ifndef ODIN_C82_PP_HOST_HPP
#define ODIN_C82_PP_HOST_HPP

#include <cstdint>
#include <initializer_list>
#include <vector>

#include "odin/machine/core/host_code.hpp"

namespace odin {

/* Host code for the prepared blocks of a PP (C82Pp::Block): the memory it is
 * written in and the few x86-64 instructions the blocks are made of
 * (c82_pp_host.cpp writes them, step by step, as run_block() does them). A
 * block works on the PP itself (rbx), counts the cycles left in rbp, finds
 * the words it fetched from r13 and calls the execute stage's handlers of
 * the interpreter. On another host, or one that refuses the memory, usable()
 * is false and the blocks run in C++ (run_block). */
class C82PpCode : public HostCode {
public:
	C82PpCode();
	/* Host code is written here (x86-64 and the memory given) */
	bool usable() const;
	/* Forgets every block and writes the way in again */
	void clear();
	/* Room left for the longest block there is? */
	bool room() const;
	/* Runs the code at `place` for the PP, with the fetched words from
	 * `host` and `left` cycles left in the slice: what the block's end
	 * returned (C82Pp::run_native) */
	uint64_t run(void *pp, uint32_t place, const uint8_t *host, int64_t left) const;
	/* The way in itself, to run the code at a host address */
	auto enter() const { return m_enter; }

#if defined(ODIN_HOST_X86_64)
	/* The x86-64 steps the writer needs, all with the PP in rbx */
	static constexpr uint8_t PP = RBX, LEFT = RBP, HOST = R13;
	using HostCode::RAX;
	using HostCode::RCX;
	using HostCode::RDX;
	/* Neither kept by a call nor an argument, on both systems (rsi and
	 * rdi are kept under Windows) */
	using HostCode::R10;
	using HostCode::R11;
	using HostCode::JNZ;
	using HostCode::JZ;
	static constexpr uint8_t JLE = 0x8E;
	using HostCode::ARG;
	using HostCode::byte;
	using HostCode::bytes;
	using HostCode::word;
	using HostCode::quad;
	using HostCode::jump_if;
	using HostCode::bind;
	using HostCode::jump_back;
	using HostCode::call_to;
	using HostCode::op_ri;
	using HostCode::shift_ri;
	/* `opcode` with ModRM naming `reg` and [base + displacement] (never
	 * rsp or r12 as the base); 64 bits when wide */
	void op_mem(uint8_t opcode, uint8_t reg, uint8_t base, int32_t displacement, bool wide = false);
	/* reg = value on 64 bits; on 32 */
	void move_quad(uint8_t reg, uint64_t value);
	void move_word(uint8_t reg, uint32_t value);
	/* mov the first argument, the PP */
	void pp_first();
	/* A jump forward, bound later */
	Jump jump_ahead();
	/* The way back to C++, where the ends of the blocks jump */
	uint32_t leave() const { return m_leave; }

	/* For the execute stage written in host code (c82_pp_host_exec.cpp):
	 * the other registers it uses (r8-r11 are neither kept by a call nor
	 * kept for the caller; r12, r14, r15 are kept by the way in) */
	using HostCode::R8;
	using HostCode::R9;
	using HostCode::R12;
	using HostCode::R14;
	using HostCode::R15;
	using HostCode::JC;
	using HostCode::JNC;
	static constexpr uint8_t JS = 0x88, JAE = 0x83;
	using HostCode::mov_ri;
	using HostCode::not_r;
	using HostCode::op_rr;
	using HostCode::flag_r;
	/* [base + index * 2^scale + displacement]; NO: no index */
	static constexpr uint8_t NO = 0xFF;
	struct Mem {
		uint8_t base, index = NO, scale = 0;
		int32_t disp = 0;
	};
	/* `opcode` with ModRM naming `reg` (or a group's step) and the memory
	 * `m`, or the register `rm`; wide: REX.W; half: the 0x66 prefix;
	 * low: `reg` or `rm` is a byte register */
	void op_m(std::initializer_list<uint8_t> opcode, uint8_t reg, Mem m, bool wide = false, bool half = false,
	          bool low = false);
	void op_r(std::initializer_list<uint8_t> opcode, uint8_t reg, uint8_t rm, bool wide = false, bool half = false,
	          bool low = false);
	/* bswap reg (32 bits) */
	void swap32(uint8_t reg);
#endif

private:
	uint64_t (*m_enter)(void *, const uint8_t *, const uint8_t *, int64_t);
};

/* A slow path out of the line of a step (c82_pp_host_exec.cpp): the jumps
 * that lead to it and where the step goes on; the step's number and the
 * slot of its execute stage's addresses; either its handler called (with
 * the cycles it adds when it reaches memory) or an external address tested
 * by plain() for the access register `reg` */
struct C82PpSlow {
	std::vector<HostCode::Jump> from;
	uint32_t back = 0;
	unsigned k = 0, se = 0;
	bool handler = false, memory = false;
	/* The step's frame written first (C82Pp::write_frame) */
	bool frame = false;
	uint32_t pc = 0, ipa = 0, ipe = 0, protect = 0;
	uint8_t form = 0, reg = 0;
	const void *op = nullptr, *in = nullptr;
	/* The access is a store (plain_store); the way out of the line
	 * of a store alone (its watched line looked at) */
	bool store = false, watch = false;
	/* slow_unless_ram's jumps for a watched line (alone looked at
	 * first), then the slow path's as from */
	std::vector<HostCode::Jump> watched;
};

} /* namespace odin */

#endif /* ODIN_C82_PP_HOST_HPP */
