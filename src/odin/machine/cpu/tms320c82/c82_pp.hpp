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

#ifndef ODIN_C82_PP_HPP
#define ODIN_C82_PP_HPP

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <utility>
#include <vector>

namespace odin {

class Tms320c82;
class C82PpCode;
struct C82PpSlow;

/* One parallel transfer (TI SPRU110 appendix A.4, table A-7) */
struct C82PpXfer {
	/* Table A-7: 4-7 post-index with modify, 8-11 pre-index, 12-15
	 * pre-index with modify; odd: subtract; 4, 5, 8, 9, 12, 13: index
	 * register, others: immediate offset */
	uint8_t mode = 0;
	/* The register loaded, stored or written by address unit arithmetic
	 * (7-bit code of table 7-1) */
	uint8_t reg = 0;
	/* The address register's code: 0-7 local (a0-a7), 8-15 global (a8-a15) */
	uint8_t ar = 0;
	/* L (bit 1) and e (bit 0): 2 zero-extended load, 3 sign-extended
	 * load, 0 store, 1 address unit arithmetic (A.4.1) */
	uint8_t le = 0;
	/* 0 byte, 1 halfword, 2 word, 3 reserved */
	uint8_t size = 0;
	/* PP-relative: 0 none, 1 reserved, 2 dba, 3 pba (Lrm/Grm, A.4.2) */
	uint8_t rel = 0;
	/* The s bit: scale the index (halfword, word) */
	bool scale = false;
	bool global = false;
	/* The immediate offset (with the s bit as its MSB for bytes), or the
	 * index register's three LSBs */
	uint16_t index = 0;

	/* Prepared at decode: the index register (index modes, else 0); the
	 * index's left shift (s); subtract; the address is the computed one
	 * (pre-index); the address register is written back; the access size
	 * (a reserved size as a word); a conditional global store picks the
	 * odd or even D register; a load to pc with an unscaled offset; a
	 * local access over the global port costs a cycle (format 1) */
	uint8_t xreg = 0, shift = 0, msize = 0;
	bool sub = false, pre = false, writeback = false, pair = false, pcload = false, divert = false;
	/* Prepared for the address stage: the base's slot in the registers (an
	 * address register's, or dba's or pba's), the index's (ZERO for an
	 * offset) and the offset (0 for a register); subtract and pre-index as
	 * masks; the address register written back (0xFF: none) */
	uint8_t base_slot = 0, index_slot = 0, wb = 0xFF;
	uint32_t offset = 0, neg = 0, pre_mask = 0;
};

/* An instruction of a parallel processor, its fields as SPRU110 appendix A
 * codes them (fig. A-1) */
struct C82PpInsn {
	/* The data unit format: A six-operand, B 5-bit immediate, C register,
	 * D 32-bit immediate, E miscellaneous; reserved: 00x and 010 */
	enum Du : uint8_t { RESERVED, A, B, C, D, E };
	/* Prepared at decode: which way the data unit goes. BASE1: formats B,
	 * C, D without multiple arithmetic or bit detection (one piece);
	 * MPY_ADD: format A's MPY||ADD (mpy_add) */
	enum Path : uint8_t { NO_DU, BASE1, GENERIC, MPY_ADD };
	/* Prepared at decode: the execute stage's work: none (NOP); one load
	 * or one store and no data unit (LOAD, STORE); a BASE1 data unit
	 * alone or with one load or store (DU, DU_LOAD, DU_STORE); anything
	 * else (GENERIC). The loads and stores here: no pc load, no
	 * condition on the transfer. One address unit arithmetic and nothing
	 * else (LEA) */
	enum Shape : uint8_t { GENERIC_SHAPE, NOP, LOAD, STORE, DU, DU_LOAD, DU_STORE, LEA };
	/* The execute stage's handler (C82Pp::EXEC): the shape, and for the
	 * BASE1 shapes the class and arithmetic too (FORM_DU + 3 x (class x 2 +
	 * A) + 0 alone, 1 with a load, 2 with a store); FORM_F: the same with
	 * F known too, for the class, A and F the game runs most (C82PP_F);
	 * FORM_LEA: the shape LEA */
	enum : uint8_t {
		FORM_GENERIC, FORM_NOP, FORM_LOAD, FORM_STORE, FORM_DU, FORM_F = FORM_DU + 48, FORM_LEA = FORM_F + 3 * 14, FORMS
	};

	/* First what each stage reads every cycle (one cache line with the
	 * decode cache's tag). Prepared: the shape, the path, eint (1) or
	 * dint (0) setting G before the next fetch (-1: neither) */
	Shape shape = GENERIC_SHAPE;
	uint8_t form = FORM_GENERIC;
	Path path = NO_DU;
	int8_t g_change = -1;
	/* An unimplemented opcode: the PP stalls (SPRU110 § 5.2.3) */
	bool illegal = false;
	Du du = RESERVED;
	/* The parallel transfer format, 1-10; 0 with format D */
	uint8_t pt = 0;
	/* Formats B, C, D: class (62-60), A (59), ALU operation (58-51) */
	uint8_t cls = 0, alu = 0;
	bool arith = false;
	/* dst (dst1) and src1: 3 LSBs; their banks (4 = D registers unless
	 * a format gives one) */
	uint8_t dst = 0, src1 = 0, dstbank = 4, s1bank = 4;
	/* src2: a D register (formats A, C) or the immediate (B, D) */
	uint8_t src2 = 0;
	bool src2_imm = false;
	/* Conditions (formats 7-10 and D): code, c (source), r (result),
	 * g (global transfer or move), protected status bits N C V Z */
	uint8_t cond = 0, prot = 0;
	bool c = false, r = false, g = false;
	/* Up to two transfers, global first when there are two */
	uint8_t xfers = 0;
	uint32_t imm = 0;
	/* Prepared: the protected status bits as a mask of sr */
	uint32_t protect = 0;
	/* Prepared for BASE1: port A forced to 0 (0) or kept (all ones); the
	 * carry-in (0: by the sign, 1: C of sr, 2: the LSB of port C); %!;
	 * code 9's inverted C(n); no status for a write to pc or sr; the F
	 * code and the carry-in for a positive and a negative rotator input */
	uint32_t a_keep = ~0u;
	uint8_t cin_sel = 0;
	/* BASE1 reads the condition (r or c) */
	bool reads_cond = false;
	/* Prepared for BASE1: source 1's slot in the registers, the even one
	 * of a pair picked by c (then s1_pair = 1: the odd one when the
	 * condition holds); s1_special: a register above 0x27 (read_reg) */
	uint8_t s1_slot = 0, s1_pair = 0;
	bool s1_special = false;
	bool bang = false, cn9 = false, sets_status = false;
	uint8_t f_by_sign[2] = {}, cin_by_sign[2] = {};

	C82PpXfer xfer[2];
	/* Prepared: the memory accesses in their order (global, then local)
	 * and the transfers' register writes in theirs (local, then global) */
	uint8_t mems = 0, writes = 0;
	uint8_t mem_order[2] = {}, write_order[2] = {};

	/* Format A: oper (bits 60-57) and the D-register fields */
	uint8_t oper = 0, src3 = 0, dst2 = 0, src4 = 0;
	/* Format E: the operation (bits 43-39) */
	uint8_t misc = 0;
	/* A move or field move by the global unit (formats 2, 3, 7, 8) */
	bool move = false, fmove = false;
	uint8_t move_dst = 0, move_src = 0;
	/* Field move: item, sign extension, size, replicate (D) */
	uint8_t itm = 0, fsize = 0;
	bool fsign = false, replicate = false;
	/* Prepared for BASE1: the F code's minterms for each sign as masks
	 * of 32 bits: F0, F0^F2, F4, F4^F6, F1, F1^F3, F5, F5^F7 (A = 0, then
	 * A = 1; B and C pick) */
	uint32_t f_masks[2][8] = {};

	uint8_t dst_code() const { return static_cast<uint8_t>(dstbank << 3 | dst); }
	uint8_t src1_code() const { return static_cast<uint8_t>(s1bank << 3 | src1); }
};

C82PpInsn c82_pp_decode(uint64_t word);

/* What the common cycles (C82Pp::steps) and the execute stage's handlers
 * read of an instruction, made from its C82PpInsn: one cache line, so that
 * a program's loops stay in the first-level cache */
struct alignas(64) C82PpOp {
	/* The cache's: the address (8-byte aligned; 1: none), the plain memory
	 * the word was fetched from and its bytes there */
	uint32_t addr = 1;
	uint8_t form = 0, xfers = 0;
	bool illegal = false;
	int8_t g_change = -1;
	const uint8_t *host = nullptr;
	uint64_t raw = 0;
	/* The address stage of each transfer (C82PpXfer): slots, write-back,
	 * shift, offset; subtract and pre-index (0 or 1) */
	struct Agen {
		uint8_t base_slot = 0, index_slot = 0, wb = 0xFF, shift = 0;
		uint16_t offset = 0;
		uint8_t neg = 0, pre = 0;
	} agen[2];
	/* The first transfer for the execute stage: register, size */
	uint8_t reg = 0, msize = 0;
	/* BASE1 (C82PpInsn): condition, source 1's slot (the even one of a
	 * pair picked by c), src2, destination code, F and the carry-in by
	 * sign, the carry-in's choice, the protected bits (N C V Z as bits
	 * 3-0); the flags below; the immediate */
	uint8_t cond = 0, s1_slot = 0, src2 = 0, dst = 0;
	uint8_t f[2] = {}, cin_by_sign[2] = {}, cin_sel = 0, prot = 0;
	/* The first transfer's sign extension and local access over the
	 * global port; port A forced to 0; BASE1 reads the condition; r;
	 * src2 is the immediate; sets the status; %!; the odd register of
	 * the pair when the condition holds; code 9's inverted C(n) */
	enum : uint16_t {
		SIGN = 1, DIVERT = 2, A_ZERO = 4, READS_COND = 8, R = 16, SRC2_IMM = 32, SETS_STATUS = 64, BANG = 128,
		S1_PAIR = 256, CN9 = 512
	};
	uint16_t flags = 0;
	uint32_t imm = 0;

	bool has(uint16_t flag) const { return flags & flag; }
	uint32_t protect() const { return static_cast<uint32_t>(prot) << 28; }
};

static_assert(sizeof(C82PpOp) == 64);

C82PpOp c82_pp_op(const C82PpInsn &in);

/* Everything a PP keeps, for save states. Plain data. */
struct C82PpState {
	/* Registers by code (table 7-1): a0-a15, x, d */
	std::array<uint32_t, 0x28> r;
	uint32_t sr, mf, pc, ipa, ipe, iprs, inten, intflg, lctl;
	std::array<uint32_t, 3> lc, lr, ls, le;
	uint32_t comm_hs, q_visible_lo, q_visible_hi;
	/* The pipeline: the instructions in the address and execute stages */
	std::array<uint64_t, 2> stage_word;
	std::array<uint32_t, 2> stage_addr, stage_ea0, stage_ea1;
	std::array<uint8_t, 2> stage_kind, stage_extra;
	/* Pseudo-instructions: sequence, step, vector bit, saved addresses */
	uint8_t seq, seq_step, seq_bit, halted, in_reset, illegal, faulted, fault_type;
	uint32_t seq_p1, seq_p2, seq_p3, reset_ipa, reset_ipe, reset_pc;
	uint64_t cycles, reset_done;
};

/* A parallel processor's registers below 0x28 (table 7-1): where each
 * reads (ZERO: the reserved codes read 0; e = sp), where each is written
 * (SINK: the reserved and read-only codes, never read), the bits a write
 * keeps (sp: its two LSBs wired to 0) and what it marks written */
namespace c82_pp_regs {

/* ZERO and SINK, then two slots holding dba and pba (§ 8.8.1.5), read
 * by the address stage, never written */
constexpr unsigned ZERO = 0x28, SINK = 0x29, DBA = 0x2A, PBA = 0x2B, SLOTS = 0x2C;

constexpr bool
write_ignored(unsigned c)
{
	return c == 5 || c == 7 || c == 0xD || c == 0xF || (c >= 0x13 && c <= 0x17) || (c >= 0x1B && c <= 0x1F);
}

constexpr std::array<uint8_t, 0x28> READ_SLOT = [] {
	std::array<uint8_t, 0x28> t{};
	for (unsigned c = 0; c < 0x28; c++)
		t[c] = static_cast<uint8_t>(c == 7 || c == 0xF ? ZERO : c == 0xE ? 6 : c);
	return t;
}();

constexpr std::array<uint8_t, 0x28> WRITE_SLOT = [] {
	std::array<uint8_t, 0x28> t{};
	for (unsigned c = 0; c < 0x28; c++)
		t[c] = static_cast<uint8_t>(write_ignored(c) ? SINK : c == 0xE ? 6 : c);
	return t;
}();

constexpr std::array<uint32_t, 0x28> WRITE_KEEP = [] {
	std::array<uint32_t, 0x28> t{};
	for (unsigned c = 0; c < 0x28; c++)
		t[c] = c == 6 || c == 0xE ? ~3u : ~0u;
	return t;
}();

constexpr std::array<uint64_t, 0x28> WRITE_MARK = [] {
	std::array<uint64_t, 0x28> t{};
	for (unsigned c = 0; c < 0x28; c++)
		t[c] = write_ignored(c) ? 0 : c == 6 || c == 0xE ? (1ull << 6 | 1ull << 0xE) : 1ull << c;
	return t;
}();

/* Table 8-36, for the status bits N C V Z (bits 3-0 of `ncvz`) */
constexpr bool
holds(unsigned code, unsigned ncvz)
{
	const bool n = (ncvz >> 3) & 1, c = (ncvz >> 2) & 1, v = (ncvz >> 1) & 1, z = ncvz & 1;

	switch (code & 15) {
	case 0:
		return true;
	case 1:
		return !n && !z;
	case 2:
		return !c || z;
	case 3:
		return c && !z;
	case 4:
		return n != v;
	case 5:
		return n != v || z;
	case 6:
		return n == v;
	case 7:
		return n == v && !z;
	case 8:
		return c;
	case 9:
		return !c;
	case 10:
		return z;
	case 11:
		return !z;
	case 12:
		return v;
	case 13:
		return !v;
	case 14:
		return n;
	default:
		return !n;
	}
}

/* Bit `code` of entry `ncvz`: whether the condition holds */
constexpr std::array<uint16_t, 16> CONDITIONS = [] {
	std::array<uint16_t, 16> t{};

	for (unsigned ncvz = 0; ncvz < 16; ncvz++)
		for (unsigned code = 0; code < 16; code++)
			if (holds(code, ncvz))
				t[ncvz] = static_cast<uint16_t>(t[ncvz] | 1u << code);
	return t;
}();

} /* namespace c82_pp_regs */

/* A parallel processor of the TMS320C82 (TI SPRU110 for the TMS320C80's
 * PP, the same on the C82 but for its memory map: SPRU154 § 1.2, 1.5; and
 * the d0 bits T and U of SPRS048 fig. 24). The three-stage pipeline (fetch,
 * address, execute) is emulated cycle by cycle, so that the delay slots,
 * the loop logic in the fetch stage and the address registers seen one
 * instruction late by the address stage behave as § 5.2 says. Timing,
 * HEURISTIC: one instruction a cycle; one more when a local access goes to
 * the global port (§ 6.3.2); a direct external access takes the minimum of
 * table 6-1 (8 cycles for a store, 11 for a load); no instruction cache, no
 * crossbar contention. */
class C82Pp {
public:
	/* Interrupt flags (fig. 9-2) */
	enum : uint32_t {
		INT_TASK = 1u << 14, INT_PTQ = 1u << 17, INT_PTERR = 1u << 18, INT_PTEND = 1u << 19,
		INT_MPMSG = 1u << 20, INT_PP0MSG = 1u << 24
	};

	C82Pp(Tms320c82 &chip, unsigned number);
	~C82Pp();
	/* Its pipeline points into itself */
	C82Pp(const C82Pp &) = delete;
	C82Pp &operator=(const C82Pp &) = delete;

	/* A hardware reset: halted, the reset sequence armed (§ 9.6) */
	void power_up();
	/* Runs until the cycle `until`. In line, the two commonest slices:
	 * a parked PP with nothing to make (parked_run's case), and blocks
	 * resumed in host code where the last slice stopped them (resume) */
	void run(uint64_t until)
	{
		if (m_park.on) {
			if (until <= m_park.until)
				return;
			if (!m_park.boards || until <= m_park.next_read) {
				m_park.until = until;
				return;
			}
			if (until < m_park.quiet) {
				count_reads(until);
				return;
			}
		} else if (m_resume && until > m_cycles) {
			const int blocks = resume(until);

			if (blocks != DONE)
				run_on(until, blocks);
			return;
		}
		run_on(until, NOT_RUN);
	}
	uint64_t cycles() const { return m_cycles; }
	/* Parked (m_park.on), whether run(until) only notes `until` or
	 * counts reads ahead (it reads no board, wakes not) */
	bool calm(uint64_t until) const
	{
		return until <= m_park.until || !m_park.boards || until <= m_park.next_read || until < m_park.quiet;
	}
	/* The board is about to be reached (Tms320c82::board_reached):
	 * the reads counted ahead (count_reads) told to it, the promise
	 * dropped */
	void board_reached()
	{
		if (m_counted.count)
			tell_counted();
		m_park.quiet = 0;
	}

	/* A command word from the MP (SPRU110 § 3.2.2; SPRU109 p. 10-70),
	 * this PP being designated */
	void command(uint32_t word, uint64_t now);
	/* A reset of this PP, as from the TC (§ 9.6.1) */
	void reset(uint64_t now);
	/* The PPERROR bits (SPRU109 fig. 3-4): halt latch, illegal
	 * instruction, fault type */
	bool halt_latch(uint64_t now);
	bool illegal() const { return m_illegal; }
	bool fault_type() const { return m_fault_type; }

	/* From the transfer controller: the request ended (Q cleared, PTEND or
	 * PTERR), or an entry with its I bit done (PTEND) */
	void list_ended(bool error);
	void entry_done()
	{
		disturb();
		m_intflg |= INT_PTEND;
	}
	/* comm H, which the TC watches (§ 12.2.4) */
	bool high_priority() const { return m_comm_hs >> 31; }
	/* An interrupt flag set by another processor */
	void post(uint32_t bits)
	{
		disturb();
		m_intflg |= bits;
	}

	/* A wait of the program, found by the board in its code: the address of
	 * its first instruction and its words (none: an empty span). Once a
	 * pass through it, from that instruction back to it, leaves this PP
	 * exactly as it was (all its state but the cycle count), with no store,
	 * no request, no command and no fault, and reads only what nobody
	 * changes meanwhile, the PP is parked: not run, its passes counted. It
	 * is woken where it would be (wake) as soon as anything it reads is
	 * written or anything is sent to it; the emulated results are the
	 * same as running it */
	void set_wait(uint32_t pc, std::span<const uint64_t> words);
	/* Whether a write of `bytes` at `addr` touches what it reads while
	 * parked or measuring a pass (its reads of the board are made while
	 * parked, each where it falls: parked_run) */
	bool watches(uint32_t addr, uint32_t bytes) const;
	/* Something it reads or holds is about to change: parked, it is
	 * brought to the end of the last slice it was given; measuring, the
	 * pass is dropped */
	void disturb();
	/* The internal watches of a parked or measuring PP: the RAM ranges
	 * (offsets in the chip's RAM, or external addresses) */
	struct Watch {
		uint32_t lo = 0, hi = 0;
		bool internal = false;
	};
	std::span<const Watch> watched() const
	{
		return { m_park.watch.data(), (m_park.on || m_park.measuring) ? m_park.watches : 0u };
	}

	void save(C82PpState &out) const;
	void load(const C82PpState &in);

	/* For the tests: the blocks in host code (c82_pp_host.cpp) or in C++
	 * (run_block) */
	void use_host_code(bool on);
	/* `bytes` bytes from `addr` of external memory are about to be
	 * written (Tms320c82::code_written, for a page whose Page::code is
	 * set): the blocks fetching in a line of 64 bytes written are dropped */
	void code_written(uint32_t addr, uint32_t bytes)
	{
		const uint64_t *const lines = m_code_lines[addr >> 20];
		const uint32_t line = (addr & 0xFFFFF) >> 6;

		/* Most often: a write within one line that no block fetches */
		if (!lines || ((addr & 63) + bytes <= 64 && !((lines[line >> 6] >> (line & 63)) & 1)))
			return;
		drop_code(addr, bytes);
	}
	/* The MP's marks of the page of `addr` cleared (C82Mp::
	 * flush_blocks): the blocks fetching there dropped */
	void code_page_cleared(uint32_t addr);
	/* For the tests */
	uint32_t reg(unsigned code)
	{
		disturb();
		return read_reg(code);
	}
	void set_reg(unsigned code, uint32_t value)
	{
		disturb();
		write_reg(code, value);
	}
	uint32_t pc()
	{
		disturb();
		return m_pc;
	}

private:
	/* What sits in the address or execute stage (§ 5.2) */
	enum Kind : uint8_t { EMPTY, INSN, PS1, PS2, PS3, PS4, PS5, RS1, RS2 };
	/* A pseudo-instruction sequence: an interrupt or the reset's */
	enum Seq : uint8_t { NONE, INTERRUPT, RESET_SEQ };
	static const C82PpInsn NO_INSN;
	static const C82PpOp NO_OP;
	struct Stage {
		Kind kind = EMPTY;
		uint32_t addr = 0;
		uint64_t word = 0;
		/* Its decoded instruction: the cache's (decoded), or its own copy
		 * when the cache's place is taken while it is in the pipeline */
		const C82PpInsn *insn = &NO_INSN;
		const C82PpOp *op = &NO_OP;
		/* The addresses the address stage made, for each transfer */
		std::array<uint32_t, 2> ea{};
		/* One more cycle in the execute stage: a local access diverted to
		 * the global port (§ 6.3.2) */
		bool divert = false;
		/* An instruction that stalls the PP at decode (kind INSN, illegal) */
		bool stop = false;
		C82PpInsn own;
		C82PpOp own_op;
	};
	/* The register writes the address stage leaves for the end of the
	 * cycle: one per transfer */
	struct Pending {
		unsigned count = 0;
		std::array<uint8_t, 2> code{};
		std::array<uint32_t, 2> value{};
	};
	/* Data unit results, written by priority (§ 8.11) */
	struct DuOut {
		bool alu = false, second = false, status = false;
		uint8_t alu_dst = 0, second_dst = 0;
		uint32_t alu_value = 0, second_value = 0;
		uint32_t sr = 0, mf = 0;
		bool mf_set = false;
		/* Bit detection found nothing */
		bool none = false;
	};

	/* c82_pp.cpp: the pipeline, registers, interrupts, memory */
	void cycle();
	/* c82_pp_ops.cpp: the common case of cycle(), cycle after cycle: false
	 * when the first one needs cycle() */
	bool steps(uint64_t until);
	uint32_t next_fetch(uint32_t pc, std::array<uint32_t, 3> &lc) const;
	void loops_changed();
	void address_stage(Stage &s, Pending &pending);
	[[gnu::noinline, gnu::cold]] void execute_pseudo(Stage &s);
	/* The cache's place for `addr`, `word` decoded there */
	unsigned decoded(uint32_t addr, uint64_t word, const uint8_t *host);
	uint32_t read_reg(unsigned code);
	void write_reg(unsigned code, uint32_t value);
	/* read_reg and write_reg, the registers below 0x28 by table */
	uint32_t get(unsigned code)
	{
		return code < 0x28 ? m_r[c82_pp_regs::READ_SLOT[code]] : read_reg(code);
	}
	/* Always in line */
	[[gnu::always_inline]] void put(unsigned code, uint32_t value)
	{
		if (code < 0x28) {
			m_r[c82_pp_regs::WRITE_SLOT[code]] = value & c82_pp_regs::WRITE_KEEP[code];
			m_written |= c82_pp_regs::WRITE_MARK[code];
		} else {
			write_reg(code, value);
		}
	}
	uint32_t read_comm() const;
	void write_comm(uint32_t value);
	void branch(uint32_t target, uint32_t bits);
	void send_command(uint32_t word);
	bool condition(unsigned code) const { return (c82_pp_regs::CONDITIONS[m_sr >> 28] >> (code & 15)) & 1; }
	bool load(uint32_t addr, unsigned size, uint32_t &value, unsigned &cycles);
	bool store(uint32_t addr, unsigned size, uint32_t value, unsigned &cycles);
	/* `host`: the plain memory the word was read from, or null */
	bool fetch_word(uint32_t addr, uint64_t &word, const uint8_t *&host);
	[[gnu::noinline, gnu::cold]] void memory_fault(uint32_t addr, bool data);
	void take_reset();
	uint32_t param_base() const { return 0x01000000u + (m_number << 12); }

	/* c82_pp_ops.cpp: the execute stage (a handler for each form of
	 * C82PpInsn) and the data unit */
	using Exec = void (*)(C82Pp &, const C82PpOp &, const C82PpInsn *, const uint32_t *);
	template <unsigned FORM>
	static void exec(C82Pp &p, const C82PpOp &op, const C82PpInsn *in, const uint32_t *ea);
	template <size_t... K>
	static constexpr std::array<Exec, sizeof...(K)> exec_table(std::index_sequence<K...>);
	static const std::array<Exec, C82PpInsn::FORMS> EXEC;
	void execute(unsigned form, const C82PpOp &op, const C82PpInsn *in, const uint32_t *ea);
	void execute_generic(const C82PpInsn &in, const uint32_t *ea);
	bool load_xfer(const C82PpXfer &t, uint32_t ea, uint32_t &value);
	bool store_xfer(const C82PpXfer &t, uint32_t ea, uint32_t value);
	bool load_op(const C82PpOp &op, uint32_t ea, uint32_t &value);
	bool store_op(const C82PpOp &op, uint32_t ea, uint32_t value);
	void apply(const DuOut &du);
	void data_unit(const C82PpInsn &in, bool cond_true, DuOut &out);
	void mpy_add(const C82PpInsn &in, bool cond_true, DuOut &out);
	void data_unit_base1(const C82PpInsn &in, bool cond_true, DuOut &out);
	template <unsigned CLS, bool ARITH>
	void base1(const C82PpInsn &in, bool cond_true, DuOut &out);
	template <unsigned CLS, bool ARITH, int F>
	uint32_t alu1(const C82PpOp &op, bool cond_true, uint32_t &ncvz) const;
	uint32_t source1(const C82PpInsn &in, bool cond_true);
	uint32_t expand_mf() const;

	Tms320c82 &m_chip;
	const unsigned m_number;
	/* Registers by code below 0x28, then ZERO, SINK, dba and pba
	 * (c82_pp_regs) */
	std::array<uint32_t, c82_pp_regs::SLOTS> m_r{};
	uint32_t m_sr = 0, m_mf = 0, m_pc = 0, m_ipa = 0, m_ipe = 0, m_iprs = 0, m_inten = 0, m_intflg = 0, m_lctl = 0;
	std::array<uint32_t, 3> m_lc{}, m_lr{}, m_ls{}, m_le{};
	/* The enabled loop ends (loops_changed) */
	std::array<uint32_t, 3> m_ends{ 1, 1, 1 };
	/* comm H and S (bits 31, 30); Q comes from the TC (§ 12.2.4) */
	uint32_t m_comm_hs = 0;
	uint64_t m_q_visible = 0;
	/* The address and execute stages, and the one the fetch fills: three
	 * places that take turns (no copy) */
	std::array<Stage, 3> m_stages{};
	Stage *m_sa = &m_stages[0], *m_se = &m_stages[1], *m_sf = &m_stages[2];
	Stage &a() { return *m_sa; }
	Stage &e() { return *m_se; }
	const Stage &a() const { return *m_sa; }
	const Stage &e() const { return *m_se; }
	Seq m_seq = NONE;
	uint8_t m_seq_step = 0, m_seq_bit = 0;
	uint32_t m_seq_p1 = 0, m_seq_p2 = 0, m_seq_p3 = 0;
	/* ipa, ipe and the G and L bits when the reset came (§ 9.6.1) */
	uint32_t m_reset_ipa = 0, m_reset_ipe = 0, m_reset_pc = 0;
	bool m_halted = true, m_in_reset = false, m_illegal = false, m_faulted = false, m_fault_type = false;
	uint64_t m_cycles = 0, m_reset_done = 0;
	/* The cycle being run: what the execute stage wrote, the status bits
	 * it protects, the cycles it adds */
	uint64_t m_written = 0;
	unsigned m_lc_written = 0;
	uint32_t m_protect = 0;
	unsigned m_extra = 0;
	bool m_pc_written = false;
	/* The branch target, and which of its G (2) and L (1) bits count */
	uint32_t m_new_pc = 0, m_new_pc_bits = 0;
	/* Decoded instructions by address (a cache of the emulator, not of
	 * the chip, checked against the word fetched): the lines steps()
	 * reads, then the rest */
	struct Cached {
		uint64_t word = 0;
		C82PpInsn insn;
	};
	std::vector<C82PpOp> m_ops;
	std::vector<Cached> m_decoded;

	/* The wait (set_wait) and the pass being measured or the parking */
	[[gnu::noinline]] void pass_boundary();
	void note_read(uint32_t addr, unsigned size, uint64_t value);
	void note_fetch(uint32_t addr);
	void watch(uint32_t lo, uint32_t hi, bool internal);
	void stop_watching();
	bool parked_run(uint64_t until);
	/* Parked, the pass's one read of the board promised to answer
	 * as in the pass before m_park.quiet (C82Bus::answers_until) and
	 * `until` before it: the reads due before `until` counted, not made,
	 * as parked_run would make them. `until` is the bus time of the
	 * slice's reads (Tms320c82::run gives the PPs the MP's cycles) */
	void count_reads(uint64_t until)
	{
		uint64_t pass = m_park.next_pass, count = 0;

		do {
			pass += m_park.period;
			count++;
		} while (pass + m_park.board[0].at < until);
		if (!m_counted.count) {
			m_counted.addr = m_park.board[0].addr;
			m_counted.size = m_park.board[0].size;
			m_counted.value = m_park.board[0].value;
		}
		m_counted.count += count;
		m_counted.at = until;
		m_park.next_pass = pass;
		m_park.next_read = pass + m_park.board[0].at;
		m_park.until = until;
	}
	[[gnu::noinline]] void tell_counted();
	void wake();
	/* A read of the board in the pass: where (the start of its cycle, from
	 * the pass's), what, and its answer */
	struct BoardRead {
		uint32_t at = 0, addr = 0;
		uint8_t size = 0;
		uint64_t value = 0;
	};
	struct Park {
		static constexpr unsigned WATCHES = 8, BOARD_READS = 4, CYCLES = 64;

		bool measuring = false, spoiled = false, on = false;
		/* Catching up (wake): the board's answers are those it gave */
		bool replaying = false;
		/* The PP at the wait's first instruction: where the pass began
		 * (state[at]) and where it is now (the other; both cleared by
		 * set_wait, their padding staying 0 since save() writes only the
		 * fields, as the cleared copy compared before) */
		std::array<C82PpState, 2> state;
		unsigned at = 0;
		/* Parked: its cycle count there, the cycles of a pass, the end of
		 * the last slice it was given; the first read of the board at or
		 * after `until`: its cycle, its pass's start, its index in `board`
		 * (kept up to date, each read lying at the start of one of the
		 * pass's cycles) */
		uint64_t base = 0, period = 0, until = 0, next_read = 0, next_pass = 0;
		unsigned next_k = 0;
		/* The bus time before which the pass's one read of the
		 * board answers as in the pass (count_reads), 0 for none */
		uint64_t quiet = 0;
		/* The cycles of the pass measured (at most CYCLES) */
		unsigned cycles = 0;
		std::array<Watch, WATCHES> watch{};
		unsigned watches = 0;
		std::array<BoardRead, BOARD_READS> board{}, replay{};
		unsigned boards = 0, replays = 0;
	};
	uint32_t m_wait_pc = ~0u;
	std::vector<uint64_t> m_wait_words;
	/* The same words as they lie in memory */
	std::vector<uint8_t> m_wait_bytes;
	Park m_park{};
	/* Reads of the board counted, not made yet: what, how many,
	 * the bus time of the last */
	struct Counted {
		uint64_t count = 0, at = 0, value = 0;
		uint32_t addr = 0;
		uint8_t size = 0;
	};
	Counted m_counted{};

	/* c82_pp_block.cpp: prepared blocks. A block is a run of
	 * cycles of the pipeline from one state (pc with G and L, the
	 * instructions in the address and execute stages): step k fetches the
	 * word at f0 + 8k, makes the addresses of the instruction fetched at
	 * step k - 1 and executes the one fetched at k - 2, as cycle() and
	 * steps() do, with what does not depend on the values worked out once:
	 * the stages' instructions, pc, the loop ends, which registers a step
	 * touches. Each fetched word is checked against memory at its step */
	struct alignas(64) BlockStep {
		/* The word fetched, its bytes as in memory */
		uint64_t raw = 0;
		/* The execute stage's instruction */
		const C82PpOp *eop = nullptr;
		const C82PpInsn *ein = nullptr;
		/* The address stage's transfers, and where their write-backs go
		 * (put(): the slot, 0xFF for none, and the bits kept) */
		C82PpOp::Agen agen[2];
		uint32_t wb_keep[2] = {};
		uint8_t wb_slot[2] = { 0xFF, 0xFF };
		uint16_t flags = 0;
		uint8_t form = 0, axfers = 0;
		/* The execute stage's memory accesses, looked at first: transfer
		 * and size */
		uint8_t mems = 0;
		uint8_t mem_x[2] = {}, mem_size[2] = {};
		/* The accesses that are stores (bit m) */
		uint8_t mem_store = 0;
		/* eint or dint fetched here (-1: neither) */
		int8_t g_change = -1;
	};
	/* BlockStep::flags: the fetch is an enabled loop end; the execute stage
	 * reaches memory; reads or writes a special register (the members
	 * read_reg and write_reg read are made up to date first); may write pc;
	 * may write a register an address stage writes back in the same cycle;
	 * the block ends after this step */
	enum : uint16_t { LOOP_END = 1, MEMORY = 2, SPECIAL = 4, BRANCH = 8, CAREFUL = 16, LAST = 32 };
	struct Block {
		/* The first fetch (8-byte aligned), pc's two LSBs, the steps */
		uint32_t f0 = 0, bits = 0, n = 0;
		/* The stages it starts with: addresses and words (bytes as in
		 * memory) */
		uint32_t ipa = 0, ipe = 0;
		uint64_t a_raw = 0, e_raw = 0;
		/* The host memory of the fetches */
		const uint8_t *host = nullptr;
		/* The enabled loop ends within it (1: none), as translated; the
		 * bytes of its fetches (0: pc's L set, no loop logic) */
		std::array<uint32_t, 3> ends{ 1, 1, 1 };
		uint32_t span = 0;
		bool dead = false;
		/* Its fetches are the wait's words (after the first): it
		 * counts the cycles of a pass measured and reaches memory by the
		 * handlers, which note what it reads (wait blocks) */
		bool wait = false;
		Block *next = nullptr;
		std::vector<BlockStep> steps;
		/* Its instructions: [0] the execute stage's at the start, [1] the
		 * address stage's, [2 + k] the one fetched at step k */
		std::vector<C82PpOp> ops;
		std::vector<C82PpInsn> insns;
		/* After step k, the block that followed last: from pc, its step;
		 * for the host code, where its step starts (null: none), its
		 * fetched words and where it reads the execute stage's addresses */
		struct Successor {
			Block *block = nullptr;
			uint32_t pc = 0, k = 0;
			const uint8_t *code = nullptr, *host = nullptr;
			int64_t slot = 0;
		};
		std::vector<Successor> successors;
		/* Its host code (c82_pp_host.cpp): where each step starts, none
		 * when empty */
		std::vector<uint32_t> entries;

		uint32_t addr(int i) const { return i >= 0 ? (f0 + 8 * static_cast<uint32_t>(i)) | bits : i == -1 ? ipa : ipe; }
		uint64_t raw(int i) const { return i >= 0 ? steps[static_cast<unsigned>(i)].raw : i == -1 ? a_raw : e_raw; }
	};
	/* Where blocks left the pipeline: pc; the instruction of index i of
	 * block b in the address stage, i - 1 in the execute stage (b null:
	 * those of m_sa and m_se), the addresses the latter was given; the
	 * cycles left in the slice */
	struct Flow {
		uint32_t pc = 0;
		Block *b = nullptr;
		int i = 0;
		uint32_t e_ea[2] = {};
		int64_t left = 0;
	};
	/* An instruction in a stage of a flow */
	struct InStage {
		uint32_t addr;
		uint64_t raw;
		const C82PpOp *op;
		const C82PpInsn *in;
	};
	enum class BlockEnd : uint8_t { NEXT, STOP, CHANGED };
	int run_blocks(uint64_t until);
	/* run()'s loop, `first` what run_blocks gave already (resume) or
	 * NOT_RUN; run_blocks' loop, `first` the end of a block resumed (or
	 * null); what the host code of a resumed block ended with, as run_blocks
	 * would have taken it */
	static constexpr int NOT_RUN = 2, DONE = 3;
	/* What the host code returns at such a slice end (c82_pp_host.cpp) */
	static constexpr uint64_t END_DONE = 5;
	void run_on(uint64_t until, int first);
	int blocks_on(uint64_t until, const BlockEnd *first);
	bool after_block(BlockEnd end);
	int resumed(uint64_t out, uint64_t until);
	/* An instruction in the address (0) or execute (1) stage of a flow
	 * (in line) */
	InStage in_stage(const Flow &f, int which) const
	{
		if (!f.b) {
			const Stage &s = which ? *m_se : *m_sa;

			return { s.addr, __builtin_bswap64(s.word), s.op, s.insn };
		}

		const int i = f.i - which;

		return { f.b->addr(i), f.b->raw(i), &f.b->ops[static_cast<unsigned>(i + 2)], &f.b->insns[static_cast<unsigned>(i + 2)] };
	}
	Block *find_block(const Flow &f, unsigned &k);
	Block *translate(const Flow &f);
	bool ends_hold(const Block &b) const;
	BlockEnd run_block(Block &b, unsigned k, Flow &f, uint64_t until);
	void materialize();
	void fill(Stage &s, uint32_t addr, uint64_t raw, const uint32_t *ea);
	void flush_blocks();
	/* The lines of 64 bytes of external memory that blocks in host
	 * code fetch from (by page of 1 MB, 256 x 64 bits), marked in the MP's
	 * too (C82Mp::mark_lines: Page::code set, every write there reaching
	 * Tms320c82::code_written): their fetched words are not checked */
	void mark_lines(const Block &b);
	void drop_code(uint32_t addr, uint32_t bytes);
	std::array<uint64_t *, 4096> m_code_lines{};
	std::vector<std::unique_ptr<uint64_t[]>> m_line_store;
	/* The members brought up to date when blocks left them behind */
	void settle_flow()
	{
		if (m_flowing)
			materialize();
	}
	struct Inside {
		Block *block = nullptr;
		unsigned k = 0;
	};
	std::vector<std::unique_ptr<Block>> m_blocks;
	size_t m_block_steps = 0;
	std::vector<Block *> m_entry = std::vector<Block *>(4096);
	std::vector<Inside> m_inside = std::vector<Inside>(4096);
	/* Blocks stopped at the end of a slice: the pipeline is m_flow's,
	 * not m_sa's and m_se's (m_pc and m_cycles are up to date) */
	bool m_flowing = false;
	/* On alone (run_alone, Tms320c82::run_long): its host code stops
	 * before a step another unit could see */
	bool m_alone = false;
	Flow m_flow;
	/* The slice ended in host code before step f.i + 1 of the live
	 * block f.b, which it noted (m_flow, m_pc, m_cycles, m_flowing: write_
	 * native's slice ends): where that step's code starts, null otherwise
	 * (cleared wherever m_flowing is, and by run_blocks). run() resumes it
	 * there after run_blocks' checks that can change between slices; the
	 * room check made after each block written (m_full) */
	const uint8_t *m_resume = nullptr;
	bool m_full = false;
	uint64_t (*m_nat_enter)(void *, const uint8_t *, const uint8_t *, int64_t) = nullptr;
	/* The first block run as run_blocks would run it from m_flow (run_native):
	 * DONE when its host code ran to the end of the slice */
	int resume(uint64_t until)
	{
		Flow &f = m_flow;

		if (((f.pc & 2) && (m_intflg & m_inten & ~1u)) || (f.pc & ~7u) == m_wait_pc || m_halted || m_illegal ||
		    m_faulted || m_in_reset || m_park.measuring || m_full)
			return NOT_RUN;

		const unsigned se = ((static_cast<unsigned>(f.i) + 1) & 1) ^ 1;
		const uint8_t *const at = m_resume;

		m_flowing = false;
		m_resume = nullptr;
		m_nat_ea[se][0] = f.e_ea[0];
		m_nat_ea[se][1] = f.e_ea[1];
		m_nat_until = until;
		m_extra = 0;

		const uint64_t out = m_nat_enter(this, at, f.b->host, static_cast<int64_t>(until - m_cycles));

		return (out & 0xFF) == END_DONE ? DONE : resumed(out, until);
	}

	/* c82_pp_host.cpp: the blocks in host code (x86-64), made with them.
	 * The steps' addresses go to m_nat_ea[k & 1], their write-backs to
	 * m_nat_back; the loop logic's copy of the counters and next fetch;
	 * the end of the slice, the cycles left when the code ends */
	void write_native(Block &b);
	void leave(Flow &f, Block &b, int i, uint32_t pc, const uint32_t *ea, int64_t left) const;
	BlockEnd run_native(Block &b, unsigned k, Flow &f, uint64_t until);
	/* In line in its two callers (c82_pp_host.cpp) */
	[[gnu::always_inline]] inline BlockEnd native_end(uint64_t out, Flow &f);
	static uint64_t plain(C82Pp &p, uint32_t addr);
	/* plain() for a store, 0 when alone (the block ends before it) */
	static uint64_t plain_store(C82Pp &p, uint32_t addr);
	/* On alone, the MP parked and the other PP quiet (Tms320c82::
	 * run_long): its blocks in host code from where it stands, as run()
	 * makes them, until `limit`; it stops where run() would go on by the
	 * interpreter (or reset, halt), and before a step whose access or
	 * register write another unit could see (m_alone) */
	void run_alone(uint64_t limit);
	friend class Tms320c82;
	/* c82_pp_host_exec.cpp: the execute stage of the commonest
	 * forms written in host code (x86-64), the handler for the rest. The
	 * accesses of step k looked at before it: their offsets in the on-chip
	 * RAM, -1 for plain external memory (the handler's), the end of the
	 * block for anything else (`stops`); then its execute stage (false:
	 * the handler is called); the slow paths written after the steps */
	void write_memory(C82PpCode &c, const BlockStep &st, unsigned k, bool handler, std::vector<C82PpSlow> &slow,
	                  std::vector<uint32_t> &stops);
	bool write_exec(C82PpCode &c, const BlockStep &st, unsigned k, uint32_t pc, uint32_t ipa, uint32_t ipe, bool frame,
	                std::vector<C82PpSlow> &slow);
	void write_frame(C82PpCode &c, uint32_t pc, uint32_t ipa, uint32_t ipe, uint32_t protect);
	/* What write_exec does for the step: written in host code, and
	 * whether it reads or writes comm */
	enum : unsigned { PLAN_WRITTEN = 1, PLAN_COMM_READ = 2, PLAN_COMM_WRITE = 4 };
	unsigned exec_plan(const BlockStep &st) const;
	void write_slow(C82PpCode &c, const C82PpSlow &s, std::vector<uint32_t> &stops);
	static void loop_before(C82Pp &p, uint32_t pc);
	static uint64_t loop_after(C82Pp &p, uint32_t pc);
	/* For the special registers written in host code: comm read
	 * before the step's execute stage (into m_nat_comm), comm written, the
	 * loop ends after a write to lctl, an le or an lrs/lrse */
	static void comm_read(C82Pp &p);
	static void comm_written(C82Pp &p, uint32_t value);
	static void loops_written(C82Pp &p);
	std::unique_ptr<C82PpCode> m_code;
	bool m_native = true;
	uint32_t m_nat_ea[2][2] = {}, m_nat_back[2] = {};
	/* N C V Z (bits 31-28) by the host's SF ZF CF OF (rflags bits 7 6
	 * 0 11), for the host code's ALU */
	std::array<uint32_t, 0x8C2> m_ncvz{};
	std::array<uint32_t, 3> m_nat_lc{};
	uint32_t m_nat_next = 0;
	/* comm as comm_read() found it */
	uint32_t m_nat_comm = 0;
	/* Tms320c82's map of the on-chip RAM (internal_offset, made once
	 * by its constructor) with the MP's data cache, which the PPs never
	 * reach, marked as no RAM (-1), for the host code's accesses */
	std::array<int16_t, 0x2000> m_nat_internal{};
	uint64_t m_nat_until = 0;
	int64_t m_nat_left = 0;
	/* Where the host code ended: the block (it goes from one to the next
	 * by itself), pc after a step */
	Block *m_nat_block = nullptr;
	uint32_t m_nat_pc = 0;
};

} /* namespace odin */

#endif /* ODIN_C82_PP_HPP */
