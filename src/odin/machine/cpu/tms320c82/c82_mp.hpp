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

#ifndef ODIN_C82_MP_HPP
#define ODIN_C82_MP_HPP

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <unordered_map>
#include <vector>

#include "odin/machine/core/host_code.hpp"
#include "odin/machine/cpu/tms320c82/c82_fpu.hpp"

namespace odin {

class Tms320c82;
class C82MpCode;
struct C82MpSlow;

/* An instruction of the master processor, its fields as TI SPRU109 ch. 10
 * codes them (fig. 10-1, tables 10-8 to 10-10, and each instruction's
 * page) */
struct C82Insn {
	enum Op : uint8_t {
		RESERVED,	/* a missing opcode (table notes: reserved) */
		ILLOP,		/* illop0, illopF: trap 38 (p. 10-106) */
		TRAP, CMND, RDCR, SWCR, BRCR, SHIFT, LOGIC, LOAD, STORE, DCACHE,
		BSR, JSR, BBZ, BBO, BCND, CMP, ADD, SUB,
		VECTOR, FADD, FSUB, FMPY, FDIV, FRND, FCMP, FSQRT, LMO, RMO, ESTOP
	};
	enum Form : uint8_t { SHORT, REGISTER, LONG };

	Op op = RESERVED;
	Form form = SHORT;
	/* 4, or 8 with a long immediate: bits 21, 20 and 12 set (§ 4.1.1) */
	uint8_t length = 4;
	uint8_t dest = 0, src2 = 0, src1 = 0;
	/* The short immediate, extended as the instruction says, or the long
	 * one */
	uint32_t imm = 0;
	/* Branches: annul (A); trap: emulation (E); add, sub, ld.u: unsigned
	 * (U); dcache: clear present flags too (F) */
	bool annul = false, emulation = false, is_unsigned = false, flush = false;
	/* Loads and stores: size as 2^size bytes, modify the base (M), scale
	 * the offset (S), direct external access (D) */
	uint8_t size = 2;
	bool modify = false, scale = false, dea = false;
	/* Shifts: merge field, endmask, rotate, i, n (table 10-25) */
	uint8_t merge = 0, endmask = 0, rotate = 0;
	bool invert = false, right = false;
	/* Boolean operations: the 4-bit truth table, bits 18-15 */
	uint8_t logic = 0;
	/* Floating point: precisions and rounding (RM for frnd) */
	uint8_t p1 = 0, p2 = 0, pd = 0, rm = 0;
	/* Vector: operation (bits 15-13), accumulator (bits 16, 11), parallel
	 * memory operation (bits 10, 6: 1 vst, 2 vld0, 3 vld1), its size
	 * (P, bit 9), Z (bit 8), its register (bits 31-27) */
	uint8_t vop = 0, acc = 0, mem = 0, memreg = 0;
	bool mem_double = false, zero = false;
};

C82Insn c82_decode(uint32_t word, uint32_t next);

class C82Mp;
struct C82MpOp;
/* A handler: the MP, the instruction, its address */
using C82MpExec = void (*)(C82Mp &, const C82MpOp &, uint32_t);

/* An instruction prepared for its handler (an emulator's cache, as
 * C82Insn): what it reads, ready; the handler made for its kind and form.
 * Kept for the word and the long immediate it was made from, both checked
 * at each step. */
struct C82MpOp {
	C82MpExec exec = nullptr;
	/* The word, and the long immediate above it (0 for none) */
	uint64_t key = 0;
	/* source1 when it is an immediate (short, extended, or long) */
	uint32_t imm = 0;
	/* The handler's own: a mask, FPST's fixed bits */
	uint32_t aux = 0;
	uint8_t dest = 0, src1 = 0, src2 = 0, length = 4;
	uint8_t a = 0, b = 0, c = 0, d = 0;
};

/* Everything the master processor keeps between two instructions, for
 * save states. Plain data. */
struct C82MpState {
	std::array<uint32_t, 32> r;
	std::array<uint64_t, 4> acc;
	uint32_t ip, pc;
	uint32_t epc, eip, config, intpen, ie, fpst, pktreq, tscale;
	uint32_t fltop, fltadr, flttag, fltdtl, fltdth, sysstk, systmp, mpc, mip;
	uint32_t ecomcntl, anastat, brk1, brk2, ilru, dlru;
	uint32_t in0p, in1p, outp;
	std::array<uint32_t, 16> itag, dtag;
	uint64_t cycles, timer_zero;
	uint8_t user, lint4, self_message, slot, fault;
	uint8_t pad[3];
};

/* The master processor of the TMS320C82 (TI SPRU109 for the TMS320C80, the
 * same MP: SPRU154 ch. 1). Timing, HEURISTIC: one cycle an instruction, one
 * more for a long immediate (§ 4.1.1, 4.3) and for the nop of an annulled
 * branch (§ 4.1.2.1); no cache, no scoreboard stall, no wait state; a
 * floating-point result is written before the next instruction, as the
 * scoreboard makes it look to the program (§ 2.2.3, 4.2). */
class C82Mp {
public:
	/* Control register numbers (table 2-1) */
	enum : uint32_t {
		EPC = 0x0000, EIP = 0x0001, CONFIG = 0x0002, INTPEN = 0x0004, IE = 0x0006, FPST = 0x0008,
		PPERROR = 0x000A, PKTREQ = 0x000D, TCOUNT = 0x000E, TSCALE = 0x000F, FLTOP = 0x0010,
		FLTADR = 0x0011, FLTTAG = 0x0012, FLTDTL = 0x0013, FLTDTH = 0x0014, SYSSTK = 0x0020,
		SYSTMP = 0x0021, MPC = 0x0030, MIP = 0x0031, ECOMCNTL = 0x0033, ANASTAT = 0x0034,
		BRK1 = 0x0039, BRK2 = 0x003A, ITAG0 = 0x0200, ILRU = 0x0300, DTAG0 = 0x0400, DLRU = 0x0500,
		IN0P = 0x4000, IN1P = 0x4001, OUTP = 0x4002
	};
	/* INTPEN and IE bits (fig. 3-2) */
	enum : uint32_t {
		INT_IE = 1u << 0, INT_FI = 1u << 2, INT_FZ = 1u << 3, INT_FO = 1u << 5, INT_FU = 1u << 6,
		INT_FX = 1u << 7, INT_TI = 1u << 10, INT_X1 = 1u << 11, INT_X2 = 1u << 12, INT_MF = 1u << 14,
		INT_IO = 1u << 15, INT_P0 = 1u << 16, INT_MI = 1u << 25, INT_PC = 1u << 26, INT_PB = 1u << 27,
		INT_BP = 1u << 28, INT_X3 = 1u << 29, INT_X4 = 1u << 30, INT_PE = 1u << 31
	};
	/* Trap numbers (table 9-2) */
	enum : unsigned { TRAP_FE = 36, TRAP_ER = 38 };

	explicit C82Mp(Tms320c82 &chip);
	~C82Mp();
	C82Mp(const C82Mp &) = delete;
	C82Mp &operator=(const C82Mp &) = delete;

	/* Power-up reset (§ 9.5) */
	void reset();
	/* Runs until the cycle `until`, or less when end_run() is called */
	void run(uint64_t until);
	void end_run()
	{
		m_until = m_cycles;
		m_check = true;
	}
	/* One instruction (with an interrupt taken first if one is due) */
	void step();

	uint64_t cycles() const { return m_cycles; }
	/* Sets INTPEN bits, as the TC, the PPs and the pins do */
	void post(uint32_t bits)
	{
		disturb();
		m_intpen |= bits;
		m_check = true;
	}
	/* LINT4, active low: INTPEN x4 follows it (§ 3.3.2) */
	void set_lint4(bool low)
	{
		disturb();
		m_lint4 = low;
		m_check = true;
	}
	/* Whether the host rounds to nearest, its default, which the fast
	 * floating point needs (c82_fast) */
	void check_host_rounding();

	/* A wait of the program, as C82Pp::set_wait: once a pass through it,
	 * from its first instruction back to it, leaves the MP as it was (all
	 * its state but the cycle count) with no store, no control register,
	 * no exception, and reads only plain memory, the MP is parked: its
	 * slices end where its steps would have ended them, without running
	 * it; anything it reads written, or an interrupt posted, brings it to
	 * where it stands (disturb) */
	void set_wait(uint32_t ip, std::span<const uint32_t> words);
	bool watches(uint32_t addr, uint32_t bytes) const;
	void disturb();
	struct Watch {
		uint32_t lo = 0, hi = 0;
		bool internal = false;
	};
	std::span<const Watch> watched() const
	{
		return { m_park.watch.data(), (m_park.on || m_park.measuring) ? m_park.watches : 0u };
	}

	void save(C82MpState &out) const;
	void load(const C82MpState &in);

	/* For the tests and the log */
	uint32_t r(unsigned n)
	{
		disturb();
		return m_r[n & 31];
	}
	void set_r(unsigned n, uint32_t v)
	{
		disturb();
		if (n & 31)
			m_r[n & 31] = v;
	}
	uint64_t accumulator(unsigned n)
	{
		disturb();
		return m_acc[n & 3];
	}
	uint32_t ip()
	{
		disturb();
		return m_ip;
	}
	void jump(uint32_t address)
	{
		disturb();
		m_ip = address;
		m_pc = address + 4;
	}
	uint32_t control(uint32_t number);
	void set_control(uint32_t number, uint32_t value)
	{
		disturb();
		write_control(number, value);
	}
	/* The blocks in host code (c82_mp_host.cpp): on where the host has
	 * them (x86-64), or off (the interpreter alone, as on other hosts) */
	void use_host_code(bool on);
	void flush_blocks();
	/* `bytes` bytes from `addr` of external memory are about to be
	 * written (by the MP, the TC or a PP: Tms320c82 calls this for a page
	 * whose Page::code is set): the blocks with a word in a line of 64
	 * bytes written are dropped, and the MP takes a whole step next */
	void code_written(uint32_t addr, uint32_t bytes)
	{
		const uint64_t *const lines = m_code_lines[addr >> 20];
		const uint32_t line = (addr & 0xFFFFF) >> 6;

		/* Most often: a write within one line that no block reads */
		if (!lines || ((addr & 63) + bytes <= 64 && !((lines[line >> 6] >> (line & 63)) & 1)))
			return;
		drop_code(addr, bytes);
	}
	/* A PP's block in host code fetches from lo to hi: its lines
	 * marked as the MP's own (code_written then tells the PPs too) */
	void mark_lines(uint32_t lo, uint32_t hi) { mark_code(lo, hi); }

private:
	struct Fault {
		bool on = false;
	};

	/* Instruction execution (c82_mp_ops.cpp) */
	void execute(const C82Insn &in, uint32_t word, uint32_t ip, uint32_t next);
	void exec_branch(const C82Insn &in, bool taken, uint32_t target);
	void exec_memory(const C82Insn &in);
	void exec_shift(const C82Insn &in);
	void exec_float(const C82Insn &in, uint32_t word);
	void exec_vector(const C82Insn &in, uint32_t word);
	bool float_allowed();
	C82FpEnv float_env(uint8_t rounding) const;
	bool float_done(const C82FpEnv &env, uint32_t word, uint8_t dest, uint8_t pd);
	uint64_t read_float(unsigned reg, uint8_t type) const;
	void write_float(unsigned reg, uint8_t type, uint64_t value);
	bool load(uint32_t address, unsigned size, uint64_t &value);
	bool store(uint32_t address, unsigned size, uint64_t value);
	void memory_fault(uint32_t address, unsigned size, uint8_t kind, uint8_t dest, uint64_t data);

	/* Control registers, exceptions (c82_mp.cpp) */
	uint32_t read_control(uint32_t number);
	void write_control(uint32_t number, uint32_t value);
	uint32_t tcount() const;
	void set_tcount(uint32_t value);
	void advance_timer();
	void exception(unsigned number, uint32_t eip, uint32_t epc);
	void take_interrupt();
	void set(unsigned reg, uint32_t value)
	{
		if (reg)
			m_r[reg] = value;
	}

	Tms320c82 &m_chip;
	std::array<uint32_t, 32> m_r{};
	std::array<uint64_t, 4> m_acc{};
	/* IP: the instruction executed next; PC: the one fetched after it, the
	 * target of a branch in its delay slot (§ 3.1) */
	uint32_t m_ip = 0, m_pc = 0;
	uint32_t m_epc = 0, m_eip = 0, m_config = 0, m_intpen = 0, m_ie = 0, m_fpst = 0, m_pktreq = 0;
	uint32_t m_tscale = 0;
	uint32_t m_fltop = 0, m_fltadr = 0, m_flttag = 0, m_fltdtl = 0, m_fltdth = 0;
	uint32_t m_sysstk = 0, m_systmp = 0, m_mpc = 0, m_mip = 0;
	uint32_t m_ecomcntl = 0, m_anastat = 0, m_brk1 = 0, m_brk2 = 0, m_ilru = 0, m_dlru = 0;
	uint32_t m_in0p = 0, m_in1p = 0, m_outp = 0;
	std::array<uint32_t, 16> m_itag{}, m_dtag{};
	uint64_t m_cycles = 0, m_until = 0;
	/* The cycle TCOUNT next reads 0 (§ 3.5) */
	uint64_t m_timer_zero = 0;
	bool m_user = false, m_lint4 = false;
	/* cmnd's message to the MP: posted after one delay slot (p. 10-71
	 * note 3) */
	uint8_t m_self_message = 0;
	/* A nonmaskable memory fault, taken after its instruction (§ 5.6.2) */
	Fault m_fault;
	/* The instruction executing: in a branch delay slot; the next one
	 * will be (FLTOP R, § 9.3.2) */
	bool m_in_slot = false, m_slot_next = false;
	/* The instruction executing and the one after it, for traps; it has a
	 * long immediate. Set by step(), and by a handler before what reads
	 * them (execute, memory_fault) */
	uint32_t m_now_ip = 0, m_now_next = 0;
	bool m_now_long = false;

	/* Decoded and prepared instructions (an emulator's cache: c82_decode
	 * depends on the word and the long immediate alone, both checked); a
	 * place an address, the two tables filled together by prepare() */
	friend struct C82MpOps;
	struct Decoded {
		uint32_t word = 0, immediate = 0;
		C82Insn insn;
	};
	static constexpr size_t DECODED = 32768;
	std::vector<Decoded> m_decoded = std::vector<Decoded>(DECODED);
	std::vector<C82MpOp> m_ops = std::vector<C82MpOp>(DECODED);
	const C82MpOp &prepared(uint32_t ip, uint32_t word, uint32_t immediate)
	{
		C82MpOp &op = m_ops[(ip >> 2) & (DECODED - 1)];

		if (op.key != (word | static_cast<uint64_t>(immediate) << 32))
			prepare(op, word, immediate);
		return op;
	}
	void prepare(C82MpOp &op, uint32_t word, uint32_t immediate);
	const C82Insn &decoded(const C82MpOp &op) const { return m_decoded[&op - m_ops.data()].insn; }

	/* What step() checks before an instruction and only an event changes: a
	 * fault, a message, the x4 pin, an interrupt to take. False: all these
	 * checks would do nothing */
	bool attention() const
	{
		return m_fault.on || m_self_message || ((m_intpen ^ (m_lint4 ? INT_X4 : 0u)) & INT_X4) ||
		       ((m_ie & INT_IE) && (m_intpen & m_ie & ~INT_IE));
	}
	/* attention() may be true: set by whatever changes what it reads; the
	 * run then takes a whole step() */
	bool m_check = true;
	/* The host rounds to nearest (check_host_rounding) */
	bool m_host_nearest = true;
	/* What the fast floating point may do (C82MpOps::fast_float), from IE,
	 * FPST and the host: bit 0, run; bit 1, round to nearest by drm. Set
	 * before each instruction of step() and at the start of fast_steps();
	 * whatever changes it sets m_check, which ends fast_steps() */
	uint8_t m_float_mode = 0;
	void set_float_mode()
	{
		constexpr uint32_t TRAPS = INT_FX | INT_FU | INT_FO | INT_FZ | INT_FI, FS = 1u << 20, DRM = 3u << 17;
		/* & between the three, the same value without branches */
		const bool run = m_host_nearest & ((m_ie & (INT_IE | TRAPS)) == INT_IE) & !(m_fpst & FS);

		m_float_mode = static_cast<uint8_t>((run ? 1 : 0) | (run && !(m_fpst & DRM) ? 2 : 0));
	}

	[[gnu::noinline]] void fast_steps();
	/* run()'s rare ways, out of line: woken first; the steps until
	 * m_until (fast_steps, the interpreter, the wait, the timer) */
	[[gnu::noinline, gnu::cold]] void wake_on();
	[[gnu::noinline]] void run_steps();

	/* Blocks of host code (c82_mp_host.cpp, x86-64 only; elsewhere the
	 * interpreter runs alone): the instructions that follow each other in
	 * one page of plain memory, written in host code or calling the
	 * prepared handler; run from fast_steps() in the normal state (no
	 * delay slot due, PC = IP + 4), they stop where fast_steps() would
	 * hand over, or where they cannot go on. Their words are not
	 * checked at each step any more: the lines of 64 bytes they read are
	 * marked (m_code_lines, Page::code) and any write there drops them
	 * first (code_written) */
	friend class C82MpCode;
	struct Block {
		uint32_t ip = 0, n = 0;
		/* Its host code, and where each step starts in it */
		uint32_t place = 0;
		std::vector<uint32_t> entries;
		/* The address after its last word */
		uint32_t end = 0;
		bool dead = false;
	};
	/* One instruction of a block being written */
	struct BlockStep {
		uint32_t ip = 0, word = 0, imm = 0;
		uint8_t length = 4;
		/* In a delay slot of the step before */
		bool slot = false;
		C82MpOp op;
		C82Insn in;
	};
	/* What the host code finds a block by: its first instruction (~0u:
	 * none), its number and its code (an address) */
	struct Link {
		uint32_t ip = ~0u, id = 0;
		uint64_t code = 0;
	};
	static constexpr size_t LINKS = 8192;
	/* A jump at a block's end to an instruction known when the block
	 * is written (a branch's target from an immediate, the instruction
	 * after it): the place of its displacement, the way through the link
	 * it takes while that instruction has no code (its stub), the
	 * instruction, the block it is in. Chained: it leads straight to the
	 * code find_block() would give (code_for), until that block is
	 * dropped */
	struct Chain {
		uint32_t at = 0, stub = 0, ip = 0, owner = 0;
	};
	void chain(const Chain &site);
	const uint32_t *code_for(uint32_t ip, uint32_t &id) const;
	void settle(uint32_t ip);
	std::unordered_map<uint32_t, std::vector<Chain>> m_waiting;
	std::vector<std::vector<Chain>> m_chained;
	std::vector<Chain> m_sites;
	bool run_blocks(uint32_t ip, uint64_t limit);
	[[gnu::noinline]] const uint8_t *find_block(uint32_t ip);
	Block *translate(uint32_t ip);
	/* The lines of 64 bytes from lo to hi marked as read by a block; the
	 * blocks that read the lines written dropped (code_written) */
	void mark_code(uint32_t lo, uint32_t hi);
	void drop_code(uint32_t addr, uint32_t bytes);
	bool write_block(Block &b, const std::vector<BlockStep> &steps, bool branch, bool end_in_slot,
	                 uint32_t next);
	/* The host code of a step's body (c82_mp_host_exec.cpp): false when
	 * the step calls its handler instead */
	bool write_body(C82MpCode &c, const std::vector<BlockStep> &steps, unsigned k, std::vector<C82MpSlow> &slow);
	void write_branch(C82MpCode &c, const BlockStep &st);
	bool write_memory(C82MpCode &c, const std::vector<BlockStep> &steps, unsigned k, std::vector<C82MpSlow> &slow);
	bool write_float(C82MpCode &c, const std::vector<BlockStep> &steps, unsigned k, std::vector<C82MpSlow> &slow);
	void write_store(C82MpCode &c, unsigned size);
	void write_data(C82MpCode &c, unsigned reg, unsigned size);
	void write_loaded(C82MpCode &c, unsigned size, unsigned dest);
	/* The routines the steps call (c82_mp_host_exec.cpp), written after
	 * the way in, and a call of one */
	enum Helper : uint8_t {
		H_LOAD8S, H_LOAD8U, H_LOAD16S, H_LOAD16U, H_LOAD32, H_LOAD64, H_STORE8, H_STORE16, H_STORE32, H_STORE64,
		H_FMPY, H_FADD, H_FDIV, H_FCMP, H_VRND_SD, H_VMAC_S, H_VMAC_D, H_CVT_IS, H_CVT_US, H_CVT_ID, H_CVT_UD,
		H_CVT_SD, HELPERS
	};
	void write_helpers();
	void call_helper(C82MpCode &c, unsigned h, std::vector<HostCode::Jump> &slow);
	std::array<uint32_t, HELPERS> m_helper{};
	void kill_block(uint32_t id);
	/* Called by the host code: a step by its prepared handler; the
	 * parallel load or store of a vector instruction alone */
	static void block_step(C82Mp &mp, uint32_t ip, uint64_t key);
	static void block_vector_memory(C82Mp &mp, uint32_t ip, uint64_t key);
	/* cmp's status word (c82_mp_ops.cpp), for the host code too */
	static uint32_t compare_all(uint32_t a, uint32_t b);
	std::unique_ptr<C82MpCode> m_code;
	bool m_native = true;
	std::vector<std::unique_ptr<Block>> m_blocks;
	std::unordered_map<uint32_t, uint32_t> m_block_at;
	/* The steps within blocks (not the first, not in a delay slot): a
	 * block there goes on from them, rather than a new one made */
	struct Inside {
		uint32_t id = 0, k = 0;
	};
	std::unordered_map<uint32_t, Inside> m_inside;
	std::vector<Link> m_links;
	size_t m_block_steps = 0;
	/* Where the last run of the blocks stopped within one (a slice's end,
	 * written by the host code): the next one goes on from there */
	uint32_t m_resume_ip = ~0u;
	const uint8_t *m_resume_code = nullptr;

	/* The wait (set_wait): the pass measured, the parking */
	void pass_boundary();
	void note_read(uint32_t addr, unsigned size);
	void watch(uint32_t lo, uint32_t hi, bool internal);
	void stop_watching();
	void wake();
	uint64_t parked_end(uint64_t until);
	/* parked_end from the pass `pass` found last (kept up to date
	 * there), nothing else changed: Tms320c82::run_long's plan */
	uint64_t parked_end_from(uint64_t &pass, uint64_t until) const;
	friend class Tms320c82;
	struct Park {
		static constexpr unsigned WATCHES = 8, STEPS = 32, NEXT = 64;

		bool measuring = false, spoiled = false, on = false;
		C82MpState start;
		/* Parked: its cycle count at the wait's first instruction, the
		 * cycles of a pass, where each of its steps starts in it; the
		 * start of the pass parked_end found last */
		uint64_t base = 0, period = 0, pass = 0;
		std::array<uint32_t, STEPS> step{};
		unsigned steps = 0;
		/* For each point of a pass of at most NEXT cycles, the first
		 * start of a step at or after it (the period: the next pass's
		 * first), from the pass's start */
		std::array<uint8_t, NEXT> next{};
		std::array<Watch, WATCHES> watch{};
		unsigned watches = 0;
	};
	uint32_t m_wait_ip = ~0u;
	std::vector<uint32_t> m_wait_words;
	Park m_park{};
	/* For each 1 MB page of the external bus, a bit for each line of
	 * 64 bytes a live block reads (null: none); last, so that the members
	 * the host code reaches stay near rbx */
	std::array<uint64_t *, 4096> m_code_lines{};
	std::vector<std::unique_ptr<uint64_t[]>> m_line_store;
};

} /* namespace odin */

#endif /* ODIN_C82_MP_HPP */
