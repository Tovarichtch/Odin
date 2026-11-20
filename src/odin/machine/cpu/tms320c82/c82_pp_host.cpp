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


#include "odin/machine/cpu/tms320c82/c82_pp_host.hpp"

#include <vector>

#include "odin/machine/cpu/tms320c82/c82_pp.hpp"
#include "odin/machine/cpu/tms320c82/tms320c82.hpp"

/* The PP's prepared blocks in host code: each step written as run_block()
 * does it, in its order, with what it knows already as constants, and the
 * execute stage's handler of the interpreter called (exec<FORM>). The end of
 * a block tells C++ which step it stopped at and why (run_native) */

namespace odin {

namespace {

#if defined(ODIN_HOST_X86_64)

/* Whether the execute stage of `in` may write memory, by the handler
 * a step calls (or its slow path, or comm_written): a store, comm written
 * (the TC asked), a command; anything for the data unit's generic way */
bool
may_write(const C82PpInsn &in)
{
	for (unsigned x = 0; x < in.xfers; x++)
		if (in.xfer[x].le == 0 || in.xfer[x].reg == 0x3E)
			return true;
	if ((in.move || in.fmove) && (in.move_dst == 0x3E || in.move_dst == 0xF))
		return true;
	return in.path == C82PpInsn::GENERIC || (in.path != C82PpInsn::NO_DU && in.dst_code() == 0x3E);
}

/* Whether the execute stage of `in` (`op` prepared) may do what another
 * unit sees other than by its memory accesses: write comm (the TC asked),
 * move to a15 (a command), anything (the data unit's generic way) */
bool
may_signal(const C82PpInsn &in, const C82PpOp &op)
{
	constexpr unsigned COMM = 0x3E;

	if (in.path == C82PpInsn::GENERIC || (in.path != C82PpInsn::NO_DU && in.dst_code() == COMM))
		return true;
	for (unsigned x = 0; x < in.xfers; x++)
		if (in.xfer[x].le != 0 && (in.xfer[x].reg & 0x7F) == COMM)
			return true;
	if ((in.move || in.fmove) && (in.move_dst == COMM || in.move_dst == 0xF))
		return true;
	switch (in.shape) {
	case C82PpInsn::LOAD:
	case C82PpInsn::LEA:
	case C82PpInsn::DU_LOAD:
		return (op.reg & 0x7F) == COMM || (in.shape == C82PpInsn::DU_LOAD && (op.dst & 0x7F) == COMM);
	case C82PpInsn::DU:
	case C82PpInsn::DU_STORE:
		return (op.dst & 0x7F) == COMM;
	default:
		return false;
	}
}

/* Whether the execute stage of `in` may write pc (branch()): the
 * destinations as traits() finds them (anything for the data unit's
 * generic way) */
bool
writes_pc(const C82PpInsn &in)
{
	const auto pc = [](unsigned code) { return (code & 0x7F) == 0x38 || (code & 0x7F) == 0x39; };

	if (in.path == C82PpInsn::GENERIC || (in.path != C82PpInsn::NO_DU && pc(in.dst_code())))
		return true;
	for (unsigned x = 0; x < in.xfers; x++)
		if (in.xfer[x].le != 0 && pc(in.xfer[x].reg))
			return true;
	return (in.move || in.fmove) && pc(in.move_dst);
}

#endif

/* The memory of a PP's host code, and the most a block can take (64 steps,
 * their ends) */
constexpr uint32_t SIZE = 2u << 20, LONGEST = 64 * 2048;

/* Why a block's code ended (bits 7-0 of what it returns, the step above):
 * before the step (end of the slice, or memory the interpreter must reach;
 * its word changed), after it (it branched, took a loop end, was the last) */
enum : uint32_t { END_STOP, END_CHANGED, END_BRANCH, END_LOOP, END_LAST };
/* Before a step, the end of the slice: END_DONE when the code notes
 * the flow itself (C82Pp::resume), END_STOP otherwise */
constexpr uint32_t END_SLICE = 6;

} /* namespace */

C82PpCode::C82PpCode() : HostCode(SIZE), m_enter(nullptr)
{
	if (usable())
		clear();
}

bool
C82PpCode::usable() const
{
#if defined(ODIN_HOST_X86_64)
	return HostCode::usable();
#else
	return false;
#endif
}

bool
C82PpCode::room() const
{
	return m_used + LONGEST <= m_size;
}

uint64_t
C82PpCode::run(void *pp, uint32_t place, const uint8_t *host, int64_t left) const
{
	return m_enter(pp, m_code + place, host, left);
}

#if defined(ODIN_HOST_X86_64)

/* The way in keeps the PP in rbx (way_in), the words fetched in r13 and the
 * cycles left in rbp, then jumps to the step; the way back returns rax */
void
C82PpCode::clear()
{
	writable();
	way_in();
	bytes({ static_cast<uint8_t>(0x49 | (ARG[2] >= 8 ? 4 : 0)), 0x89, static_cast<uint8_t>(0xC0 | ((ARG[2] & 7) << 3) | 5) });	/* mov r13, the third argument */
	bytes({ static_cast<uint8_t>(0x48 | (ARG[3] >= 8 ? 4 : 0)), 0x89, static_cast<uint8_t>(0xC0 | ((ARG[3] & 7) << 3) | RBP) });	/* mov rbp, the fourth */
	way_in_end();
	way_out();
	m_enter = reinterpret_cast<uint64_t (*)(void *, const uint8_t *, const uint8_t *, int64_t)>(m_code);
	ready();
}

void
C82PpCode::op_mem(uint8_t opcode, uint8_t reg, uint8_t base, int32_t displacement, bool wide)
{
	const uint8_t rex = static_cast<uint8_t>(0x40 | (wide ? 8 : 0) | (reg >= 8 ? 4 : 0) | (base >= 8 ? 1 : 0));

	if (rex != 0x40)
		byte(rex);
	bytes({ opcode, static_cast<uint8_t>(0x80 | ((reg & 7) << 3) | (base & 7)) });
	word(static_cast<uint32_t>(displacement));
}

void
C82PpCode::move_quad(uint8_t reg, uint64_t value)
{
	bytes({ static_cast<uint8_t>(0x48 | (reg >= 8 ? 1 : 0)), static_cast<uint8_t>(0xB8 | (reg & 7)) });	/* mov reg, value */
	quad(value);
}

void
C82PpCode::move_word(uint8_t reg, uint32_t value)
{
	mov_ri(reg, value);
}

void
C82PpCode::pp_first()
{
	bytes({ static_cast<uint8_t>(0x48 | (ARG[0] >= 8 ? 1 : 0)), 0x89, static_cast<uint8_t>(0xC0 | (RBX << 3) | (ARG[0] & 7)) });	/* mov the first argument, rbx */
}

HostCode::Jump
C82PpCode::jump_ahead()
{
	byte(0xE9);					/* jmp ... */
	m_used += 4;
	return Jump{ m_used - 4 };
}

void
C82PpCode::op_m(std::initializer_list<uint8_t> opcode, uint8_t reg, Mem m, bool wide, bool half, bool low)
{
	const bool sib = m.index != NO || (m.base & 7) == RSP;
	const uint8_t rex = static_cast<uint8_t>(0x40 | (wide ? 8 : 0) | (reg >= 8 ? 4 : 0) |
	                                         (m.index != NO && m.index >= 8 ? 2 : 0) | (m.base >= 8 ? 1 : 0));

	if (half)
		byte(0x66);
	if (rex != 0x40 || (low && reg >= 4))
		byte(rex);
	bytes(opcode);
	/* mod 10: a 32-bit displacement */
	byte(static_cast<uint8_t>(0x80 | (reg & 7) << 3 | (sib ? RSP : m.base & 7)));
	if (sib)
		byte(static_cast<uint8_t>(m.scale << 6 | ((m.index == NO ? RSP : m.index) & 7) << 3 | (m.base & 7)));
	word(static_cast<uint32_t>(m.disp));
}

void
C82PpCode::op_r(std::initializer_list<uint8_t> opcode, uint8_t reg, uint8_t rm, bool wide, bool half, bool low)
{
	const uint8_t rex = static_cast<uint8_t>(0x40 | (wide ? 8 : 0) | (reg >= 8 ? 4 : 0) | (rm >= 8 ? 1 : 0));

	if (half)
		byte(0x66);
	if (rex != 0x40 || (low && (reg >= 4 || rm >= 4)))
		byte(rex);
	bytes(opcode);
	byte(static_cast<uint8_t>(0xC0 | (reg & 7) << 3 | (rm & 7)));
}

void
C82PpCode::swap32(uint8_t reg)
{
	if (reg >= 8)
		byte(0x41);
	bytes({ 0x0F, static_cast<uint8_t>(0xC8 | (reg & 7)) });	/* bswap reg */
}

#else

void C82PpCode::clear() {}

#endif

C82Pp::~C82Pp() = default;

void
C82Pp::use_host_code(bool on)
{
	disturb();
	flush_blocks();
	m_native = on;
}

/* For the host code: whether the execute stage reaches plain memory there
 * (run_block's test); the loop logic before the address stage and after the
 * execute stage */
uint64_t
C82Pp::plain(C82Pp &p, uint32_t addr)
{
	return (addr < 0x02000000u ? p.m_chip.pp_ram(addr) : p.m_chip.pp_code(addr)) != nullptr;
}

uint64_t
C82Pp::plain_store(C82Pp &p, uint32_t addr)
{
	return !p.m_alone && plain(p, addr);
}

void
C82Pp::loop_before(C82Pp &p, uint32_t pc)
{
	p.m_nat_lc = p.m_lc;
	p.m_nat_next = p.next_fetch(pc, p.m_nat_lc);
}

uint64_t
C82Pp::loop_after(C82Pp &p, uint32_t pc)
{
	for (unsigned n = 0; n < 3; n++)
		if (!((p.m_lc_written >> n) & 1))
			p.m_lc[n] = p.m_nat_lc[n];
	return p.m_nat_next != pc + 8;
}

void
C82Pp::comm_read(C82Pp &p)
{
	p.m_nat_comm = p.read_comm();
}

void
C82Pp::comm_written(C82Pp &p, uint32_t value)
{
	p.write_comm(value);
}

void
C82Pp::loops_written(C82Pp &p)
{
	p.loops_changed();
}

/* The step's frame: what read_reg and write_reg read made up to date (pc,
 * the cycles, the stages' addresses, the protected status bits) */
void
C82Pp::write_frame(C82PpCode &c, uint32_t pc, uint32_t ipa, uint32_t ipe, uint32_t protect)
{
#if defined(ODIN_HOST_X86_64)
	using C = C82PpCode;
	const auto at = [this](const void *member) { return HostCode::place_in(this, member); };
	const int32_t ADDR = HostCode::place_in(&m_stages[0], &m_stages[0].addr);

	c.op_mem(0xC7, 0, C::PP, at(&m_pc));					/* mov dword [pc], pc */
	c.word(pc);
	c.op_mem(0x8B, C::RAX, C::PP, at(&m_nat_until), true);			/* mov rax, [until] */
	c.bytes({ 0x48, 0x29, 0xE8 });						/* sub rax, rbp */
	c.op_mem(0x89, C::RAX, C::PP, at(&m_cycles), true);			/* mov [cycles], rax */
	c.op_mem(0x8B, C::RAX, C::PP, at(&m_sa), true);				/* mov rax, [m_sa] */
	c.op_mem(0xC7, 0, C::RAX, ADDR);					/* mov dword [rax + addr], ipa */
	c.word(ipa);
	c.op_mem(0x8B, C::RAX, C::PP, at(&m_se), true);				/* mov rax, [m_se] */
	c.op_mem(0xC7, 0, C::RAX, ADDR);					/* mov dword [rax + addr], ipe */
	c.word(ipe);
	c.op_mem(0xC7, 0, C::PP, at(&m_protect));				/* mov dword [protect], ... */
	c.word(protect);
#else
	(void)c;
	(void)pc;
	(void)ipa;
	(void)ipe;
	(void)protect;
#endif
}

/* The block's steps in host code, as run_block() does them (see there for
 * each part) */
void
C82Pp::write_native(Block &b)
{
#if defined(ODIN_HOST_X86_64)
	if (!m_native)
		return;
	if (!m_code) {
		m_code = std::make_unique<C82PpCode>();
		m_nat_enter = m_code->enter();
		const Tms320c82::RamMap map = m_chip.ram_map();

		for (uint32_t i = 0; i < m_nat_internal.size(); i++)
			m_nat_internal[i] = map.internal[i] == map.dcache ? int16_t{ -1 } : map.internal[i];
		for (uint32_t f = 0; f < m_ncvz.size(); f++)
			m_ncvz[f] = ((f >> 7) & 1) << 31 | (f & 1) << 30 | ((f >> 11) & 1) << 29 | ((f >> 6) & 1) << 28;
	}

	C82PpCode &c = *m_code;

	if (!c.usable()) {
		m_native = false;
		return;
	}
	if (!c.room())
		return;
	c.writable();

	using C = C82PpCode;
	const auto at = [this](const void *member) { return HostCode::place_in(this, member); };
	const int32_t R = at(m_r.data()), EXTRA = at(&m_extra), WRITTEN = at(&m_written), PC_WRITTEN = at(&m_pc_written),
	              LC_WRITTEN = at(&m_lc_written), PC = at(&m_pc), CYCLES = at(&m_cycles), BACK = at(m_nat_back),
	              UNTIL = at(&m_nat_until), LEFT = at(&m_nat_left),
	              NEW_PC = at(&m_new_pc), NEW_PC_BITS = at(&m_new_pc_bits), NAT_NEXT = at(&m_nat_next),
	              NAT_PC = at(&m_nat_pc), NAT_BLOCK = at(&m_nat_block), HALTED = at(&m_halted), INTFLG = at(&m_intflg),
	              INTEN = at(&m_inten), ENDS = at(m_ends.data()), PARK_CYCLES = at(&m_park.cycles),
	              MEASURING = at(&m_park.measuring), FLOW_PC = at(&m_flow.pc), FLOW_B = at(&m_flow.b),
	              FLOW_I = at(&m_flow.i), FLOW_EA = at(m_flow.e_ea), FLOW_LEFT = at(&m_flow.left),
	              FLOWING = at(&m_flowing), RESUME = at(&m_resume), ALONE = at(&m_alone);
	const Block::Successor &model = b.successors[0];
	const int32_t S_BLOCK = HostCode::place_in(&model, &model.block), S_PC = HostCode::place_in(&model, &model.pc),
	              S_CODE = HostCode::place_in(&model, &model.code), S_HOST = HostCode::place_in(&model, &model.host),
	              S_SLOT = HostCode::place_in(&model, &model.slot), DEAD = HostCode::place_in(&b, &b.dead),
	              B_ENDS = HostCode::place_in(&b, b.ends.data()), B_F0 = HostCode::place_in(&b, &b.f0),
	              B_SPAN = HostCode::place_in(&b, &b.span);
	const auto ea = [&](unsigned slot, unsigned x) { return at(&m_nat_ea[slot][x]); };
	struct End {
		HostCode::Jump jump;
		uint32_t code;
	};
	std::vector<End> ends;
	std::vector<C82PpSlow> slow;
	std::vector<uint32_t> stops;

	/* A block in external memory has its lines marked (mark_lines):
	 * a write there drops it first (code_written), its words are not
	 * checked; but a step after one whose call may write memory looks at
	 * whether the block was dropped then (the block itself wrote there).
	 * One in the PPs' RAMs checks each word it fetches */
	const bool checked = b.f0 < 0x02000000u;
	bool after_write = false;
	/* The budget: a step right after another that ended with sub
	 * rbp, 1 (no end looked at after it) tests its flags (jle: rbp <= 0, no
	 * overflow); entered from elsewhere, by its entry out of line (test
	 * rbp, rbp, then that jle) */
	bool by_sub = false;
	struct Entry {
		unsigned k;
		uint32_t jle;
	};
	std::vector<Entry> entries;

	b.entries.assign(b.n, 0);
	for (unsigned k = 0; k < b.n; k++) {
		const BlockStep &st = b.steps[k];
		const unsigned flags = st.flags;
		const uint32_t pc = (b.f0 + 8 * k) | b.bits;
		const unsigned sa = k & 1, se = sa ^ 1;

		const bool dead_check = after_write && !checked;

		b.entries[k] = c.here();
		if (dead_check) {
			c.move_quad(C::RAX, reinterpret_cast<uintptr_t>(&b.dead));
			c.bytes({ 0x80, 0x38, 0x00 });		/* cmp byte [rax], 0 */
			ends.push_back({ c.jump_if(C::JNZ), k << 8 | END_CHANGED });
		}
		after_write = may_write(*st.ein);
		/* The end of the slice; the word fetched */
		if (by_sub && !dead_check)
			entries.push_back({ k, c.here() });
		else
			c.bytes({ 0x48, 0x85, 0xED });		/* test rbp, rbp */
		ends.push_back({ c.jump_if(C::JLE), k << 8 | END_SLICE });
		if (checked) {
			c.op_mem(0x8B, C::RAX, C::HOST, static_cast<int32_t>(8 * k), true);	/* mov rax, [r13 + 8k] */
			c.move_quad(C::RCX, st.raw);
			c.bytes({ 0x48, 0x39, 0xC8 });			/* cmp rax, rcx */
			ends.push_back({ c.jump_if(C::JNZ), k << 8 | END_CHANGED });
		}
		/* How the execute stage is made (below) */
		const unsigned plan = st.form != C82PpInsn::FORM_NOP && !(b.wait && (flags & MEMORY)) ? exec_plan(st) : 0;
		const bool handler = st.form != C82PpInsn::FORM_NOP && !(plan & PLAN_WRITTEN);

		/* Alone, the block ends before a step whose execute stage
		 * may write comm (the TC asked) or a15 (a command), or may do
		 * anything (the generic way); its stores are looked at by
		 * write_memory (by the handler) or on their slow path */
		if (st.form != C82PpInsn::FORM_NOP && may_signal(*st.ein, *st.eop)) {
			c.op_mem(0x80, 7, C::PP, ALONE);			/* cmp byte [m_alone], 0 */
			c.byte(0);
			ends.push_back({ c.jump_if(C::JNZ), k << 8 | END_STOP });
		}
		/* The execute stage's memory: on-chip RAM, plain or not
		 * (written here, plain() only for external addresses) */
		if (flags & MEMORY) {
			write_memory(c, st, k, handler, slow, stops);
			for (uint32_t from : stops)
				ends.push_back({ HostCode::Jump{ from }, k << 8 | END_STOP });
			stops.clear();
		}
		/* A wait block counts the cycle (its limit: run_blocks) */
		if (b.wait)
			c.op_mem(0xFF, 0, C::PP, PARK_CYCLES);			/* inc dword [m_park.cycles] */
		if (flags & LOOP_END) {
			c.pp_first();
			c.move_word(C::ARG[1], pc);
			c.call_to(reinterpret_cast<uintptr_t>(&C82Pp::loop_before));
		}
		/* The address stage: eax the base, ecx the index; edx the address
		 * written back */
		for (unsigned x = 0; x < st.axfers; x++) {
			const C82PpOp::Agen &g = st.agen[x];
			const bool back = st.wb_slot[x] != 0xFF;

			c.op_mem(0x8B, C::RAX, C::PP, R + 4 * g.base_slot);	/* mov eax, base */
			if (g.index_slot == c82_pp_regs::ZERO) {
				const uint32_t index = (((0u + g.offset) << g.shift) ^ (0u - g.neg)) + g.neg;

				c.op_mem(0x89, C::RAX, C::PP, ea(sa, x));		/* mov [address], eax */
				if (g.pre && index) {
					c.op_mem(0x81, 0, C::PP, ea(sa, x));		/* add dword [address], index */
					c.word(index);
				}
				if (back) {
					if (index)
						c.op_ri(0, C::RAX, index);		/* add eax, index */
					c.op_mem(0x89, C::RAX, C::PP, BACK + 4 * static_cast<int32_t>(x));
				}
				continue;
			}
			c.op_mem(0x8B, C::RCX, C::PP, R + 4 * g.index_slot);	/* mov ecx, index */
			if (g.offset)
				c.op_ri(0, C::RCX, g.offset);			/* add ecx, offset */
			if (g.shift)
				c.shift_ri(4, C::RCX, g.shift);			/* shl ecx, shift */
			if (g.neg)
				c.bytes({ 0xF7, 0xD9 });			/* neg ecx */
			c.bytes({ 0x8D, 0x14, 0x08 });				/* lea edx, [rax + rcx] */
			c.op_mem(0x89, g.pre ? C::RDX : C::RAX, C::PP, ea(sa, x));	/* mov [address], edx or eax */
			if (back)
				c.op_mem(0x89, C::RDX, C::PP, BACK + 4 * static_cast<int32_t>(x));
		}
		/* The execute stage, what read_reg and write_reg read first */
		const uint32_t ipa = k >= 1 ? pc - 8 : b.ipa, ipe = k >= 2 ? pc - 16 : k == 1 ? b.ipa : b.ipe;

		/* A wait block reaches memory by the handler (load() notes
		 * what a pass measured reads). The frame (what read_reg and
		 * write_reg read) only where C++ reads it: the handler, on the step
		 * (as before) or on its slow paths (written there, before the
		 * call), comm's helpers (on the step); m_written, m_pc_written and
		 * m_lc_written cleared only where they are read after (the write-
		 * backs when careful, the end when the step may branch, the loop
		 * end), and as before for the handler */
		const bool frame = (flags & SPECIAL) && (handler || (plan & (PLAN_COMM_READ | PLAN_COMM_WRITE)));
		const bool branch = (flags & BRANCH) && (handler || writes_pc(*st.ein));

		if (frame)
			write_frame(c, pc, ipa, ipe, st.eop->protect());
		if (handler ? (flags & (SPECIAL | CAREFUL | BRANCH | LOOP_END)) : (flags & CAREFUL)) {
			c.op_mem(0xC7, 0, C::PP, WRITTEN, true);		/* mov qword [m_written], 0 */
			c.word(0);
		}
		if (handler ? (flags & (SPECIAL | CAREFUL | BRANCH | LOOP_END)) : branch) {
			c.op_mem(0xC6, 0, C::PP, PC_WRITTEN);			/* mov byte [m_pc_written], 0 */
			c.byte(0);
		}
		if (handler ? (flags & (SPECIAL | CAREFUL | BRANCH | LOOP_END)) : (flags & LOOP_END)) {
			c.op_mem(0xC7, 0, C::PP, LC_WRITTEN);			/* mov dword [m_lc_written], 0 */
			c.word(0);
		}

		const bool written = (plan & PLAN_WRITTEN) && write_exec(c, st, k, pc, ipa, ipe, (flags & SPECIAL) && !frame, slow);

		if (st.form != C82PpInsn::FORM_NOP && !written) {
			c.pp_first();
			c.move_quad(C::ARG[1], reinterpret_cast<uintptr_t>(st.eop));
			c.move_quad(C::ARG[2], reinterpret_cast<uintptr_t>(st.ein));
			c.op_mem(0x8D, C::ARG[3], C::PP, ea(se, 0), true);	/* lea the fourth argument, the addresses */
			c.call_to(reinterpret_cast<uintptr_t>(EXEC[st.form]));
		}
		/* The write-backs (put(), its mark only when careful) */
		for (unsigned x = 0; x < st.axfers; x++) {
			if (st.wb_slot[x] == 0xFF)
				continue;

			HostCode::Jump skip{ 0 };
			const unsigned code = st.agen[x].wb;

			if (flags & CAREFUL) {
				c.op_mem(0x8B, C::RAX, C::PP, WRITTEN, true);	/* mov rax, [m_written] */
				c.bytes({ 0x48, 0x0F, 0xBA, 0xE0, static_cast<uint8_t>(code) });	/* bt rax, code */
				skip = c.jump_if(0x82);				/* jc skip */
			}
			c.op_mem(0x8B, C::RAX, C::PP, BACK + 4 * static_cast<int32_t>(x));	/* mov eax, written back */
			if (st.wb_keep[x] != ~0u)
				c.op_ri(4, C::RAX, st.wb_keep[x]);		/* and eax, kept */
			c.op_mem(0x89, C::RAX, C::PP, R + 4 * st.wb_slot[x]);	/* mov [register], eax */
			if (flags & CAREFUL) {
				c.op_mem(0x81, 1, C::PP, WRITTEN, true);	/* or qword [m_written], mark */
				c.word(static_cast<uint32_t>(c82_pp_regs::WRITE_MARK[code]));
				c.bind(skip);
			}
		}
		/* The cycles (those a written execute stage adds: on its way) */
		if ((flags & MEMORY) && !written) {
			c.op_mem(0x8B, C::RAX, C::PP, EXTRA);			/* mov eax, [m_extra] */
			c.bytes({ 0x48, 0x29, 0xC5 });				/* sub rbp, rax */
			c.op_mem(0xC7, 0, C::PP, EXTRA);			/* mov dword [m_extra], 0 */
			c.word(0);
		}
		c.bytes({ 0x48, 0x83, 0xED, 0x01 });			/* sub rbp, 1 */
		by_sub = !branch && !(flags & (LOOP_END | LAST));
		/* The ends after the step */
		if (branch) {
			c.op_mem(0x80, 7, C::PP, PC_WRITTEN);			/* cmp byte [m_pc_written], 0 */
			c.byte(0);
			ends.push_back({ c.jump_if(C::JNZ), k << 8 | END_BRANCH });
		}
		if (flags & LOOP_END) {
			c.pp_first();
			c.move_word(C::ARG[1], pc);
			c.call_to(reinterpret_cast<uintptr_t>(&C82Pp::loop_after));
			c.bytes({ 0x85, 0xC0 });				/* test eax, eax */
			ends.push_back({ c.jump_if(C::JNZ), k << 8 | END_LOOP });
		}
		if (flags & LAST)
			ends.push_back({ c.jump_ahead(), k << 8 | END_LAST });
	}
	for (const Entry &e : entries) {
		b.entries[e.k] = c.here();
		c.bytes({ 0x48, 0x85, 0xED });			/* test rbp, rbp */
		c.jump_back(e.jle);
	}
	for (const C82PpSlow &s : slow) {
		write_slow(c, s, stops);
		for (uint32_t from : stops)
			ends.push_back({ HostCode::Jump{ from }, s.k << 8 | END_STOP });
		stops.clear();
	}
	for (End end : ends) {
		const unsigned j = end.code >> 8, why = end.code & 0xFF;
		const uint32_t pc = (b.f0 + 8 * j) | b.bits;
		const int8_t g = b.steps[j].g_change;
		std::vector<HostCode::Jump> out;

		c.bind(end.jump);
		static_assert(END_DONE != END_SLICE && END_DONE > END_LAST);
		if (why == END_SLICE) {
			/* The flow as leave() and run_blocks() would note it
			 * (not halted: run_blocks materializes), step j to resume */
			const unsigned xfers = b.ops[j].xfers;

			c.op_mem(0x80, 7, C::PP, HALTED);			/* cmp byte [m_halted], 0 */
			c.byte(0);
			out.push_back(c.jump_if(C::JNZ));
			c.op_mem(0xC7, 0, C::PP, FLOW_PC);			/* mov dword [f.pc], pc */
			c.word(pc);
			c.move_quad(C::RAX, reinterpret_cast<uintptr_t>(&b));
			c.op_mem(0x89, C::RAX, C::PP, FLOW_B, true);		/* mov [f.b], rax */
			c.op_mem(0xC7, 0, C::PP, FLOW_I);			/* mov dword [f.i], j - 1 */
			c.word(j - 1);
			if (xfers) {
				/* mov eax or rax, the addresses: e_ea[1] 0 for one */
				c.op_mem(0x8B, C::RAX, C::PP, ea((j & 1) ^ 1, 0), xfers > 1);
				c.op_mem(0x89, C::RAX, C::PP, FLOW_EA, true);	/* mov [f.e_ea], rax */
			} else {
				c.op_mem(0xC7, 0, C::PP, FLOW_EA, true);	/* mov qword [f.e_ea], 0 */
				c.word(0);
			}
			c.op_mem(0x89, C::LEFT, C::PP, FLOW_LEFT, true);	/* mov [f.left], rbp */
			c.op_mem(0xC7, 0, C::PP, PC);				/* mov dword [m_pc], pc */
			c.word(pc);
			c.op_mem(0x8B, C::RAX, C::PP, UNTIL, true);		/* mov rax, [until] */
			c.bytes({ 0x48, 0x29, 0xE8 });				/* sub rax, rbp */
			c.op_mem(0x89, C::RAX, C::PP, CYCLES, true);		/* mov [cycles], rax */
			c.op_mem(0xC6, 0, C::PP, FLOWING);			/* mov byte [m_flowing], 1 */
			c.byte(1);
			c.move_quad(C::RAX, c.address(b.entries[j]));
			c.op_mem(0x89, C::RAX, C::PP, RESUME, true);		/* mov [m_resume], rax */
			c.move_word(C::RAX, j << 8 | static_cast<uint32_t>(END_DONE));
			c.jump_back(c.leave());
			for (const HostCode::Jump &jump : out)
				c.bind(jump);
			end.code = j << 8 | END_STOP;
		}
		if (why == END_BRANCH || why == END_LOOP || why == END_LAST) {
			/* pc after the step in edx (run_block), kept for C++ */
			if (why == END_LAST) {
				c.move_word(C::RDX, g >= 0 ? (((pc + 8) & ~2u) | static_cast<uint32_t>(g) << 1) : pc + 8);
			} else {
				if (why == END_LOOP) {
					c.op_mem(0x8B, C::RDX, C::PP, NAT_NEXT);	/* mov edx, [m_nat_next] */
				} else {
					c.op_mem(0x8B, C::RCX, C::PP, NEW_PC_BITS);	/* mov ecx, [m_new_pc_bits] */
					c.bytes({ 0xF7, 0xD1 });			/* not ecx */
					c.op_ri(4, C::RCX, 3);				/* and ecx, 3: the bits kept */
					c.op_mem(0x8B, C::RDX, C::PP, NEW_PC);		/* mov edx, [m_new_pc] */
					c.bytes({ 0x41, 0x89, 0xCA });			/* mov r10d, ecx */
					c.op_ri(1, C::R10, 4);				/* or r10d, 4 */
					c.bytes({ 0x41, 0xF7, 0xD2, 0x44, 0x21, 0xD2 });	/* not r10d / and edx, r10d */
					c.move_word(C::R10, pc);			/* mov r10d, pc */
					c.bytes({ 0x41, 0x21, 0xCA, 0x44, 0x09, 0xD2 });	/* and r10d, ecx / or edx, r10d */
				}
				if (g >= 0) {
					c.op_ri(4, C::RDX, ~2u);			/* and edx, ~2 */
					if (g)
						c.op_ri(1, C::RDX, 2);			/* or edx, 2 */
				}
			}
			c.op_mem(0x89, C::RDX, C::PP, NAT_PC);			/* mov [m_nat_pc], edx */
			/* On to the block that followed last when run_blocks()
			 * would: cycles left, not halted, not the wait, no
			 * interrupt to take; the same pc and loop ends, alive,
			 * in host code */
			c.bytes({ 0x48, 0x85, 0xED });				/* test rbp, rbp */
			out.push_back(c.jump_if(C::JLE));
			if (b.wait) {
				/* Not while a pass is measured (run_blocks
				 * looks at where it goes) */
				c.op_mem(0x80, 7, C::PP, MEASURING);		/* cmp byte [m_park.measuring], 0 */
				c.byte(0);
				out.push_back(c.jump_if(C::JNZ));
			}
			c.op_mem(0x80, 7, C::PP, HALTED);			/* cmp byte [m_halted], 0 */
			c.byte(0);
			out.push_back(c.jump_if(C::JNZ));
			c.bytes({ 0x89, 0xD0 });				/* mov eax, edx */
			c.op_ri(4, C::RAX, ~7u);				/* and eax, ~7 */
			c.op_ri(7, C::RAX, m_wait_pc);				/* cmp eax, the wait */
			out.push_back(c.jump_if(C::JZ));
			c.bytes({ 0xF6, 0xC2, 0x02 });				/* test dl, 2 */
			const HostCode::Jump no_g = c.jump_if(C::JZ);
			c.op_mem(0x8B, C::RAX, C::PP, INTFLG);			/* mov eax, [m_intflg] */
			c.op_mem(0x23, C::RAX, C::PP, INTEN);			/* and eax, [m_inten] */
			c.op_ri(4, C::RAX, ~1u);				/* and eax, ~1 */
			out.push_back(c.jump_if(C::JNZ));
			c.bind(no_g);
			c.move_quad(C::RAX, reinterpret_cast<uintptr_t>(&b.successors[j]));
			c.op_mem(0x39, C::RDX, C::RAX, S_PC);			/* cmp [successor pc], edx */
			out.push_back(c.jump_if(C::JNZ));
			c.op_mem(0x8B, C::RCX, C::RAX, S_BLOCK, true);		/* mov rcx, [successor block] */
			c.bytes({ 0x48, 0x85, 0xC9 });				/* test rcx, rcx */
			out.push_back(c.jump_if(C::JZ));
			c.op_mem(0x80, 7, C::RCX, DEAD);			/* cmp byte [rcx + dead], 0 */
			c.byte(0);
			out.push_back(c.jump_if(C::JNZ));
			for (unsigned n = 0; n < 3; n++) {
				/* ends_hold() */
				c.op_mem(0x8B, C::RAX, C::PP, ENDS + 4 * static_cast<int32_t>(n));	/* mov eax, [m_ends + 4n] */
				c.op_mem(0x3B, C::RAX, C::RCX, B_ENDS + 4 * static_cast<int32_t>(n));	/* cmp eax, [rcx + ends + 4n] */
				const HostCode::Jump same = c.jump_if(C::JZ);
				c.op_mem(0x83, 7, C::RCX, B_ENDS + 4 * static_cast<int32_t>(n));	/* cmp dword [rcx + ends + 4n], 1 */
				c.byte(1);
				out.push_back(c.jump_if(C::JNZ));
				c.op_mem(0x2B, C::RAX, C::RCX, B_F0);			/* sub eax, [rcx + f0] */
				c.op_mem(0x3B, C::RAX, C::RCX, B_SPAN);			/* cmp eax, [rcx + span] */
				out.push_back(c.jump_if(0x82));				/* jb out */
				c.bind(same);
			}
			c.move_quad(C::RAX, reinterpret_cast<uintptr_t>(&b.successors[j]));
			c.op_mem(0x8B, C::RCX, C::RAX, S_CODE, true);		/* mov rcx, [successor code] */
			c.bytes({ 0x48, 0x85, 0xC9 });				/* test rcx, rcx */
			out.push_back(c.jump_if(C::JZ));
			c.op_mem(0x63, C::R10, C::RAX, S_SLOT, true);		/* movsxd r10, [successor slot] */
			c.op_mem(0x8B, C::R11, C::PP, ea(j & 1, 0), true);	/* mov r11, the addresses made */
			c.bytes({ 0x4E, 0x89, 0x1C, 0x13 });			/* mov [rbx + r10], r11 */
			c.op_mem(0x8B, C::HOST, C::RAX, S_HOST, true);		/* mov r13, [successor host] */
			c.bytes({ 0xFF, 0xE1 });				/* jmp rcx */
			for (const HostCode::Jump &jump : out)
				c.bind(jump);
		}
		c.op_mem(0x89, C::LEFT, C::PP, LEFT, true);			/* mov [m_nat_left], rbp */
		c.move_quad(C::RCX, reinterpret_cast<uintptr_t>(&b));
		c.op_mem(0x89, C::RCX, C::PP, NAT_BLOCK, true);			/* mov [m_nat_block], rcx */
		c.move_word(C::RAX, end.code);
		c.jump_back(c.leave());
	}
	c.ready();
	if (!checked)
		mark_lines(b);
#else
	(void)b;
#endif
}

/* A block's steps in host code from step k: the same ends and the same flow
 * as run_block() */
C82Pp::BlockEnd
C82Pp::run_native(Block &b, unsigned k, Flow &f, uint64_t until)
{
	const unsigned se = (k & 1) ^ 1;

	m_nat_ea[se][0] = f.e_ea[0];
	m_nat_ea[se][1] = f.e_ea[1];
	m_nat_until = until;
	m_extra = 0;

	return native_end(m_code->run(this, b.entries[k], b.host, f.left), f);
}

/* What the host code returned, the flow as it ended (END_DONE: noted by the
 * code itself, before a step with no cycle left) */
C82Pp::BlockEnd
C82Pp::native_end(uint64_t out, Flow &f)
{
	const unsigned j = static_cast<unsigned>(out >> 8), why = static_cast<unsigned>(out & 0xFF);

	if (why == END_DONE)
		return BlockEnd::STOP;

	/* The block it ended in, the next ones being taken by the code itself */
	Block &end = *m_nat_block;

	if (why == END_STOP || why == END_CHANGED) {
		leave(f, end, static_cast<int>(j) - 1, (end.f0 + 8 * j) | end.bits, m_nat_ea[(j & 1) ^ 1], m_nat_left);
		return why == END_STOP ? BlockEnd::STOP : BlockEnd::CHANGED;
	}
	leave(f, end, static_cast<int>(j), m_nat_pc, m_nat_ea[j & 1], m_nat_left);
	return BlockEnd::NEXT;
}

/* A block resumed by run() (resume) that did not end with the slice:
 * as run_blocks() goes on after run_native() */
int
C82Pp::resumed(uint64_t out, uint64_t until)
{
	const BlockEnd end = native_end(out, m_flow);

	return blocks_on(until, &end);
}

} /* namespace odin */
