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

#include <algorithm>
#include <cstring>

#include "odin/machine/cpu/tms320c82/c82_pp_host.hpp"
#include "odin/machine/cpu/tms320c82/tms320c82.hpp"

/* The PP's prepared blocks (c82_pp.hpp, Block): made, found, left. A block
 * does what cycle() and steps() do, cycle for cycle; what it leaves to them:
 * the pseudo-instructions, the wait and its parking, interrupts, illegal
 * instructions, and every cycle whose execute stage reaches the board or a
 * faulting address */

namespace odin {

namespace {

/* Steps in a block at most, and in all the blocks of a PP (then they are all
 * dropped) */
constexpr unsigned MAX_STEPS = 64;
constexpr size_t ALL_STEPS = 1u << 15;

/* What the execute stage of an instruction does besides plain registers and
 * memory: reads or writes a special register (code 0x28 up); may write pc,
 * always does; writes a special register after which the block ends; the
 * registers it may write as m_written marks them; its memory accesses
 * (transfer, size) in their order */
struct Traits {
	bool special = false, branch = false, always = false, ends = false;
	uint64_t marks = 0;
	uint8_t mems = 0;
	uint8_t mem_x[2] = {}, mem_size[2] = {};
	/* The accesses that are stores (bit m) */
	uint8_t mem_store = 0;
};

/* A register written by the execute stage. Writes to sr, mf, iprs only
 * change what later instructions read; to pc they branch; any other
 * (interrupts, comm, loops) ends the block */
void
wrote(Traits &t, unsigned code)
{
	code &= 0x7F;
	if (code < 0x28) {
		t.marks |= c82_pp_regs::WRITE_MARK[code];
		return;
	}
	t.special = true;
	if (code == 0x38 || code == 0x39)
		t.branch = true;
	else if (code != 0x29 && code != 0x2A && code != 0x3B)
		t.ends = true;
}

void
memory(Traits &t, unsigned x, unsigned size, bool store)
{
	t.mem_store = static_cast<uint8_t>(t.mem_store | (store ? 1u << t.mems : 0u));
	t.mem_x[t.mems] = static_cast<uint8_t>(x);
	t.mem_size[t.mems++] = static_cast<uint8_t>(size);
}

/* The traits by the shapes of exec(); the generic one as execute_generic
 * may do anything: always made up to date first, careful, a possible
 * branch */
Traits
traits(const C82PpInsn &in, const C82PpOp &op)
{
	Traits t;

	switch (in.shape) {
	case C82PpInsn::NOP:
		break;
	case C82PpInsn::LOAD:
		wrote(t, op.reg);
		t.always = t.branch;
		memory(t, 0, op.msize, false);
		break;
	case C82PpInsn::STORE:
		t.special = (op.reg & 0x7F) >= 0x28;
		memory(t, 0, op.msize, true);
		break;
	case C82PpInsn::LEA:
		wrote(t, op.reg);
		t.always = t.branch;
		break;
	case C82PpInsn::DU:
	case C82PpInsn::DU_LOAD:
	case C82PpInsn::DU_STORE:
		wrote(t, op.dst);
		t.always = t.branch && !op.has(C82PpOp::R);
		if (in.shape == C82PpInsn::DU_LOAD)
			wrote(t, op.reg);
		else if (in.shape == C82PpInsn::DU_STORE)
			t.special |= (op.reg & 0x7F) >= 0x28;
		if (in.shape != C82PpInsn::DU)
			memory(t, 0, op.msize, in.shape == C82PpInsn::DU_STORE);
		break;
	default:
		t.special = t.branch = true;
		t.marks = ~0ull;
		if (in.path != C82PpInsn::NO_DU)
			wrote(t, in.dst_code());
		for (unsigned k = 0; k < in.xfers; k++)
			if (in.xfer[k].le != 0)
				wrote(t, in.xfer[k].reg);
		if (in.move || in.fmove) {
			if (in.move_dst == 0xF && in.move)
				t.ends = true;
			else
				wrote(t, in.move_dst);
		}
		for (unsigned j = 0; j < in.mems; j++)
			memory(t, in.mem_order[j], in.xfer[in.mem_order[j]].msize, in.xfer[in.mem_order[j]].le == 0);
		break;
	}
	return t;
}

unsigned
slot_of(uint32_t addr)
{
	return (addr >> 3) & 4095;
}

} /* namespace */

void
C82Pp::flush_blocks()
{
	if (m_code)
		m_code->clear();
	m_blocks.clear();
	m_block_steps = 0;
	std::fill(m_entry.begin(), m_entry.end(), nullptr);
	std::fill(m_inside.begin(), m_inside.end(), Inside{});
	m_resume = nullptr;
	m_full = false;
	/* No line fetched any more (the MP's marks stay: writes there
	 * only reach code_written for nothing) */
	for (uint64_t *lines : m_code_lines)
		if (lines)
			std::fill(lines, lines + 256, 0);
}

/* The lines of the block's fetches, in the PP's marks and the MP's */
void
C82Pp::mark_lines(const Block &b)
{
	const uint64_t hi = static_cast<uint64_t>(b.f0) + 8 * static_cast<uint64_t>(b.n);

	for (uint64_t at = b.f0 & ~63ull; at < hi; at += 64) {
		const uint32_t line = static_cast<uint32_t>(at);
		uint64_t *&lines = m_code_lines[line >> 20];

		if (!lines) {
			m_line_store.push_back(std::make_unique<uint64_t[]>(256));
			lines = m_line_store.back().get();
		}
		lines[(line & 0xFFFFF) >> 12] |= 1ull << ((line >> 6) & 63);
	}
	/* A block ends at 2^32 at most: its last line starts below 2^32 - 1 */
	m_chip.mp().mark_lines(b.f0, static_cast<uint32_t>(std::min<uint64_t>(hi, 0xFFFFFFFFu)));
}

/* Each line written that a block fetches from: the blocks fetching
 * there dropped (all of them: the line is then fetched by none), and the
 * resumption forgotten. A block in host code that made the write itself
 * (a handler it called) ends before its next step (write_native) */
void
C82Pp::drop_code(uint32_t addr, uint32_t bytes)
{
	for (uint64_t at = addr & ~63ull; at < static_cast<uint64_t>(addr) + bytes; at += 64) {
		const uint32_t line = static_cast<uint32_t>(at);
		uint64_t *const lines = m_code_lines[line >> 20];
		const uint64_t bit = 1ull << ((line >> 6) & 63);

		if (!lines || !(lines[(line & 0xFFFFF) >> 12] & bit))
			continue;
		lines[(line & 0xFFFFF) >> 12] &= ~bit;
		for (const std::unique_ptr<Block> &b : m_blocks)
			if (!b->dead && b->f0 < at + 64 && static_cast<uint64_t>(b->f0) + 8 * static_cast<uint64_t>(b->n) > at)
				b->dead = true;
		m_resume = nullptr;
	}
}

void
C82Pp::code_page_cleared(uint32_t addr)
{
	uint64_t *const lines = m_code_lines[addr >> 20];

	if (!lines)
		return;
	std::fill(lines, lines + 256, 0);
	for (const std::unique_ptr<Block> &b : m_blocks)
		if (!b->dead && (b->f0 >> 20) == (addr >> 20))
			b->dead = true;
	m_resume = nullptr;
}

/* The loop ends within the block as when it was made (the host code makes
 * the same test: write_native) */
bool
C82Pp::ends_hold(const Block &b) const
{
	for (unsigned n = 0; n < 3; n++)
		if (m_ends[n] != b.ends[n] && (b.ends[n] != 1 || m_ends[n] - b.f0 < b.span))
			return false;
	return true;
}

/* A block that starts from this state: at one of its steps k >= 2 (the two
 * steps before it fetched the stages' words), or at its first step (the
 * same stages) */
C82Pp::Block *
C82Pp::find_block(const Flow &f, unsigned &k)
{
	const uint32_t at = f.pc & ~7u, bits = f.pc & 3;
	const unsigned slot = slot_of(at);
	const Inside &in = m_inside[slot];
	const InStage a = in_stage(f, 0), e = in_stage(f, 1);

	if (Block *b = in.block; b && !b->dead && b->f0 + 8 * in.k == at && b->bits == bits && a.addr == ((at - 8) | bits) &&
	                         e.addr == ((at - 16) | bits) && b->steps[in.k - 1].raw == a.raw &&
	                         b->steps[in.k - 2].raw == e.raw && ends_hold(*b)) {
		k = in.k;
		return b;
	}
	for (Block **p = &m_entry[slot]; *p;) {
		Block *b = *p;

		if (b->dead) {
			*p = b->next;
			continue;
		}
		if (b->f0 == at && b->bits == bits && b->ipa == a.addr && b->ipe == e.addr && b->a_raw == a.raw &&
		    b->e_raw == e.raw && ends_hold(*b)) {
			k = 0;
			return b;
		}
		p = &b->next;
	}
	return nullptr;
}

/* A block from this state: the words from pc on, in one piece of host
 * memory (a RAM block of 4 KB, a page of 1 MB), up to an illegal word or the
 * wait's first one (left out), a fetched eint or dint, an execute stage that
 * always branches or ends blocks (kept) */
C82Pp::Block *
C82Pp::translate(const Flow &f)
{
	const uint32_t at = f.pc & ~7u;
	const uint8_t *const host = m_chip.pp_code(at);

	if (!host)
		return nullptr;

	const uint32_t room = (at >= 0x02000000u ? 0x100000u - (at & 0xFFFFFu) : 0x1000u - (at & 0xFFFu)) / 8;
	auto made = std::make_unique<Block>();
	Block &b = *made;
	std::vector<Traits> tr;

	const InStage a = in_stage(f, 0), e = in_stage(f, 1);

	/* A block starting in the wait's words stays in them (none
	 * other reaches them: the wait's first one ends blocks) */
	const uint32_t wait_bytes = 8 * static_cast<uint32_t>(m_wait_words.size());

	b.wait = at - m_wait_pc < wait_bytes;
	b.f0 = at;
	b.bits = f.pc & 3;
	b.host = host;
	b.ipa = a.addr;
	b.ipe = e.addr;
	b.a_raw = a.raw;
	b.e_raw = e.raw;
	b.ops.reserve(MAX_STEPS + 2);
	b.insns.reserve(MAX_STEPS + 2);
	for (const InStage &s : { e, a }) {
		b.ops.push_back(*s.op);
		b.insns.push_back(*s.in);
		tr.push_back(traits(*s.in, *s.op));
	}

	unsigned n = 0;

	while (n < MAX_STEPS && n < room) {
		const uint32_t addr = at + 8 * n;

		if (addr == m_wait_pc || (b.wait && addr - m_wait_pc >= wait_bytes))
			break;

		const uint64_t word = c82_get(host + 8 * n, 3);
		const unsigned slot = slot_of(addr);

		if (m_ops[slot].addr == addr && m_decoded[slot].word == word) {
			b.insns.push_back(m_decoded[slot].insn);
			b.ops.push_back(m_ops[slot]);
		} else {
			b.insns.push_back(c82_pp_decode(word));
			b.ops.push_back(c82_pp_op(b.insns.back()));
		}
		if (b.insns.back().illegal) {
			b.insns.pop_back();
			b.ops.pop_back();
			break;
		}
		tr.push_back(traits(b.insns.back(), b.ops.back()));
		n++;
		/* Step n - 1 executes instruction n - 3, at index n - 1 */
		if (tr[n - 1].ends || tr[n - 1].always || b.insns.back().g_change >= 0)
			break;
	}
	if (!n)
		return nullptr;
	b.n = n;
	b.span = b.bits & 1 ? 0 : 8 * n;
	for (unsigned l = 0; l < 3; l++)
		b.ends[l] = m_ends[l] - at < b.span ? m_ends[l] : 1u;
	b.steps.resize(n);
	b.successors.resize(n);
	for (unsigned k = 0; k < n; k++) {
		BlockStep &s = b.steps[k];
		const Traits &t = tr[k];
		const uint32_t addr = at + 8 * k;

		const C82PpOp &aop = b.ops[k + 1];

		std::memcpy(&s.raw, host + 8 * k, 8);
		s.eop = &b.ops[k];
		s.ein = &b.insns[k];
		s.form = s.eop->form;
		s.axfers = aop.xfers;
		for (unsigned x = 0; x < s.axfers; x++) {
			s.agen[x] = aop.agen[x];
			if (aop.agen[x].wb != 0xFF) {
				s.wb_slot[x] = c82_pp_regs::WRITE_SLOT[aop.agen[x].wb];
				s.wb_keep[x] = c82_pp_regs::WRITE_KEEP[aop.agen[x].wb];
			}
		}
		s.mems = t.mems;
		s.mem_store = t.mem_store;
		for (unsigned m = 0; m < 2; m++) {
			s.mem_x[m] = t.mem_x[m];
			s.mem_size[m] = t.mem_size[m];
		}
		s.g_change = b.insns[k + 2].g_change;
		if (!(b.bits & 1) && (addr == b.ends[0] || addr == b.ends[1] || addr == b.ends[2]))
			s.flags |= LOOP_END;
		if (t.mems)
			s.flags |= MEMORY;
		if (t.special)
			s.flags |= SPECIAL;
		if (t.branch)
			s.flags |= BRANCH;
		/* The execute stage may write a register written back, or the
		 * first write-back marks the second's (which is then left out) */
		const uint8_t wb0 = aop.agen[0].wb, wb1 = aop.agen[1].wb;

		for (unsigned x = 0; x < s.axfers; x++)
			if (aop.agen[x].wb != 0xFF && ((t.marks >> aop.agen[x].wb) & 1))
				s.flags |= CAREFUL;
		if (s.axfers == 2 && wb0 != 0xFF && wb1 != 0xFF && ((c82_pp_regs::WRITE_MARK[wb0] >> wb1) & 1))
			s.flags |= CAREFUL;
		if (k == n - 1)
			s.flags |= LAST;
	}

	const unsigned slot = slot_of(at);

	b.next = m_entry[slot];
	m_entry[slot] = &b;
	for (unsigned k = 2; k < n; k++)
		m_inside[slot_of(at + 8 * k)] = Inside{ &b, k };
	m_block_steps += n;
	m_blocks.push_back(std::move(made));
	return &b;
}

/* The flow as a block leaves it: pc, the instruction of index i of the block
 * in the address stage, i - 1 in the execute stage with the addresses `ea`
 * (those of its transfers only: a stage is fetched with none) */
void
C82Pp::leave(Flow &f, Block &b, int i, uint32_t pc, const uint32_t *ea, int64_t left) const
{
	const unsigned xfers = b.ops[static_cast<unsigned>(i + 1)].xfers;

	f.pc = pc;
	f.b = &b;
	f.i = i;
	f.e_ea[0] = xfers > 0 ? ea[0] : 0;
	f.e_ea[1] = xfers > 1 ? ea[1] : 0;
	f.left = left;
}

/* A stage as the interpreter fills it, its instruction from the decoding
 * cache (made again there when it was replaced) */
void
C82Pp::fill(Stage &s, uint32_t addr, uint64_t raw, const uint32_t *ea)
{
	const uint64_t word = __builtin_bswap64(raw);
	const uint32_t at = addr & ~7u;
	const unsigned k = slot_of(at);

	if (m_ops[k].addr != at || m_decoded[k].word != word)
		decoded(at, word, m_chip.pp_code(at));
	s.kind = INSN;
	s.addr = addr;
	s.word = word;
	s.ea = { ea ? ea[0] : 0u, ea ? ea[1] : 0u };
	s.divert = false;
	s.insn = &m_decoded[k].insn;
	s.op = &m_ops[k];
	s.stop = s.insn->illegal;
}

/* Back to the interpreter: the address and execute stages as m_flow has them
 * (pc and the cycles are up to date) */
void
C82Pp::materialize()
{
	const InStage a = in_stage(m_flow, 0), e = in_stage(m_flow, 1);

	m_flowing = false;
	m_resume = nullptr;
	m_sa = &m_stages[0];
	m_se = &m_stages[1];
	m_sf = &m_stages[2];
	m_sa->insn = m_se->insn = &NO_INSN;
	fill(*m_sa, a.addr, a.raw, nullptr);
	fill(*m_se, e.addr, e.raw, m_flow.e_ea);
}

/* run()'s first try: blocks as long as they can go, from where the last
 * slice left them or from the stages. 1: they ran to the end of the slice
 * (or the PP halted); 0: the next cycle is the interpreter's (the stages
 * made up to date); -1: a state blocks never start from (pseudo-
 * instructions, parked, a pass measured, an empty stage, an illegal
 * instruction about to stall) */
int
C82Pp::run_blocks(uint64_t until)
{
	Flow &f = m_flow;

	/* Too many blocks, or no room for their host code: all dropped, from
	 * the stages */
	if (m_block_steps > ALL_STEPS || (m_code && !m_code->room())) {
		settle_flow();
		flush_blocks();
	}
	if (!m_flowing) {
		if (m_seq != NONE || m_park.on || m_sa->kind != INSN || m_se->kind != INSN || m_sa->stop)
			return -1;
		f.pc = m_pc;
		f.b = nullptr;
		f.i = 0;
		f.e_ea[0] = m_se->ea[0];
		f.e_ea[1] = m_se->ea[1];
	}
	/* Not while the blocks run: what they call may settle the PP
	 * (disturb) */
	m_flowing = false;
	m_resume = nullptr;
	f.left = static_cast<int64_t>(until - m_cycles);
	return blocks_on(until, nullptr);
}

/* After a block in run_blocks' loop: false when the loop stops there (before
 * a step, cycles left) */
inline bool
C82Pp::after_block(BlockEnd end)
{
	/* The cycles of a pass measured counted by wait blocks without
	 * their limit: past it, spoiled as cycle() would have */
	if (m_park.measuring) {
		if (m_park.cycles > Park::CYCLES) {
			m_park.cycles = Park::CYCLES;
			m_park.spoiled = true;
		}
		if (m_park.spoiled)
			stop_watching();
	}
	if (end == BlockEnd::STOP && m_flow.left > 0)
		return false;
	if (end == BlockEnd::CHANGED)
		m_flow.b->dead = true;
	return true;
}

/* run_blocks' loop; `first`: the end of the block run() resumed (resume),
 * the loop's first block, run_blocks' checks before it made there */
int
C82Pp::blocks_on(uint64_t until, const BlockEnd *first)
{
	Flow &f = m_flow;

	const bool on = !first || after_block(*first);

	while (on && f.left > 0 && !((f.pc & 2) && (m_intflg & m_inten & ~1u)) && (f.pc & ~7u) != m_wait_pc && !m_halted) {
		unsigned k = 0;
		Block *b = f.b;

		/* A pass measured fetches out of the wait's words: spoiled
		 * (note_fetch), what it watches no longer matters (steps()) */
		if (m_park.measuring && (f.pc & ~7u) - m_wait_pc >= 8 * m_wait_words.size()) {
			m_park.spoiled = true;
			stop_watching();
		}

		/* On from where the slice ended, or the block that followed
		 * last, or one found or made */
		if (b && !b->dead && f.i + 1 < static_cast<int>(b->n) && f.pc == b->addr(f.i + 1)) {
			k = static_cast<unsigned>(f.i + 1);
		} else if (const Block::Successor *next = b && f.i >= 0 ? &b->successors[static_cast<unsigned>(f.i)] : nullptr;
		           next && next->block && next->pc == f.pc && !next->block->dead && ends_hold(*next->block)) {
			k = next->k;
			b = next->block;
		} else {
			b = find_block(f, k);
			if (!b) {
				if (!(b = translate(f)))
					break;
				write_native(*b);
				/* run_blocks' room check, for resume */
				m_full = m_block_steps > ALL_STEPS || (m_code && !m_code->room());
			}
			if (f.b && f.i >= 0)
				f.b->successors[static_cast<unsigned>(f.i)] = {
					b, f.pc, k, b->entries.empty() ? nullptr : reinterpret_cast<const uint8_t *>(m_code->address(b->entries[k])),
					b->host, HostCode::place_in(this, m_nat_ea[(k & 1) ^ 1])
				};
		}

		/* Alone, only blocks in host code (they stop before what
		 * the others could see) */
		if (m_alone && b->entries.empty())
			break;
		if (!after_block(b->entries.empty() ? run_block(*b, k, f, until) : run_native(*b, k, f, until)))
			break;
	}
	m_pc = f.pc;
	m_cycles = until - static_cast<uint64_t>(f.left);
	if (f.left > 0 || m_halted) {
		materialize();
		return m_halted ? 1 : 0;
	}
	m_flowing = true;
	return 1;
}

} /* namespace odin */
