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


#include "odin/machine/cpu/tms320c82/c82_mp_host.hpp"

#include <algorithm>
#include <cstring>
#include <utility>
#include <vector>

#include "odin/machine/cpu/tms320c82/c82_mp.hpp"
#include "odin/machine/cpu/tms320c82/tms320c82.hpp"

/* The MP's blocks in host code: each step as fast_steps() does it, in its
 * order (the cycles counted, the instruction), its body written out
 * (c82_mp_host_exec.cpp) or its prepared handler called; the state written
 * where fast_steps() would hand over. Their words are not checked at each
 * step: any write into a line of 64 bytes they read drops them first
 * (code_written). x86-64 only: on other hosts (AArch64 too, for now)
 * fast_steps() interprets alone. */

namespace odin {

namespace {

/* The memory of the MP's host code: the steps' part first, then the ways out
 * of their line; the most one block can take in each */
constexpr uint32_t SIZE = 24u << 20, HOT = 8u << 20, LONGEST = 160u << 10;
/* Steps in a block at most, and in all the blocks (then all are dropped) */
constexpr unsigned MAX_STEPS = 64;
constexpr size_t ALL_STEPS = 1u << 17;

bool
is_branch(C82Insn::Op op)
{
	return op == C82Insn::BSR || op == C82Insn::JSR || op == C82Insn::BBZ || op == C82Insn::BBO || op == C82Insn::BCND;
}

} /* namespace */

C82MpCode::C82MpCode() : HostCode(SIZE), m_enter(nullptr) {}

bool
C82MpCode::usable() const
{
#if defined(ODIN_HOST_X86_64)
	return HostCode::usable();
#else
	return false;
#endif
}

bool
C82MpCode::room() const
{
	const uint32_t hot = m_cold ? m_other : m_used, cold = m_cold ? m_used : m_other;

	return hot + LONGEST <= HOT && cold + LONGEST <= m_size;
}

void
C82MpCode::hot()
{
	if (m_cold)
		std::swap(m_used, m_other);
	m_cold = false;
}

void
C82MpCode::cold()
{
	if (!m_cold)
		std::swap(m_used, m_other);
	m_cold = true;
}

#if defined(ODIN_HOST_X86_64)

/* The way in keeps the MP in rbx (way_in), the pages in r14, the slice's
 * end in r15 and the cycles left in rbp, then jumps to the block; the two
 * ways back, with the cycles from rbp or as they are */
void
C82MpCode::clear()
{
	writable();
	m_cold = false;
	m_other = HOT;
	way_in();
	op_r(0, { 0x89 }, ARG[2], PAGES, true);				/* mov r14, the third argument */
	op_r(0, { 0x89 }, ARG[3], LIMIT, true);				/* mov r15, the fourth */
	op_r(0, { 0x89 }, LIMIT, LEFT, true);				/* mov rbp, r15 */
	op_m(0, { 0x2B }, LEFT, Mem{ MP, NO, 0, m_cycles_at }, true);	/* sub rbp, [cycles] */
	way_in_end();
	m_leave_left = m_used;
	op_r(0, { 0x89 }, LIMIT, RCX, true);				/* mov rcx, r15 */
	op_r(0, { 0x29 }, LEFT, RCX, true);				/* sub rcx, rbp */
	op_m(0, { 0x89 }, RCX, Mem{ MP, NO, 0, m_cycles_at }, true);	/* mov [cycles], rcx */
	m_leave = m_used;
	way_out();
	m_enter = reinterpret_cast<uint64_t (*)(void *, const uint8_t *, const void *, uint64_t)>(m_code);
	ready();
}

void
C82MpCode::op_m(uint8_t prefix, std::initializer_list<uint8_t> opcode, uint8_t reg, Mem m, bool wide, bool low)
{
	const bool sib = m.index != NO || (m.base & 7) == RSP;
	const uint8_t rex = static_cast<uint8_t>(0x40 | (wide ? 8 : 0) | (reg >= 8 ? 4 : 0) |
	                                         (m.index != NO && m.index >= 8 ? 2 : 0) | (m.base >= 8 ? 1 : 0));
	const bool near = m.disp == static_cast<int8_t>(m.disp);

	if (prefix)
		byte(prefix);
	if (rex != 0x40 || (low && reg >= 4))
		byte(rex);
	bytes(opcode);
	/* mod 01 (8-bit displacement) or 10 (32-bit) */
	byte(static_cast<uint8_t>((near ? 0x40 : 0x80) | (reg & 7) << 3 | (sib ? RSP : m.base & 7)));
	if (sib)
		byte(static_cast<uint8_t>(m.scale << 6 | ((m.index == NO ? RSP : m.index) & 7) << 3 | (m.base & 7)));
	if (near)
		byte(static_cast<uint8_t>(m.disp));
	else
		word(static_cast<uint32_t>(m.disp));
}

void
C82MpCode::op_r(uint8_t prefix, std::initializer_list<uint8_t> opcode, uint8_t reg, uint8_t rm, bool wide, bool low)
{
	const uint8_t rex = static_cast<uint8_t>(0x40 | (wide ? 8 : 0) | (reg >= 8 ? 4 : 0) | (rm >= 8 ? 1 : 0));

	if (prefix)
		byte(prefix);
	if (rex != 0x40 || (low && (reg >= 4 || rm >= 4)))
		byte(rex);
	bytes(opcode);
	byte(static_cast<uint8_t>(0xC0 | (reg & 7) << 3 | (rm & 7)));
}

void
C82MpCode::align8()
{
	while (m_used & 7)
		byte(0xCC);					/* int3 */
}

void
C82MpCode::relink(uint32_t at, uint32_t place)
{
	const int32_t displacement = static_cast<int32_t>(place - (at + 4));

	std::memcpy(m_code + at, &displacement, 4);
}

void
C82MpCode::move32(uint8_t reg, uint32_t value)
{
	mov_ri(reg, value);
}

void
C82MpCode::move64(uint8_t reg, uint64_t value)
{
	if (value <= 0xFFFFFFFFu) {
		mov_ri(reg, static_cast<uint32_t>(value));
		return;
	}
	bytes({ static_cast<uint8_t>(0x48 | (reg >= 8 ? 1 : 0)), static_cast<uint8_t>(0xB8 | (reg & 7)) });	/* mov reg, value */
	quad(value);
}

void
C82MpCode::swap(uint8_t reg, bool wide)
{
	if (wide || reg >= 8)
		byte(static_cast<uint8_t>(0x40 | (wide ? 8 : 0) | (reg >= 8 ? 1 : 0)));
	bytes({ 0x0F, static_cast<uint8_t>(0xC8 | (reg & 7)) });	/* bswap reg */
}

HostCode::Jump
C82MpCode::jump_if(Cc cc)
{
	return HostCode::jump_if(static_cast<uint8_t>(0x80 | cc));
}

HostCode::Jump
C82MpCode::jump_ahead()
{
	byte(0xE9);					/* jmp ... */
	m_used += 4;
	return Jump{ m_used - 4 };
}

void
C82MpCode::jump_if_to(Cc cc, uint32_t place)
{
	bytes({ 0x0F, static_cast<uint8_t>(0x80 | cc) });	/* j<cc> place */
	jump_to(place);
}

#else

void C82MpCode::clear() {}
/* No blocks on this host: never called */
void C82MpCode::relink(uint32_t, uint32_t) {}

#endif

C82Mp::~C82Mp() = default;

void
C82Mp::use_host_code(bool on)
{
	disturb();
	flush_blocks();
	m_native = on;
}

void
C82Mp::flush_blocks()
{
	if (m_code && m_code->usable()) {
		m_code->clear();
		write_helpers();
	}
	m_blocks.clear();
	m_block_at.clear();
	m_inside.clear();
	std::fill(m_links.begin(), m_links.end(), Link{});
	m_block_steps = 0;
	m_resume_ip = ~0u;
	m_waiting.clear();
	m_chained.clear();
	/* No line read by a block any more */
	for (uint32_t page = 0; page < m_code_lines.size(); page++)
		if (m_code_lines[page]) {
			std::fill(m_code_lines[page], m_code_lines[page] + 256, 0);
			m_chip.set_code_page(page << 20, false);
		}
}

void
C82Mp::mark_code(uint32_t lo, uint32_t hi)
{
	for (uint64_t at = lo & ~63ull; at < hi; at += 64) {
		const uint32_t line = static_cast<uint32_t>(at);
		uint64_t *&lines = m_code_lines[line >> 20];

		if (!lines) {
			m_line_store.push_back(std::make_unique<uint64_t[]>(256));
			lines = m_line_store.back().get();
		}
		lines[(line & 0xFFFFF) >> 12] |= 1ull << ((line >> 6) & 63);
		m_chip.set_code_page(line, true);
	}
}

/* Each line written that a block reads: the blocks with a word in it
 * dropped (all of them: the line is then read by none), the next run
 * started afresh, and a whole step taken first (m_check): when the MP itself
 * writes there from a block, the block stops after that step */
void
C82Mp::drop_code(uint32_t addr, uint32_t bytes)
{
	for (uint64_t at = addr & ~63ull; at < static_cast<uint64_t>(addr) + bytes; at += 64) {
		const uint32_t line = static_cast<uint32_t>(at);
		uint64_t *const lines = m_code_lines[line >> 20];
		const uint64_t bit = 1ull << ((line >> 6) & 63);

		if (!lines || !(lines[(line & 0xFFFFF) >> 12] & bit))
			continue;
		lines[(line & 0xFFFFF) >> 12] &= ~bit;
		for (uint32_t id = 0; id < m_blocks.size(); id++) {
			const Block &b = *m_blocks[id];

			if (!b.dead && b.ip < line + 64 && b.end > line)
				kill_block(id);
		}
		m_resume_ip = ~0u;
		m_check = true;
	}
}

void
C82Mp::kill_block(uint32_t id)
{
	if (id >= m_blocks.size() || m_blocks[id]->dead)
		return;

	Block &b = *m_blocks[id];

	b.dead = true;
	if (const auto it = m_block_at.find(b.ip); it != m_block_at.end() && it->second == id)
		m_block_at.erase(it);
	/* Its links: from its first step, and into steps within it */
	for (Link &link : m_links)
		if (link.ip != ~0u && link.id == id)
			link = Link{};
	/* The jumps chained into it: through their links again, waiting */
	if (id < m_chained.size() && !m_chained[id].empty()) {
		m_code->writable();
		for (const Chain &site : m_chained[id]) {
			m_code->relink(site.at, site.stub);
			if (!m_blocks[site.owner]->dead)
				m_waiting[site.ip].push_back(site);
		}
		m_code->ready();
		m_chained[id].clear();
	}
}

/* The code find_block() gives for `ip` without making any: the block that
 * starts there, else a live block's step there; null when none */
const uint32_t *
C82Mp::code_for(uint32_t ip, uint32_t &id) const
{
	if (const auto it = m_block_at.find(ip); it != m_block_at.end()) {
		id = it->second;
		return &m_blocks[id]->entries[0];
	}
	if (const auto in = m_inside.find(ip); in != m_inside.end() && !m_blocks[in->second.id]->dead) {
		id = in->second.id;
		return &m_blocks[id]->entries[in->second.k];
	}
	return nullptr;
}

/* A jump sent to the code of its instruction when there is some, else
 * left on its link, waiting */
void
C82Mp::chain(const Chain &site)
{
	uint32_t id = 0;

	if (m_blocks[site.owner]->dead)
		return;
	if (const uint32_t *place = code_for(site.ip, id)) {
		m_code->relink(site.at, *place);
		m_chained[id].push_back(site);
	} else {
		m_waiting[site.ip].push_back(site);
	}
}

/* The jumps waiting for `ip`, now that it may have code */
void
C82Mp::settle(uint32_t ip)
{
	const auto it = m_waiting.find(ip);

	if (it == m_waiting.end())
		return;

	const std::vector<Chain> sites = std::move(it->second);

	m_waiting.erase(it);
	for (const Chain &site : sites)
		chain(site);
}

/* From fast_steps(), in the normal state at `ip` (not the wait): the block
 * there run (on from where the last run stopped, or from its first
 * instruction); false when there is none (the interpreter takes the step) */
bool
C82Mp::run_blocks(uint32_t ip, uint64_t limit)
{
	const uint8_t *code = m_resume_code;

	if (ip != m_resume_ip) {
		const Link &link = m_links.empty() ? Link{} : m_links[(ip >> 2) & (LINKS - 1)];

		code = link.ip == ip ? reinterpret_cast<const uint8_t *>(link.code) : find_block(ip);
		if (!code)
			return false;
	}
	m_resume_ip = ~0u;
	m_code->run(this, code, m_chip.mp_pages(), limit);
	return true;
}

/* The block that starts at `ip`, found or made, and its link: its code from
 * its first instruction; null when there is none */
const uint8_t *
C82Mp::find_block(uint32_t ip)
{
#if !defined(ODIN_HOST_X86_64)
	/* No blocks written for this host: the interpreter alone */
	m_native = false;
	return nullptr;
#endif
	if (!m_code) {
		m_code = std::make_unique<C82MpCode>();
		m_code->m_cycles_at = HostCode::place_in(this, &m_cycles);
		m_links.assign(LINKS, Link{});
		if (m_code->usable()) {
			m_code->clear();
			write_helpers();
		}
	}
	if (!m_code->usable()) {
		m_native = false;
		return nullptr;
	}
	if (m_block_steps > ALL_STEPS || !m_code->room())
		flush_blocks();

	Block *b = nullptr;

	if (const auto it = m_block_at.find(ip); it != m_block_at.end()) {
		b = m_blocks[it->second].get();
	} else if (const auto in = m_inside.find(ip); in != m_inside.end() && !m_blocks[in->second.id]->dead) {
		/* Within a block: on from that step */
		const Block &within = *m_blocks[in->second.id];
		Link &link = m_links[(ip >> 2) & (LINKS - 1)];

		link.ip = ip;
		link.id = in->second.id;
		link.code = m_code->address(within.entries[in->second.k]);
		return reinterpret_cast<const uint8_t *>(link.code);
	} else if (!(b = translate(ip))) {
		return nullptr;
	}

	/* The host code finds it there from the end of another block */
	Link &link = m_links[(ip >> 2) & (LINKS - 1)];

	link.ip = ip;
	link.id = static_cast<uint32_t>(m_block_at[ip]);
	link.code = m_code->address(b->place);
	return reinterpret_cast<const uint8_t *>(link.code);
}

/* A block from `ip`: the instructions that follow in its page of plain
 * memory, up to a branch (with its delay slot when that is a plain short
 * instruction, not a branch, not the wait), or before the wait's first
 * instruction, or MAX_STEPS steps, or a long immediate out of the page */
C82Mp::Block *
C82Mp::translate(uint32_t ip)
{
	const uint8_t *const page = m_chip.mp_code(ip & ~0xFFFFFu);

	if (!page || ip == m_wait_ip)
		return nullptr;
	if (!m_code->room())
		flush_blocks();

	const auto fetch = [&](uint32_t at, BlockStep &st) {
		if (at >> 20 != ip >> 20)
			return false;
		st.ip = at;
		st.word = static_cast<uint32_t>(c82_get(page + (at & 0xFFFFF), 2));
		st.length = (st.word & 0x00301000u) == 0x00301000u ? 8 : 4;
		if (st.length == 8 && (at + 4) >> 20 != ip >> 20)
			return false;
		st.imm = st.length == 8 ? static_cast<uint32_t>(c82_get(page + ((at + 4) & 0xFFFFF), 2)) : 0;

		const C82MpOp &op = prepared(at, st.word, st.imm);

		st.op = op;
		st.in = decoded(op);
		return true;
	};
	std::vector<BlockStep> steps;
	uint32_t at = ip;
	bool branch = false, end_in_slot = false;

	while (steps.size() < MAX_STEPS) {
		BlockStep st;

		if ((!steps.empty() && at == m_wait_ip) || !fetch(at, st))
			break;
		steps.push_back(st);
		at += st.length;
		if (!is_branch(st.in.op))
			continue;
		branch = true;
		if (!st.in.annul) {
			BlockStep slot;

			if (at != m_wait_ip && fetch(at, slot) && slot.length == 4 && !is_branch(slot.in.op)) {
				slot.slot = true;
				steps.push_back(slot);
			} else {
				end_in_slot = true;
			}
		}
		break;
	}
	if (steps.empty())
		return nullptr;

	auto b = std::make_unique<Block>();
	const uint32_t id = static_cast<uint32_t>(m_blocks.size());

	b->ip = ip;
	b->n = static_cast<uint32_t>(steps.size());
	b->end = steps.back().ip + steps.back().length;
	if (!write_block(*b, steps, branch, end_in_slot, at))
		return nullptr;
	mark_code(b->ip, b->end);
	m_block_steps += steps.size();
	m_block_at[ip] = id;
	for (unsigned k = 1; k < steps.size(); k++)
		if (!steps[k].slot)
			m_inside[steps[k].ip] = Inside{ id, k };
	m_blocks.push_back(std::move(b));
	m_chained.resize(m_blocks.size());
	/* Its own jumps, then those waiting for its steps */
	m_code->writable();
	for (Chain &site : m_sites) {
		site.owner = id;
		chain(site);
	}
	settle(ip);
	for (unsigned k = 1; k < steps.size(); k++)
		if (!steps[k].slot)
			settle(steps[k].ip);
	m_code->ready();
	return m_blocks.back().get();
}

/* Called by the host code with the state written as fast_steps() has it
 * when it calls a handler */
void
C82Mp::block_step(C82Mp &mp, uint32_t ip, uint64_t key)
{
	const C82MpOp &op = mp.prepared(ip, static_cast<uint32_t>(key), static_cast<uint32_t>(key >> 32));

	op.exec(mp, op, ip);
}

/* The steps of a block in host code (see the top of the file) */
bool
C82Mp::write_block(Block &b, const std::vector<BlockStep> &steps, bool branch, bool end_in_slot,
                   uint32_t next)
{
#if defined(ODIN_HOST_X86_64)
	C82MpCode &c = *m_code;
	using C = C82MpCode;
	using Mem = C82MpCode::Mem;
	const auto at = [this](const void *member) { return HostCode::place_in(this, member); };
	const int32_t IP = at(&m_ip), PC = at(&m_pc), CYCLES = at(&m_cycles), CHECK = at(&m_check),
	              SLOT_NEXT = at(&m_slot_next), IN_SLOT = at(&m_in_slot);
	const size_t n = steps.size();
	struct Exit {
		HostCode::Jump jump;
		unsigned k;
		/* The budget's test after the last step */
		bool last = false;
	};
	std::vector<Exit> exits;
	std::vector<C82MpSlow> slow;
	std::vector<HostCode::Jump> kept;
	/* The cycles a step counts: 1, 2 with a long immediate, 1 more for
	 * the nop of an annulled branch */
	const auto cycles = [&](unsigned k) {
		const BlockStep &st = steps[k];

		return static_cast<uint8_t>(st.length / 4 + (is_branch(st.in.op) && st.in.annul ? 1 : 0));
	};
	/* The state fast_steps() has when it calls step k's handler: cycles,
	 * IP and PC after the fetch, in a delay slot or not */
	const auto materialize = [&](unsigned k) {
		const BlockStep &st = steps[k];

		c.op_r(0, { 0x89 }, C::LIMIT, C::RAX, true);			/* mov rax, r15 */
		c.op_r(0, { 0x29 }, C::LEFT, C::RAX, true);			/* sub rax, rbp */
		c.op_r(0, { 0x83 }, 0, C::RAX, true);				/* add rax, cycles */
		c.byte(cycles(k));
		c.op_m(0, { 0x89 }, C::RAX, Mem{ C::MP, C::NO, 0, CYCLES }, true);	/* mov [cycles], rax */
		if (st.slot) {
			c.op_m(0, { 0x89 }, C::NEXT, Mem{ C::MP, C::NO, 0, IP });	/* mov [ip], r12d */
			c.op_m(0, { 0x8D }, C::RAX, Mem{ C::NEXT, C::NO, 0, 4 });	/* lea eax, [r12 + 4] */
			c.op_m(0, { 0x89 }, C::RAX, Mem{ C::MP, C::NO, 0, PC });	/* mov [pc], eax */
		} else {
			c.op_m(0, { 0xC7 }, 0, Mem{ C::MP, C::NO, 0, IP });		/* mov dword [ip], next */
			c.word(st.ip + st.length);
			c.op_m(0, { 0xC7 }, 0, Mem{ C::MP, C::NO, 0, PC });		/* mov dword [pc], next + 4 */
			c.word(st.ip + st.length + 4);
		}
		c.op_m(0, { 0xC6 }, 0, Mem{ C::MP, C::NO, 0, IN_SLOT });		/* mov byte [in_slot], slot */
		c.byte(st.slot ? 1 : 0);
	};
	/* After a handler: on when it left m_check clear, the cycles back */
	const auto after_call = [&](unsigned k) {
		c.op_m(0, { 0x80 }, 7, Mem{ C::MP, C::NO, 0, CHECK });		/* cmp byte [check], 0 */
		c.byte(0);
		kept.push_back(c.jump_if(C::NE));
		c.op_r(0, { 0x89 }, C::LIMIT, C::LEFT, true);			/* mov rbp, r15 */
		c.op_m(0, { 0x2B }, C::LEFT, Mem{ C::MP, C::NO, 0, CYCLES }, true);	/* sub rbp, [cycles] */
		c.op_r(0, { 0x83 }, 0, C::LEFT, true);				/* add rbp, cycles */
		c.byte(cycles(k));
	};
	const auto call = [&](unsigned k, uintptr_t target) {
		c.op_r(0, { 0x89 }, C::MP, C::ARG[0], true);			/* mov the first argument, rbx */
		c.move32(C::ARG[1], steps[k].ip);
		c.move64(C::ARG[2], steps[k].word | static_cast<uint64_t>(steps[k].imm) << 32);
		c.call_to(target);
	};

	c.writable();
	b.place = c.here();
	b.entries.assign(n, 0);
	for (unsigned k = 0; k < n; k++) {
		const BlockStep &st = steps[k];

		b.entries[k] = c.here();
		if (is_branch(st.in.op)) {
			write_branch(c, st);
		} else if (!write_body(c, steps, k, slow)) {
			materialize(k);
			call(k, reinterpret_cast<uintptr_t>(&C82Mp::block_step));
			after_call(k);
		}
		c.op_r(0, { 0x83 }, 5, C::LEFT, true);				/* sub rbp, cycles */
		c.byte(cycles(k));
		exits.push_back({ c.jump_if(C::LE), k + 1, k + 1 == n });
	}

	/* On to the next block. An instruction known now (a branch's
	 * target from an immediate, the one after the delay slot or the
	 * block) by a jump of its own (chained later); a target from a
	 * register by the link of its first instruction */
	const BlockStep *const br = branch ? &*std::find_if(steps.begin(), steps.end(),
	                                                    [](const BlockStep &st) { return is_branch(st.in.op); })
	                                   : nullptr;
	const bool known = !br || (br->in.form != C82Insn::REGISTER && br->in.op != C82Insn::JSR);
	std::vector<std::pair<HostCode::Jump, uint32_t>> sites;

	m_sites.clear();
	if (!end_in_slot && known) {
		if (br) {
			/* As write_branch: r12d the target when taken, else after */
			const uint32_t target = br->ip + 4 * br->op.imm;
			const uint32_t after = br->ip + br->length + (br->in.annul ? 0 : 4);
			const bool always = br->in.op == C82Insn::BSR || (br->in.op == C82Insn::BCND && br->op.a == 7);
			const bool never = br->in.op == C82Insn::BCND && br->op.a == 0;

			if (!always && !never) {
				c.op_ri(7, C::NEXT, target);				/* cmp r12d, target */
				sites.emplace_back(c.jump_if(C::E), target);
			}
			sites.emplace_back(c.jump_ahead(), always ? target : after);
		} else {
			sites.emplace_back(c.jump_ahead(), next);
		}
	} else if (!end_in_slot) {
		{
			c.op_r(0, { 0x89 }, C::NEXT, C::RAX);				/* mov eax, r12d */
			c.shift_ri(5, C::RAX, 2);					/* shr eax, 2 */
			c.op_ri(4, C::RAX, LINKS - 1);					/* and eax, LINKS - 1 */
			c.shift_ri(4, C::RAX, 4);					/* shl eax, 4 */
			c.move64(C::RCX, reinterpret_cast<uintptr_t>(m_links.data()));
			c.op_m(0, { 0x39 }, C::NEXT, Mem{ C::RCX, C::RAX, 0, 0 });	/* cmp [rcx + rax], r12d */
			exits.push_back({ c.jump_if(C::NE), static_cast<unsigned>(n) });
			c.op_m(0, { 0xFF }, 4, Mem{ C::RCX, C::RAX, 0, 8 });		/* jmp [rcx + rax + 8] */
		}
	} else {
		exits.push_back({ c.jump_ahead(), static_cast<unsigned>(n) });
	}

	/* The ways out of the line of the steps, apart */
	c.cold();
	/* Each jump's stub: by the link of its instruction, else out */
	for (const auto &[jump, ip] : sites) {
		const uint32_t stub = c.here();

		c.bind(jump);
		c.move64(C::RCX, reinterpret_cast<uintptr_t>(&m_links[(ip >> 2) & (LINKS - 1)]));
		c.op_m(0, { 0x81 }, 7, Mem{ C::RCX, C::NO, 0, 0 });		/* cmp dword [rcx], ip */
		c.word(ip);
		exits.push_back({ c.jump_if(C::NE), static_cast<unsigned>(n) });
		c.op_m(0, { 0xFF }, 4, Mem{ C::RCX, C::NO, 0, 8 });		/* jmp [rcx + 8] */
		m_sites.push_back(Chain{ jump.from, stub, ip, 0 });
	}
	for (const C82MpSlow &s : slow) {
		for (const HostCode::Jump &j : s.from)
			c.bind(j);
		if (s.kind == C82MpSlow::SIGNED_OVERFLOW) {
			/* The result, then io posted: m_check, the state after the step */
			if (s.dest)
				c.op_m(0, { 0x89 }, C::RAX, Mem{ C::MP, C::NO, 0, at(&m_r[s.dest]) });	/* mov [r(dest)], eax */
			c.op_m(0, { 0x81 }, 1, Mem{ C::MP, C::NO, 0, at(&m_intpen) });	/* or dword [intpen], io */
			c.word(INT_IO);
			c.op_m(0, { 0xC6 }, 0, Mem{ C::MP, C::NO, 0, CHECK });	/* mov byte [check], 1 */
			c.byte(1);
			materialize(s.k);
			kept.push_back(c.jump_ahead());
			continue;
		}
		materialize(s.k);
		call(s.k, reinterpret_cast<uintptr_t>(s.kind == C82MpSlow::HANDLER ? &C82Mp::block_step : &C82Mp::block_vector_memory));
		after_call(s.k);
		c.jump_back(s.back);
	}
	/* Before step k (or after the last one): the state fast_steps() has
	 * there. Before a step not in a delay slot (the slice ended): 2k to the
	 * block's common way out, which finds the step's address and its code
	 * in a table (IP, PC, where the next run goes on) */
	c.align8();
	const uint32_t table = c.here();
	std::vector<HostCode::Jump> common;

	for (unsigned k = 0; k < n; k++) {
		c.word(steps[k].ip);
		c.word(0);
		c.quad(c.address(b.entries[k]));
	}
	for (const Exit &e : exits) {
		c.bind(e.jump);
		if (e.k < n && !steps[e.k].slot) {
			c.move32(C::RCX, 2 * e.k);
			common.push_back(c.jump_ahead());
			continue;
		}
		if (e.k < n) {
			const BlockStep &st = steps[e.k];

			c.op_m(0, { 0xC7 }, 0, Mem{ C::MP, C::NO, 0, IP });		/* mov dword [ip], its address */
			c.word(st.ip);
			c.op_m(0, { 0x89 }, C::NEXT, Mem{ C::MP, C::NO, 0, PC });	/* mov [pc], r12d */
			c.op_m(0, { 0xC6 }, 0, Mem{ C::MP, C::NO, 0, SLOT_NEXT });	/* mov byte [slot_next], 1 */
			c.byte(1);
		} else if (end_in_slot) {
			c.op_m(0, { 0xC7 }, 0, Mem{ C::MP, C::NO, 0, IP });		/* mov dword [ip], the slot */
			c.word(next);
			c.op_m(0, { 0x89 }, C::NEXT, Mem{ C::MP, C::NO, 0, PC });	/* mov [pc], r12d */
			c.op_m(0, { 0xC6 }, 0, Mem{ C::MP, C::NO, 0, SLOT_NEXT });	/* mov byte [slot_next], 1 */
			c.byte(1);
		} else if (branch) {
			c.op_m(0, { 0x89 }, C::NEXT, Mem{ C::MP, C::NO, 0, IP });	/* mov [ip], r12d */
			c.op_m(0, { 0x8D }, C::RAX, Mem{ C::NEXT, C::NO, 0, 4 });	/* lea eax, [r12 + 4] */
			c.op_m(0, { 0x89 }, C::RAX, Mem{ C::MP, C::NO, 0, PC });	/* mov [pc], eax */
			if (e.last) {
				/* The slice ended after the last step: the next
				 * run goes on where run_blocks() would find it, by the
				 * link of r12d when there is one (only the MP's
				 * find_block writes links; dropping or flushing blocks
				 * forgets the note) */
				c.op_r(0, { 0x89 }, C::NEXT, C::RAX);				/* mov eax, r12d */
				c.shift_ri(5, C::RAX, 2);					/* shr eax, 2 */
				c.op_ri(4, C::RAX, LINKS - 1);					/* and eax, LINKS - 1 */
				c.shift_ri(4, C::RAX, 4);					/* shl eax, 4 */
				c.move64(C::RCX, reinterpret_cast<uintptr_t>(m_links.data()));
				c.op_m(0, { 0x39 }, C::NEXT, Mem{ C::RCX, C::RAX, 0, 0 });	/* cmp [rcx + rax], r12d */
				const HostCode::Jump none = c.jump_if(C::NE);
				c.op_m(0, { 0x89 }, C::NEXT, Mem{ C::MP, C::NO, 0, at(&m_resume_ip) });	/* mov [resume_ip], r12d */
				c.op_m(0, { 0x8B }, C::RAX, Mem{ C::RCX, C::RAX, 0, 8 }, true);	/* mov rax, [rcx + rax + 8] */
				c.op_m(0, { 0x89 }, C::RAX, Mem{ C::MP, C::NO, 0, at(&m_resume_code) }, true);	/* mov [resume_code], rax */
				c.bind(none);
			}
		} else {
			c.op_m(0, { 0xC7 }, 0, Mem{ C::MP, C::NO, 0, IP });		/* mov dword [ip], next */
			c.word(next);
			c.op_m(0, { 0xC7 }, 0, Mem{ C::MP, C::NO, 0, PC });		/* mov dword [pc], next + 4 */
			c.word(next + 4);
			if (e.last) {
				/* The same by the link of `next` */
				c.move64(C::RCX, reinterpret_cast<uintptr_t>(&m_links[(next >> 2) & (LINKS - 1)]));
				c.op_m(0, { 0x81 }, 7, Mem{ C::RCX, C::NO, 0, 0 });		/* cmp dword [rcx], next */
				c.word(next);
				const HostCode::Jump none = c.jump_if(C::NE);
				c.op_m(0, { 0xC7 }, 0, Mem{ C::MP, C::NO, 0, at(&m_resume_ip) });	/* mov dword [resume_ip], next */
				c.word(next);
				c.op_m(0, { 0x8B }, C::RAX, Mem{ C::RCX, C::NO, 0, 8 }, true);	/* mov rax, [rcx + 8] */
				c.op_m(0, { 0x89 }, C::RAX, Mem{ C::MP, C::NO, 0, at(&m_resume_code) }, true);	/* mov [resume_code], rax */
				c.bind(none);
			}
		}
		c.jump_back(c.leave_left());
	}
	if (!common.empty()) {
		for (const HostCode::Jump &j : common)
			c.bind(j);
		/* ecx = 2k */
		c.move64(C::RDX, c.address(table));
		c.op_m(0, { 0x8B }, C::R9, Mem{ C::RDX, C::RCX, 3, 0 });	/* mov r9d, [rdx + rcx * 8]: its address */
		c.op_m(0, { 0x89 }, C::R9, Mem{ C::MP, C::NO, 0, IP });		/* mov [ip], r9d */
		c.op_m(0, { 0x8D }, C::R10, Mem{ C::R9, C::NO, 0, 4 });		/* lea r10d, [r9 + 4] */
		c.op_m(0, { 0x89 }, C::R10, Mem{ C::MP, C::NO, 0, PC });		/* mov [pc], r10d */
		/* The next run goes on from here */
		c.op_m(0, { 0x89 }, C::R9, Mem{ C::MP, C::NO, 0, at(&m_resume_ip) });	/* mov [resume_ip], r9d */
		c.op_m(0, { 0x8B }, C::RCX, Mem{ C::RDX, C::RCX, 3, 8 }, true);	/* mov rcx, [rdx + rcx * 8 + 8]: its code */
		c.op_m(0, { 0x89 }, C::RCX, Mem{ C::MP, C::NO, 0, at(&m_resume_code) }, true);	/* mov [resume_code], rcx */
		c.jump_back(c.leave_left());
	}
	for (const HostCode::Jump &j : kept)
		c.bind(j);
	if (!kept.empty())
		c.jump_back(c.leave_kept());
	c.hot();
	c.ready();
	return true;
#else
	(void)b;
	(void)steps;
	(void)branch;
	(void)end_in_slot;
	(void)next;
	return false;
#endif
}

} /* namespace odin */
