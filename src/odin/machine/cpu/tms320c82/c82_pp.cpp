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
#include <bit>
#include <cstdio>
#include <cstring>

#include "odin/common/repeats.hpp"
#include "odin/machine/cpu/tms320c82/c82_pp_host.hpp"
#include "odin/machine/cpu/tms320c82/tms320c82.hpp"

namespace odin {

namespace {

/* sr: N C V Z, MSS, R, Msize, Asize (sr register, PP:7-36) */
constexpr uint32_t SR_BITS = 0xF00001FFu;
/* A direct external access, in cycles of the execute stage (table 6-1,
 * page hit; HEURISTIC: the board's memory timing is not known) */
constexpr unsigned DEA_LOAD = 11, DEA_STORE = 8;
/* A PP's software reset, from the command to its halt latch set again:
 * the TC acknowledges once its pipeline drained (§ 9.6.1). HEURISTIC: 4
 * cycles, the shortest hardware reset (§ 9.6) */
constexpr uint64_t RESET_CYCLES = 4;
/* The parameter RAM of a PP on the C82: 4 KB (SPRU154 fig. 1-4) */
constexpr uint32_t PARAM_SIZE = 0x1000;

} /* namespace */

C82Pp::C82Pp(Tms320c82 &chip, unsigned number) : m_chip(chip), m_number(number), m_ops(4096), m_decoded(4096)
{
	m_r[c82_pp_regs::DBA] = m_number << 12;
	m_r[c82_pp_regs::PBA] = param_base();
}

/* A hardware reset (§ 9.6): HEURISTIC, the registers it leaves undefined
 * are 0; then as a software reset, acknowledged at once */
void
C82Pp::power_up()
{
	m_r.fill(0);
	m_r[c82_pp_regs::DBA] = m_number << 12;
	m_r[c82_pp_regs::PBA] = param_base();
	m_sr = m_mf = m_pc = m_iprs = m_inten = m_intflg = m_lctl = 0;
	m_lc.fill(0);
	m_lr.fill(0);
	m_ls.fill(0);
	m_le.fill(0);
	m_comm_hs = 0;
	m_q_visible = 0;
	a() = Stage{};
	e() = Stage{};
	m_cycles = 0;
	m_illegal = m_faulted = m_fault_type = false;
	m_in_reset = false;
	m_flowing = false;
	m_resume = nullptr;
	stop_watching();
	take_reset();
}

/* The reset command (§ 9.6.1): the halt latch reads 0 until the TC
 * acknowledges; a reset always ends halted, even with unhalt */
void
C82Pp::reset(uint64_t now)
{
	disturb();
	m_in_reset = true;
	m_halted = false;
	m_reset_done = now + RESET_CYCLES;
}

/* The acknowledge (§ 9.6.1): halted, H P Q S cleared, the TC's requests of
 * this PP dropped, accesses and cache requests cancelled, the loop ends
 * disabled; the two reset pseudo-instructions then PS3-PS5 for the task
 * vector wait for the unhalt (§ 9.6.2) */
void
C82Pp::take_reset()
{
	settle_flow();
	m_in_reset = false;
	m_halted = true;
	m_comm_hs = 0;
	m_chip.tc().drop(1 + m_number);
	m_lctl &= ~0x888u;
	loops_changed();
	m_reset_ipa = a().addr;
	m_reset_ipe = e().addr;
	m_reset_pc = m_pc;
	a().kind = EMPTY;
	e().kind = EMPTY;
	a().stop = e().stop = false;
	m_seq = RESET_SEQ;
	m_seq_step = 0;
	m_seq_bit = 14;
	m_illegal = m_faulted = m_fault_type = false;
}

bool
C82Pp::halt_latch(uint64_t now)
{
	if (m_in_reset && now >= m_reset_done)
		take_reset();
	return m_halted;
}

/* From the MP (fig. 3-3; § 3.2.2): reset wins over unhalt, unhalt over
 * halt; task and message set their flags. HEURISTIC: the instruction cache
 * reset does nothing, no cache being emulated */
void
C82Pp::command(uint32_t word, uint64_t now)
{
	disturb();
	if (word >> 31)
		reset(now);
	else if ((word >> 29) & 1) {
		if (!m_in_reset) {
			if (m_halted && m_cycles < now)
				m_cycles = now;
			m_halted = false;
		}
	} else if ((word >> 30) & 1)
		m_halted = true;
	if ((word >> 14) & 1)
		m_intflg |= INT_TASK;
	if ((word >> 13) & 1)
		m_intflg |= INT_MPMSG;
}

/* A move to a15 (§ 3.2): to itself a halt (bit 30) or a cache reset (bit
 * 28) with its own designator; a message (bit 13) to the MP (bit 8) and
 * to the PPs designated; the bits only the MP may send are ignored.
 * HEURISTIC: the store of the first delay slot is done at the unhalt, not
 * during the halt (§ 3.2.1) */
void
C82Pp::send_command(uint32_t word)
{
	m_chip.visible();
	m_park.spoiled = true;
	if ((word >> m_number) & 1 && (word >> 30) & 1)
		m_halted = true;
	if ((word >> 13) & 1) {
		if ((word >> 8) & 1)
			m_chip.mp().post(C82Mp::INT_P0 << m_number);
		for (unsigned pp = 0; pp < 2; pp++)
			if ((word >> pp) & 1)
				m_chip.pp(pp).post(INT_PP0MSG << m_number);
	}
}

/* The TC ended the request: Q cleared, PTEND, or PTERR on an error
 * (SPRU261 § 4.3.2 step 6, § 9.4) */
void
C82Pp::list_ended(bool error)
{
	disturb();
	m_intflg |= error ? INT_PTERR : INT_PTEND;
}

/* comm (comm register; SPRU154 fig. 2-2): H, S, Q one instruction after
 * P (§ 12.2.4), M = 1 on the C82, the PP number. HEURISTIC: P reads 0, no
 * suspend acknowledge being emulated */
uint32_t
C82Pp::read_comm() const
{
	const bool q = m_chip.tc().queued(1 + m_number) && m_cycles >= m_q_visible;

	return m_comm_hs | (q ? 1u << 29 : 0) | (1u << 8) | m_number;
}

/* P submits when S is or goes to 0: queued, or PTQ when a request is
 * already queued (fig. 12-1, 12-2) */
void
C82Pp::write_comm(uint32_t value)
{
	const bool suspend = (value >> 30) & 1;

	m_comm_hs = value & 0xC0000000u;
	if (suspend && m_chip.tc().queued(1 + m_number))
		repeats::print(stderr, "PP%u: suspending a packet transfer: not emulated yet\n", m_number);
	if ((value >> 28) & 1 && !suspend) {
		m_chip.visible();
		m_park.spoiled = true;
		if (m_chip.tc().queued(1 + m_number)) {
			m_intflg |= INT_PTQ;
		} else {
			m_chip.tc().request(1 + m_number, m_cycles);
			m_q_visible = m_cycles + 2;
		}
	}
}

/* Registers by code (table 7-1). HEURISTIC: reserved codes, the write-only
 * lrs/lrse and the cache tags (no cache emulated) read 0 */
uint32_t
C82Pp::read_reg(unsigned code)
{
	code &= 0x7F;
	if (code < 0x28)
		return get(code);
	switch (code) {
	case 0x29:
		return m_sr;
	case 0x2A:
		return m_mf;
	case 0x38:
		return m_pc;
	case 0x39:
		return a().addr;
	case 0x3A:
		return e().addr;
	case 0x3B:
		return m_iprs;
	case 0x3C:
		return m_inten;
	case 0x3D:
		return m_intflg;
	case 0x3E:
		return read_comm();
	case 0x3F:
		return m_lctl;
	case 0x60:
	case 0x61:
	case 0x62:
		return m_lc[code & 3];
	case 0x64:
	case 0x65:
	case 0x66:
		return m_lr[code & 3];
	case 0x70:
	case 0x71:
	case 0x72:
		return m_ls[code & 3];
	case 0x74:
	case 0x75:
	case 0x76:
		return m_le[code & 3];
	default:
		return 0;
	}
}

/* A write to pc in the execute stage: the target is fetched after the two
 * delay slots; G and L stay unless `bits` says (§ 10.3; pc register) */
void
C82Pp::branch(uint32_t target, uint32_t bits)
{
	m_pc_written = true;
	m_new_pc = target;
	m_new_pc_bits = bits;
}

/* HEURISTIC: writes to reserved or read-only codes are left out */
void
C82Pp::write_reg(unsigned code, uint32_t value)
{
	code &= 0x7F;
	if (code < 0x28) {
		put(code, value);
		return;
	}
	switch (code) {
	case 0x29:
		/* Protected status bits keep their value (§ 8.9.4) */
		m_sr = ((m_sr & m_protect) | (value & ~m_protect)) & SR_BITS;
		return;
	case 0x2A:
		m_mf = value;
		return;
	case 0x38:
		/* call: iprs = PC + 1, its three LSBs 0 (§ 10.4, iprs register) */
		m_iprs = (m_pc & ~7u) + 8;
		branch(value, 0);
		return;
	case 0x39:
		branch(value, 0);
		return;
	case 0x3B:
		m_iprs = value;
		return;
	case 0x3C:
		m_inten = value;
		return;
	case 0x3D:
		/* W of inten: 1 sets, 0 clears; bit 0 is not written (fig. 9-2,
		 * § 9.3) */
		if (m_inten & 1)
			m_intflg |= value & ~1u;
		else
			m_intflg &= ~(value & ~1u);
		return;
	case 0x3E:
		write_comm(value);
		return;
	case 0x3F:
		m_lctl = value & 0xFFFu;
		loops_changed();
		return;
	case 0x60:
	case 0x61:
	case 0x62:
		m_lc[code & 3] = value;
		m_lc_written |= 1u << (code & 3);
		return;
	case 0x64:
	case 0x65:
	case 0x66:
		/* A reload register writes its counter too (lr registers) */
		m_lr[code & 3] = m_lc[code & 3] = value;
		m_lc_written |= 1u << (code & 3);
		return;
	case 0x68:
	case 0x69:
	case 0x6A:
	case 0x6C:
	case 0x6D:
	case 0x6E: {
		/* lrse/lrs: count, start = PC + 1 (and end for lrse), the loop
		 * end enabled with its own counter (lrs, lrse registers) */
		const unsigned n = code & 3;

		m_lr[n] = m_lc[n] = value;
		m_lc_written |= 1u << n;
		m_ls[n] = (m_pc & ~7u) + 8;
		if (code < 0x6C)
			m_le[n] = m_ls[n];
		m_lctl = (m_lctl & ~(0xFu << (4 * n))) | ((8u | (n + 1)) << (4 * n));
		loops_changed();
		return;
	}
	case 0x70:
	case 0x71:
	case 0x72:
		m_ls[code & 3] = value;
		return;
	case 0x74:
	case 0x75:
	case 0x76:
		m_le[code & 3] = value;
		loops_changed();
		return;
	default:
		return;
	}
}

/* The enabled loop ends (29 MSBs), for steps(); 1 for a disabled one */
void
C82Pp::loops_changed()
{
	for (unsigned n = 0; n < 3; n++)
		m_ends[n] = (m_lctl >> (4 * n + 3)) & 1 ? m_le[n] & ~7u : 1;
}

/* The loop logic of the fetch stage (§ 5.3, § 11.2-11.4): at an enabled
 * loop end (29 MSBs), le2 first, the first with a nonzero designated
 * counter (or none) branches to its start and counts down; the zero
 * counters on the way are reloaded; L inhibits it (pc register) */
uint32_t
C82Pp::next_fetch(uint32_t pc, std::array<uint32_t, 3> &lc) const
{
	const uint32_t sequential = ((pc & ~7u) + 8) | (pc & 3);

	if ((pc & 1) || !(m_lctl & 0x888u))
		return sequential;
	for (int n = 2; n >= 0; n--) {
		if (!((m_lctl >> (4 * n + 3)) & 1) || ((m_le[n] ^ pc) & ~7u))
			continue;

		const unsigned lcd = (m_lctl >> (4 * n)) & 7;
		const uint32_t start = (m_ls[n] & ~7u) | (pc & 3);

		/* HEURISTIC: a reserved designator (1xx) as none */
		if (lcd == 0 || lcd > 3)
			return start;
		if (lc[lcd - 1]) {
			lc[lcd - 1]--;
			return start;
		}
		lc[lcd - 1] = m_lr[lcd - 1];
	}
	return sequential;
}

const C82PpInsn C82Pp::NO_INSN{};
const C82PpOp C82Pp::NO_OP{};

unsigned
C82Pp::decoded(uint32_t addr, uint64_t word, const uint8_t *host)
{
	const unsigned k = (addr >> 3) & 4095;
	Cached &c = m_decoded[k];
	C82PpOp &op = m_ops[k];

	if (op.addr != addr || c.word != word) {
		for (Stage *s : { &a(), &e() })
			if (s->insn == &c.insn) {
				s->own = c.insn;
				s->insn = &s->own;
				s->own_op = op;
				s->op = &s->own_op;
			}
		c.word = word;
		c.insn = c82_pp_decode(word);
		op = c82_pp_op(c.insn);
		op.addr = addr;
	}
	op.host = host;
	op.raw = __builtin_bswap64(word);
	return k;
}

bool
C82Pp::fetch_word(uint32_t addr, uint64_t &word, const uint8_t *&host)
{
	if (m_park.measuring)
		note_fetch(addr);
	host = m_chip.pp_code(addr);
	if (host) {
		word = c82_get(host, 3);
		return true;
	}
	if (!m_chip.pp_fetch(addr, word)) {
		memory_fault(addr, false);
		return false;
	}
	return true;
}

/* A memory fault (SPRU109 § 3.3.4.1, table 5-5; SPRU110 § 8.8.3.1): the PP
 * stalls until reset, the address in 0x0100#0F8, FLTSTS x, mf to the MP */
void
C82Pp::memory_fault(uint32_t addr, bool data)
{
	m_park.spoiled = true;
	m_faulted = true;
	m_fault_type = data;
	m_chip.set_internal32(param_base() + 0xF8, addr);
	m_chip.fault(1u << (24 + m_number));
}

bool
C82Pp::load(uint32_t addr, unsigned size, uint32_t &value, unsigned &cycles)
{
	uint64_t v = 0;

	if (const uint8_t *p = addr < 0x02000000u ? m_chip.pp_ram(addr) : nullptr) {
		/* On-chip RAM: the word's bytes from addr, those of the size
		 * kept (a PP's access is at most a word, and the RAM it reaches
		 * has another block after it) */
		v = c82_get(p, 2) >> (32 - (8u << size));
		if (m_park.measuring)
			note_read(addr, size, v);
		value = static_cast<uint32_t>(v);
		return true;
	}
	if (m_park.replaying && m_chip.place(addr) == Tms320c82::PLACE_BOARD) {
		/* Catching up: this read was made where it fell (parked_run) */
		for (unsigned k = 0; k < m_park.replays; k++)
			if (m_park.replay[k].addr == addr && m_park.replay[k].size == size)
				v = m_park.replay[k].value;
	} else if (!m_chip.pp_load(addr, size, v)) {
		memory_fault(addr, true);
		return false;
	}
	if (m_park.measuring)
		note_read(addr, size, v);
	if (addr >= 0x02000000u)
		cycles += DEA_LOAD - 1;
	value = static_cast<uint32_t>(v);
	return true;
}

bool
C82Pp::store(uint32_t addr, unsigned size, uint32_t value, unsigned &cycles)
{
	m_park.spoiled = true;
	if (!m_chip.pp_store(addr, size, value)) {
		memory_fault(addr, true);
		return false;
	}
	if (addr >= 0x02000000u)
		cycles += DEA_STORE - 1;
	return true;
}

/* The address stage (§ 6.4.1.1, table A-7, § 8.8.1): base (an address
 * register or dba/pba) +/- index (an index register or the offset, scaled
 * by the size); pre- or post-indexed; the address register written back
 * at the end of the cycle */
void
C82Pp::address_stage(Stage &s, Pending &pending)
{
	const C82PpInsn &in = *s.insn;

	for (unsigned k = 0; k < in.xfers; k++) {
		const C82PpXfer &t = in.xfer[k];
		/* The base and index as prepare() picked them (C82PpXfer) */
		const uint32_t base = m_r[t.base_slot];
		const uint32_t index = (((m_r[t.index_slot] + t.offset) << t.shift) ^ t.neg) - t.neg;

		s.ea[k] = base + (index & t.pre_mask);
		if (t.wb != 0xFF) {
			pending.code[pending.count] = t.wb;
			pending.value[pending.count++] = base + index;
		}
	}
}

/* The pseudo-instructions (§ 9.5, fig. 9-4; § 9.6, fig. 9-5) in the
 * execute stage. HEURISTIC: which pseudo-instruction does which push; the
 * reset leaves the word at 0x0100#FF0 unwritten (fig. 9-5 shows none) */
void
C82Pp::execute_pseudo(Stage &s)
{
	const uint32_t sp = m_r[6];
	const uint32_t top = param_base() + PARAM_SIZE;
	unsigned extra = 0;

	if (m_seq == INTERRUPT) {
		switch (s.kind) {
		case PS1:
			/* The PC for the return, L = 0 */
			store(sp - 4, 2, m_seq_p3 & ~1u, extra);
			break;
		case PS2:
			/* The IPA value, L = 1 */
			store(sp - 8, 2, m_seq_p2, extra);
			break;
		case PS3: {
			/* The IPE value with G, L = 1; then the vector */
			uint32_t vector = 0;

			store(sp - 12, 2, m_seq_p1, extra);
			if (load(param_base() + 0x180 + 4u * m_seq_bit, 2, vector, extra))
				branch(vector, 3);
			break;
		}
		case PS4:
			store(sp - 16, 2, m_sr, extra);
			write_reg(6, sp - 16);
			break;
		default:
			m_seq = NONE;
			break;
		}
	} else {
		switch (s.kind) {
		case RS1:
			/* sp shifted left by one, with G and L (fig. 9-5) */
			store(top - 4, 2, (sp << 1 & ~3u) | (m_reset_pc & 3), extra);
			break;
		case RS2:
			store(top - 8, 2, m_reset_ipa, extra);
			break;
		case PS3: {
			uint32_t vector = 0;

			store(top - 12, 2, m_reset_ipe, extra);
			if (load(param_base() + 0x180 + 4u * m_seq_bit, 2, vector, extra))
				branch(vector, 3);
			break;
		}
		case PS4:
			write_reg(6, top - 16);
			break;
		default:
			m_seq = NONE;
			break;
		}
	}
	m_extra += extra;
}

/* One cycle of the FAE pipeline (§ 5.2): the fetch stage (interrupts,
 * eint/dint, loops), the address stage (registers as they were at the start
 * of the cycle), the execute stage; then the address registers, the pc and
 * the loop counters, the execute stage's writes winning (HEURISTIC: § 5.2.2.1
 * calls a clash unpredictable) */
void
C82Pp::cycle()
{
	if ((m_pc & ~7u) == m_wait_pc && m_seq == NONE) {
		pass_boundary();
		if (m_park.on)
			return;
	}
	if (m_park.measuring) {
		if (m_park.cycles == Park::CYCLES)
			m_park.spoiled = true;
		else
			m_park.cycles++;
	}
	/* An unimplemented opcode reaching decode stalls the PP and tells the
	 * MP (§ 5.2.3; SPRU109 § 3.3.4.2). HEURISTIC: the instruction in the
	 * execute stage that cycle does not complete either */
	Stage &as = *m_sa, &es = *m_se;

	if (as.stop) {
		m_illegal = true;
		m_chip.mp().post(C82Mp::INT_PE);
		repeats::print(stderr, "PP%u: illegal instruction %016llX at 0x%08X\n", m_number,
		               static_cast<unsigned long long>(as.word), as.addr & ~7u);
		return;
	}
	/* An enabled flag with G: the five pseudo-instructions, the lowest
	 * flag first (§ 9.4, § 9.5) */
	if (m_seq == NONE && (m_pc & 2)) {
		const uint32_t pending = m_intflg & m_inten & ~1u;

		if (pending) {
			m_seq = INTERRUPT;
			m_seq_step = 0;
			m_seq_bit = static_cast<uint8_t>(std::countr_zero(pending));
		}
	}

	m_written = 0;
	m_lc_written = 0;
	m_extra = 0;
	m_pc_written = false;
	m_protect = 0;

	/* Filled in place as a fresh stage would be (its other fields are
	 * not read before they are written) */
	Stage &fetched = *m_sf;
	/* The loop counters as the loop logic leaves them (only when a loop
	 * end is enabled: they stay as they are otherwise) */
	std::array<uint32_t, 3> lc;
	bool loops = false;
	uint32_t next = m_pc;
	int g_change = -1;

	if (m_seq != NONE && m_seq_step < 5) {
		/* PS1 and PS2 replace two fetches but the PC moves on: those two
		 * instructions are fetched again on return, their loops already
		 * counted (L = 1 in their stacked addresses); PS3-PS5 hold the PC
		 * (ipa register: the first two pseudo-instructions) */
		const unsigned step = m_seq_step++;

		fetched.word = 0;
		fetched.insn = &NO_INSN;
		fetched.op = &NO_OP;
		fetched.ea = {};
		fetched.divert = false;
		fetched.kind = static_cast<Kind>(m_seq == RESET_SEQ && step < 2 ? RS1 + step : PS1 + step);
		fetched.stop = false;
		fetched.addr = m_pc;
		if (m_seq == INTERRUPT && step < 2) {
			fetched.addr = m_pc | 1;
			if (step == 0)
				m_seq_p1 = m_pc | 1;
			else
				m_seq_p2 = m_pc | 1;
			loops = true;
			lc = m_lc;
			next = next_fetch(m_pc, lc);
		} else if (m_seq == INTERRUPT && step == 2) {
			m_seq_p3 = m_pc;
		}
	} else {
		uint64_t word = 0;
		const uint8_t *host = nullptr;

		if (!fetch_word(m_pc & ~7u, word, host))
			return;
		fetched.kind = INSN;
		fetched.addr = m_pc;
		fetched.word = word;
		fetched.ea = {};
		fetched.divert = false;
		const unsigned k = decoded(m_pc & ~7u, word, host);

		fetched.insn = &m_decoded[k].insn;
		fetched.op = &m_ops[k];
		fetched.stop = fetched.insn->illegal;
		/* eint and dint act before the next fetch (§ 9.2.1, § 8.7) */
		g_change = fetched.insn->g_change;
		if (!(m_pc & 1) && (m_lctl & 0x888u)) {
			loops = true;
			lc = m_lc;
			next = next_fetch(m_pc, lc);
		} else {
			next = ((m_pc & ~7u) + 8) | (m_pc & 3);
		}
	}

	Pending pending;

	if (as.kind == INSN)
		address_stage(as, pending);
	if (es.kind == INSN) {
		m_protect = es.insn->protect;
		EXEC[es.op->form](*this, *es.op, es.insn, es.ea.data());
	} else if (es.kind != EMPTY) {
		execute_pseudo(es);
	}
	if (m_faulted)
		return;

	/* The address registers written back, unless the execute stage wrote
	 * them (sp: code 6, C82PpXfer::wb) */
	for (unsigned k = 0; k < pending.count; k++)
		if (!((m_written >> pending.code[k]) & 1))
			put(pending.code[k], pending.value[k]);
	/* A software branch wins over the loop logic, which then counts
	 * nothing (§ 5.3.4) */
	if (m_pc_written) {
		const uint32_t keep = 3u & ~m_new_pc_bits;

		m_pc = (m_new_pc & ~(4u | keep)) | (m_pc & keep);
	} else {
		m_pc = next;
		if (loops)
			for (unsigned n = 0; n < 3; n++)
				if (!((m_lc_written >> n) & 1))
					m_lc[n] = lc[n];
	}
	if (g_change >= 0)
		m_pc = (m_pc & ~2u) | (static_cast<uint32_t>(g_change) << 1);

	Stage *const done = m_se;

	m_se = m_sa;
	m_sa = m_sf;
	m_sf = done;
	m_cycles += 1 + m_extra;
}

/* `first` NOT_RUN, or what run_blocks gave for a block resumed (run()):
 * the loop goes on from there */
void
C82Pp::run_on(uint64_t until, int first)
{
	if (first == NOT_RUN && m_park.on) {
		/* parked_run's case with nothing to make */
		if (until <= m_park.until)
			return;
		if (!m_park.boards || until <= m_park.next_read) {
			m_park.until = until;
			return;
		}
		if (parked_run(until))
			return;
	}
	for (int blocks = first;; blocks = NOT_RUN) {
		if (blocks == NOT_RUN) {
			if (m_cycles >= until)
				return;
			if (m_in_reset) {
				if (m_cycles < m_reset_done) {
					m_cycles = std::min(until, m_reset_done);
					continue;
				}
				take_reset();
			}
			if (m_halted || m_illegal || m_faulted) {
				m_cycles = until;
				return;
			}
			blocks = run_blocks(until);
		}
		if (blocks > 0)
			continue;
		if (steps(blocks == 0 ? m_cycles + 1 : until))
			continue;
		cycle();
		if (m_park.on) {
			m_park.until = m_cycles;
			if (parked_run(until))
				return;
		}
	}
}

/* run_on() until it would step by the interpreter, take a reset or
 * stop for a halt; alone (m_alone), the host code also stops before a step
 * another unit could see, and blocks without host code are not run */
void
C82Pp::run_alone(uint64_t limit)
{
	int blocks = NOT_RUN;

	if (m_resume && limit > m_cycles && (blocks = resume(limit)) == DONE)
		return;
	for (;;) {
		if (blocks == NOT_RUN) {
			if (m_cycles >= limit || m_in_reset || m_halted || m_illegal || m_faulted)
				return;
			blocks = run_blocks(limit);
		}
		if (blocks <= 0)
			return;
		blocks = NOT_RUN;
	}
}

/* A slice of a parked PP: the reads of the board its cycles would make
 * (those that start from where it stands to `until`) are made, in order,
 * with the bus time of the slice, as running it would. They are all looked
 * at first (C82Bus::peek): if one would answer otherwise than in the pass,
 * none is made, the PP is woken and the slice is run. False when woken.
 * A read lies at the start of one of its cycles (note_read), so the reads
 * from where it stands are those from next_read on, in the order of
 * `board` (the order of the pass), pass after pass */
bool
C82Pp::parked_run(uint64_t until)
{
	if (until <= m_park.until)
		return true;
	if (m_park.boards && until > m_park.next_read) {
		/* The slice's only read, made if it answers as in the pass
		 * (C82Bus::read_if: the board looks and reads in one pass) */
		unsigned then_k = m_park.next_k + 1;
		uint64_t then_pass = m_park.next_pass;

		if (then_k == m_park.boards) {
			then_k = 0;
			then_pass += m_park.period;
		}
		if (const uint64_t after = then_pass + m_park.board[then_k].at; after >= until) {
			const BoardRead &r = m_park.board[m_park.next_k];

			if (!m_chip.read_if(r.addr, r.size, r.value)) {
				wake();
				return false;
			}
			/* How long it keeps answering so (count_reads) */
			if (m_park.boards == 1 && m_chip.counted_reads())
				m_park.quiet = m_chip.answers_until(r.addr, r.size, r.value);
			m_park.next_read = after;
			m_park.next_pass = then_pass;
			m_park.next_k = then_k;
			m_park.until = until;
			return true;
		}
		for (int make = 0; make < 2; make++) {
			uint64_t at = m_park.next_read, pass = m_park.next_pass;
			unsigned k = m_park.next_k;

			while (at < until) {
				const BoardRead &r = m_park.board[k];
				uint64_t value = 0;

				if (make) {
					m_chip.pp_load(r.addr, r.size, value);
				} else if (!m_chip.peek(r.addr, r.size, value) || value != r.value) {
					wake();
					return false;
				}
				if (++k == m_park.boards) {
					k = 0;
					pass += m_park.period;
				}
				at = pass + m_park.board[k].at;
			}
			if (make) {
				m_park.next_read = at;
				m_park.next_pass = pass;
				m_park.next_k = k;
			}
		}
	}
	m_park.until = until;
	return true;
}

void
C82Pp::tell_counted()
{
	const Counted counted = m_counted;

	m_counted.count = 0;
	m_chip.reads_made(counted.addr, counted.size, counted.at, counted.value, counted.count);
}

void
C82Pp::set_wait(uint32_t pc, std::span<const uint64_t> words)
{
	stop_watching();
	settle_flow();
	flush_blocks();
	m_wait_pc = words.empty() ? ~0u : pc & ~7u;
	m_wait_words.assign(words.begin(), words.end());
	m_wait_bytes.assign(8 * words.size(), 0);
	for (size_t k = 0; k < words.size(); k++)
		c82_put(&m_wait_bytes[8 * k], 3, words[k]);
	std::memset(m_park.state.data(), 0, sizeof(m_park.state));
}

namespace {

/* memcmp(&a, &b, sizeof(a)) == 0 in line, 8 bytes at a time */
bool
same_state(const C82PpState &a, const C82PpState &b)
{
	static_assert(sizeof(C82PpState) % 8 == 0);
	const auto *const x = reinterpret_cast<const uint8_t *>(&a), *const y = reinterpret_cast<const uint8_t *>(&b);
	uint64_t diff = 0;

	for (size_t i = 0; i < sizeof(C82PpState); i += 8) {
		uint64_t u, v;

		std::memcpy(&u, x + i, 8);
		std::memcpy(&v, y + i, 8);
		diff |= u ^ v;
	}
	return diff == 0;
}

} /* namespace */

/* The fetch stage is at the wait's first instruction: the state is kept
 * there; at the next pass, if it is the same but for the cycles and
 * nothing spoiled the pass, the PP is parked */
void
C82Pp::pass_boundary()
{
	/* The pass a wake catches up (wake) is not measured: it nearly
	 * always leaves the wait, and parking a pass later changes nothing
	 * emulated (it is only a way to run it) */
	if (m_park.replaying)
		return;

	C82PpState &now = m_park.state[m_park.at ^ 1];
	const C82PpState &start = m_park.state[m_park.at];

	save(now);
	if (m_park.measuring && !m_park.spoiled) {
		const uint64_t period = m_cycles - start.cycles;

		now.cycles = start.cycles;
		/* The Q bit already seen; nothing else a pass reads changes with
		 * time but the board's answers, looked at each time (parked_run) */
		if (period && same_state(now, start) && !m_halted && !m_in_reset &&
		    !m_illegal && !m_faulted && m_q_visible <= start.cycles) {
			m_park.measuring = false;
			m_park.on = true;
			m_park.quiet = 0;
			m_park.base = m_park.next_pass = m_cycles;
			m_park.period = period;
			/* It stands at the pass's start (run() sets `until` there) */
			m_park.next_k = 0;
			m_park.next_read = m_cycles + m_park.board[0].at;
			return;
		}
		now.cycles = m_cycles;
	}
	stop_watching();

	/* The wait's words in memory, as the board found them: compared in
	 * place when they lie in one page of memory or one block of RAM */
	const uint8_t *const code = m_chip.pp_code(m_wait_pc);
	const size_t room = m_wait_pc >= 0x02000000 ? 0x100000 - (m_wait_pc & 0xFFFFF) : 0x1000 - (m_wait_pc & 0xFFF);

	if (code && m_wait_bytes.size() <= room) {
		if (std::memcmp(code, m_wait_bytes.data(), m_wait_bytes.size()) != 0)
			return;
	} else {
		for (size_t k = 0; k < m_wait_words.size(); k++) {
			uint64_t word = 0;

			if (!m_chip.pp_fetch(m_wait_pc + 8 * static_cast<uint32_t>(k), word) || word != m_wait_words[k])
				return;
		}
	}
	m_park.at ^= 1;
	m_park.measuring = true;
	m_park.spoiled = false;
	m_park.cycles = 0;
	watch(m_wait_pc, m_wait_pc + 8 * static_cast<uint32_t>(m_wait_words.size()), false);
}

/* A load made while measuring a pass: plain memory is watched; a read of
 * the board is kept, where it falls and what it answered (the board must
 * be able to tell an answer without making the read: C82Bus::peek) */
void
C82Pp::note_read(uint32_t addr, unsigned size, uint64_t value)
{
	const unsigned bytes = 1u << size;
	const int place = m_chip.place(addr);
	uint64_t seen = 0;

	if (place == Tms320c82::PLACE_NONE) {
		m_park.spoiled = true;
	} else if (place == Tms320c82::PLACE_BOARD) {
		if (m_park.boards == Park::BOARD_READS || !m_chip.peek(addr, size, seen)) {
			m_park.spoiled = true;
			return;
		}
		/* Two reads of one address in a pass must agree (catching up
		 * gives back one answer for each) */
		for (unsigned k = 0; k < m_park.boards; k++)
			if (m_park.board[k].addr == addr && m_park.board[k].size == size && m_park.board[k].value != value)
				m_park.spoiled = true;
		m_park.board[m_park.boards++] =
			BoardRead{ static_cast<uint32_t>(m_cycles - m_park.state[m_park.at].cycles), addr, static_cast<uint8_t>(size), value };
	} else {
		watch(addr, addr + bytes, place == Tms320c82::PLACE_INTERNAL);
	}
}

/* The pass stays in the wait's words */
void
C82Pp::note_fetch(uint32_t addr)
{
	if (addr < m_wait_pc || addr - m_wait_pc >= 8 * m_wait_words.size())
		m_park.spoiled = true;
}

void
C82Pp::watch(uint32_t lo, uint32_t hi, bool internal)
{
	for (unsigned k = 0; k < m_park.watches; k++)
		if (m_park.watch[k].internal == internal && lo >= m_park.watch[k].lo && hi <= m_park.watch[k].hi)
			return;
	if (m_park.watches == Park::WATCHES) {
		m_park.spoiled = true;
		return;
	}
	m_park.watch[m_park.watches++] = Watch{ lo, hi, internal };
	m_chip.watch_added(1 + m_number, lo, hi, internal);
}

bool
C82Pp::watches(uint32_t addr, uint32_t bytes) const
{
	if (!m_park.on && !m_park.measuring)
		return false;
	for (unsigned k = 0; k < m_park.watches; k++)
		if (addr < m_park.watch[k].hi && addr + bytes > m_park.watch[k].lo)
			return true;
	return false;
}

void
C82Pp::stop_watching()
{
	const bool had = m_park.on || m_park.measuring;

	m_park.on = m_park.measuring = false;
	m_park.watches = m_park.boards = 0;
	if (had)
		m_chip.watch_changed(1 + m_number);
}

void
C82Pp::disturb()
{
	settle_flow();
	if (m_park.on) {
		wake();
		/* The catch-up may end in blocks */
		settle_flow();
	}
	/* Also a pass begun while catching up: what it read is changing */
	stop_watching();
}

/* Back to where the PP stands after its last slice: the whole passes since
 * it was parked counted, then the pass in which that slice ended run as
 * usual; its reads of the board were made already (parked_run), they give
 * back their answers */
void
C82Pp::wake()
{
	const uint64_t target = m_park.until, base = m_park.base, period = m_park.period;

	m_park.replay = m_park.board;
	m_park.replays = m_park.boards;
	stop_watching();
	if (target > base) {
		m_cycles = base + (target - base - 1) / period * period;
		m_park.replaying = true;
		/* The wait's first word (steps() stops there at once), then
		 * blocks where they can as in run() (steps() makes the
		 * same cycles as cycle()). Blocks never read the board: the
		 * interpreter does, and load() gives back the answers */
		if (!steps(target))
			cycle();
		while (m_cycles < target && !m_halted && !m_illegal && !m_faulted) {
			const int blocks = run_blocks(target);

			if (blocks > 0)
				continue;
			if (!steps(blocks == 0 ? m_cycles + 1 : target))
				cycle();
		}
		m_park.replaying = false;
		if (m_cycles < target)
			m_cycles = target;
	}
}

void
C82Pp::save(C82PpState &out) const
{
	/* memcpy, in line (std::copy_n called memmove) */
	std::memcpy(out.r.data(), m_r.data(), sizeof(out.r));
	out.sr = m_sr;
	out.mf = m_mf;
	out.pc = m_pc;
	out.iprs = m_iprs;
	out.inten = m_inten;
	out.intflg = m_intflg;
	out.lctl = m_lctl;
	out.lc = m_lc;
	out.lr = m_lr;
	out.ls = m_ls;
	out.le = m_le;
	out.comm_hs = m_comm_hs;
	out.q_visible_lo = static_cast<uint32_t>(m_q_visible);
	out.q_visible_hi = static_cast<uint32_t>(m_q_visible >> 32);
	const Stage *stages[2] = { &a(), &e() };
	for (unsigned k = 0; k < 2; k++) {
		out.stage_word[k] = stages[k]->word;
		out.stage_addr[k] = stages[k]->addr;
		out.stage_ea0[k] = stages[k]->ea[0];
		out.stage_ea1[k] = stages[k]->ea[1];
		out.stage_kind[k] = stages[k]->kind;
		out.stage_extra[k] = 0;
		if (m_flowing) {
			/* The stages as blocks left them (materialize) */
			const InStage s = in_stage(m_flow, static_cast<int>(k));

			out.stage_word[k] = __builtin_bswap64(s.raw);
			out.stage_addr[k] = s.addr;
			out.stage_ea0[k] = k ? m_flow.e_ea[0] : 0;
			out.stage_ea1[k] = k ? m_flow.e_ea[1] : 0;
			out.stage_kind[k] = INSN;
		}
	}
	out.ipa = out.stage_addr[0];
	out.ipe = out.stage_addr[1];
	out.seq = m_seq;
	out.seq_step = m_seq_step;
	out.seq_bit = m_seq_bit;
	out.halted = m_halted;
	out.in_reset = m_in_reset;
	out.illegal = m_illegal;
	out.faulted = m_faulted;
	out.fault_type = m_fault_type;
	out.seq_p1 = m_seq_p1;
	out.seq_p2 = m_seq_p2;
	out.seq_p3 = m_seq_p3;
	out.reset_ipa = m_reset_ipa;
	out.reset_ipe = m_reset_ipe;
	out.reset_pc = m_reset_pc;
	out.cycles = m_cycles;
	out.reset_done = m_reset_done;
}

void
C82Pp::load(const C82PpState &in)
{
	stop_watching();
	m_flowing = false;
	m_resume = nullptr;
	/* A state comes with the memories its code is fetched from,
	 * written behind the blocks (their words are not all checked) */
	flush_blocks();
	std::memcpy(m_r.data(), in.r.data(), sizeof(in.r));
	m_sr = in.sr;
	m_mf = in.mf;
	m_pc = in.pc;
	m_iprs = in.iprs;
	m_inten = in.inten;
	m_intflg = in.intflg;
	m_lctl = in.lctl;
	m_le = in.le;
	loops_changed();
	m_lc = in.lc;
	m_lr = in.lr;
	m_ls = in.ls;
	m_comm_hs = in.comm_hs;
	m_q_visible = static_cast<uint64_t>(in.q_visible_hi) << 32 | in.q_visible_lo;
	Stage *stages[2] = { &a(), &e() };
	for (unsigned k = 0; k < 2; k++) {
		*stages[k] = Stage{};
		stages[k]->word = in.stage_word[k];
		stages[k]->addr = in.stage_addr[k];
		stages[k]->ea = { in.stage_ea0[k], in.stage_ea1[k] };
		stages[k]->kind = static_cast<Kind>(in.stage_kind[k]);
		stages[k]->stop = false;
		if (stages[k]->kind == INSN) {
			stages[k]->own = c82_pp_decode(in.stage_word[k]);
			stages[k]->insn = &stages[k]->own;
			stages[k]->own_op = c82_pp_op(stages[k]->own);
			stages[k]->op = &stages[k]->own_op;
			stages[k]->stop = stages[k]->own.illegal;
		}
	}
	a().addr = in.ipa;
	e().addr = in.ipe;
	m_seq = static_cast<Seq>(in.seq);
	m_seq_step = in.seq_step;
	m_seq_bit = in.seq_bit;
	m_halted = in.halted != 0;
	m_in_reset = in.in_reset != 0;
	m_illegal = in.illegal != 0;
	m_faulted = in.faulted != 0;
	m_fault_type = in.fault_type != 0;
	m_seq_p1 = in.seq_p1;
	m_seq_p2 = in.seq_p2;
	m_seq_p3 = in.seq_p3;
	m_reset_ipa = in.reset_ipa;
	m_reset_ipe = in.reset_ipe;
	m_reset_pc = in.reset_pc;
	m_cycles = in.cycles;
	m_reset_done = in.reset_done;
}

} /* namespace odin */
