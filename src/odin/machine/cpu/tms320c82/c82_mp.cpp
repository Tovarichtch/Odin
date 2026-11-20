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

#include "odin/machine/cpu/tms320c82/c82_mp.hpp"

#include <algorithm>
#include <bit>
#include <cfenv>
#include <cstdio>
#include <cstring>

#include "odin/common/repeats.hpp"
#include "odin/machine/cpu/tms320c82/c82_mp_host.hpp"
#include "odin/machine/cpu/tms320c82/tms320c82.hpp"

namespace odin {

namespace {

/* CONFIG (SPRU109 fig. 3-7; SPRS048 fig. 10): big-endian (E = 0, set by
 * READY at reset on this board), Type 2: two PPs (§ 3.6.1), Deriv 2: the
 * C82. HEURISTIC: Release, "the TMS320C82 version number", is not given;
 * the program never reads it. */
constexpr uint32_t CONFIG_FIXED = (2u << 12) | (2u << 8);
/* Its R, T, H, X bits, the only ones written (§ 3.6) */
constexpr uint32_t CONFIG_WRITTEN = 0x78000000u;
/* FPST: fm at reset (§ 3.3.3); the bits written: drm, fm, fs, sm and the
 * accumulated status (bits 17-26, § 3.3.3.2-3.3.3.4) */
constexpr uint32_t FPST_FM = 1u << 19, FPST_WRITTEN = 0x07FE0000u;
/* PKTREQ (§ 3.2) */
enum : uint32_t { REQ_P = 1, REQ_Q = 2, REQ_S = 4, REQ_F = 8, REQ_I = 16 };
/* MP interrupt and trap vectors (SPRS048 tables 3 and 4) */
constexpr uint32_t VECTORS = 0x01010180;

int32_t
short_signed(uint32_t w)
{
	return static_cast<int32_t>(w << 17) >> 17;
}

} /* namespace */

C82Insn
c82_decode(uint32_t w, uint32_t next)
{
	C82Insn in;

	in.dest = static_cast<uint8_t>((w >> 27) & 31);
	in.src2 = static_cast<uint8_t>((w >> 22) & 31);
	in.src1 = static_cast<uint8_t>(w & 31);
	/* A long immediate follows when bits 21, 20 and 12 are set (§ 4.1.1) */
	if ((w & 0x00301000u) == 0x00301000u)
		in.length = 8;

	if ((w & 0x00300000u) != 0x00300000u) {
		/* Short immediate (table 10-8) */
		const uint32_t op = (w >> 15) & 0x7F, u = w & 0x7FFF;
		const uint32_t s = static_cast<uint32_t>(short_signed(w));

		in.form = C82Insn::SHORT;
		in.imm = u;
		if (op >= 0x08 && op <= 0x0F) {
			in.op = C82Insn::SHIFT;
			in.merge = static_cast<uint8_t>(op & 7);
			in.invert = (w >> 11) & 1;
			in.right = (w >> 10) & 1;
			in.endmask = static_cast<uint8_t>((w >> 5) & 31);
			in.rotate = static_cast<uint8_t>(w & 31);
			return in;
		}
		if (op >= 0x10 && op <= 0x1F) {
			static constexpr bool LOGIC_CODES[16] = { false, true, true, false, true, false, true, true,
				                                  true, true, false, true, false, true, true, false };

			in.op = LOGIC_CODES[op & 15] ? C82Insn::LOGIC : C82Insn::RESERVED;
			in.logic = static_cast<uint8_t>(op & 15);
			return in;
		}
		if (op >= 0x20 && op <= 0x3F) {
			const uint32_t group = (op >> 3) & 3;

			in.modify = (op >> 2) & 1;
			in.size = static_cast<uint8_t>(op & 3);
			in.imm = s;
			if (group == 3) {
				in.op = in.size ? C82Insn::RESERVED : C82Insn::DCACHE;
				in.flush = (w >> 27) & 1;
			} else {
				in.op = group == 2 ? C82Insn::STORE : C82Insn::LOAD;
				in.is_unsigned = group == 1;
				/* ld.u: halfword and byte only (table 10-20) */
				if (in.is_unsigned && in.size >= 2)
					in.op = C82Insn::RESERVED;
			}
			return in;
		}
		switch (op) {
		case 0x00:
			in.op = C82Insn::ILLOP;
			break;
		case 0x01:
			in.op = C82Insn::TRAP;
			in.emulation = (w >> 27) & 1;
			break;
		case 0x02:
			in.op = C82Insn::CMND;
			break;
		case 0x04:
			in.op = C82Insn::RDCR;
			break;
		case 0x05:
			in.op = C82Insn::SWCR;
			break;
		case 0x06:
			in.op = C82Insn::BRCR;
			break;
		case 0x40:
		case 0x41:
		case 0x44:
		case 0x45:
		case 0x48:
		case 0x49:
		case 0x4A:
		case 0x4B:
		case 0x4C:
		case 0x4D: {
			static constexpr C82Insn::Op BRANCHES[] = { C82Insn::BSR, C82Insn::RESERVED, C82Insn::JSR,
				                                    C82Insn::RESERVED, C82Insn::BBZ, C82Insn::BBO, C82Insn::BCND };

			in.op = BRANCHES[(op >> 1) & 7];
			in.annul = op & 1;
			in.imm = s;
			break;
		}
		case 0x50:
			in.op = C82Insn::CMP;
			in.imm = s;
			break;
		case 0x58:
		case 0x59:
		case 0x5A:
		case 0x5B:
			in.op = op & 2 ? C82Insn::SUB : C82Insn::ADD;
			in.is_unsigned = op & 1;
			in.imm = s;
			break;
		default:
			in.op = C82Insn::RESERVED;
			break;
		}
		return in;
	}

	/* Register and long-immediate forms (tables 10-9, 10-10) */
	const uint32_t op = (w >> 13) & 0x7F;

	in.form = (w >> 12) & 1 ? C82Insn::LONG : C82Insn::REGISTER;
	if (in.form == C82Insn::LONG)
		in.imm = next;
	if (op >= 0x08 && op <= 0x0F) {
		/* No long immediate form of the shifts */
		in.op = in.form == C82Insn::LONG ? C82Insn::RESERVED : C82Insn::SHIFT;
		in.merge = static_cast<uint8_t>(op & 7);
		in.invert = (w >> 11) & 1;
		in.right = (w >> 10) & 1;
		in.endmask = static_cast<uint8_t>((w >> 5) & 31);
		return in;
	}
	if (op >= 0x10 && op <= 0x1F) {
		static constexpr bool LOGIC_CODES[16] = { false, true, true, false, true, false, true, true,
			                                  true, true, false, true, false, true, true, false };

		in.op = LOGIC_CODES[op & 15] ? C82Insn::LOGIC : C82Insn::RESERVED;
		in.logic = static_cast<uint8_t>(op & 15);
		return in;
	}
	if (op >= 0x20 && op <= 0x3F) {
		const uint32_t group = (op >> 3) & 3;

		in.modify = (op >> 2) & 1;
		in.size = static_cast<uint8_t>(op & 3);
		if (group == 3) {
			in.op = in.size ? C82Insn::RESERVED : C82Insn::DCACHE;
			in.flush = (w >> 27) & 1;
		} else {
			in.op = group == 2 ? C82Insn::STORE : C82Insn::LOAD;
			in.is_unsigned = group == 1;
			in.scale = (w >> 11) & 1;
			in.dea = (w >> 10) & 1;
			if (in.is_unsigned && in.size >= 2)
				in.op = C82Insn::RESERVED;
		}
		return in;
	}
	if (op >= 0x60 && op <= 0x6F) {
		/* Vector (table 10-10): bits 21-17 = 11110 */
		in.op = C82Insn::VECTOR;
		in.vop = static_cast<uint8_t>((w >> 13) & 7);
		in.acc = static_cast<uint8_t>((((w >> 16) & 1) << 1) | ((w >> 11) & 1));
		in.mem = static_cast<uint8_t>((((w >> 10) & 1) << 1) | ((w >> 6) & 1));
		in.mem_double = (w >> 9) & 1;
		in.zero = (w >> 8) & 1;
		in.memreg = in.dest;
		if (in.vop == 4) {
			in.pd = static_cast<uint8_t>((w >> 7) & 3);
			in.p1 = static_cast<uint8_t>((w >> 5) & 1);
		} else if (in.vop == 5) {
			in.pd = static_cast<uint8_t>((w >> 7) & 1);
			in.p1 = static_cast<uint8_t>(FP_INT + ((w >> 5) & 1));
		} else {
			in.pd = static_cast<uint8_t>((w >> 7) & 1);
			in.p1 = static_cast<uint8_t>((w >> 5) & 1);
		}
		return in;
	}
	if (op >= 0x70 && op <= 0x77) {
		/* Floating point: bits 21-16 = 111110 */
		static constexpr C82Insn::Op FLOATS[] = { C82Insn::FADD, C82Insn::FSUB, C82Insn::FMPY, C82Insn::FDIV,
			                                  C82Insn::FRND, C82Insn::FCMP, C82Insn::RESERVED, C82Insn::FSQRT };

		in.op = FLOATS[op & 7];
		in.pd = static_cast<uint8_t>((w >> 9) & 3);
		in.p2 = static_cast<uint8_t>((w >> 7) & 3);
		in.rm = in.p2;
		in.p1 = static_cast<uint8_t>((w >> 5) & 3);
		return in;
	}
	if (op >= 0x78) {
		/* Miscellaneous: bits 21-16 = 111111 */
		static constexpr C82Insn::Op MISC[] = { C82Insn::LMO, C82Insn::RMO, C82Insn::RESERVED, C82Insn::RESERVED,
			                                C82Insn::RESERVED, C82Insn::RESERVED, C82Insn::ESTOP, C82Insn::ILLOP };

		in.op = MISC[op & 7];
		return in;
	}
	switch (op) {
	case 0x01:
		in.op = C82Insn::TRAP;
		in.emulation = (w >> 27) & 1;
		break;
	case 0x02:
		in.op = C82Insn::CMND;
		break;
	case 0x04:
		in.op = C82Insn::RDCR;
		break;
	case 0x05:
		in.op = C82Insn::SWCR;
		break;
	case 0x06:
		in.op = C82Insn::BRCR;
		break;
	case 0x40:
	case 0x41:
		in.op = C82Insn::BSR;
		in.annul = op & 1;
		break;
	case 0x44:
	case 0x45:
		in.op = C82Insn::JSR;
		in.annul = op & 1;
		break;
	case 0x48:
	case 0x49:
		in.op = C82Insn::BBZ;
		in.annul = op & 1;
		break;
	case 0x4A:
	case 0x4B:
		in.op = C82Insn::BBO;
		in.annul = op & 1;
		break;
	case 0x4C:
	case 0x4D:
		in.op = C82Insn::BCND;
		in.annul = op & 1;
		break;
	case 0x50:
		in.op = C82Insn::CMP;
		break;
	case 0x58:
	case 0x59:
	case 0x5A:
	case 0x5B:
		in.op = op & 2 ? C82Insn::SUB : C82Insn::ADD;
		in.is_unsigned = op & 1;
		break;
	default:
		in.op = C82Insn::RESERVED;
		break;
	}
	return in;
}

C82Mp::C82Mp(Tms320c82 &chip) : m_chip(chip)
{
	/* Every place starts with the word 0 prepared */
	prepare(m_ops[0], 0, 0);
	std::fill(m_ops.begin() + 1, m_ops.end(), m_ops[0]);
	std::fill(m_decoded.begin() + 1, m_decoded.end(), m_decoded[0]);
	check_host_rounding();
}

void
C82Mp::check_host_rounding()
{
	m_host_nearest = std::fegetround() == FE_TONEAREST;
}

/* The state after a power-up reset (§ 9.5). HEURISTIC: what TI calls
 * undefined (r1-r31, the accumulators, the other control registers, the
 * timer) starts at 0. */
void
C82Mp::reset()
{
	stop_watching();
	m_r.fill(0);
	m_acc.fill(0);
	m_ip = 0xFFFFFFF8u;
	m_pc = 0xFFFFFFFCu;
	m_epc = m_eip = 0;
	m_config = CONFIG_FIXED;
	m_intpen = m_ie = 0;
	m_fpst = FPST_FM;
	m_pktreq = 0;
	m_tscale = 0;
	m_fltop = m_fltadr = m_flttag = m_fltdtl = m_fltdth = 0;
	m_sysstk = m_systmp = m_mpc = m_mip = 0;
	m_ecomcntl = m_anastat = m_brk1 = m_brk2 = 0;
	/* HEURISTIC: the "known state" of the LRU registers (§ 9.5) is not
	 * given */
	m_ilru = m_dlru = 0;
	m_in0p = m_in1p = m_outp = 0;
	m_itag.fill(0);
	m_dtag.fill(0);
	m_timer_zero = m_cycles;
	m_user = false;
	m_self_message = 0;
	m_fault = Fault{};
	m_in_slot = m_slot_next = false;
	m_check = true;
}

/* What runs in most slices (parked all through it, or resumed in the
 * host code) kept short, in line in Tms320c82::run; the rest out of line
 * (wake_on, run_steps), in the same order */
void
C82Mp::run(uint64_t until)
{
	if (m_park.on) {
		/* No step of the run reaches the timer's zero */
		if (until <= m_timer_zero) {
			if (until > m_cycles)
				m_cycles = parked_end(until);
			return;
		}
		wake_on();
	}
	m_until = until;
	/* The slice going on in the host code where the last one stopped
	 * (a block's step, never the wait): fast_steps()' first turn, its tests
	 * kept, without its frame; then on as before when the slice is not
	 * over */
	if (m_ip == m_resume_ip && !m_check && !m_park.measuring && m_ip != m_wait_ip && m_native && !m_slot_next &&
	    m_pc == m_ip + 4) {
		const uint64_t limit = std::min(until, m_timer_zero);

		if (m_cycles < limit) {
			set_float_mode();
			m_resume_ip = ~0u;
			m_code->run(this, m_resume_code, m_chip.mp_pages(), limit);
			if (m_cycles >= until)
				return;
		}
	}
	run_steps();
}

void
C82Mp::wake_on()
{
	wake();
}

void
C82Mp::run_steps()
{
	while (m_cycles < m_until) {
		if (!m_check && !m_park.measuring) {
			fast_steps();
			if (m_cycles >= m_until)
				break;
		}
		if (m_ip == m_wait_ip) {
			pass_boundary();
			if (m_park.on) {
				if (m_until <= m_timer_zero) {
					m_cycles = parked_end(m_until);
					return;
				}
				wake();
			}
		}
		if (m_cycles >= m_timer_zero)
			advance_timer();
		if (m_park.measuring) {
			if (m_park.steps == Park::STEPS)
				m_park.spoiled = true;
			else
				m_park.step[m_park.steps++] = static_cast<uint32_t>(m_cycles - m_park.start.cycles);
		}
		step();
		m_check = attention();
	}
}

/* The steps where step() and the run's loop would only fetch and execute:
 * nothing to check (attention() false, no measured pass), not at the wait,
 * the timer's zero ahead, the code in plain external memory, read straight
 * from it. Returns at the first step that is not one of these. */
void
C82Mp::fast_steps()
{
	set_float_mode();
	const uint64_t limit = std::min(m_until, m_timer_zero);
	const uint32_t wait_ip = m_wait_ip;
	C82MpOp *const ops = m_ops.data();
	/* The 1 MB page of the code, as mp_code gives it */
	uint32_t page = ~0u;
	const uint8_t *base = nullptr;

	while (m_cycles < limit && !m_check) {
		const uint32_t ip = m_ip;

		if (ip == wait_ip)
			return;
		/* A block of host code, from the normal state */
		if (m_native && !m_slot_next && m_pc == ip + 4 && run_blocks(ip, limit))
			continue;
		if (ip >> 20 != page) {
			base = m_chip.mp_code(ip & ~0xFFFFFu);
			if (!base)
				return;
			page = ip >> 20;
		}

		const uint32_t word = static_cast<uint32_t>(c82_get(base + (ip & 0xFFFFF), 2));
		C82MpOp &op = ops[(ip >> 2) & (DECODED - 1)];
		uint32_t next = m_pc;

		/* Prepared for this word: a short one is checked; a long one (or
		 * another word) has its immediate read and checked too */
		if (static_cast<uint32_t>(op.key) != word || op.length == 8) {
			uint32_t immediate = 0;

			if ((word & 0x00301000u) == 0x00301000u) {
				/* The long immediate, at PC (§ 4.1.1) */
				if (next >> 20 != page)
					return;
				immediate = static_cast<uint32_t>(c82_get(base + (next & 0xFFFFF), 2));
				next += 4;
			}
			if (op.key != (word | static_cast<uint64_t>(immediate) << 32))
				prepare(op, word, immediate);
		}
		m_in_slot = m_slot_next;
		m_slot_next = false;
		m_cycles += op.length >> 2;
		m_ip = next;
		m_pc = next + 4;
		op.exec(*this, op, ip);
	}
}

void
C82Mp::set_wait(uint32_t ip, std::span<const uint32_t> words)
{
	stop_watching();
	/* Blocks end before the wait */
	flush_blocks();
	m_wait_ip = words.empty() ? ~0u : ip;
	m_wait_words.assign(words.begin(), words.end());
}

/* At the wait's first instruction, as C82Pp::pass_boundary. Parked only
 * with no interrupt to take, no fault or message on its way, the timer's
 * zero ahead */
void
C82Mp::pass_boundary()
{
	C82MpState now;

	std::memset(&now, 0, sizeof(now));
	save(now);
	if (m_park.measuring && !m_park.spoiled) {
		const uint64_t period = m_cycles - m_park.start.cycles;

		now.cycles = m_park.start.cycles;
		if (period && std::memcmp(&now, &m_park.start, sizeof(now)) == 0 && !m_fault.on && !m_self_message &&
		    !((m_ie & INT_IE) && (m_intpen & m_ie & ~INT_IE)) && m_timer_zero > m_cycles) {
			m_park.measuring = false;
			m_park.on = true;
			m_park.base = m_park.pass = m_cycles;
			m_park.period = period;
			for (unsigned at = 0, k = 0; at < period && at < Park::NEXT; at++) {
				while (k < m_park.steps && m_park.step[k] < at)
					k++;
				m_park.next[at] = static_cast<uint8_t>(k < m_park.steps ? m_park.step[k] : period);
			}
			return;
		}
		now.cycles = m_cycles;
	}
	stop_watching();
	for (size_t k = 0; k < m_wait_words.size(); k++) {
		uint64_t word = 0;

		if (!m_chip.mp_fetch(m_wait_ip + 4 * static_cast<uint32_t>(k), word) || word != m_wait_words[k])
			return;
	}
	m_park.start = now;
	m_park.measuring = true;
	m_park.spoiled = false;
	m_park.steps = 0;
	watch(m_wait_ip, m_wait_ip + 4 * static_cast<uint32_t>(m_wait_words.size()), false);
}

/* Where a run to `until` ends while parked: the first start of a step at
 * or after it, the passes repeating from `base`. The pass holding `until`
 * found from the last one (a slice is short), or by a division */
uint64_t
C82Mp::parked_end(uint64_t until)
{
	return parked_end_from(m_park.pass, until);
}

uint64_t
C82Mp::parked_end_from(uint64_t &found, uint64_t until) const
{
	uint64_t pass = found;

	if (until - pass >= 8 * m_park.period)
		pass = m_park.base + (until - m_park.base) / m_park.period * m_park.period;
	else
		while (until - pass >= m_park.period)
			pass += m_park.period;
	found = pass;

	const uint64_t rest = until - pass;

	if (m_park.period <= Park::NEXT)
		return pass + m_park.next[rest];
	for (unsigned k = 0; k < m_park.steps; k++)
		if (m_park.step[k] >= rest)
			return pass + m_park.step[k];
	return pass + m_park.period;
}

/* Back to the state where its last run ended: the whole passes counted,
 * the steps of the last one run as usual */
void
C82Mp::wake()
{
	const uint64_t target = m_cycles;

	m_cycles = m_park.base + (target - m_park.base) / m_park.period * m_park.period;
	stop_watching();
	while (m_cycles < target)
		step();
	m_check = true;
}

void
C82Mp::note_read(uint32_t addr, unsigned size)
{
	const Tms320c82::Place place = m_chip.place(addr);

	if (place == Tms320c82::PLACE_INTERNAL || place == Tms320c82::PLACE_MEMORY)
		watch(addr, addr + (1u << size), place == Tms320c82::PLACE_INTERNAL);
	else
		m_park.spoiled = true;
}

void
C82Mp::watch(uint32_t lo, uint32_t hi, bool internal)
{
	for (unsigned k = 0; k < m_park.watches; k++)
		if (m_park.watch[k].internal == internal && lo >= m_park.watch[k].lo && hi <= m_park.watch[k].hi)
			return;
	if (m_park.watches == Park::WATCHES) {
		m_park.spoiled = true;
		return;
	}
	m_park.watch[m_park.watches++] = Watch{ lo, hi, internal };
	m_chip.watch_added(lo, hi, internal);
}

bool
C82Mp::watches(uint32_t addr, uint32_t bytes) const
{
	if (!m_park.on && !m_park.measuring)
		return false;
	for (unsigned k = 0; k < m_park.watches; k++)
		if (addr < m_park.watch[k].hi && addr + bytes > m_park.watch[k].lo)
			return true;
	return false;
}

void
C82Mp::stop_watching()
{
	const bool had = m_park.on || m_park.measuring;

	m_park.on = m_park.measuring = false;
	m_park.watches = 0;
	if (had)
		m_chip.watch_changed();
}

void
C82Mp::disturb()
{
	if (m_park.on)
		wake();
	stop_watching();
}

void
C82Mp::step()
{
	/* A nonmaskable memory fault of the last instruction: interrupt 14,
	 * whatever IE says (§ 3.3.2.6, 5.6.2) */
	if (m_fault.on) {
		m_fault.on = false;
		exception(14, m_ip, m_pc);
		return;
	}
	if (m_self_message && --m_self_message == 0)
		m_intpen |= INT_MI;
	m_intpen = m_lint4 ? m_intpen | INT_X4 : m_intpen & ~INT_X4;
	if ((m_ie & INT_IE) && (m_intpen & m_ie & ~INT_IE)) {
		take_interrupt();
		return;
	}

	const uint32_t ip = m_ip;
	uint64_t word = 0;

	m_in_slot = m_slot_next;
	m_slot_next = false;
	if (m_park.measuring && (ip < m_wait_ip || ip - m_wait_ip >= 4 * m_wait_words.size()))
		m_park.spoiled = true;
	if (!m_chip.mp_fetch(ip, word)) {
		/* An instruction-cache fault: FLTOP i, EPC the address (§ 3.8.1,
		 * table 5-4) */
		m_fltop |= 1u << 12;
		exception(14, ip, ip);
		return;
	}

	uint32_t next = m_pc;
	uint64_t immediate = 0;

	m_cycles++;
	if ((word & 0x00301000u) == 0x00301000u) {
		/* The long immediate is fetched at PC, one cycle more (§ 4.1.1) */
		if (m_park.measuring && (m_pc < m_wait_ip || m_pc - m_wait_ip >= 4 * m_wait_words.size()))
			m_park.spoiled = true;
		if (!m_chip.mp_fetch(m_pc, immediate)) {
			m_fltop |= 1u << 12;
			exception(14, ip, m_pc);
			return;
		}
		next = m_pc + 4;
		m_cycles++;
	}
	m_ip = next;
	m_pc = next + 4;
	m_now_ip = ip;
	m_now_next = next;
	m_now_long = (word & 0x00301000u) == 0x00301000u;

	const C82MpOp &op = prepared(ip, static_cast<uint32_t>(word), static_cast<uint32_t>(immediate));

	set_float_mode();
	op.exec(*this, op, ip);
}

void
C82Mp::take_interrupt()
{
	/* The lowest bit first (§ 3.3.2) */
	const uint32_t pending = m_intpen & m_ie & ~INT_IE;

	exception(static_cast<unsigned>(std::countr_zero(pending)), m_ip, m_pc);
}

/* PC and IP saved in EPC and EIP, ie in EPC bit 0, the user mode in bit 1;
 * interrupts off, supervisor mode; the vector of N read (§ 9.2, fig. 9-1).
 * HEURISTIC: two cycles. */
void
C82Mp::exception(unsigned number, uint32_t eip, uint32_t epc)
{
	uint64_t vector = 0;

	m_park.spoiled = true;
	m_check = true;
	m_eip = eip;
	m_epc = (epc & ~3u) | (m_ie & INT_IE) | (m_user ? 2u : 0u);
	m_ie &= ~INT_IE;
	m_user = false;
	m_slot_next = false;
	if (!m_chip.mp_load(VECTORS + 4 * number, 2, vector))
		repeats::print(stderr, "mp: the vector of exception %u cannot be read\n", number);
	m_ip = static_cast<uint32_t>(vector) & ~3u;
	m_pc = m_ip + 4;
	m_cycles += 2;
}

uint32_t
C82Mp::tcount() const
{
	if (m_cycles <= m_timer_zero)
		return static_cast<uint32_t>(m_timer_zero - m_cycles);

	const uint64_t period = static_cast<uint64_t>(m_tscale) + 1;

	return static_cast<uint32_t>(m_tscale - (m_cycles - m_timer_zero - 1) % period);
}

void
C82Mp::set_tcount(uint32_t value)
{
	m_timer_zero = m_cycles + value;
}

/* TCOUNT went down to 0: ti, and TSCALE again, TSCALE + 1 cycles a turn
 * (§ 3.5, example 3-1) */
void
C82Mp::advance_timer()
{
	const uint64_t period = static_cast<uint64_t>(m_tscale) + 1;

	m_intpen |= INT_TI;
	m_check = true;
	m_timer_zero += ((m_cycles - m_timer_zero) / period + 1) * period;
}

uint32_t
C82Mp::control(uint32_t number)
{
	disturb();
	return read_control(number);
}

uint32_t
C82Mp::read_control(uint32_t n)
{
	m_park.spoiled = true;
	if (n >= ITAG0 && n < ITAG0 + 16)
		return m_itag[n - ITAG0];
	if (n >= DTAG0 && n < DTAG0 + 16)
		return m_dtag[n - DTAG0];
	switch (n) {
	case EPC:
		return m_epc;
	case EIP:
		return m_eip;
	case CONFIG:
		return m_config;
	case INTPEN:
		return m_lint4 ? m_intpen | INT_X4 : m_intpen & ~INT_X4;
	case IE:
		return m_ie;
	case FPST:
		return m_fpst;
	case PPERROR:
		return m_chip.pperror();
	case PKTREQ:
		return (m_pktreq & (REQ_S | REQ_F | REQ_I)) | (m_chip.tc().queued() ? REQ_Q : 0u);
	case TCOUNT:
		return tcount();
	case TSCALE:
		return m_tscale;
	case FLTOP:
		return m_fltop;
	case FLTADR:
		return m_fltadr;
	case FLTTAG:
		return m_flttag;
	case FLTDTL:
		return m_fltdtl;
	case FLTDTH:
		return m_fltdth;
	case SYSSTK:
		return m_sysstk;
	case SYSTMP:
		return m_systmp;
	case MPC:
		return m_mpc;
	case MIP:
		return m_mip;
	case ECOMCNTL:
		return m_ecomcntl;
	case ANASTAT:
		return m_anastat;
	case BRK1:
		return m_brk1;
	case BRK2:
		return m_brk2;
	case ILRU:
		return m_ilru;
	case DLRU:
		return m_dlru;
	case IN0P:
		return m_in0p;
	case IN1P:
		return m_in1p;
	case OUTP:
		return m_outp;
	default:
		/* An undefined number: undefined, no error (§ 10.5 note 2).
		 * HEURISTIC: 0 */
		repeats::print(stderr, "mp: control register 0x%04X is not defined\n", n);
		return 0;
	}
}

void
C82Mp::write_control(uint32_t n, uint32_t v)
{
	m_park.spoiled = true;
	m_check = true;
	/* User mode: control registers below 0x4000 are not written, without
	 * an error (§ 9.6) */
	if (m_user && n < 0x4000)
		return;
	if (n >= ITAG0 && n < ITAG0 + 16) {
		m_itag[n - ITAG0] = v;
		return;
	}
	if (n >= DTAG0 && n < DTAG0 + 16) {
		m_dtag[n - DTAG0] = v;
		return;
	}
	switch (n) {
	case EPC:
		m_epc = v;
		break;
	case EIP:
		m_eip = v;
		break;
	case CONFIG:
		m_config = (m_config & ~CONFIG_WRITTEN) | (v & CONFIG_WRITTEN);
		/* X allows the TC to take the XPTs (SPRU261 § 7.2) */
		m_chip.tc().sample_xpt(m_cycles);
		break;
	case INTPEN:
		/* A 1 clears its bit; x4 follows the pin; mf stays while FLTSTS
		 * holds a fault (§ 3.3.2, 5.6.3) */
		m_intpen &= ~v;
		if (m_chip.fltsts())
			m_intpen |= INT_MF;
		break;
	case IE:
		m_ie = v;
		break;
	case FPST:
		m_fpst = (m_fpst & ~FPST_WRITTEN) | (v & FPST_WRITTEN);
		break;
	case PKTREQ: {
		const uint32_t suspend = m_pktreq & REQ_S;

		m_pktreq = v & (REQ_S | REQ_F | REQ_I);
		if ((v & REQ_S) && m_chip.tc().queued())
			repeats::print(stderr, "mp: packet transfer suspension: not emulated yet\n");
		/* P submits when Q is 0 and S is or goes to 0; with Q set, the
		 * request is refused with pb (§ 3.2, § 3.3.2.10) */
		if (v & REQ_P) {
			if (m_chip.tc().queued())
				m_intpen |= INT_PB;
			else if (!(v & REQ_S) || !suspend)
				m_chip.tc().request(m_cycles);
		}
		break;
	}
	case TCOUNT:
		set_tcount(v);
		end_run();
		break;
	case TSCALE:
		m_tscale = v;
		break;
	case FLTOP:
		m_fltop = v;
		break;
	case FLTADR:
		m_fltadr = v;
		break;
	case FLTTAG:
		m_flttag = v;
		break;
	case FLTDTL:
		m_fltdtl = v;
		break;
	case FLTDTH:
		m_fltdth = v;
		break;
	case SYSSTK:
		m_sysstk = v;
		break;
	case SYSTMP:
		m_systmp = v;
		break;
	case MPC:
		m_mpc = v;
		break;
	case MIP:
		m_mip = v;
		break;
	case ECOMCNTL:
		m_ecomcntl = v;
		break;
	case ANASTAT:
		m_anastat = v;
		break;
	case BRK1:
		m_brk1 = v;
		break;
	case BRK2:
		m_brk2 = v;
		break;
	case ILRU:
		m_ilru = v;
		break;
	case DLRU:
		m_dlru = v;
		break;
	case IN0P:
		m_in0p = v;
		break;
	case IN1P:
		m_in1p = v;
		break;
	case OUTP:
		m_outp = v;
		break;
	case PPERROR:
		break;
	default:
		repeats::print(stderr, "mp: control register 0x%04X is not defined\n", n);
		break;
	}
}

void
C82Mp::save(C82MpState &out) const
{
	out.r = m_r;
	out.acc = m_acc;
	out.ip = m_ip;
	out.pc = m_pc;
	out.epc = m_epc;
	out.eip = m_eip;
	out.config = m_config;
	out.intpen = m_intpen;
	out.ie = m_ie;
	out.fpst = m_fpst;
	out.pktreq = m_pktreq;
	out.tscale = m_tscale;
	out.fltop = m_fltop;
	out.fltadr = m_fltadr;
	out.flttag = m_flttag;
	out.fltdtl = m_fltdtl;
	out.fltdth = m_fltdth;
	out.sysstk = m_sysstk;
	out.systmp = m_systmp;
	out.mpc = m_mpc;
	out.mip = m_mip;
	out.ecomcntl = m_ecomcntl;
	out.anastat = m_anastat;
	out.brk1 = m_brk1;
	out.brk2 = m_brk2;
	out.ilru = m_ilru;
	out.dlru = m_dlru;
	out.in0p = m_in0p;
	out.in1p = m_in1p;
	out.outp = m_outp;
	out.itag = m_itag;
	out.dtag = m_dtag;
	out.cycles = m_cycles;
	out.timer_zero = m_timer_zero;
	out.user = m_user;
	out.lint4 = m_lint4;
	out.self_message = m_self_message;
	out.slot = m_slot_next;
	out.fault = m_fault.on;
}

void
C82Mp::load(const C82MpState &in)
{
	stop_watching();
	m_r = in.r;
	m_r[0] = 0;
	m_acc = in.acc;
	m_ip = in.ip;
	m_pc = in.pc;
	m_epc = in.epc;
	m_eip = in.eip;
	m_config = in.config;
	m_intpen = in.intpen;
	m_ie = in.ie;
	m_fpst = in.fpst;
	m_pktreq = in.pktreq;
	m_tscale = in.tscale;
	m_fltop = in.fltop;
	m_fltadr = in.fltadr;
	m_flttag = in.flttag;
	m_fltdtl = in.fltdtl;
	m_fltdth = in.fltdth;
	m_sysstk = in.sysstk;
	m_systmp = in.systmp;
	m_mpc = in.mpc;
	m_mip = in.mip;
	m_ecomcntl = in.ecomcntl;
	m_anastat = in.anastat;
	m_brk1 = in.brk1;
	m_brk2 = in.brk2;
	m_ilru = in.ilru;
	m_dlru = in.dlru;
	m_in0p = in.in0p;
	m_in1p = in.in1p;
	m_outp = in.outp;
	m_itag = in.itag;
	m_dtag = in.dtag;
	m_cycles = in.cycles;
	m_timer_zero = in.timer_zero;
	m_user = in.user != 0;
	m_lint4 = in.lint4 != 0;
	m_self_message = in.self_message;
	m_slot_next = in.slot != 0;
	m_fault.on = in.fault != 0;
	m_until = m_cycles;
	m_check = true;
}

} /* namespace odin */
