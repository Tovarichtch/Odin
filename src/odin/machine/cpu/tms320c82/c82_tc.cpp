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

#include "odin/machine/cpu/tms320c82/c82_tc.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "odin/common/repeats.hpp"
#include "odin/machine/cpu/tms320c82/tms320c82.hpp"

namespace odin {

namespace {

/* The linked list start addresses of the MP and of PP n (SPRU261 fig.
 * 4-1, 4-2) */
constexpr uint32_t MP_LIST = 0x010100FC, PP_LIST = 0x010000FC;
/* MP PKTREQ: F high priority, I urgent (SPRU109 § 3.2) */
constexpr uint32_t PKTREQ_F = 8, PKTREQ_I = 16;
/* The linked list start address of XPT n: 0x010100FC - 4n (fig. 7-1) */
constexpr uint32_t XPT_LISTS = 0x010100FC;
/* CONFIG X, XPTs allowed (§ 7.2); FLTSTS XPT, bits 8-5 (§ 1.3.4) */
constexpr uint32_t CONFIG_X = 1u << 27, FLTSTS_XPT = 15u << 5;

/* Long-form PT Options (SPRU261 fig. 6-17) */
enum : uint32_t {
	/* Short form (fig. 5-3): count in the 16 LSBs; src update, dst update */
	COUNT = 0xFFFFu, SU = 1u << 22, DU_SHORT = 1u << 25,
	DUM = 3u << 0, DTM = 7u << 4, SUM = 3u << 8, STM = 7u << 12, PAM = 7u << 16, X = 1u << 19, SF = 1u << 20,
	RSB = 1u << 21, RSC = 1u << 22, RA = 1u << 23, RDB = 1u << 24, RDC = 1u << 25, I = 1u << 28,
	PTS = 3u << 29, S = 1u << 31
};

/* The parameters of one side of a dimensioned transfer (§ 6.2, 6.5) */
struct Side {
	uint32_t start, a, b, c, b_pitch, c_pitch;
	bool reverse_b, reverse_c;

	uint64_t bytes() const { return static_cast<uint64_t>(a) * (b + 1ull) * (c + 1ull); }
	/* Where an update mode puts the start address (tables 6-2, 6-4) */
	uint32_t updated(uint32_t mode) const
	{
		const uint32_t last_patch = start + (reverse_c ? 0 - c * c_pitch : c * c_pitch);

		if (mode == 1) {
			const uint32_t last_line = last_patch + (reverse_b ? 0 - b * b_pitch : b * b_pitch);

			return last_line + (reverse_b ? 0 - b_pitch : b_pitch);
		}
		return last_patch + (reverse_c ? 0 - c_pitch : c_pitch);
	}
};

/* The addresses of a side's bytes in their order: within a line, then line
 * after line (b + 1 of them), then patch after patch; RA addresses the lines
 * backwards (§ 6.6.10). Counted, with no division for each byte */
class Walk {
public:
	Walk(const Side &side, bool reverse_a) : m_side(side), m_reverse_a(reverse_a), m_patch(side.start), m_line(side.start) {}

	uint32_t address() const { return m_line + (m_reverse_a ? 0 - m_in_line : m_in_line); }
	void next()
	{
		if (++m_in_line < m_side.a)
			return;
		m_in_line = 0;
		next_line();
	}
	/* The first byte of the line, and the next line (at its first byte) */
	uint32_t line() const { return m_line; }
	void next_line()
	{
		if (m_lines++ < m_side.b) {
			m_line += m_side.reverse_b ? 0 - m_side.b_pitch : m_side.b_pitch;
			return;
		}
		m_lines = 0;
		m_patch += m_side.reverse_c ? 0 - m_side.c_pitch : m_side.c_pitch;
		m_line = m_patch;
	}

private:
	const Side &m_side;
	bool m_reverse_a;
	uint32_t m_patch, m_line, m_in_line = 0, m_lines = 0;
};

/* One side as the table holds it: source at words 2, 4, 6, 8, 10 or
 * destination at 3, 5, 7, 9, 11 (fig. 6-16) */
Side
side_of(const std::array<uint32_t, 16> &t, unsigned dst, uint32_t options)
{
	Side s;

	s.start = t[2 + dst];
	s.a = t[4 + dst] & 0xFFFF;
	s.b = t[4 + dst] >> 16;
	s.c = t[6 + dst];
	s.b_pitch = t[8 + dst];
	s.c_pitch = t[10 + dst];
	s.reverse_b = options & (dst ? RDB : RSB);
	s.reverse_c = options & (dst ? RDC : RSC);
	return s;
}

/* The column accesses of a peripheral device transfer: one for each 64-bit
 * doubleword the memory side's addresses pass through, in their order;
 * all eight CAS/DQM in a read, those of the bytes written in a write
 * (§ 8.3.4, 10.5.5, 14.1.2, 14.1.4) */
class Columns {
public:
	Columns(Tms320c82 &chip, bool write) : m_chip(chip), m_write(write) {}

	void pass(uint32_t addr)
	{
		if ((addr & ~7u) != m_at) {
			flush();
			m_at = addr & ~7u;
		}
		m_strobes = static_cast<uint8_t>(m_strobes | 1u << (addr & 7));
	}
	/* pass() for each of `bytes` bytes from `addr` up, or down with
	 * `reverse`: a doubleword at a time */
	void pass_run(uint32_t addr, uint32_t bytes, bool reverse)
	{
		while (bytes) {
			const uint32_t in = addr & 7, take = std::min(bytes, reverse ? in + 1 : 8 - in);

			if ((addr & ~7u) != m_at) {
				flush();
				m_at = addr & ~7u;
			}
			m_strobes = static_cast<uint8_t>(m_strobes | ((1u << take) - 1) << (reverse ? in + 1 - take : in));
			addr = reverse ? addr - take : addr + take;
			bytes -= take;
		}
	}
	void flush()
	{
		if (m_strobes)
			m_chip.tc_peripheral(m_at, m_write, m_write ? m_strobes : 0xFF);
		m_strobes = 0;
	}

private:
	Tms320c82 &m_chip;
	bool m_write;
	uint32_t m_at = 1;
	uint8_t m_strobes = 0;
};

/* The bytes from `addr` to the end of its 4 KB block of on-chip addresses
 * or its 1 MB page */
uint32_t
room(uint32_t addr)
{
	return addr >= 0x02000000 ? 0x100000 - (addr & 0xFFFFF) : 0x1000 - (addr & 0xFFF);
}

} /* namespace */

C82Tc::C82Tc(Tms320c82 &chip) : m_chip(chip) {}

void
C82Tc::reset()
{
	m_table.fill(0);
	m_done_at = ~0ull;
	m_entry = 0;
	m_queued.fill(false);
	m_stuck.fill(false);
	m_active = NOBODY;
	m_last = REQUESTERS - 1;
	m_short = false;
	m_peripheral = NORMAL;
	/* The pins are the board's: they stay as it holds them */
	m_xpt_waiting = m_xpt = 0;
	m_xpt_acked = false;
	m_halted = false;
}

void
C82Tc::not_yet(unsigned who, const char *what, uint32_t entry)
{
	static const char *const NAMES[REQUESTERS] = { "MP", "PP0", "PP1" };

	if (who == XPT)
		repeats::print(stderr, "tc: XPT%u's %s at 0x%08X: not emulated yet; the XPT ends there\n", m_xpt, what,
		               entry);
	else
		repeats::print(stderr,
		               "tc: %s's %s at 0x%08X: not emulated yet; the request stays queued\n",
		               NAMES[who], what, entry);
}

/* What is not emulated yet: a processor's request stays queued without
 * holding the TC for the others; an XPT, which never waits (§ 7.5), ends
 * there. HEURISTIC both. False, for start() */
bool
C82Tc::stall(unsigned who, const char *what, uint32_t entry, uint64_t now)
{
	not_yet(who, what, entry);
	if (who == XPT)
		xpt_over(now);
	else
		m_stuck[who] = true;
	return false;
}

/* A peripheral device transfer (§ 8.3): the device reads the memory when
 * the dst start address is 0 and writes it when the src start address is
 * 0; the memory side must be off chip (§ 8.3.1, 8.3.2, 9.4). False when the
 * request ended in an error or waits */
bool
C82Tc::peripheral(unsigned who, uint32_t entry, uint32_t from, uint32_t to, uint64_t now)
{
	/* HEURISTIC: § 8.3 does not say what neither address at 0 does */
	if (to != 0 && from != 0)
		return stall(who, "peripheral device packet transfer with no address at 0", entry, now);
	if ((to == 0 ? from : to) < 0x02000000) {
		repeats::print(stderr, "tc: peripheral device packet transfer at 0x%08X: on-chip memory side\n", entry);
		stop_list(who, true, now);
		return false;
	}
	m_peripheral = to == 0 ? DEVICE_READS : DEVICE_WRITES;
	return true;
}

uint32_t
C82Tc::list_address(unsigned who) const
{
	if (who == XPT)
		return XPT_LISTS - 4u * m_xpt;
	return who == MP ? MP_LIST : PP_LIST + ((who - PP0) << 12);
}

/* Urgent MP requests, then high, then low priority (§ 2.1): the MP's
 * PKTREQ I and F, a PP's comm H */
unsigned
C82Tc::level(unsigned who) const
{
	if (who == MP) {
		const uint32_t pktreq = m_chip.mp().control(C82Mp::PKTREQ);

		return (pktreq & PKTREQ_I) ? 2 : (pktreq & PKTREQ_F) ? 1 : 0;
	}
	return m_chip.pp(who - PP0).high_priority() ? 1 : 0;
}

/* Q is set and P cleared on the next clock (§ 4.3.2) */
bool
C82Tc::request(unsigned who, uint64_t now)
{
	if (m_queued[who])
		return false;
	m_queued[who] = true;
	m_stuck[who] = false;
	if (m_active == NOBODY)
		arbitrate(now + 1);
	return true;
}

void
C82Tc::drop(unsigned who)
{
	m_queued[who] = false;
	m_stuck[who] = false;
	if (m_active == who) {
		m_active = NOBODY;
		m_done_at = ~0ull;
	}
	if (m_halted && m_halted_who == who)
		m_halted = false;
}

void
C82Tc::set_xpt(unsigned number, uint64_t now)
{
	m_pins = static_cast<uint8_t>(number & 15);
	sample_xpt(now);
}

/* The TC takes the first code it synchronizes, with X set, no XPT fault in
 * FLTSTS, none waiting, and none being served whose access code has not
 * gone out yet (§ 7.2, 7.4.3, 7.5.2). HEURISTIC: synchronized at once
 * (SPRS048 p. 139: over several cycles, how many is not said) */
void
C82Tc::sample_xpt(uint64_t now)
{
	if (!m_pins || m_xpt_waiting || (m_xpt && !m_xpt_acked) ||
	    !(m_chip.mp().control(C82Mp::CONFIG) & CONFIG_X) || (m_chip.fltsts() & FLTSTS_XPT))
		return;
	m_xpt_waiting = m_pins;
	if (!m_xpt)
		serve_xpt(now);
}

/* An XPT goes before every processor's request (fig. 2-1). A short-form
 * one goes at once: the entry in progress is halted, its registers kept
 * loaded, and goes on after it (§ 7.4.2, 1.2.7; SPRS048 p. 57). A
 * long-form one suspends it once PTMIN has passed (§ 7.4, 7.4.1).
 * HEURISTIC: suspension is not emulated, it waits for the end of that
 * entry. False when it waits */
bool
C82Tc::serve_xpt(uint64_t now)
{
	const uint32_t first = m_chip.internal32(XPT_LISTS - 4u * m_xpt_waiting);
	const bool short_form = m_chip.internal32(first + 4) & SF;

	if (m_active != NOBODY && !short_form)
		return false;
	if (m_active != NOBODY) {
		m_halted = true;
		m_halted_table = m_table;
		m_halted_left = m_done_at > now ? m_done_at - now : 0;
		m_halted_entry = m_entry;
		m_halted_who = m_active;
		m_halted_short = m_short;
		m_halted_peripheral = m_peripheral;
		m_active = NOBODY;
		m_done_at = ~0ull;
	}
	m_xpt = m_xpt_waiting;
	m_xpt_waiting = 0;
	m_xpt_acked = false;
	start(XPT, now);
	return true;
}

/* The XPT's list is over, at its end or by an error: the entry it halted
 * goes on, and the pins are sampled again */
void
C82Tc::xpt_over(uint64_t now)
{
	m_xpt = 0;
	m_xpt_acked = false;
	m_active = NOBODY;
	m_done_at = ~0ull;
	if (m_halted) {
		m_halted = false;
		m_table = m_halted_table;
		m_entry = m_halted_entry;
		m_active = m_halted_who;
		m_short = m_halted_short;
		m_peripheral = m_halted_peripheral;
		m_done_at = now + m_halted_left;
		m_chip.mp().end_run();
	}
	sample_xpt(now);
	arbitrate(now);
}

/* The XPT's access code goes out at the row time of its first access off
 * the chip (table 10-2; § 7.4.3): the device may let go of the pins, and
 * the TC samples them again. An XPT that stays on chip gives none (§
 * 7.4.3). HEURISTIC: at the start of the entry, the data moving at its
 * end */
void
C82Tc::acknowledge(uint32_t from, uint32_t to, uint64_t now)
{
	if (m_active != XPT || m_xpt_acked || (from < 0x02000000 && to < 0x02000000 && !m_peripheral))
		return;
	m_xpt_acked = true;
	m_chip.xpt_acknowledged(m_xpt);
	sample_xpt(now);
}

/* The highest level first, round robin among equals from the processor
 * after the last one served (§ 2.1, 2.2). HEURISTIC: CONFIG T (no round
 * robin) is not looked at */
void
C82Tc::arbitrate(uint64_t now)
{
	if (m_xpt_waiting && !m_xpt && serve_xpt(now))
		return;
	while (m_active == NOBODY) {
		int best = -1;
		unsigned best_level = 0;

		for (unsigned k = 1; k <= REQUESTERS; k++) {
			const unsigned who = (m_last + k) % REQUESTERS;

			if (!m_queued[who] || m_stuck[who])
				continue;

			const unsigned l = level(who);

			if (best < 0 || l > best_level) {
				best = static_cast<int>(who);
				best_level = l;
			}
		}
		if (best < 0 || start(static_cast<unsigned>(best), now))
			return;
	}
}

/* An error ends the list: Q cleared, bp (MP) or PTERR (PP) (§ 9.4), an
 * XPT's number in FLTSTS and mf (§ 7.5.2); the end of the last entry: Q
 * cleared, pc or PTEND (§ 4.3.2 step 6, § 5.4.7), nothing for an XPT (§
 * 7.5.1) */
void
C82Tc::stop_list(unsigned who, bool error, uint64_t now)
{
	if (who == XPT) {
		if (error)
			m_chip.fault(static_cast<uint32_t>(m_xpt) << 5);
		xpt_over(now);
		return;
	}
	/* A waiting PP sees its Q bit change after its last slice */
	if (who != MP)
		m_chip.pp(who - PP0).disturb();
	m_queued[who] = false;
	m_stuck[who] = false;
	if (who == MP) {
		m_chip.mp().post(error ? C82Mp::INT_BP : C82Mp::INT_PC);
		m_chip.mp().end_run();
	} else {
		m_chip.pp(who - PP0).list_ended(error);
	}
}

/* A fault in the data moved: FLTSTS M (MP), PPn (PP) or the XPT's
 * number, and mf (§ 1.3.4; SPRU109 tables 5-6, 5-7). A faulted XPT ends at
 * once and no other is taken until the MP clears its bits (§ 7.5.2); a
 * processor's request would wait until then: not emulated yet */
void
C82Tc::faulted(unsigned who, uint64_t now)
{
	if (who == XPT) {
		stop_list(XPT, true, now);
		return;
	}
	m_chip.fault(who == MP ? 1u : 1u << (16 + who - PP0));
	not_yet(who, "suspension of a faulted packet transfer", m_entry);
	m_stuck[who] = true;
	arbitrate(now);
}

/* An entry with I done: pc (MP) or PTEND (PP) (§ 5.4.7, § 6.6.13) */
void
C82Tc::entry_done(unsigned who)
{
	if (who == MP)
		m_chip.mp().post(C82Mp::INT_PC);
	else
		m_chip.pp(who - PP0).entry_done();
}

/* Reads the entry the list start address points at (§ 4.3.2); false when
 * the request ended or waits on what is not emulated */
bool
C82Tc::start(unsigned who, uint64_t now)
{
	const uint32_t entry = m_chip.internal32(list_address(who));

	m_entry = entry;
	/* The list must be on chip, 16-byte aligned (§ 9.4): an error before
	 * any transfer, nothing suspended */
	if ((entry & 15) || !m_chip.tc_internal_ok(entry) || !m_chip.tc_internal_ok(entry + 15)) {
		repeats::print(stderr, "tc: bad packet transfer list at 0x%08X\n", entry);
		stop_list(who, true, now);
		return false;
	}

	const uint32_t options = m_chip.internal32(entry + 4);

	if (options & SF) {
		/* Short form (ch. 5): next entry, options and count, source,
		 * destination (fig. 5-2) */
		m_table.fill(0);
		for (unsigned i = 0; i < 4; i++)
			m_table[i] = m_chip.internal32(entry + 4 * i);
		if (options & PTS) {
			return stall(who, "resumption of a suspended packet transfer", entry, now);
		}

		const uint32_t pam = (options & PAM) >> 16;

		/* Reserved access modes 1xx (table 5-1; § 9.4) */
		if (pam >= 4) {
			repeats::print(stderr, "tc: packet transfer at 0x%08X: reserved mode (options 0x%08X)\n", entry,
			               options);
			stop_list(who, true, now);
			return false;
		}
		if (pam > 1) {
			return stall(who, "VRAM read transfer", entry, now);
		}
		m_peripheral = NORMAL;
		if (pam == 1) {
			const bool exchange = options & X;

			if (!peripheral(who, entry, m_table[exchange ? 3 : 2], m_table[exchange ? 2 : 3], now))
				return false;
		}
		m_short = true;
		m_active = static_cast<uint8_t>(who);
		/* A peripheral transfer has only its memory side's cycles */
		m_done_at = now + 2 + (m_peripheral ? 1 : 2) * ((static_cast<uint64_t>(options & COUNT) + 7) / 8);
		m_chip.mp().end_run();
		acknowledge(m_table[(options & X) ? 3 : 2], m_table[(options & X) ? 2 : 3], now);
		return true;
	}
	if (!m_chip.tc_internal_ok(entry + 63)) {
		repeats::print(stderr, "tc: bad packet transfer list at 0x%08X\n", entry);
		stop_list(who, true, now);
		return false;
	}
	for (unsigned i = 0; i < m_table.size(); i++)
		m_table[i] = m_chip.internal32(entry + 4 * i);
	if (options & PTS) {
		return stall(who, "resumption of a suspended packet transfer", entry, now);
	}

	const uint32_t stm = (options & STM) >> 12, dtm = (options & DTM) >> 4, pam = (options & PAM) >> 16;

	/* Reserved modes are errors (tables 6-3, 6-5, 6-6); X swaps the modes
	 * too (table 6-8) */
	if (stm == 2 || (dtm >= 1 && dtm <= 3) || pam == 3) {
		repeats::print(stderr, "tc: packet transfer at 0x%08X: reserved mode (options 0x%08X)\n", entry, options);
		stop_list(who, true, now);
		return false;
	}
	if (stm || dtm) {
		return stall(who, "guided or fill packet transfer", entry, now);
	}
	if (pam > 1) {
		return stall(who, "special access mode (block write, transparency)", entry, now);
	}

	const Side src = side_of(m_table, 0, options), dst = side_of(m_table, 1, options);

	m_peripheral = NORMAL;
	if (pam == 1) {
		/* X: source and destination exchanged as they are loaded (§ 6.6.6) */
		const bool exchange = options & X;

		if (!peripheral(who, entry, (exchange ? dst : src).start, (exchange ? src : dst).start, now))
			return false;
	} else if (dst.bytes() > src.bytes()) {
		/* The destination may not take more bytes than the source gives
		 * (§ 9.4); the device's side of a peripheral transfer is Don't
		 * Care (fig. A-6, A-42) */
		repeats::print(stderr, "tc: packet transfer at 0x%08X: more destination than source bytes\n", entry);
		stop_list(who, true, now);
		return false;
	}
	m_short = false;
	m_active = static_cast<uint8_t>(who);
	if (m_peripheral) {
		const bool exchange = options & X;
		const Side &memory = (m_peripheral == DEVICE_READS) != exchange ? src : dst;

		m_done_at = now + 8 + (memory.bytes() + 7) / 8;
	} else {
		m_done_at = now + 8 + (src.bytes() + 7) / 8 + (dst.bytes() + 7) / 8;
	}
	m_chip.mp().end_run();
	acknowledge(((options & X) ? dst : src).start, ((options & X) ? src : dst).start, now);
	return true;
}

void
C82Tc::advance(uint64_t now)
{
	while (now >= m_done_at)
		end();
}

/* The data moved, the updates written back, the list advanced (§ 5.3.1,
 * 5.4.4-5.4.6, 6.5.1, 6.6.1, 6.6.3, 9.3) */
void
C82Tc::end()
{
	const unsigned who = m_active;
	const uint32_t options = m_table[1];
	const uint64_t done = m_done_at;
	std::array<uint32_t, 16> table = m_table;

	m_done_at = ~0ull;
	m_active = NOBODY;
	if (who != XPT)
		m_last = static_cast<uint8_t>(who);
	if (m_short) {
		/* Contiguous bytes, backwards with RA (fig. 5-5); X exchanges the
		 * addresses, the updates staying with their table words (§ 5.4.2) */
		const bool exchange = options & X, reverse = options & RA;
		const uint32_t count = options & COUNT;
		const uint32_t from = exchange ? m_table[3] : m_table[2], to = exchange ? m_table[2] : m_table[3];
		Columns columns(m_chip, m_peripheral == DEVICE_WRITES);

		if (m_peripheral)
			columns.pass_run(m_peripheral == DEVICE_READS ? from : to, count, reverse);
		for (uint32_t n = 0; !m_peripheral && n < count;) {
			/* A run of plain memory on both sides, the two apart: what the
			 * bytes one by one would do, at once */
			const uint32_t run = reverse ? 1 : std::min({ count - n, room(from + n), room(to + n) });
			const uint8_t *const src = reverse ? nullptr : m_chip.tc_plain_read(from + n, run);
			uint8_t *const dst = src ? m_chip.tc_plain_write(to + n, run) : nullptr;
			const uintptr_t s = reinterpret_cast<uintptr_t>(src), d = reinterpret_cast<uintptr_t>(dst);

			if (dst && (d + run <= s || s + run <= d)) {
				std::memcpy(dst, src, run);
				n += run;
				continue;
			}
			for (const uint32_t end = n + run; n < end; n++) {
				uint64_t value = 0;
				const uint32_t step = reverse ? 0 - n : n;

				if (!m_chip.tc_read(from + step, 0, value) || !m_chip.tc_write(to + step, 0, value)) {
					faulted(who, done);
					return;
				}
			}
		}
		columns.flush();
		if (options & SU)
			table[2] = reverse ? m_table[2] - count : m_table[2] + count;
		if (options & DU_SHORT)
			table[3] = reverse ? m_table[3] - count : m_table[3] + count;
		for (unsigned i = 2; i < 4; i++)
			if (table[i] != m_table[i])
				m_chip.set_internal32(m_entry + 4 * i, table[i]);
	} else {
		/* X: source and destination exchanged as they are loaded (§ 6.6.6) */
		const bool exchange = options & X;
		const Side src = side_of(m_table, exchange ? 1 : 0, options), dst = side_of(m_table, exchange ? 0 : 1, options);
		const bool reverse_a = options & RA;
		const uint64_t bytes = std::min(src.bytes(), dst.bytes());

		if (m_peripheral) {
			const Side &memory = m_peripheral == DEVICE_READS ? src : dst;
			Columns columns(m_chip, m_peripheral == DEVICE_WRITES);

			Walk at(memory, reverse_a);

			/* Line after line, as the bytes one by one */
			for (uint64_t n = 0; memory.a && n < memory.bytes(); n += memory.a, at.next_line())
				columns.pass_run(at.line(), memory.a, reverse_a);
			columns.flush();
		}
		Walk from_at(src, reverse_a), to_at(dst, reverse_a);

		for (uint64_t n = 0; !m_peripheral && n < bytes; n++, from_at.next(), to_at.next()) {
			uint64_t value = 0;
			const uint32_t from = from_at.address(), to = to_at.address();

			if (!m_chip.tc_read(from, 0, value) || !m_chip.tc_write(to, 0, value)) {
				faulted(who, done);
				return;
			}
		}

		const uint32_t sum = (options & SUM) >> 8, dum = options & DUM;

		if (sum) {
			table[2] = side_of(m_table, 0, options).updated(sum);
			if (sum == 3)
				table[1] ^= RSC;
		}
		if (dum) {
			table[3] = side_of(m_table, 1, options).updated(dum);
			if (dum == 3)
				table[1] ^= RDC;
		}
		for (unsigned i = 1; i < 4; i++)
			if (table[i] != m_table[i])
				m_chip.set_internal32(m_entry + 4 * i, table[i]);
	}
	m_chip.set_internal32(list_address(who), m_table[0]);

	if (who == XPT) {
		/* An XPT's list runs to its end; its I bit, like S, only
		 * gives the device the End of XPT status (§ 7.5, 7.5.1),
		 * which nothing on this board was seen to need */
		if (options & S)
			xpt_over(done);
		else
			start(XPT, done);
		return;
	}
	if (options & S)
		stop_list(who, false, done);
	else if (options & I)
		entry_done(who);
	arbitrate(done);
}

void
C82Tc::save(C82TcState &out) const
{
	out.table = m_table;
	out.done_at = m_done_at;
	out.entry = m_entry;
	for (unsigned k = 0; k < REQUESTERS; k++) {
		out.queued[k] = m_queued[k];
		out.stuck[k] = m_stuck[k];
	}
	out.active = m_active;
	out.last = m_last;
	out.short_form = m_short;
	out.peripheral = m_peripheral;
	out.halted_table = m_halted_table;
	out.halted_left = m_halted_left;
	out.halted_entry = m_halted_entry;
	out.pins = m_pins;
	out.xpt_waiting = m_xpt_waiting;
	out.xpt = m_xpt;
	out.xpt_acked = m_xpt_acked;
	out.halted = m_halted;
	out.halted_who = m_halted_who;
	out.halted_short = m_halted_short;
	out.halted_peripheral = m_halted_peripheral;
}

void
C82Tc::load(const C82TcState &in)
{
	m_table = in.table;
	m_done_at = in.done_at;
	m_entry = in.entry;
	for (unsigned k = 0; k < REQUESTERS; k++) {
		m_queued[k] = in.queued[k] != 0;
		m_stuck[k] = in.stuck[k] != 0;
	}
	m_active = in.active;
	m_last = in.last;
	m_short = in.short_form != 0;
	m_peripheral = in.peripheral;
	m_halted_table = in.halted_table;
	m_halted_left = in.halted_left;
	m_halted_entry = in.halted_entry;
	m_pins = in.pins;
	m_xpt_waiting = in.xpt_waiting;
	m_xpt = in.xpt;
	m_xpt_acked = in.xpt_acked != 0;
	m_halted = in.halted != 0;
	m_halted_who = in.halted_who;
	m_halted_short = in.halted_short != 0;
	m_halted_peripheral = in.halted_peripheral;
}

} /* namespace odin */
