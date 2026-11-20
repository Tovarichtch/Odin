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

#include "odin/machine/cpu/tms320c82/tms320c82.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdio>

#include "odin/common/repeats.hpp"

namespace odin {

namespace {

/* TC registers (SPRU261 table 1-1) */
enum : uint32_t { REFCNTL = 0x000, PTMIN = 0x004, PTMAX = 0x008, FLTSTS = 0x00C };
/* FLTSTS XPT, bits 8-5 (§ 1.3.4) */
constexpr uint32_t XPT_FAULT = 15u << 5;

/* HEURISTIC: the MP, the PPs and the TC take turns by slices of this many
 * cycles of their common clock */
constexpr uint64_t SLICE = 16;

} /* namespace */

Tms320c82::Tms320c82(C82Bus &bus)
	: m_bus(bus), m_mp(*this), m_tc(*this), m_pp{ C82Pp(*this, 0), C82Pp(*this, 1) }
{
	const auto block = [this](uint32_t base, int16_t value) { m_internal[base >> 12] = value; };

	/* SPRS048 fig. 3 */
	m_internal.fill(NONE);
	block(0x00000000, PP0_DATA0 * 4096);
	block(0x00001000, PP1_DATA0 * 4096);
	block(0x00008000, PP0_DATA1 * 4096);
	block(0x00009000, PP1_DATA1 * 4096);
	block(0x01000000, PP0_PARAM * 4096);
	block(0x01001000, PP1_PARAM * 4096);
	block(0x01010000, MP_PARAM * 4096);
	block(0x01810000, MP_DCACHE * 4096);
	block(0x01801000, CACHE);
	block(0x01803000, CACHE);
	block(0x01818000, CACHE);
	block(0x01820000, TC_REGISTERS);
	reset();
}

/* After a power-up reset (SPRU109 § 9.5; SPRU261 § 1.3). HEURISTIC: the
 * on-chip RAM starts at 0 */
void
Tms320c82::reset()
{
	board_reached();
	m_ram.fill(0);
	m_refcntl = 0xFFFF0020u;
	m_ptmin = m_ptmax = 0x10000;
	m_fltsts = 0;
	m_eint.fill(false);
	m_mp.reset();
	m_tc.reset();
	/* All PPs halted, their reset sequence armed (SPRU110 § 9.6) */
	for (C82Pp &pp : m_pp)
		pp.power_up();
}

void
Tms320c82::run_rounded(uint64_t until)
{
	while (m_mp.cycles() < until) {
		/* The MP parked, slices made together when they can */
		if (m_mp.m_park.on && m_long && run_long(until))
			continue;
		m_mp.run(std::min({ until, m_mp.cycles() + SLICE, m_tc.next_event() }));

		const uint64_t now = m_mp.cycles();

		/* Each PP's run in a place of its own (its jump into its
		 * host code predicted apart); the TC only when an entry ends */
		m_pp[0].run(now);
		m_pp[1].run(now);
		if (now >= m_tc.next_event())
			m_tc.advance(now);
	}
	board_reached();
}

/* Slices with the MP parked through them, made as the slices one by one
 * (the same results, checked bit for bit). Planned from its pass (the MP's run()
 * for each), until `until`, the TC's next event, the MP's timer (it would
 * wake), a parked PP's first read of the board due past its promise
 * (C82Bus::answers_until, m_park.quiet), LONG slices. The PP that is not
 * quiet (one at most) goes on alone through them (8, 16, 32... planned at a
 * time): its blocks in host code, stopping before any step another unit
 * could see or the interpreter's. The others then as in run(), in the
 * slices' order, but only where something is made: a parked PP's reads of
 * the board (each at its slice's end, the MP's cycles then); the slice the
 * alone PP stopped in made whole from the end of the one before (it goes on
 * there as in run(), the run ends with it); else the end of the last. False
 * when two PPs are not quiet, the one is not where blocks resume or measures
 * a pass, or one slice at most is planned */
bool
Tms320c82::run_long(uint64_t until)
{
	const auto quiet = [](const C82Pp &pp) {
		return pp.m_park.on || ((pp.m_halted || pp.m_illegal || pp.m_faulted) && !pp.m_in_reset);
	};
	C82Pp *alone = nullptr;

	for (C82Pp &pp : m_pp) {
		if (quiet(pp))
			continue;
		if (alone || !pp.m_resume || pp.m_park.measuring)
			return false;
		alone = &pp;
	}
	/* MEASURED: all parked, planned slices cost more than one by one;
	 * they are made one by one without the calls (run_calm) */
	if (!alone)
		return run_calm(until);

	const uint64_t start = m_mp.m_cycles, event = m_tc.next_event(), zero = m_mp.m_timer_zero;
	uint64_t hold = ~0ull;

	/* A parked PP's reads due before its promise (m_park.quiet,
	 * the board reached by nothing else) answer as before: counted; past
	 * it, or with none, the first read due ends the plan before its slice */
	for (const C82Pp &pp : m_pp)
		if (pp.m_park.on && pp.m_park.boards)
			hold = std::min(hold, std::max(pp.m_park.quiet, pp.m_park.next_read + 1));

	uint64_t end = start, pass = m_mp.m_park.pass;
	unsigned n = 0;
	bool bound = false;
	const auto plan = [&](unsigned upto) {
		while (n < upto && !bound) {
			const uint64_t aim = std::min({ until, end + SLICE, event });

			if (aim > zero)
				break;
			if (aim > end)
				end = m_mp.parked_end_from(pass, aim);
			/* Each read of this slice is made at its end */
			if (end >= hold)
				break;
			m_plan[n++] = { end, pass };
			bound = end >= until || end >= event;
		}
		bound |= n < upto || n == LONG;
	};

	plan(alone ? 8 : LONG);
	if (n < 2)
		return false;
	if (alone) {
		alone->m_alone = m_alone = true;
		for (unsigned upto = 16;; upto = std::min(2 * upto, LONG)) {
			alone->run_alone(m_plan[n - 1].end);
			if (bound || alone->m_cycles < m_plan[n - 1].end)
				break;
			plan(upto);
		}
		alone->m_alone = m_alone = false;
	}

	/* The slice the alone PP stopped in (n: none) */
	const uint64_t ahead = alone ? alone->m_cycles : ~0ull;
	unsigned here = 0;

	while (here < n && ahead >= m_plan[here].end)
		here++;

	const bool quiet0 = quiet(m_pp[0]), quiet1 = quiet(m_pp[1]);
	const auto due = [this, alone]() {
		uint64_t at = ~0ull;

		for (const C82Pp &pp : m_pp)
			if (&pp != alone && pp.m_park.on && pp.m_park.boards)
				at = std::min(at, pp.m_park.next_read);
		return at;
	};
	/* The others as run() makes them, alone left out, to `k`'s end */
	const auto others = [this, alone](unsigned k) {
		m_mp.m_cycles = m_plan[k].end;
		m_mp.m_park.pass = m_plan[k].pass;
		for (C82Pp &pp : m_pp)
			if (&pp != alone)
				pp.run(m_plan[k].end);
	};

	/* Each slice with a read due, before `here` and the last */
	for (unsigned k = 0; k < here && k + 1 < n; k++)
		if (m_plan[k].end > due())
			others(k);
	if (here < n) {
		if (here > 0)
			others(here - 1);
		m_mp.m_cycles = m_plan[here].end;
		m_mp.m_park.pass = m_plan[here].pass;
		m_pp[0].run(m_plan[here].end);
		m_pp[1].run(m_plan[here].end);
		if (m_plan[here].end >= m_tc.next_event())
			m_tc.advance(m_plan[here].end);
		return true;
	}
	others(n - 1);
	/* For the tests: none woke in the run (the TC's end may wake) */
	if ((quiet0 && !quiet(m_pp[0])) || (quiet1 && !quiet(m_pp[1])))
		m_breaches++;
	if (m_plan[n - 1].end >= m_tc.next_event())
		m_tc.advance(m_plan[n - 1].end);
	return true;
}

/* The MP and both PPs parked: the slices as run() makes them, without
 * its calls: the MP's run() is, parked and not reaching its timer's zero,
 * its cycles set to parked_end(); each PP's run() (the same) only notes
 * `until` or counts its reads; the TC when an entry ends, and then
 * no more. It stops before a slice in which the MP would wake (its timer)
 * or a PP would read the board (C82Pp::calm). False when none is made */
bool
Tms320c82::run_calm(uint64_t until)
{
	C82Pp &p0 = m_pp[0], &p1 = m_pp[1];

	if (!p0.m_park.on || !p1.m_park.on)
		return false;

	const uint64_t event = m_tc.next_event(), zero = m_mp.m_timer_zero;
	uint64_t end = m_mp.m_cycles, pass = m_mp.m_park.pass;
	bool made = false;

	/* While a slice is SLICE cycles long (its end before `until`, the TC's
	 * event and the timer's zero), its end and the MP's pass by the offset
	 * of where the MP stands in its pass: parked_end_from(.., u + period) =
	 * parked_end_from(.., u) + period (it finds the pass of `u`, from base,
	 * then the first start of a step from there), so a table of `period`
	 * slices from base tells them all */
	const uint64_t last = std::min({ until, event, zero });
	const C82Mp::Park &park = m_mp.m_park;

	if (park.period <= CALM && end + SLICE <= last) {
		const uint64_t period = park.period;

		for (uint64_t o = 0, cursor = park.base; o < period; o++) {
			const uint64_t from = park.base + o, to = m_mp.parked_end_from(cursor, from + SLICE);

			m_calm[o] = { to - from, cursor - from, static_cast<uint32_t>((o + to - from) % period) };
		}

		const auto reads = [](const C82Pp &pp) { return pp.m_park.boards ? pp.m_park.next_read : ~0ull; };
		uint64_t due = std::min(reads(p0), reads(p1));
		uint32_t o = static_cast<uint32_t>((end - park.base) % period);

		for (;;) {
			const Calm &t = m_calm[o];
			const uint64_t to = end + t.delta;

			/* A PP's reads due in the slice: counted (run()), or the
			 * slice left to the loop below (it stops before it) */
			if (to > due) {
				if (!p0.calm(to) || !p1.calm(to))
					break;
				p0.run(to);
				p1.run(to);
				due = std::min(reads(p0), reads(p1));
			}
			pass = end + t.pass;
			end = to;
			o = t.next;
			made = true;
			if (end >= event || end + SLICE > last)
				break;
		}
		if (made) {
			m_mp.m_cycles = end;
			m_mp.m_park.pass = pass;
			p0.run(end);
			p1.run(end);
			if (end >= event) {
				m_tc.advance(end);
				return true;
			}
		}
	}
	while (end < until) {
		const uint64_t aim = std::min({ until, end + SLICE, event });

		if (aim > zero)
			break;
		if (aim > end)
			end = m_mp.parked_end_from(pass, aim);
		if (!p0.calm(end) || !p1.calm(end))
			break;
		m_mp.m_cycles = end;
		m_mp.m_park.pass = pass;
		p0.run(end);
		p1.run(end);
		made = true;
		if (end >= event) {
			m_tc.advance(end);
			break;
		}
	}
	return made;
}

void
Tms320c82::set_eint(unsigned n, bool high)
{
	static constexpr uint32_t BITS[3] = { C82Mp::INT_X1, C82Mp::INT_X2, C82Mp::INT_X3 };

	if (n < 1 || n > 3)
		return;
	if (high && !m_eint[n - 1])
		m_mp.post(BITS[n - 1]);
	m_eint[n - 1] = high;
}

Tms320c82::MpMap
Tms320c82::mp_map() const
{
	const auto from_pages = [this](const void *at) {
		return static_cast<int32_t>(static_cast<const uint8_t *>(at) - reinterpret_cast<const uint8_t *>(m_pages.data()));
	};

	return { static_cast<int32_t>(sizeof(Page)), static_cast<int32_t>(offsetof(Page, writable)),
		 static_cast<int32_t>(offsetof(Page, watched)), from_pages(m_ram.data()), from_pages(m_internal.data()),
		 from_pages(m_watched_lines.data()), static_cast<int32_t>(offsetof(Page, code)) };
}

void
Tms320c82::map_external(uint32_t base, uint32_t size, uint8_t *bytes, bool writable)
{
	/* The MP's blocks read their code from these pages */
	m_mp.flush_blocks();
	for (uint32_t page = 0; page < size >> 20; page++)
		m_pages[(base >> 20) + page] = Page{ bytes + (static_cast<size_t>(page) << 20), writable };
}

uint64_t
Tms320c82::external_read(uint32_t addr, unsigned size)
{
	const Page &page = m_pages[addr >> 20];

	if (page.bytes)
		return c82_get(page.bytes + (addr & 0xFFFFF), size);
	board_reached();
	switch (size) {
	case 0:
		return m_bus.read8(addr);
	case 1:
		return m_bus.read16(addr);
	case 2:
		return m_bus.read32(addr);
	default:
		return m_bus.read64(addr);
	}
}

void
Tms320c82::external_write(uint32_t addr, unsigned size, uint64_t value)
{
	const Page &page = m_pages[addr >> 20];

	if (page.bytes) {
		if (page.writable) {
			if (page.code)
				code_written(addr, 1u << size);
			if (page.watched && external_watched(addr, 1u << size))
				written(addr, 1u << size);
			c82_put(page.bytes + (addr & 0xFFFFF), size, value);
		}
		return;
	}
	board_reached();
	switch (size) {
	case 0:
		m_bus.write8(addr, static_cast<uint8_t>(value));
		break;
	case 1:
		m_bus.write16(addr, static_cast<uint16_t>(value));
		break;
	case 2:
		m_bus.write32(addr, static_cast<uint32_t>(value));
		break;
	default:
		m_bus.write64(addr, value);
		break;
	}
}

bool
Tms320c82::mp_fetch(uint32_t addr, uint64_t &word)
{
	if (addr >= 0x02000000) {
		word = external_read(addr, 2);
		return true;
	}

	const int32_t offset = internal_offset(addr);

	if (offset < 0 || offset == MP_PARAM * 4096 || offset == MP_DCACHE * 4096)
		return false;
	word = c82_get(m_ram.data() + offset + (addr & 0xFFC), 2);
	return true;
}

/* TC registers: 32 bits wide, no 64-bit access (§ 5.1.2) */
bool
Tms320c82::mp_load_other(uint32_t addr, unsigned size, uint64_t &value)
{
	const int32_t offset = internal_offset(addr);

	if (offset != TC_REGISTERS || (addr & 0xFFF) >= 0x200 || size == 3)
		return false;

	const uint64_t word = read_tc_register(addr & ~3u);

	/* HEURISTIC: a narrower read takes its bytes of the word */
	value = size == 2 ? word : (word >> (8 * (4 - (1u << size) - (addr & 3)))) & ((1ull << (8 << size)) - 1);
	return true;
}

/* TC registers take 32-bit stores only (§ 5.1.2): HEURISTIC, others are
 * left out */
bool
Tms320c82::mp_store_other(uint32_t addr, unsigned size, uint64_t value)
{
	const int32_t offset = internal_offset(addr);

	if (offset != TC_REGISTERS || (addr & 0xFFF) >= 0x200 || size == 3)
		return false;
	if (size != 2) {
		repeats::print(stderr, "c82: a %u-byte store to the TC register 0x%08X is left out\n", 1u << size, addr);
		return true;
	}
	write_tc_register(addr, static_cast<uint32_t>(value));
	return true;
}

/* The TC reaches the data and parameter RAMs (tables 5-6, 5-7); the
 * caches and registers fault */
bool
Tms320c82::tc_internal_ok(uint32_t addr) const
{
	if (addr >= 0x02000000)
		return false;

	const int32_t offset = internal_offset(addr);

	return offset >= 0 && offset != MP_DCACHE * 4096;
}

bool
Tms320c82::tc_read(uint32_t addr, unsigned size, uint64_t &value)
{
	if (addr >= 0x02000000) {
		value = external_read(addr, size);
		return true;
	}
	if (!tc_internal_ok(addr))
		return false;
	value = c82_get(m_ram.data() + internal_offset(addr) + (addr & 0xFFF), size);
	return true;
}

bool
Tms320c82::tc_write(uint32_t addr, unsigned size, uint64_t value)
{
	if (addr >= 0x02000000) {
		external_write(addr, size, value);
		return true;
	}
	if (!tc_internal_ok(addr))
		return false;
	written_ram(internal_offset(addr), addr, 1u << size);
	c82_put(m_ram.data() + internal_offset(addr) + (addr & 0xFFF), size, value);
	return true;
}

/* A PP fetches from external memory and the PPs' RAMs (table 5-5) */
bool
Tms320c82::pp_fetch(uint32_t addr, uint64_t &word)
{
	if (addr >= 0x02000000) {
		word = external_read(addr, 3);
		return true;
	}

	const int32_t offset = internal_offset(addr);

	if (offset < 0 || offset >= MP_PARAM * 4096)
		return false;
	word = c82_get(m_ram.data() + offset + (addr & 0xFF8), 3);
	return true;
}

/* The shared RAMs, the MP parameter RAM on the C82, external memory by
 * DEA; the caches, the registers and the rest fault (SPRU109 § 3.3.4.1) */
bool
Tms320c82::pp_load(uint32_t addr, unsigned size, uint64_t &value)
{
	if (addr >= 0x02000000) {
		value = external_read(addr, size);
		return true;
	}

	const int32_t offset = internal_offset(addr);

	if (offset < 0 || offset == MP_DCACHE * 4096)
		return false;
	value = c82_get(m_ram.data() + offset + (addr & 0xFFF), size);
	return true;
}

bool
Tms320c82::pp_store(uint32_t addr, unsigned size, uint64_t value)
{
	if (addr >= 0x02000000) {
		external_write(addr, size, value);
		return true;
	}

	const int32_t offset = internal_offset(addr);

	if (offset < 0 || offset == MP_DCACHE * 4096)
		return false;
	written_ram(offset, addr, 1u << size);
	c82_put(m_ram.data() + offset + (addr & 0xFFF), size, value);
	return true;
}

uint32_t
Tms320c82::internal32(uint32_t addr) const
{
	const int32_t offset = addr < 0x02000000 ? internal_offset(addr) : NONE;

	return offset >= 0 ? static_cast<uint32_t>(c82_get(m_ram.data() + offset + (addr & 0xFFC), 2)) : 0;
}

void
Tms320c82::set_internal32(uint32_t addr, uint32_t value)
{
	const int32_t offset = addr < 0x02000000 ? internal_offset(addr) : NONE;

	if (offset >= 0) {
		written_ram(offset, addr & ~3u, 4);
		c82_put(m_ram.data() + offset + (addr & 0xFFC), 2, value);
	}
}

uint64_t
Tms320c82::read_tc_register(uint32_t addr)
{
	visible();
	switch (addr & 0xFFF) {
	case REFCNTL:
		return m_refcntl;
	case PTMIN:
		return m_ptmin;
	case PTMAX:
		return m_ptmax;
	case FLTSTS:
		return m_fltsts;
	default:
		/* HEURISTIC: the reserved registers read 0 */
		return 0;
	}
}

void
Tms320c82::write_tc_register(uint32_t addr, uint32_t value)
{
	visible();
	switch (addr & 0xFFF) {
	case REFCNTL:
		/* Refresh: not emulated (the memories keep their data) */
		m_refcntl = value;
		break;
	case PTMIN:
		m_ptmin = value & 0xFFFFFF;
		break;
	case PTMAX:
		m_ptmax = value & 0xFFFFFF;
		break;
	case FLTSTS:
		/* A 1 clears its bit (§ 1.3.4); the faulted request would go on:
		 * not emulated yet. Cleared XPT bits let the TC
		 * sample the /XPT pins again (§ 1.3.4, 7.5.2) */
		if (m_fltsts & value & ~XPT_FAULT)
			repeats::print(stderr, "tc: resuming a faulted request: not emulated yet\n");
		m_fltsts &= ~value;
		if (value & XPT_FAULT)
			m_tc.sample_xpt(m_mp.cycles());
		break;
	default:
		break;
	}
}

void
Tms320c82::fault(uint32_t bits)
{
	visible();
	m_fltsts |= bits;
	m_mp.post(C82Mp::INT_MF);
}

/* cmnd (SPRU109 p. 10-70): bits 0-1 PP0-PP1 (PP2, PP3 do not exist on the
 * C82), 8 MP, 9 TC, 10 VC (none on the C82); 31 reset, 30 halt, 29
 * unhalt, 28 ICR, 27 DCR, 14 task, 13 message (SPRU110 fig. 3-3) */
void
Tms320c82::command(uint32_t word)
{
	visible();
	const uint64_t now = m_mp.cycles();

	for (unsigned pp = 0; pp < 2; pp++)
		if ((word >> pp) & 1)
			m_pp[pp].command(word, now);
	if ((word >> 31) && ((word >> 9) & 1)) {
		settle();
		/* A TC reset resets the PPs too (SPRU110 § 9.6) */
		repeats::print(stderr, "c82: TC reset: its requests are dropped\n");
		m_tc.reset();
		for (C82Pp &pp : m_pp)
			pp.reset(now);
	}
}

/* PPERROR (SPRU109 fig. 3-4; SPRS048 fig. 14): f bits 0-1, i bits 8-9, h
 * bits 16-17 */
uint32_t
Tms320c82::pperror()
{
	const uint64_t now = m_mp.cycles();
	uint32_t v = 0;

	for (unsigned pp = 0; pp < 2; pp++) {
		if (m_pp[pp].halt_latch(now))
			v |= 1u << (16 + pp);
		if (m_pp[pp].illegal())
			v |= 1u << (8 + pp);
		if (m_pp[pp].fault_type())
			v |= 1u << pp;
	}
	return v;
}

Tms320c82::Place
Tms320c82::place(uint32_t addr) const
{
	if (addr < 0x02000000)
		return internal_offset(addr) >= 0 ? PLACE_INTERNAL : PLACE_NONE;
	return m_pages[addr >> 20].bytes ? PLACE_MEMORY : PLACE_BOARD;
}

void
Tms320c82::watch_added(unsigned unit, uint32_t lo, uint32_t hi, bool internal)
{
	if (internal) {
		for (uint32_t a = lo & ~63u; a < hi; a += 64) {
			const int32_t offset = internal_offset(a);

			if (offset >= 0) {
				const uint64_t line = 1ull << ((a & 0xFFF) >> 6);

				m_unit_lines[unit][offset >> 12] |= line;
				m_watched_lines[offset >> 12] |= line;
			}
		}
		return;
	}
	/* At most 8 watches a processor: the list holds them all, once
	 * remade (the unit's list holds this range already) */
	if (m_external_count == m_external.size()) {
		external_again();
		return;
	}
	m_external[m_external_count++] = Range{ lo, hi };
	m_pages[lo >> 20].watched = true;
}

/* The unit's lines dropped, the union made again from the three; the
 * external ranges from the lists, as before */
void
Tms320c82::watch_changed(unsigned unit)
{
	m_unit_lines[unit].fill(0);
	for (unsigned b = 0; b < BLOCKS; b++)
		m_watched_lines[b] = m_unit_lines[0][b] | m_unit_lines[1][b] | m_unit_lines[2][b];
	external_again();
}

void
Tms320c82::external_again()
{
	for (unsigned k = 0; k < m_external_count; k++)
		m_pages[m_external[k].lo >> 20].watched = false;
	m_external_count = 0;

	/* At most 8 watches a processor (C82Pp, C82Mp: Park::WATCHES), 24 in
	 * all: m_external holds them */
	const auto add = [this](uint32_t lo, uint32_t hi, bool internal) {
		if (!internal) {
			m_external[m_external_count++] = Range{ lo, hi };
			m_pages[lo >> 20].watched = true;
		}
	};

	for (const C82Pp &pp : m_pp)
		for (const C82Pp::Watch &w : pp.watched())
			add(w.lo, w.hi, w.internal);
	for (const C82Mp::Watch &w : m_mp.watched())
		add(w.lo, w.hi, w.internal);
}

void
Tms320c82::written(uint32_t addr, uint32_t bytes)
{
	visible();
	if (m_mp.watches(addr, bytes))
		m_mp.disturb();
	for (C82Pp &pp : m_pp)
		if (pp.watches(addr, bytes))
			pp.disturb();
}

void
Tms320c82::settle()
{
	board_reached();
	m_mp.disturb();
	for (C82Pp &pp : m_pp)
		pp.disturb();
}

void
Tms320c82::save(C82ChipState &out)
{
	settle();
	out.ram = m_ram;
	out.refcntl = m_refcntl;
	out.ptmin = m_ptmin;
	out.ptmax = m_ptmax;
	out.fltsts = m_fltsts;
	for (unsigned pp = 0; pp < 2; pp++)
		m_pp[pp].save(out.pp[pp]);
	for (unsigned n = 0; n < 3; n++)
		out.eint[n] = m_eint[n];
}

void
Tms320c82::load(const C82ChipState &in)
{
	board_reached();
	/* A state comes with the board's memories, written behind the
	 * MP's blocks */
	m_mp.flush_blocks();
	m_ram = in.ram;
	m_refcntl = in.refcntl;
	m_ptmin = in.ptmin;
	m_ptmax = in.ptmax;
	m_fltsts = in.fltsts;
	for (unsigned pp = 0; pp < 2; pp++)
		m_pp[pp].load(in.pp[pp]);
	for (unsigned n = 0; n < 3; n++)
		m_eint[n] = in.eint[n] != 0;
}

} /* namespace odin */
