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

#ifndef ODIN_TMS320C82_HPP
#define ODIN_TMS320C82_HPP

#include <array>
#include <cstdint>
#include <cstring>
#include <span>

#include "odin/machine/cpu/tms320c82/c82_mp.hpp"
#include "odin/machine/cpu/tms320c82/c82_pp.hpp"
#include "odin/machine/cpu/tms320c82/c82_tc.hpp"

namespace odin {

/* What the board puts on the C82's external bus: everything from
 * 0x02000000 up (SPRS048 fig. 3). Bytes in address order, the C82 being
 * big-endian on this board (its boot reads "_mag", 0x5F6D6167, at
 * 0xC0000000). */
class C82Bus {
public:
	virtual ~C82Bus() = default;
	virtual uint8_t read8(uint32_t addr) = 0;
	virtual uint16_t read16(uint32_t addr) = 0;
	virtual uint32_t read32(uint32_t addr) = 0;
	virtual uint64_t read64(uint32_t addr) = 0;
	virtual void write8(uint32_t addr, uint8_t value) = 0;
	virtual void write16(uint32_t addr, uint16_t value) = 0;
	virtual void write32(uint32_t addr, uint32_t value) = 0;
	virtual void write64(uint32_t addr, uint64_t value) = 0;
	/* A column access of a peripheral device packet transfer (SPRU261
	 * § 8.3.4; AD[35:32] = 0010 or 0011 at row time, table 10-2) at the
	 * 64-bit doubleword `addr`. In a read the memory drives the data bus
	 * and the device latches it; in a write the device drives the bytes
	 * whose CAS/DQM are active (bit n: the byte at addr + n) into the
	 * memory. With no device on the board nothing latches the data, and
	 * HEURISTIC, the memory keeps its bytes in a write */
	virtual void pdpt_read(uint32_t) {}
	virtual void pdpt_write(uint32_t, uint8_t) {}
	/* The TC serves the board's XPT `number`: its code went out on
	 * AD[39:36] at row time (SPRU261 § 7.4.3, table 10-2); the device may
	 * let go of the /XPT pins */
	virtual void xpt_acknowledged(unsigned) {}
	/* For a waiting PP (C82Pp::set_wait): what a read of 2^size bytes at
	 * `addr` would answer at the bus time `now`, without making it (no
	 * side effect); false when the board cannot tell */
	virtual bool peek(uint32_t, unsigned, uint64_t, uint64_t &) { return false; }
	/* For a waiting PP: the read of 2^size bytes at `addr` at the
	 * bus time `now`, made only when it would answer `value` (true);
	 * nothing done otherwise. By default peek(), then the read */
	virtual bool read_if(uint32_t addr, unsigned size, uint64_t now, uint64_t value)
	{
		uint64_t seen = 0;

		if (!peek(addr, size, now, seen) || seen != value)
			return false;
		switch (size) {
		case 0:
			read8(addr);
			break;
		case 1:
			read16(addr);
			break;
		case 2:
			read32(addr);
			break;
		default:
			read64(addr);
			break;
		}
		return true;
	}
	/* For a waiting PP whose read of 2^size bytes at `addr` just
	 * answered `value` at the bus time `now`: the bus time before which
	 * the same read would still answer `value`, the board reached by
	 * nothing else (Tms320c82::board_reached) and nothing set from outside
	 * Tms320c82::run meanwhile. By default 0: no promise */
	virtual uint64_t answers_until(uint32_t, unsigned, uint64_t, uint64_t) { return 0; }
	/* `count` such reads made, the last at the bus time `now`, each
	 * answering `value` as answers_until() promised: what they do. Only
	 * for reads a board promised; by default, the reads */
	virtual void reads_made(uint32_t addr, unsigned size, uint64_t, uint64_t, uint64_t count)
	{
		for (uint64_t n = 0; n < count; n++) {
			if (size == 0)
				read8(addr);
			else if (size == 1)
				read16(addr);
			else if (size == 2)
				read32(addr);
			else
				read64(addr);
		}
	}
};

/* The chip's own memories and registers, for save states. Plain data. */
struct C82ChipState {
	std::array<uint8_t, 8 * 4096> ram;
	uint32_t refcntl, ptmin, ptmax, fltsts;
	std::array<C82PpState, 2> pp;
	std::array<uint8_t, 3> eint;
	uint8_t pad[5];
};

/* The TMS320C82 (TI SPRS048): a master processor, two parallel processors
 * and a transfer controller sharing 44 KB of on-chip RAM through a crossbar.
 * Big-endian, as on this board; one clock for the MP, the TC and the PPs,
 * which take turns by slices of a few cycles (HEURISTIC). */
class Tms320c82 {
public:
	/* On-chip RAM blocks of 4 KB, in this order (SPRS048 fig. 3) */
	enum Block : uint8_t {
		PP0_DATA0, PP1_DATA0, PP0_DATA1, PP1_DATA1, PP0_PARAM, PP1_PARAM, MP_PARAM, MP_DCACHE, BLOCKS
	};

	explicit Tms320c82(C82Bus &bus);

	/* Power-up reset of the chip, the MP running (/HREQ low, § 9.4) */
	void reset();
	/* Runs everything until the cycle `until` */
	void run(uint64_t until)
	{
		m_mp.check_host_rounding();
		run_rounded(until);
	}
	/* run() with the host's rounding read before (check_host_rounding)
	 * and not changed since: the runs of a frame (Rollext::run_frame) */
	void check_host_rounding() { m_mp.check_host_rounding(); }
	void run_rounded(uint64_t until);
	/* The bus time: the MP's cycles, which end each slice */
	uint64_t cycles() const { return m_mp.cycles(); }

	/* /EINT1-/EINT3: an interrupt on each rising edge; /LINT4: while low
	 * (SPRS048 signal descriptions; SPRU109 § 9.2.5) */
	void set_eint(unsigned n, bool high);
	void set_lint4(bool low) { m_mp.set_lint4(low); }
	/* /XPT3-/XPT0: the number of the request the board holds, 0 for none
	 * (SPRS048 p. 9; SPRU261 table 7-1) */
	void set_xpt(unsigned number) { m_tc.set_xpt(number, cycles()); }
	void xpt_acknowledged(unsigned number)
	{
		board_reached();
		m_bus.xpt_acknowledged(number);
	}

	/* Whole 1 MB pages of the external bus that are plain memory, read
	 * and written here without the board (bytes in address order) */
	void map_external(uint32_t base, uint32_t size, uint8_t *bytes, bool writable);

	C82Mp &mp() { return m_mp; }
	C82Tc &tc() { return m_tc; }
	C82Pp &pp(unsigned n) { return m_pp[n & 1]; }
	/* An on-chip RAM block, for tests and dumps */
	std::span<uint8_t> block(Block b) { return { m_ram.data() + b * 4096u, 4096 }; }
	std::span<const uint8_t> block(Block b) const { return { m_ram.data() + b * 4096u, 4096 }; }

	/* The MP's instruction fetches: external memory and the PPs' RAMs;
	 * false for the MP parameter RAM, the caches, the registers and
	 * invalid addresses (SPRU109 table 5-4) */
	bool mp_fetch(uint32_t addr, uint64_t &word);
	/* The plain memory behind mp_fetch at `addr`: a page of external
	 * memory; null for anything else */
	const uint8_t *mp_code(uint32_t addr) const
	{
		if (addr < 0x02000000)
			return nullptr;

		const uint8_t *bytes = m_pages[addr >> 20].bytes;

		return bytes ? bytes + (addr & 0xFFFFF) : nullptr;
	}
	/* The MP's loads and stores: false when the address faults
	 * (SPRU109 § 5.6, tables 5-2 and 5-3) */
	inline bool mp_load(uint32_t addr, unsigned size, uint64_t &value);
	inline bool mp_store(uint32_t addr, unsigned size, uint64_t value, bool user);
	/* The TC's: false when the address faults (tables 5-6, 5-7) */
	bool tc_read(uint32_t addr, unsigned size, uint64_t &value);
	bool tc_write(uint32_t addr, unsigned size, uint64_t value);
	bool tc_internal_ok(uint32_t addr) const;
	/* For the TC's copies: the plain memory behind the `bytes` bytes from
	 * `addr`, within one 4 KB block of on-chip addresses or one 1 MB page,
	 * when tc_read (tc_write) on each of them would only read (write) it:
	 * no fault, no board, no byte left out, no line or range a waiting
	 * processor reads; null otherwise */
	inline const uint8_t *tc_plain_read(uint32_t addr, uint32_t bytes) const;
	inline uint8_t *tc_plain_write(uint32_t addr, uint32_t bytes);
	/* A column access of its peripheral device transfers, off chip */
	void tc_peripheral(uint32_t addr, bool write, uint8_t strobes)
	{
		board_reached();
		if (write)
			m_bus.pdpt_write(addr, strobes);
		else
			m_bus.pdpt_read(addr);
	}
	/* The PPs' fetches (SPRU109 table 5-5), loads and stores (SPRU110
	 * § 2.1, § 8.8.3.1; SPRU154 note p. 1-8: the MP parameter RAM too):
	 * false when the address faults */
	bool pp_fetch(uint32_t addr, uint64_t &word);
	/* The plain memory behind pp_fetch at `addr` (8-byte aligned): the
	 * PPs' RAMs or a page of external memory; null for anything else */
	const uint8_t *pp_code(uint32_t addr) const
	{
		if (addr >= 0x02000000) {
			const Page &page = m_pages[addr >> 20];

			return page.bytes ? page.bytes + (addr & 0xFFFFF) : nullptr;
		}

		const int32_t offset = internal_offset(addr);

		return offset >= 0 && offset < MP_PARAM * 4096 ? m_ram.data() + offset + (addr & 0xFF8) : nullptr;
	}
	/* The on-chip RAM behind pp_load at `addr` (below 0x02000000), null
	 * when it faults */
	const uint8_t *pp_ram(uint32_t addr) const
	{
		const int32_t offset = internal_offset(addr);

		return offset >= 0 && offset != MP_DCACHE * 4096 ? m_ram.data() + offset + (addr & 0xFFF) : nullptr;
	}
	bool pp_load(uint32_t addr, unsigned size, uint64_t &value);
	bool pp_store(uint32_t addr, unsigned size, uint64_t value);
	/* For the PPs' host code (c82_pp_host_exec.cpp): the on-chip RAM, the
	 * offset of each 4 KB of the addresses below 0x02000000 in it (or a
	 * negative code), its lines watched, and the block the PPs never reach */
	struct RamMap {
		const uint8_t *ram;
		const int16_t *internal;
		const uint64_t *watched;
		int32_t dcache;
	};
	RamMap ram_map() const { return { m_ram.data(), m_internal.data(), m_watched_lines.data(), MP_DCACHE * 4096 }; }
	/* For the MP's host code (c82_mp_host_exec.cpp): the pages of the
	 * external bus (`page` bytes apart: their bytes, then the bytes
	 * writable and watched), and in bytes from them the on-chip RAM, its
	 * map (internal_offset) and its lines watched */
	struct MpMap {
		int32_t page, writable, watched, ram, internal, lines, code;
	};
	const void *mp_pages() const { return m_pages.data(); }
	MpMap mp_map() const;
	uint32_t internal32(uint32_t addr) const;
	void set_internal32(uint32_t addr, uint32_t value);
	/* The MP's blocks read code in the page of `addr` (C82Mp::
	 * code_written is then told of every write there) */
	void set_code_page(uint32_t addr, bool code)
	{
		m_pages[addr >> 20].code = code;
		/* The MP's marks cleared (its flush_blocks), the PPs'
		 * there with them (they mark the same lines) */
		if (!code) {
			m_pp[0].code_page_cleared(addr);
			m_pp[1].code_page_cleared(addr);
		}
	}

	/* cmnd from the MP (SPRU109 p. 10-70) */
	void command(uint32_t word);
	/* The PPERROR register (SPRU109 § 3.3.4; SPRS048 fig. 14) */
	uint32_t pperror();
	uint32_t fltsts() const { return m_fltsts; }
	void fault(uint32_t bits);

	/* Waiting PPs first brought to where they stand */
	void save(C82ChipState &out);
	void load(const C82ChipState &in);

	/* For waiting PPs (C82Pp::set_wait): where an address leads */
	enum Place : int { PLACE_NONE, PLACE_INTERNAL, PLACE_MEMORY, PLACE_BOARD };
	Place place(uint32_t addr) const;
	bool peek(uint32_t addr, unsigned size, uint64_t &value)
	{
		board_reached();
		return m_bus.peek(addr, size, cycles(), value);
	}
	/* A read of the board (PLACE_BOARD: pp_load reaches the bus
	 * there) made only when it answers `value` (C82Bus::read_if) */
	bool read_if(uint32_t addr, unsigned size, uint64_t value)
	{
		board_reached();
		return m_bus.read_if(addr, size, cycles(), value);
	}
	/* C82Bus::answers_until for the read just made by read_if() */
	uint64_t answers_until(uint32_t addr, unsigned size, uint64_t value)
	{
		return m_bus.answers_until(addr, size, cycles(), value);
	}
	/* Reads a waiting PP counted ahead (C82Pp::board_reached), told
	 * to the board */
	void reads_made(uint32_t addr, unsigned size, uint64_t now, uint64_t value, uint64_t count)
	{
		m_bus.reads_made(addr, size, now, value, count);
	}
	/* What the processors watch, by unit (0 the MP, 1 + n PP n): a
	 * range added to the unit's list (the tables only grow), or the
	 * unit's list emptied; without a unit, the MP's */
	[[gnu::noinline]] void watch_added(unsigned unit, uint32_t lo, uint32_t hi, bool internal);
	[[gnu::noinline]] void watch_changed(unsigned unit);
	void watch_added(uint32_t lo, uint32_t hi, bool internal) { watch_added(0, lo, hi, internal); }
	void watch_changed() { watch_changed(0); }
	/* Every waiting PP brought to where it stands (states, tests) */
	void settle();
	/* The long runs (run_long) on (by default) or off (the slices one
	 * by one: the reference of the tests); what a PP on alone did that
	 * another unit could see, or a parked one woken in one (0: tests) */
	void set_long_runs(bool on) { m_long = on; }
	/* The reads counted ahead on (by default) or off (made
	 * one by one: the reference of the tests) */
	void set_counted_reads(bool on) { m_counted = on; }
	bool counted_reads() const { return m_counted; }
	uint64_t long_breaches() const { return m_breaches; }
	/* Before what another unit could see (the board, the TC, a
	 * command, a watched line, a fault; code: only by an external store,
	 * which plain_store stops): never while a PP is on alone (it stops
	 * before) */
	void visible()
	{
		if (m_alone) [[unlikely]]
			m_breaches++;
	}

private:
	struct Page {
		uint8_t *bytes = nullptr;
		bool writable = false;
		/* A waiting PP reads it */
		bool watched = false;
		/* The MP's blocks read code in it (or the PPs') */
		bool code = false;
	};
	/* Before a write that a waiting processor may read */
	void written(uint32_t addr, uint32_t bytes);
	/* Before a write in a page whose Page::code is set: the
	 * PPs' blocks and the MP's that fetch there dropped */
	void code_written(uint32_t addr, uint32_t bytes)
	{
		m_pp[0].code_written(addr, bytes);
		m_pp[1].code_written(addr, bytes);
		m_mp.code_written(addr, bytes);
	}
	/* Whether a write in a watched page touches an external range a
	 * waiting processor reads (written() decides for each) */
	bool external_watched(uint32_t addr, uint32_t bytes) const
	{
		for (unsigned k = 0; k < m_external_count; k++)
			if (addr < m_external[k].hi && addr + bytes > m_external[k].lo)
				return true;
		return false;
	}
	/* written() if the write touches a watched line of on-chip RAM */
	void written_ram(int32_t offset, uint32_t addr, uint32_t bytes)
	{
		const uint64_t lines = 1ull << ((addr & 0xFFF) >> 6) | 1ull << (((addr + bytes - 1) & 0xFFF) >> 6);

		if (m_watched_lines[offset >> 12] & lines)
			written(addr, bytes);
	}

	/* The on-chip block of an address below 0x02000000: its offset in
	 * m_ram, or one of these */
	enum : int16_t { NONE = -1, TC_REGISTERS = -2, CACHE = -3 };
	int32_t internal_offset(uint32_t addr) const
	{
		return m_internal[(addr >> 12) & 0x1FFF];
	}
	uint64_t read_tc_register(uint32_t addr);
	void write_tc_register(uint32_t addr, uint32_t value);
	/* mp_load and mp_store below 0x02000000 but in a RAM block */
	bool mp_load_other(uint32_t addr, unsigned size, uint64_t &value);
	bool mp_store_other(uint32_t addr, unsigned size, uint64_t value);
	uint64_t external_read(uint32_t addr, unsigned size);
	void external_write(uint32_t addr, unsigned size, uint64_t value);
	/* Before anything reaches the board (each call of m_bus but
	 * those of the PPs' counted reads), at the end of run() and before
	 * states: the reads the waiting PPs counted ahead told to it first,
	 * their promises dropped (C82Pp::board_reached). Never while a
	 * PP is on alone (visible) */
	void board_reached()
	{
		visible();
		m_pp[0].board_reached();
		m_pp[1].board_reached();
	}

	C82Bus &m_bus;
	C82Mp m_mp;
	C82Tc m_tc;
	std::array<C82Pp, 2> m_pp;
	std::array<uint8_t, BLOCKS * 4096> m_ram{};
	std::array<int16_t, 0x2000> m_internal{};
	std::array<Page, 4096> m_pages{};
	uint32_t m_refcntl = 0, m_ptmin = 0, m_ptmax = 0, m_fltsts = 0;
	std::array<bool, 3> m_eint{};
	/* What waiting processors read: lines of 64 bytes of the on-chip
	 * blocks (a bit each), external ranges (their pages marked
	 * Page::watched) */
	struct Range {
		uint32_t lo = 0, hi = 0;
	};
	std::array<uint64_t, BLOCKS> m_watched_lines{};
	std::array<Range, 24> m_external{};
	unsigned m_external_count = 0;
	/* The lines of each unit (0 the MP, 1 + n PP n), m_watched_lines
	 * being their union */
	std::array<std::array<uint64_t, BLOCKS>, 3> m_unit_lines{};
	/* The external ranges again from the processors' lists */
	void external_again();

	/* Slices with the MP parked through them, one PP at most not
	 * quiet; false when not worth it (see tms320c82.cpp) */
	bool run_long(uint64_t until);
	bool run_calm(uint64_t until);
	/* run_calm: for each offset in the parked MP's pass (at most CALM
	 * cycles) of where a slice starts, the cycles to its end, to the pass
	 * the MP's run() finds, the offset of its end */
	static constexpr unsigned CALM = 64;
	struct Calm {
		uint64_t delta, pass;
		uint32_t next;
	};
	std::array<Calm, CALM> m_calm{};
	static constexpr unsigned LONG = 128;
	/* Each slice planned: its end (the MP's cycles), the MP's pass then */
	struct Planned {
		uint64_t end, pass;
	};
	std::array<Planned, LONG> m_plan{};
	bool m_long = true, m_alone = false, m_counted = true;
	uint64_t m_breaches = 0;
};

/* Big-endian access to bytes in address order */
inline uint64_t
c82_get(const uint8_t *p, unsigned size)
{
	switch (size) {
	case 0:
		return *p;
	case 1: {
		uint16_t v;

		std::memcpy(&v, p, 2);
		return __builtin_bswap16(v);
	}
	case 2: {
		uint32_t v;

		std::memcpy(&v, p, 4);
		return __builtin_bswap32(v);
	}
	default: {
		uint64_t v;

		std::memcpy(&v, p, 8);
		return __builtin_bswap64(v);
	}
	}
}

inline void
c82_put(uint8_t *p, unsigned size, uint64_t value)
{
	switch (size) {
	case 0:
		*p = static_cast<uint8_t>(value);
		break;
	case 1: {
		const uint16_t v = __builtin_bswap16(static_cast<uint16_t>(value));

		std::memcpy(p, &v, 2);
		break;
	}
	case 2: {
		const uint32_t v = __builtin_bswap32(static_cast<uint32_t>(value));

		std::memcpy(p, &v, 4);
		break;
	}
	default: {
		const uint64_t v = __builtin_bswap64(value);

		std::memcpy(p, &v, 8);
		break;
	}
	}
}

inline const uint8_t *
Tms320c82::tc_plain_read(uint32_t addr, uint32_t bytes) const
{
	if (addr >= 0x02000000) {
		const uint8_t *const page = m_pages[addr >> 20].bytes;

		return page && (addr & 0xFFFFF) + bytes <= 0x100000 ? page + (addr & 0xFFFFF) : nullptr;
	}

	const int32_t offset = internal_offset(addr);

	if (offset < 0 || offset == MP_DCACHE * 4096 || (addr & 0xFFF) + bytes > 0x1000)
		return nullptr;
	return m_ram.data() + offset + (addr & 0xFFF);
}

/* external_watched and written_ram for the whole run: none of its bytes
 * would call written(). A run ending at 2^32 is left to the bytes */
inline uint8_t *
Tms320c82::tc_plain_write(uint32_t addr, uint32_t bytes)
{
	if (addr >= 0x02000000) {
		const Page &page = m_pages[addr >> 20];

		if (!page.bytes || !page.writable || (addr & 0xFFFFF) + bytes > 0x100000 || addr + bytes < addr ||
		    (page.watched && external_watched(addr, bytes)))
			return nullptr;
		if (page.code)
			code_written(addr, bytes);
		return page.bytes + (addr & 0xFFFFF);
	}

	const int32_t offset = internal_offset(addr);

	if (offset < 0 || offset == MP_DCACHE * 4096 || (addr & 0xFFF) + bytes > 0x1000)
		return nullptr;

	const unsigned first = (addr & 0xFFF) >> 6, last = ((addr + bytes - 1) & 0xFFF) >> 6;
	const uint64_t lines = (~0ull >> (63 - last)) & (~0ull << first);

	return m_watched_lines[offset >> 12] & lines ? nullptr : m_ram.data() + offset + (addr & 0xFFF);
}

/* The MP reads all on-chip RAM, its data cache RAM (for tests), the TC
 * registers; anything else below 0x02000000 faults (tables 5-2, 5-3) */
inline bool
Tms320c82::mp_load(uint32_t addr, unsigned size, uint64_t &value)
{
	if (addr >= 0x02000000) {
		const uint8_t *bytes = m_pages[addr >> 20].bytes;

		value = bytes ? c82_get(bytes + (addr & 0xFFFFF), size) : external_read(addr, size);
		return true;
	}

	const int32_t offset = internal_offset(addr);

	if (offset >= 0) {
		value = c82_get(m_ram.data() + offset + (addr & 0xFFF), size);
		return true;
	}
	return mp_load_other(addr, size, value);
}

/* As loads; in user mode the MP parameter RAM and data cache are not
 * written (table 5-3) */
inline bool
Tms320c82::mp_store(uint32_t addr, unsigned size, uint64_t value, bool user)
{
	if (addr >= 0x02000000) {
		const Page &page = m_pages[addr >> 20];

		if (page.bytes && page.writable && !page.watched && !page.code)
			c82_put(page.bytes + (addr & 0xFFFFF), size, value);
		else
			external_write(addr, size, value);
		return true;
	}

	const int32_t offset = internal_offset(addr);

	if (offset >= 0) {
		if (user && (offset == MP_PARAM * 4096 || offset == MP_DCACHE * 4096))
			return false;
		written_ram(offset, addr, 1u << size);
		c82_put(m_ram.data() + offset + (addr & 0xFFF), size, value);
		return true;
	}
	return mp_store_other(addr, size, value);
}

} /* namespace odin */

#endif /* ODIN_TMS320C82_HPP */
