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

#ifndef ODIN_C82_TC_HPP
#define ODIN_C82_TC_HPP

#include <array>
#include <cstdint>

namespace odin {

class Tms320c82;

/* The transfer controller's state, for save states. Plain data. */
struct C82TcState {
	std::array<uint32_t, 16> table;
	uint64_t done_at;
	uint32_t entry;
	std::array<uint8_t, 3> queued, stuck;
	uint8_t active, last, short_form, peripheral;
	uint8_t pad[6];
	/* The XPTs, and the entry a short-form XPT halted */
	uint32_t halted_entry;
	uint64_t halted_left;
	std::array<uint32_t, 16> halted_table;
	uint8_t pins, xpt_waiting, xpt, xpt_acked, halted, halted_who, halted_short, halted_peripheral;
};

/* The transfer controller of the TMS320C82 (TI SPRU261). Emulated: the
 * packet transfer requests of the MP (SPRU109 § 3.2) and of the two PPs
 * (SPRU110 ch. 12), short form (ch. 5) and long-form dimensioned transfers
 * (ch. 6), normal and peripheral device access modes (§ 8.3), served by
 * priority and round robin (§ 2.1, 2.2), and the external transfer
 * requests of the XPT pins (ch. 7). Not yet: guided and fill
 * transfers, the other special access modes (VRAM read transfers, block
 * write, transparency), suspension and resumption. A request that needs one
 * of them stays queued and is told once; it does not hold the TC for the
 * other processors (HEURISTIC). Timing, HEURISTIC: 64 bits a cycle on each
 * side (SPRU113: 64-bit buses), source and destination one after the
 * other, the table read 64 bits a cycle; no wait state, page, refresh or
 * cache service; the data move at the end of an entry; the TC changes
 * processor between two entries, never inside one (no suspension); a
 * short-form XPT halts the entry in progress, which goes on after it
 * (§ 7.4.2). */
class C82Tc {
public:
	/* The requesting processors, in round-robin order (§ 2.2) */
	enum : unsigned { MP, PP0, PP1, REQUESTERS };

	explicit C82Tc(Tms320c82 &chip);

	void reset();
	/* P set by `who`: false when its request is already queued */
	bool request(unsigned who, uint64_t now);
	/* Its Q bit */
	bool queued(unsigned who) const { return m_queued[who]; }
	/* A PP reset abandons its requests, with no interrupt (SPRU110 § 9.6.1) */
	void drop(unsigned who);
	/* The MP's PKTREQ (SPRU109 § 3.2) */
	bool request(uint64_t now) { return request(MP, now); }
	bool queued() const { return queued(MP); }
	/* The /XPT[3:0] pins as the board holds them: the number of its
	 * request, 0 for none (code 1111; table 7-1). A request stays until
	 * the TC acknowledges it (§ 7.2) */
	void set_xpt(unsigned number, uint64_t now);
	/* The pins sampled again: CONFIG X or FLTSTS changed (§ 7.2, 1.3.4) */
	void sample_xpt(uint64_t now);
	/* The cycle of the next end of an entry, ~0 when none */
	uint64_t next_event() const { return m_done_at; }
	void advance(uint64_t now);

	void save(C82TcState &out) const;
	void load(const C82TcState &in);

private:
	static constexpr uint8_t NOBODY = 0xFF;
	/* An XPT being served, in place of a processor */
	static constexpr unsigned XPT = REQUESTERS;
	/* A peripheral device transfer's direction (§ 8.3.1, 8.3.2) */
	enum : uint8_t { NORMAL, DEVICE_READS, DEVICE_WRITES };

	void arbitrate(uint64_t now);
	bool start(unsigned who, uint64_t now);
	bool serve_xpt(uint64_t now);
	void xpt_over(uint64_t now);
	void end();
	void stop_list(unsigned who, bool error, uint64_t now);
	void faulted(unsigned who, uint64_t now);
	void entry_done(unsigned who);
	unsigned level(unsigned who) const;
	uint32_t list_address(unsigned who) const;
	void not_yet(unsigned who, const char *what, uint32_t entry);
	bool stall(unsigned who, const char *what, uint32_t entry, uint64_t now);
	bool peripheral(unsigned who, uint32_t entry, uint32_t from, uint32_t to, uint64_t now);
	void acknowledge(uint32_t from, uint32_t to, uint64_t now);

	Tms320c82 &m_chip;
	std::array<uint32_t, 16> m_table{};
	uint64_t m_done_at = ~0ull;
	uint32_t m_entry = 0;
	std::array<bool, REQUESTERS> m_queued{}, m_stuck{};
	uint8_t m_active = NOBODY, m_last = REQUESTERS - 1;
	bool m_short = false;
	uint8_t m_peripheral = NORMAL;
	/* The XPT pins, the request synchronized and waiting, the one being
	 * served and whether its access code went out (§ 7.2, 7.4.3) */
	uint8_t m_pins = 0, m_xpt_waiting = 0, m_xpt = 0;
	bool m_xpt_acked = false;
	/* The processor's entry a short-form XPT halted, its registers kept
	 * loaded (§ 7.4.2): its table, the cycles it had left */
	bool m_halted = false;
	std::array<uint32_t, 16> m_halted_table{};
	uint64_t m_halted_left = 0;
	uint32_t m_halted_entry = 0;
	uint8_t m_halted_who = NOBODY, m_halted_peripheral = NORMAL;
	bool m_halted_short = false;
};

} /* namespace odin */

#endif /* ODIN_C82_TC_HPP */
