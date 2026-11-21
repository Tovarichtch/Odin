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

#include "odin/machine/cpu/pic16c71x/pic16c71x.hpp"

#include <algorithm>
#include <limits>
#include <utility>

namespace odin {

Pic16c71x::Pic16c71x(std::vector<uint16_t> program, unsigned ram, uint64_t clock)
	: m_program(std::move(program)), m_pc_mask(static_cast<uint16_t>(m_program.size() - 1)),
	  m_ram_size(std::min<unsigned>(ram, 0x44)), m_clock(clock)
{
	reset();
}

void
Pic16c71x::reset()
{
	m_cycles = 0;
	m_adc_end = 0;
	m_stack = {};
	m_pc = 0;
	m_sp = 0;
	m_w = 0;
	m_status = STATUS_TO | STATUS_PD;
	m_fsr = 0;
	m_pclath = 0;
	m_intcon = 0;
	m_option = 0xFF;
	m_tmr0 = m_prescale = m_inhibit = 0;
	m_porta = m_portb = 0;
	m_trisa = 0x1F;
	m_trisb = 0xFF;
	m_pcon = 0;
	m_adcon0 = m_adcon1 = m_adres = m_adc_sample = 0;
	m_sleeping = false;
	m_ram = {};
	m_wait_last = m_wait_turn = 0;
}

void
Pic16c71x::run_to(uint64_t cycle)
{
	while (m_cycles < cycle) {
		if (m_sleeping) {
			/* Woken by an enabled interrupt (§ 8.8); Timer0 is stopped */
			if (((m_intcon & INTCON_ADIE) && (m_adcon0 & ADCON0_ADIF)) ||
			    (m_intcon & (m_intcon >> 3) & (INTCON_INTF | INTCON_RBIF))) {
				m_sleeping = false;
				interrupt_if_due();
				continue;
			}
			pass(std::min(cycle, m_adc_end ? m_adc_end : cycle) - m_cycles);
			continue;
		}
		if (m_pc == m_wait_pc) {
			const uint64_t turn = m_wait_last ? m_cycles - m_wait_last : 0;

			if (turn && turn == m_wait_turn) {
				const uint64_t until = std::min(cycle, next_event());

				if (until > m_cycles)
					pass((until - m_cycles - 1) / turn * turn);
			}
			if (turn)
				m_wait_turn = turn;
			m_wait_last = m_cycles;
		}
		step();
	}
}

void
Pic16c71x::pass(uint64_t n)
{
	m_cycles += n;
	/* Timer0 counts the instruction clock (§ 6), held for two cycles after
	 * a write, through the prescaler unless the watchdog has it */
	if (!m_sleeping && !(m_option & OPTION_T0CS)) {
		const uint64_t held = std::min<uint64_t>(n, m_inhibit);
		const unsigned shift = tmr0_shift();
		const uint64_t counts = m_prescale + n - held;
		const uint64_t tmr0 = m_tmr0 + (counts >> shift);

		m_inhibit = static_cast<uint8_t>(m_inhibit - held);
		m_prescale = static_cast<uint8_t>(counts & ((1u << shift) - 1));
		if (tmr0 > 0xFF)
			m_intcon |= INTCON_T0IF;
		m_tmr0 = static_cast<uint8_t>(tmr0);
	}
	if (m_adc_end && m_cycles >= m_adc_end) {
		/* The result in ADRES, GO cleared, ADIF set (§ 7) */
		m_adres = m_adc_sample;
		m_adcon0 = static_cast<uint8_t>((m_adcon0 & ~ADCON0_GO) | ADCON0_ADIF);
		m_adc_end = 0;
	}
}

uint64_t
Pic16c71x::next_event() const
{
	uint64_t at = m_adc_end ? m_adc_end : std::numeric_limits<uint64_t>::max();

	if (!(m_option & OPTION_T0CS)) {
		const unsigned shift = tmr0_shift();

		at = std::min(at, m_cycles + m_inhibit + ((static_cast<uint64_t>(0x100 - m_tmr0) << shift) - m_prescale));
	}
	return at;
}

/* An enabled flag with GIE set: PC pushed, GIE cleared, two dummy cycles,
 * then 0x0004 (figures 8-19 and 6-4). The flag is sampled at the end of
 * each instruction, not at its Q1, so the vector may come one instruction
 * early. */
void
Pic16c71x::interrupt_if_due()
{
	if (!(m_intcon & INTCON_GIE))
		return;
	if (!(m_intcon & (m_intcon >> 3) & 7) && !((m_intcon & INTCON_ADIE) && (m_adcon0 & ADCON0_ADIF)))
		return;
	push(m_pc);
	m_pc = 4;
	m_intcon &= static_cast<uint8_t>(~INTCON_GIE);
	/* The loop pass it interrupts is not measured; the one proven before is
	 * kept */
	m_wait_last = 0;
	pass(2);
}

uint8_t
Pic16c71x::address(uint8_t f) const
{
	return f ? static_cast<uint8_t>((m_status & STATUS_RP0) << 2 | f) : m_fsr;
}

uint8_t
Pic16c71x::read(uint8_t f) const
{
	const uint8_t a = address(f);
	const bool bank1 = a & 0x80;

	switch (a & 0x7F) {
	case 0x00:
		return 0;
	case 0x01:
		return bank1 ? m_option : m_tmr0;
	case 0x02:
		return static_cast<uint8_t>(m_pc);
	case 0x03:
		return m_status;
	case 0x04:
		return m_fsr;
	case 0x05: {
		if (bank1)
			return m_trisa;
		/* The RA0-RA3 pins ADCON1 makes analog read 0; nothing drives
		 * the digital inputs */
		static constexpr uint8_t DIGITAL[4] = { 0x10, 0x10, 0x1C, 0x1F };

		return static_cast<uint8_t>(m_porta & ~m_trisa & DIGITAL[m_adcon1 & 3]);
	}
	case 0x06:
		return bank1 ? m_trisb : portb();
	case 0x07:
		return bank1 ? m_pcon : 0;
	case 0x08:
		return bank1 ? m_adcon1 : m_adcon0;
	case 0x09:
		return m_adres;
	case 0x0A:
		return m_pclath;
	case 0x0B:
		return m_intcon;
	default: {
		const unsigned at = (a & 0x7Fu) - 0x0Cu;

		return at < m_ram_size ? m_ram[at] : 0;
	}
	}
}

void
Pic16c71x::write(uint8_t f, uint8_t v, bool flags)
{
	const uint8_t a = address(f);
	const bool bank1 = a & 0x80;

	switch (a & 0x7F) {
	case 0x00:
		break;
	case 0x01:
		if (bank1) {
			m_option = v;
		} else {
			/* The prescaler is cleared and the next two increments
			 * are held (the write's own cycle is counted after
			 * it) */
			m_tmr0 = v;
			m_inhibit = 3;
			if (!(m_option & OPTION_PSA))
				m_prescale = 0;
		}
		break;
	case 0x02:
		/* A computed jump: PCLATH above PCL, one more cycle */
		m_pc = static_cast<uint16_t>(((m_pclath & 0x1F) << 8 | v) & m_pc_mask);
		m_pc_written = true;
		break;
	case 0x03: {
		/* TO and PD are never written; Z, DC and C are not when
		 * the instruction sets them (§ 4.2.2.1) */
		const uint8_t kept = flags ? (STATUS_TO | STATUS_PD | STATUS_Z | STATUS_DC | STATUS_C)
		                           : (STATUS_TO | STATUS_PD);

		m_status = static_cast<uint8_t>((m_status & kept) | (v & ~kept));
		break;
	}
	case 0x04:
		m_fsr = v;
		break;
	case 0x05:
		(bank1 ? m_trisa : m_porta) = v & 0x1F;
		break;
	case 0x06:
		(bank1 ? m_trisb : m_portb) = v;
		break;
	case 0x07:
		if (bank1)
			m_pcon = v & 3;
		break;
	case 0x08:
		if (bank1)
			m_adcon1 = v & 3;
		else
			write_adcon0(v);
		break;
	case 0x09:
		m_adres = v;
		break;
	case 0x0A:
		m_pclath = v & 0x1F;
		break;
	case 0x0B:
		m_intcon = v;
		break;
	default: {
		const unsigned at = (a & 0x7Fu) - 0x0Cu;

		if (at < m_ram_size)
			m_ram[at] = v;
		break;
	}
	}
}

/* GO set with the converter on: the selected channel is sampled and
 * converted, 9.5 TAD after a TOSC/2 start (table 11-7: parameters 131,
 * 134); with the RC clock, TAD is its typical 4 µs and the start a cycle
 * later. Clearing GO during a conversion aborts it and keeps ADRES
 * (§ 7.4). */
void
Pic16c71x::write_adcon0(uint8_t v)
{
	const bool was = m_adcon0 & ADCON0_GO;

	m_adcon0 = v;
	if (!(v & ADCON0_GO)) {
		m_adc_end = 0;
	} else if (!was && (v & ADCON0_ADON)) {
		const unsigned clock = (v >> 6) & 3;
		/* In eighths of an instruction cycle (TOSC / 2) */
		const uint64_t tad = clock == 3 ? m_clock / 125'000 : 4u << (2 * clock);
		const uint64_t start = clock == 3 ? 8 : 1;

		m_adc_sample = m_analog[(v >> 3) & 3];
		m_adc_end = m_cycles + 1 + (start + 19 * tad / 2 + 7) / 8;
	}
}

void
Pic16c71x::push(uint16_t v)
{
	m_stack[m_sp] = v;
	m_sp = (m_sp + 1) & 7;
}

uint16_t
Pic16c71x::pop()
{
	m_sp = (m_sp - 1) & 7;
	return m_stack[m_sp];
}

uint8_t
Pic16c71x::add(uint8_t a, uint8_t b)
{
	const unsigned r = a + b;

	m_status = static_cast<uint8_t>((m_status & ~(STATUS_C | STATUS_DC)) | (r > 0xFF ? STATUS_C : 0) |
	                                (((a & 0xF) + (b & 0xF)) > 0xF ? STATUS_DC : 0));
	set_z(static_cast<uint8_t>(r));
	return static_cast<uint8_t>(r);
}

uint8_t
Pic16c71x::sub(uint8_t a, uint8_t b)
{
	const uint8_t r = static_cast<uint8_t>(a - b);

	m_status = static_cast<uint8_t>((m_status & ~(STATUS_C | STATUS_DC)) | (a >= b ? STATUS_C : 0) |
	                                ((a & 0xF) >= (b & 0xF) ? STATUS_DC : 0));
	set_z(r);
	return r;
}

/* One instruction (table 9-2), then Timer0, the converter and the
 * interrupts for its cycles */
void
Pic16c71x::step()
{
	const uint16_t op = m_program[m_pc];
	const uint8_t f = op & 0x7F;
	const bool to_f = op & 0x80;
	unsigned n = 1;
	const auto store = [&](uint8_t v, bool flags) {
		if (to_f)
			write(f, v, flags);
		else
			m_w = v;
	};
	const auto store_z = [&](uint8_t v) {
		set_z(v);
		store(v, true);
	};
	const auto skip = [&](bool yes) {
		if (yes) {
			m_pc = (m_pc + 1) & m_pc_mask;
			n = 2;
		}
	};

	m_pc = (m_pc + 1) & m_pc_mask;
	m_pc_written = false;
	switch (op >> 12) {
	case 0:
		switch ((op >> 8) & 0xF) {
		case 0x0:
			if (to_f) {
				write(f, m_w, false);	/* MOVWF */
			} else if (op == 0x0008) {
				m_pc = pop();		/* RETURN */
				n = 2;
			} else if (op == 0x0009) {
				m_pc = pop();		/* RETFIE */
				m_intcon |= INTCON_GIE;
				n = 2;
			} else if (op == 0x0063) {
				/* SLEEP: a conversion on the oscillator's clock is
				 * aborted (§ 7.6) */
				m_status = static_cast<uint8_t>((m_status | STATUS_TO) & ~STATUS_PD);
				if (((m_adcon0 >> 6) & 3) != 3) {
					m_adc_end = 0;
					m_adcon0 &= static_cast<uint8_t>(~ADCON0_GO);
				}
				m_sleeping = true;
			} else if (op == 0x0064) {
				/* CLRWDT */
				m_status |= STATUS_TO | STATUS_PD;
				if (m_option & OPTION_PSA)
					m_prescale = 0;
			}
			/* NOP, and the codes the table does not list */
			break;
		case 0x1:
			store_z(0);		/* CLRF, CLRW */
			break;
		case 0x2:
			store(sub(read(f), m_w), true);	/* SUBWF */
			break;
		case 0x3:
			store_z(static_cast<uint8_t>(read(f) - 1));	/* DECF */
			break;
		case 0x4:
			store_z(read(f) | m_w);	/* IORWF */
			break;
		case 0x5:
			store_z(read(f) & m_w);	/* ANDWF */
			break;
		case 0x6:
			store_z(read(f) ^ m_w);	/* XORWF */
			break;
		case 0x7:
			store(add(read(f), m_w), true);	/* ADDWF */
			break;
		case 0x8:
			store_z(read(f));	/* MOVF */
			break;
		case 0x9:
			store_z(static_cast<uint8_t>(~read(f)));	/* COMF */
			break;
		case 0xA:
			store_z(static_cast<uint8_t>(read(f) + 1));	/* INCF */
			break;
		case 0xB: {
			const uint8_t r = static_cast<uint8_t>(read(f) - 1);	/* DECFSZ */
			const uint16_t back = m_program[m_pc];
			const unsigned whole = 3u * r + 2;

			/* A delay loop, DECFSZ on a RAM register then GOTO back to
			 * it, is counted in one go when no interrupt can come during
			 * it: the register ends at 0, 3 cycles per pass and 2 for
			 * the last */
			if (to_f && r && f >= 0x0C && f - 0x0Cu < m_ram_size && (back & 0x3800) == 0x2800 &&
			    (((m_pclath & 0x18) << 8 | (back & 0x7FF)) & m_pc_mask) == ((m_pc - 1) & m_pc_mask) &&
			    (!(m_intcon & INTCON_GIE) || next_event() >= m_cycles + whole)) {
				write(f, 0, false);
				m_pc = (m_pc + 1) & m_pc_mask;
				n = whole;
				break;
			}
			store(r, false);
			skip(r == 0);
			break;
		}
		case 0xC: {
			const uint8_t v = read(f);	/* RRF */
			const uint8_t r = static_cast<uint8_t>(v >> 1 | (m_status & STATUS_C) << 7);

			m_status = static_cast<uint8_t>((m_status & ~STATUS_C) | (v & 1));
			store(r, true);
			break;
		}
		case 0xD: {
			const uint8_t v = read(f);	/* RLF */
			const uint8_t r = static_cast<uint8_t>(v << 1 | (m_status & STATUS_C));

			m_status = static_cast<uint8_t>((m_status & ~STATUS_C) | v >> 7);
			store(r, true);
			break;
		}
		case 0xE: {
			const uint8_t v = read(f);	/* SWAPF */

			store(static_cast<uint8_t>(v << 4 | v >> 4), false);
			break;
		}
		default: {
			const uint8_t r = static_cast<uint8_t>(read(f) + 1);	/* INCFSZ */

			store(r, false);
			skip(r == 0);
			break;
		}
		}
		break;
	case 1: {
		const uint8_t bit = static_cast<uint8_t>(1u << ((op >> 7) & 7));

		switch ((op >> 10) & 3) {
		case 0:
			write(f, read(f) & static_cast<uint8_t>(~bit), false);	/* BCF */
			break;
		case 1:
			write(f, read(f) | bit, false);	/* BSF */
			break;
		case 2:
			skip(!(read(f) & bit));		/* BTFSC */
			break;
		default:
			skip(read(f) & bit);		/* BTFSS */
			break;
		}
		break;
	}
	case 2:
		/* CALL, GOTO: PCLATH<4:3> above the 11 bits */
		if (!(op & 0x800))
			push(m_pc);
		m_pc = static_cast<uint16_t>(((m_pclath & 0x18) << 8 | (op & 0x7FF)) & m_pc_mask);
		n = 2;
		break;
	default: {
		const uint8_t k = static_cast<uint8_t>(op);

		switch ((op >> 8) & 0xF) {
		case 0x0: case 0x1: case 0x2: case 0x3:
			m_w = k;			/* MOVLW */
			break;
		case 0x4: case 0x5: case 0x6: case 0x7:
			m_w = k;			/* RETLW */
			m_pc = pop();
			n = 2;
			break;
		case 0x8:
			m_w |= k;			/* IORLW */
			set_z(m_w);
			break;
		case 0x9:
			m_w &= k;			/* ANDLW */
			set_z(m_w);
			break;
		case 0xA:
			m_w ^= k;			/* XORLW */
			set_z(m_w);
			break;
		case 0xC: case 0xD:
			m_w = sub(k, m_w);		/* SUBLW */
			break;
		case 0xE: case 0xF:
			m_w = add(k, m_w);		/* ADDLW */
			break;
		default:
			break;
		}
		break;
	}
	}
	if (m_pc_written)
		n = 2;
	pass(n);
	interrupt_if_due();
}

void
Pic16c71x::save(Pic16c71xState &out) const
{
	out.cycles = m_cycles;
	out.adc_end = m_adc_end;
	out.stack = m_stack;
	out.pc = m_pc;
	out.sp = m_sp;
	out.w = m_w;
	out.status = m_status;
	out.fsr = m_fsr;
	out.pclath = m_pclath;
	out.intcon = m_intcon;
	out.option = m_option;
	out.tmr0 = m_tmr0;
	out.prescale = m_prescale;
	out.inhibit = m_inhibit;
	out.porta = m_porta;
	out.portb = m_portb;
	out.trisa = m_trisa;
	out.trisb = m_trisb;
	out.pcon = m_pcon;
	out.adcon0 = m_adcon0;
	out.adcon1 = m_adcon1;
	out.adres = m_adres;
	out.adc_sample = m_adc_sample;
	out.rb_in = m_rb_in;
	out.sleeping = m_sleeping;
	out.ram = m_ram;
}

void
Pic16c71x::load(const Pic16c71xState &in)
{
	m_cycles = in.cycles;
	m_adc_end = in.adc_end;
	m_stack = in.stack;
	m_pc = static_cast<uint16_t>(in.pc & m_pc_mask);
	m_sp = in.sp & 7;
	m_w = in.w;
	m_status = in.status;
	m_fsr = in.fsr;
	m_pclath = in.pclath;
	m_intcon = in.intcon;
	m_option = in.option;
	m_tmr0 = in.tmr0;
	m_prescale = in.prescale;
	m_inhibit = in.inhibit;
	m_porta = in.porta;
	m_portb = in.portb;
	m_trisa = in.trisa;
	m_trisb = in.trisb;
	m_pcon = in.pcon;
	m_adcon0 = in.adcon0;
	m_adcon1 = in.adcon1;
	m_adres = in.adres;
	m_adc_sample = in.adc_sample;
	m_rb_in = in.rb_in;
	m_sleeping = in.sleeping;
	m_ram = in.ram;
	m_wait_last = m_wait_turn = 0;
}

} /* namespace odin */
