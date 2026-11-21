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

#ifndef ODIN_PIC16C71X_HPP
#define ODIN_PIC16C71X_HPP

#include <array>
#include <cstdint>
#include <vector>

namespace odin {

/* Everything the chip keeps, plain data */
struct Pic16c71xState {
	uint64_t cycles, adc_end;
	std::array<uint16_t, 8> stack;
	uint16_t pc;
	uint8_t sp, w, status, fsr, pclath, intcon, option, tmr0, prescale, inhibit;
	uint8_t porta, portb, trisa, trisb, pcon, adcon0, adcon1, adres, adc_sample, rb_in, sleeping;
	std::array<uint8_t, 0x44> ram;

	bool operator==(const Pic16c71xState &) const = default;
};

/* Microchip's PIC16C710 and PIC16C711, following data sheet DS30272A: the
 * 35 instructions of 14 bits, each taking one instruction cycle (four clock
 * periods), or two for a jump or a skip; the two register banks, Timer0 and
 * its prescaler, the 8-bit A/D converter and the interrupts they raise. The
 * board drives PORTB's inputs and reads its pins; the analog inputs are
 * given as the codes the converter makes of them (VREF = VDD).
 *
 * Not modelled: the watchdog (its fuse is in the configuration word, not in
 * the program), Timer0 counting on RA4 (T0CS = 1), and the flags for an
 * RB0/INT edge and a change on RB4-RB7 (INTF, RBIF): only the program sets
 * them. */
class Pic16c71x {
public:
	/* `program`: its words (512 for the PIC16C710, 1024 for the PIC16C711);
	 * `ram`: its general purpose registers from 0x0C (36 bytes, 68 for the
	 * PIC16C711); `clock`: its oscillator, for the converter's own RC clock */
	Pic16c71x(std::vector<uint16_t> program, unsigned ram, uint64_t clock);

	/* Power-on: the registers as the data sheet's table 4-1 gives them,
	 * and 0 for those it leaves unknown */
	void reset();
	/* Runs instructions until `cycle` (instruction cycles from reset); the
	 * last one may end past it */
	void run_to(uint64_t cycle);
	uint64_t cycles() const { return m_cycles; }

	/* The levels the board drives on PORTB: only the pins set as inputs
	 * (TRISB set) take them */
	void set_portb_in(uint8_t levels)
	{
		if (levels != m_rb_in)
			m_wait_last = m_wait_turn = 0;
		m_rb_in = levels;
	}
	/* PORTB's pins: the latch on its outputs, the board's levels on its
	 * inputs */
	uint8_t portb() const { return static_cast<uint8_t>((m_portb & ~m_trisb) | (m_rb_in & m_trisb)); }
	/* AN0 to AN3, as the converter's codes; one is sampled when GO is set */
	void set_analog(unsigned channel, uint8_t code) { m_analog[channel & 3] = code; }

	/* A proven wait in the board's program (its tables): once two passes of
	 * its loop take the same time with PORTB's inputs steady (a pass cut
	 * short by an interrupt does not count), the whole passes left before
	 * the next Timer0 or converter event are counted, not run */
	void set_wait_pc(uint16_t pc) { m_wait_pc = pc; }

	void save(Pic16c71xState &out) const;
	void load(const Pic16c71xState &in);

private:
	enum : uint8_t { STATUS_C = 0x01, STATUS_DC = 0x02, STATUS_Z = 0x04, STATUS_PD = 0x08, STATUS_TO = 0x10,
		         STATUS_RP0 = 0x20 };
	enum : uint8_t { INTCON_RBIF = 0x01, INTCON_INTF = 0x02, INTCON_T0IF = 0x04, INTCON_RBIE = 0x08,
		         INTCON_INTE = 0x10, INTCON_T0IE = 0x20, INTCON_ADIE = 0x40, INTCON_GIE = 0x80 };
	enum : uint8_t { ADCON0_ADON = 0x01, ADCON0_ADIF = 0x02, ADCON0_GO = 0x04 };
	enum : uint8_t { OPTION_PSA = 0x08, OPTION_T0CS = 0x20 };

	void step();
	/* `n` instruction cycles pass: Timer0 counts and a conversion may end */
	void pass(uint64_t n);
	void interrupt_if_due();
	/* The cycle of the next Timer0 overflow or end of conversion */
	uint64_t next_event() const;
	/* Timer0's prescaler as a shift: none when the watchdog has it */
	unsigned tmr0_shift() const { return (m_option & OPTION_PSA) ? 0 : (m_option & 7) + 1; }

	uint8_t read(uint8_t f) const;
	/* `flags`: the instruction sets Z, DC or C, so its write to STATUS
	 * leaves those bits alone (§ 4.2.2.1) */
	void write(uint8_t f, uint8_t v, bool flags);
	uint8_t address(uint8_t f) const;
	void write_adcon0(uint8_t v);
	void push(uint16_t v);
	uint16_t pop();
	void set_z(uint8_t v) { m_status = static_cast<uint8_t>((m_status & ~STATUS_Z) | (v ? 0 : STATUS_Z)); }
	uint8_t add(uint8_t a, uint8_t b);
	/* a - b, C and DC set when nothing is borrowed */
	uint8_t sub(uint8_t a, uint8_t b);

	std::vector<uint16_t> m_program;
	uint16_t m_pc_mask;
	unsigned m_ram_size;
	uint64_t m_clock;
	std::array<uint8_t, 4> m_analog{};

	uint64_t m_cycles = 0;
	/* The cycle at which the conversion in progress ends, 0 for none */
	uint64_t m_adc_end = 0;
	std::array<uint16_t, 8> m_stack{};
	uint16_t m_pc = 0;
	uint8_t m_sp = 0, m_w = 0, m_status = 0, m_fsr = 0, m_pclath = 0, m_intcon = 0, m_option = 0;
	/* Timer0, its prescaler's count, and the cycles it is held for after
	 * a write */
	uint8_t m_tmr0 = 0, m_prescale = 0, m_inhibit = 0;
	uint8_t m_porta = 0, m_portb = 0, m_trisa = 0, m_trisb = 0, m_pcon = 0;
	uint8_t m_adcon0 = 0, m_adcon1 = 0, m_adres = 0, m_adc_sample = 0;
	uint8_t m_rb_in = 0;
	bool m_sleeping = false, m_pc_written = false;
	std::array<uint8_t, 0x44> m_ram{};

	uint16_t m_wait_pc = 0xFFFF;
	uint64_t m_wait_last = 0, m_wait_turn = 0;
};

} /* namespace odin */

#endif /* ODIN_PIC16C71X_HPP */
