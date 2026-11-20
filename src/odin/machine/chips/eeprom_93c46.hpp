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

#ifndef ODIN_EEPROM_93C46_HPP
#define ODIN_EEPROM_93C46_HPP

#include <array>
#include <bit>
#include <cstdint>
#include <span>

namespace odin {

/* 93C46 serial EEPROM, 1 Kbit, Microwire. ORG low: 128 x 8, 7 address
 * bits; ORG high: 64 x 16, 6 address bits. Pins: CS, SK, DI in, DO out.
 * Instructions, the dummy zero bit before read data and the READY/BUSY
 * output after a write come from the 93C46 data sheets (Microchip
 * 93C46B, ST M93C46). The 93C66 is the same chip with 4 Kbit: 512 x 8 or
 * 256 x 16, 9 or 8 address bits (Microchip 93C66A/B/C data sheet).
 * `Bytes` is the size: 128 for the 93C46, 512 for the 93C66. */
/* Everything the chip keeps between two pin changes. Plain data. */
template <unsigned Bytes>
struct MicrowireEepromState {
	std::array<uint8_t, Bytes> data;
	uint32_t busy_cycles;
	uint32_t out_shift;
	uint16_t shift;
	uint16_t address;
	uint8_t state;
	uint8_t cs, sk, di, org;
	uint8_t write_enabled;
	uint8_t bits;
	uint8_t opcode;
	uint8_t dout;
};

template <unsigned Bytes>
class MicrowireEeprom {
public:
	static constexpr uint32_t SIZE = Bytes;

	/* `clock_hz` is the clock of the cycles passed to tick(). */
	explicit MicrowireEeprom(uint32_t clock_hz);

	void reset();

	/* Pin levels. Edges are detected here. */
	void set_pins(bool cs, bool sk, bool di, bool org);
	bool do_read() const;

	/* Advances the write cycle timer. */
	void tick(uint32_t cycles);

	std::span<uint8_t> data() { return m_data; }
	void load(std::span<const uint8_t> in);

	void save(MicrowireEepromState<Bytes> &out) const;
	void load(const MicrowireEepromState<Bytes> &in);
	/* For the tests */
	bool write_enabled() const { return m_write_enabled; }

private:
	enum class State : uint8_t {
		IDLE,		/* CS low */
		START,		/* CS high, waiting for the start bit */
		COMMAND,	/* shifting opcode and address in */
		READING,	/* shifting data out */
		WRITING,	/* shifting write data in */
		DONE		/* command taken, waiting for CS to drop */
	};

	void on_cs_rise();
	void on_cs_fall();
	void on_sk_rise();
	void execute();
	void execute_write();
	/* log2 of the cells: 7 or 6 for the 93C46, 9 or 8 for the 93C66 */
	uint8_t address_bits() const { return static_cast<uint8_t>(std::bit_width(Bytes) - (m_org ? 2 : 1)); }
	uint8_t data_bits() const { return m_org ? 16 : 8; }
	uint16_t cell_read(uint16_t index) const;
	void cell_write(uint16_t index, uint16_t value);
	void start_write_cycle();

	std::array<uint8_t, SIZE> m_data;
	uint32_t m_write_cycles;
	uint32_t m_busy_cycles;
	State m_state;
	bool m_cs, m_sk, m_di, m_org;
	bool m_write_enabled;
	uint8_t m_bits;
	uint16_t m_shift;
	uint8_t m_opcode;
	uint16_t m_address;
	uint32_t m_out_shift;
	bool m_do;
};

using Eeprom93c46State = MicrowireEepromState<128>;
using Eeprom93c46 = MicrowireEeprom<128>;

} /* namespace odin */

#endif /* ODIN_EEPROM_93C46_HPP */
