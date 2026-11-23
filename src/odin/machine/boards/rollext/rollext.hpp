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

#ifndef ODIN_ROLLEXT_HPP
#define ODIN_ROLLEXT_HPP

#include <array>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

#include "odin/machine/boards/rollext/d3plus.hpp"
#include "odin/machine/boards/rollext/rollext_clocks.hpp"
#include "odin/machine/chips/eeprom_93c46.hpp"
#include "odin/machine/core/learned_ranges.hpp"
#include "odin/machine/cpu/tms320c82/tms320c82.hpp"

namespace odin {

/* Gaelco's board of Rolling Extreme (1999): one TMS320C82, its SDRAM, the
 * program ROMs and flash, ports, a 93C66, two serial ADCs (TLC549), two
 * FLEX 10K loaded by the program, the 3D chip D3PLUS-4 and two dual DACs
 * (TDA1543). What the program does with it was read in its code, whose
 * addresses are cited. Emulated: the C82, the memory and ports, the 3D
 * chip, the sound, the cabinet's controls and outputs; not yet the link
 * between two cabinets. */
class Rollext final : public C82Bus {
public:
	struct Roms {
		/* ic17 and ic18 interleaved by 4 bytes (the section rom0 of
		 * odin-games.json): ic17 bytes 0-3 and ic18 bytes 4-7 of each
		 * 64-bit word (the reset vector at 0xFFFFFFF8) */
		std::vector<uint8_t> rom0;
		/* ic38, the flash, words big-endian in the file */
		std::vector<uint8_t> flash;
		/* The 3D chip's texture ROMs ic47, ic60, ic45, ic58 (the
		 * sections texture0-3) */
		std::array<std::vector<uint8_t>, 4> texture;
	};

	/* The C82's clock (rollext_clock::C82_HZ) */
	static constexpr uint64_t CLOCK_HZ = rollext_clock::C82_HZ;
	/* /EINT1 is /CLR of the EPF10K50, the chip that makes the syncs on
	 * the 90 MHz side (sheet INT_MAS): a frame keeps its time whatever the
	 * C82's clock: FRAME_PERIODS of the board's oscillator (HEURISTIC,
	 * 57.208 Hz) */
	static constexpr uint64_t FRAME_PERIODS = rollext_clock::FRAME_PERIODS, BOARD_HZ = rollext_clock::BOARD_HZ;
	static constexpr uint64_t FRAME_GCD = std::gcd(CLOCK_HZ * FRAME_PERIODS, BOARD_HZ);
	/* The C82 cycle where frame n starts: rounded down, no drift (874,000
	 * cycles a frame at 50 MHz) */
	static constexpr uint64_t frame_end(uint64_t n)
	{
		return n * (CLOCK_HZ * FRAME_PERIODS / FRAME_GCD) / (BOARD_HZ / FRAME_GCD);
	}
	/* A frame for the host's pacing: 17,480,000 ns, exactly */
	static constexpr uint64_t FRAME_NANOSECONDS = FRAME_PERIODS * 1'000'000'000 / BOARD_HZ;
	static_assert(FRAME_PERIODS * 1'000'000'000 % BOARD_HZ == 0);
	/* HEURISTIC: a sample period of the four DACs, and with it an XPT2
	 * request, every 2048 periods of the board's 90 MHz oscillator:
	 * 43,945.3 Hz, DEDUCED from the pitches of the sounds. The EPF10K100
	 * sends the DACs their words (S-CH1, S-CH2, WS)
	 * and asks for them (SO-REQ, through the EPM7160): its divider, in the
	 * FPGA, is not known */
	static constexpr uint64_t SOUND_CLOCK_HZ = rollext_clock::BOARD_HZ;
	static constexpr uint64_t SOUND_DIVIDER = 2048;
	/* The two TLC549 (interface sheet: IC6 on JP5, the handlebar or the
	 * seat; IC5 on JP6, nothing plugged in this game) */
	static constexpr unsigned ADCS = 2;
	/* The picture, as the 3D chip draws it */
	static constexpr unsigned WIDTH = D3plus::WIDTH, HEIGHT = D3plus::HEIGHT;

	explicit Rollext(Roms roms);
	~Rollext() override;

	void set_name(const std::string &name) { m_name = name; }
	void set_verbose(bool verbose) { m_verbose = verbose; }
	/* Power-up */
	void reset();
	void run_frame();
	unsigned frame() const { return m_s.frame; }

	/* The switches closed (1 = pressed): 0xA0000000 bits 0-3,
	 * 0xA0000002 bits 0-7, 0xA0000004 bits 0-7, 0xA0000006 bits 0-5
	 * (the program: 0x400D3ED0); they read 0 when closed. The cabinet's, by
	 * the pins of JP9 and JP10 in order (UR p. 34): 0xA0000000 bits 0-3
	 * SOUND+, SERVICE, SOUND-, TEST; 0xA0000002 bits 2-7 RIGHT ARM, LEFT
	 * ARM, LEFT LEG, RIGHT LEG, VIEW CHANGE, START (bits 0-1, pins 1-2,
	 * not wired); 0xA0000006 bit 0 COIN */
	void set_switches(const std::array<uint16_t, 4> &closed) { m_closed = closed; }
	/* The two analog inputs, 0-255: channel 0 the potentiometer of the
	 * handlebar or the seat (JP5), channel 1 JP6's; the program reads
	 * channels 1-3 and uses none */
	void set_analog(const std::array<uint8_t, ADCS> &levels) { m_analog = levels; }

	/* The outputs the program drives, output n at bit n - 1 (GameOutput):
	 * 1 COUNTER, bit 0 of 0xA0000004, a pulse for each coin (0x40003C10)
	 * and for START in OUTPUT TEST (0x4000EA84); 2 LAMP START, its bit 1,
	 * blinking in OUTPUT TEST (0x4000EB3C); 3 BLOCK, the coin lock (JP3-2),
	 * its bit 2, which the program never sets; 4 bit 6 of 0xA0000004,
	 * cleared in the attract mode when its sound is off (0x40005038): the
	 * amplifiers' MUTING (interface sheet, Q1), the four channels on */
	uint8_t outputs() const;
	/* The handlebar's motor of the upright: the signed byte the program
	 * writes into bits 0-7 of 0xA0000006 at each frame (0x4003E6A8), -128
	 * to 127, its spring and its jolts. DEDUCED: positive to the left, as
	 * its spring grows with the handlebar's position to the right
	 * (0x40003524-0x400035C0). The interface board sends bits 0-6 through
	 * a R-2R ladder as a level on F-MOT1 and bit 7 alone on F-MOT2; the
	 * motor's board, not in the schematic, is not known. */
	int motor() const { return static_cast<int8_t>(m_s.out[3] & 0xFF); }
	/* Of that byte, the jolts the program adds to the spring (the ground's
	 * bumps, a fall: 0x40013C8C-0x40013D44), in its steps: the force at
	 * 0x400D390C less the spring, which is -10 times the filter at
	 * 0x400D3878 (0x40003588-0x400035BC), times the gain at 0x400D38A4
	 * and 63.5 (0x40003604-0x4003E6B0). For the gamepad alone. */
	double motor_jolt() const;

	/* The range the program has learned for its handlebar, or the seat,
	 * since power-on (channel 0; none before it has set it, or for another
	 * program), and widening it to cover `kept` (learned_ranges.hpp). The
	 * game keeps it in RAM only: it learns it again at each power-on */
	std::optional<LearnedRanges> learned_ranges() const;
	void widen_ranges(const LearnedRanges &kept);

	/* The 93C66 kept between two sessions: its 512 bytes as the chip holds
	 * them (kept.bin); a missing file leaves it blank */
	bool load_kept(const std::string &path);
	bool save_kept(const std::string &path) const;
	/* A byte of the settings (at 0x400D2E30, `byte` 0 to 0xEF) in each of
	 * the two copies the program keeps whose sum is right, the sum made
	 * again (0x4001085C, 0x40010C24); only when it holds `only_from`, if
	 * given. False when no copy is right yet (a blank chip: the program
	 * writes its own settings at its first start). */
	bool set_eeprom_setting(uint32_t byte, uint8_t value, std::optional<uint8_t> only_from = std::nullopt);

	bool save_state(const std::string &path);
	bool load_state(const std::string &path);
	/* For the tests: ODIN_DUMPS=1, each memory in <game>-<memory>.bin */
	void dump_memories() const;
	/* For the tests: ODIN_SOUND_DUMP, a line for each frame (F), XPT2
	 * request (X), its acknowledgement (A) and frame the DACs got (D) */
	void open_sound_dump(const std::string &path);

	/* The four DACs' samples of the frame, as the other Gaelco boards give
	 * theirs (Gaelco3d): left and right; then the woofer's channel and the
	 * fourth, the speakers in the seat of the sit-down cabinet
	 * (AudioOut::woofer_pairs) */
	const std::vector<int16_t> &sound() const { return m_sound; }
	const std::vector<int16_t> &option_sound() const { return m_option_sound; }
	/* RGBA bytes, WIDTH x HEIGHT, top line first: the picture the 3D chip
	 * last finished */
	const std::vector<uint32_t> &picture() const { return m_d3plus.picture(); }

	Tms320c82 &chip() { return *m_chip; }
	D3plus &d3plus() { return m_d3plus; }
	std::vector<uint8_t> &sdram() { return m_sdram; }

	uint8_t read8(uint32_t addr) override;
	uint16_t read16(uint32_t addr) override;
	uint32_t read32(uint32_t addr) override;
	uint64_t read64(uint32_t addr) override;
	void write8(uint32_t addr, uint8_t value) override;
	void write16(uint32_t addr, uint16_t value) override;
	void write32(uint32_t addr, uint32_t value) override;
	void write64(uint32_t addr, uint64_t value) override;
	void pdpt_read(uint32_t addr) override;
	void pdpt_write(uint32_t addr, uint8_t strobes) override;
	void xpt_acknowledged(unsigned number) override;
	bool peek(uint32_t addr, unsigned size, uint64_t now, uint64_t &value) override;
	bool read_if(uint32_t addr, unsigned size, uint64_t now, uint64_t value) override;
	uint64_t answers_until(uint32_t addr, unsigned size, uint64_t now, uint64_t value) override;
	void reads_made(uint32_t addr, unsigned size, uint64_t now, uint64_t value, uint64_t count) override;

private:
	/* What the board keeps besides its memories and chips, for states */
	struct State {
		std::array<uint16_t, 4> out;
		uint64_t eeprom_tick;
		/* The DACs' sample periods since power-on */
		uint64_t sound_periods;
		uint32_t frame, fpga_clocks, video;
		/* The DACs' latch: the 8 bytes written at 0x60000000 */
		std::array<uint8_t, 8> dac;
		std::array<uint8_t, ADCS> adc_shift, adc_held;
		uint8_t fpga_done, fpga_clock, adc_select, adc_clock, adc_clocks;
		/* XPT2 held on the /XPT pins */
		uint8_t xpt2;
		uint8_t pad[2];
	};
	/* Saved as it is: no byte left to chance */
	static_assert(std::has_unique_object_representations_v<State>);

	/* The C82 cycle where the n-th sample period starts, rounded up */
	static constexpr uint64_t PERIOD_GCD = std::gcd(SOUND_DIVIDER * CLOCK_HZ, SOUND_CLOCK_HZ);
	static constexpr uint64_t period_cycle(uint64_t n)
	{
		return (n * (SOUND_DIVIDER * CLOCK_HZ / PERIOD_GCD) + SOUND_CLOCK_HZ / PERIOD_GCD - 1) /
		       (SOUND_CLOCK_HZ / PERIOD_GCD);
	}
	void sound_period();

	uint64_t read(uint32_t addr, unsigned size);
	void write(uint32_t addr, unsigned size, uint64_t value);
	uint64_t ports(uint32_t addr, unsigned size, const uint64_t *peek);
	uint16_t port_read(unsigned port, const uint64_t *peek = nullptr);
	void port_write(unsigned port, uint16_t value);
	void unmapped(const char *what, uint32_t addr);

	std::string m_name = "rollext";
	bool m_verbose = false;
	std::unique_ptr<Tms320c82> m_chip;
	std::vector<uint8_t> m_rom0, m_flash, m_sdram, m_palette, m_table;
	MicrowireEeprom<512> m_eeprom;
	D3plus m_d3plus;
	State m_s{};
	std::array<uint16_t, 4> m_closed{};
	std::array<uint8_t, ADCS> m_analog{ 0x80, 0x80 };
	std::vector<int16_t> m_sound, m_option_sound;
	std::FILE *m_sound_dump = nullptr;
};

} /* namespace odin */

#endif /* ODIN_ROLLEXT_HPP */
