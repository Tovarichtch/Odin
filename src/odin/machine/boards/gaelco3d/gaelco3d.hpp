// license:BSD-3-Clause
// copyright-holders:Aaron Giles, Couriersud
/*
 *  ██████╗ ██████╗ ██╗███╗   ██╗
 * ██╔═══██╗██╔══██╗██║████╗  ██║
 * ██║   ██║██║  ██║██║██╔██╗ ██║
 * ██║   ██║██║  ██║██║██║╚██╗██║
 * ╚██████╔╝██████╔╝██║██║ ╚████║
 *  ╚═════╝ ╚═════╝ ╚═╝╚═╝  ╚═══╝
 *
 * Copyright (C) 2026 Réda Chérif-Touil
 * Copyright (C) 2026 Daniel Martin
 *
 * Odin is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * Odin is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with Odin.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ODIN_GAELCO3D_HPP
#define ODIN_GAELCO3D_HPP

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "odin/machine/boards/gaelco3d/gaelco3d_video.hpp"
#include "odin/machine/cpu/adsp2115/adsp2115.hpp"
#include "odin/machine/cpu/m68000/m68000.hpp"
#include "odin/machine/cpu/m68020/m68020.hpp"
#include "odin/machine/cpu/pic16c71x/pic16c71x.hpp"
#include "odin/machine/cpu/tms32031/tms32031.hpp"

namespace odin {
class GaelcoLink;
struct NetLinkSettings;

class Eeprom93c66 {
public:
	static constexpr unsigned WORDS = 256;

	struct State {
		std::array<uint16_t, WORDS> data;
		uint32_t shift, accum;
		uint16_t address;
		uint8_t state, command, bits, cs, clk, di, locked;
		uint8_t pad[3];

		bool operator==(const State &) const = default;
	};

	Eeprom93c66();
	void set_cs(bool on);
	void set_clk(bool on);
	void set_di(bool on) { m_s.di = on; }
	bool do_read() const;
	std::span<uint16_t> data() { return m_s.data; }
	const State &state() const { return m_s; }
	void set_state(const State &s) { m_s = s; }

private:
	void command();

	State m_s;
};

class Gaelco3d {
public:
	struct Roms {
		std::vector<uint8_t> program;
		std::vector<uint8_t> tms;
		std::vector<uint8_t> adsp;
		std::vector<uint8_t> texture;
		std::vector<uint8_t> texmask;
		std::vector<uint8_t> boot;
		/* Football Power: the program of each ball's PIC16C710 */
		std::vector<uint8_t> pic;
		bool kick_sensors = false;
		/* Speed Up and Surf Planet: a 68000 at 15 MHz and the TMS320C31
		 * at 60 MHz */
		bool m68000 = false;
		/* Speed Up's speakers, wheel motor and settings */
		bool speed_up = false;
		bool surf_planet = false;
	};

	static constexpr unsigned WIDTH = Gaelco3dVideo::WIDTH, HEIGHT = Gaelco3dVideo::HEIGHT;

	static constexpr uint32_t SOUND_RATE = 16'000'000 / (2 * 6 * 16 * 4);

	/* One frame of the board is one field on its screen. A line is 720 dots
	 * at 11.25 MHz, 64 us: 45 MHz divided by 4 (Speed Up's schematic,
	 * sheet 4; the same 45 MHz on Football Power's board), the line length
	 * in the board designer's slides. A field is 260.5 lines, 521 half
	 * lines of 1/31250 s, 59.98 a second: MEASURED, the attract sequences
	 * run at 59.98 fields a second in three videos of real boards of both
	 * kinds, and 521 is the only whole number of half lines that gives that
	 * rate. No document states it */
	static constexpr uint64_t HALF_LINES_PER_SECOND = 31'250, HALF_LINES_PER_FIELD = 521;
	uint64_t frame_nanoseconds() const { return HALF_LINES_PER_FIELD * 1'000'000'000 / HALF_LINES_PER_SECOND; }

	explicit Gaelco3d(Roms roms);
	~Gaelco3d();

	void set_name(const std::string &name) { m_name = name; }
	void reset();
	void run_frame();

	void set_switches(const std::array<uint16_t, 4> &switches) { m_switches = switches; }
	void set_analog(const std::array<uint8_t, 4> &analog) { m_analog = analog; }
	/* Football Power: the accelerometers under each ball, as its PIC's
	 * converter reads them: AN0 (ADXL150), AN1 (ADXL250 Y), AN2 (ADXL250 X) */
	using BallSensors = std::array<std::array<uint8_t, 3>, 2>;
	static constexpr std::array<uint8_t, 3> BALL_AT_REST = { 0x80, 0x80, 0x80 };
	void set_ball_sensors(const BallSensors &sensors);

	/* RGBA bytes, WIDTH x HEIGHT, top line first */
	const std::vector<uint32_t> &picture() const { return m_video.picture(); }
	/* One picture per field, or both fields as the board keeps them
	 * (Gaelco3dVideo::set_one_picture) */
	void set_one_picture(bool on) { m_video.set_one_picture(on); }

	const std::vector<int16_t> &sound() const { return m_sound; }
	/* The second pair: Speed Up's seat speakers; on the other games, the
	 * woofer channel and the fourth channel (AudioOut::woofer_pairs) */
	const std::vector<int16_t> &option_sound() const { return m_option_sound; }
	/* Speed Up's woofer, one sample a frame */
	const std::vector<int16_t> &woofer_sound() const { return m_woofer_sound; }

	uint8_t outputs() const { return m_outlatch; }
	/* Speed Up's wheel motor, -128 to 127, positive to the left */
	int16_t motor() const { return m_motor; }
	/* Of that command, as the program made it: the part the wheel's
	 * position gives (its spring), whether the spring was on, and the hit
	 * played (cars +-1 to +-3, a wall +-4; 0: none); none where its
	 * routines are not found */
	struct MotorParts {
		int spring = 0;
		bool spring_on = false;
		int hit = 0;
	};
	std::optional<MotorParts> motor_parts() const
	{
		return m_spring_at ? std::optional<MotorParts>(m_motor_parts) : std::nullopt;
	}
	unsigned frame() const { return m_frame; }

	/* False when the EEPROM keeps no settings yet */
	bool set_eeprom_setting(uint32_t byte, unsigned shift, unsigned bits, uint8_t value);
	bool load_kept(const std::string &path);
	bool save_kept(const std::string &path) const;
	bool save_state(const std::string &path);
	bool load_state(const std::string &path);
	void set_verbose(bool verbose) { m_verbose = verbose; }

	bool set_link(const NetLinkSettings &link);
	const GaelcoLink *link() const { return m_link.get(); }
	bool link_failed() const { return m_link_failed; }

private:
	struct Main : M68020Bus {
		explicit Main(Gaelco3d &b) : board(b) {}
		uint8_t read8(uint32_t addr) override;
		uint16_t read16(uint32_t addr) override;
		uint32_t read32(uint32_t addr) override;
		void write8(uint32_t addr, uint8_t value) override;
		void write16(uint32_t addr, uint16_t value) override;
		void write32(uint32_t addr, uint32_t value) override;
		bool waiting(uint32_t pc) override;
		Gaelco3d &board;
	};
	struct Main16 : M68000Bus {
		explicit Main16(Gaelco3d &b) : board(b) {}
		uint16_t read16(uint32_t addr, unsigned) override { return board.m_main_bus.read16(addr & 0xFFFFFF); }
		uint8_t read8(uint32_t addr, unsigned) override { return board.m_main_bus.read8(addr & 0xFFFFFF); }
		void write16(uint32_t addr, uint16_t value, unsigned) override { board.m_main_bus.write16(addr & 0xFFFFFF, value); }
		void write8(uint32_t addr, uint8_t value, unsigned) override { board.m_main_bus.write8(addr & 0xFFFFFF, value); }
		bool waiting(uint32_t pc) override;
		Gaelco3d &board;
	};
	struct Dsp : Tms32031Bus {
		explicit Dsp(Gaelco3d &b) : board(b) {}
		uint32_t read(uint32_t addr) override;
		void write(uint32_t addr, uint32_t value) override;
		void iack(uint32_t addr) override;
		Gaelco3d &board;
	};
	struct Sound : Adsp2115Bus {
		explicit Sound(Gaelco3d &b) : board(b) {}
		uint16_t data_read(uint16_t addr) override;
		void data_write(uint16_t addr, uint16_t value) override;
		void control_written(unsigned reg, uint16_t value) override;
		void sport_tx(unsigned port, uint32_t value) override;
		bool waiting(uint32_t pc) override;
		Gaelco3d &board;
	};

	uint32_t io_read(uint32_t addr, unsigned size);
	void io_write(uint32_t addr, uint32_t value, unsigned size);
	void main_latch(unsigned bit, bool on);
	void out_latch(unsigned bit, bool on);
	/* Runs the balls' PICs up to the main CPU's time */
	void balls_to_main();
	void autobuffer();
	void run_adsp_to(uint64_t to);
	/* The ADSP runs on its own thread, up to the same moments in the same
	 * order (it takes its interrupts where each run starts). Only its
	 * status byte, its latch and its samples reach the rest of the board:
	 * anything that reads or writes them waits for it first */
	void adsp_to(uint64_t to);
	void adsp_send();
	void adsp_settle();
	void adsp_loop();
	void start_autobuffer();
	void stop_autobuffer();
	void tell_once(const char *what, uint32_t addr, uint32_t value);
	void set_ipl();
	uint64_t main_cycles() const { return m_main16 ? m_main16->cycles() : m_main->cycles(); }
	void main_run(uint32_t cycles);
	void end_main_run();
	uint8_t ram8(uint32_t addr) const
	{
		return static_cast<uint8_t>(m_ram[(addr & 0xFFFF) >> 2] >> (8 * (3 - (addr & 3))));
	}
	uint16_t ram16(uint32_t addr) const { return static_cast<uint16_t>(ram8(addr) << 8 | ram8(addr + 1)); }

	Main m_main_bus;
	Main16 m_main16_bus;
	Dsp m_dsp_bus;
	Sound m_sound_bus;
	std::unique_ptr<M68020> m_main;
	std::unique_ptr<M68000> m_main16;
	std::unique_ptr<Tms32031> m_tms;
	std::unique_ptr<Adsp2115> m_adsp;
	uint64_t m_main_clock = 0, m_tms_cycles = 0;
	Roms m_roms;
	std::vector<uint32_t> m_tms_rom;
	std::string m_name;
	Gaelco3dVideo m_video;

	std::array<uint32_t, 0x4000> m_ram{};
	std::array<uint32_t, 0x4000> m_palette_ram{};

	Eeprom93c66 m_eeprom;
	uint8_t m_mainlatch = 0, m_outlatch = 0;
	int16_t m_motor = 0;
	/* Where Speed Up's program keeps the wheel's position, its gain, whether
	 * the spring is on and the hit played (0: not found), and the parts of
	 * the last command */
	uint32_t m_spring_at = 0, m_gain_at = 0, m_spring_on_at = 0, m_hit_at = 0;
	MotorParts m_motor_parts;
	bool m_tms_reset = true;
	uint32_t m_wait_counter = 0;
	/* The 68000's frame wait: its option or ready cell, its frame cell */
	struct Wait16 {
		bool routine = false;
		uint32_t flag = 0, cell = 0;
	} m_wait16;
	unsigned m_adsp_wait_at = 4;
	bool m_vblank_irq = false;

	void serial_write(uint8_t byte);
	uint8_t serial_read();
	void serve_link();
	void drain_link();
	void pace_link();
	void link_send(uint16_t word);
	bool m_serial_ready = true, m_serial_irq = false, m_sending = false;
	bool m_rts = false, m_other_rts = true;
	std::array<bool, 16> m_rts_of{};
	std::array<std::vector<uint16_t>, 16> m_held;
	unsigned m_cabinet = 0;
	uint8_t m_serial_in = 0;
	std::deque<uint16_t> m_rx;
	/* The byte this board drives on the link bus, and whether the program
	 * has seen the bus free since the last byte it read */
	uint8_t m_out_byte = 0;
	bool m_seen_free = true;
	bool m_link_open = false;
	bool m_paced = false;
	int m_ahead = 0;
	std::unique_ptr<GaelcoLink> m_link;
	bool m_link_failed = false;

	uint8_t m_sound_latch = 0, m_sound_status = 0;
	bool m_sound_pending = false;
	uint32_t m_adsp_bank = 0;
	unsigned m_ab_ireg = 0;
	int32_t m_ab_base = 0;
	uint32_t m_ab_incs = 0, m_ab_size = 0;
	uint64_t m_ab_period = 0, m_ab_next = 0;
	bool m_ab_on = false;

	std::array<uint16_t, 4> m_switches{};
	std::array<uint8_t, 4> m_analog{ 0x80, 0x80, 0x80, 0x80 };
	std::array<uint8_t, 4> m_analog_shift{};
	/* The TLC548s' last conversions, and the clock pulses of the read in
	 * progress */
	std::array<uint8_t, 4> m_adc_sample{ 0x80, 0x80, 0x80, 0x80 };
	uint8_t m_adc_clocks = 0;
	/* Football Power's balls A and B (bits 29 and 28 of long 0x51002C) */
	std::array<std::unique_ptr<Pic16c71x>, 2> m_balls;

	std::vector<int16_t> m_sound, m_option_sound, m_woofer_sound;

	std::array<uint64_t, 3> m_base{};
	uint64_t m_slices = 0;
	unsigned m_frame = 0;
	bool m_verbose = false;
	/* What tell_once has told, and at which address */
	std::mutex m_told_mutex;
	std::set<std::pair<std::string, uint32_t>> m_told;

	/* The moments the ADSP runs up to: those given (written by the board),
	 * those sent to its thread a few at a time, and those it has run */
	static constexpr uint32_t ADSP_GOALS = 1024, ADSP_BATCH = 16;
	std::array<uint64_t, ADSP_GOALS> m_adsp_goals{};
	uint32_t m_adsp_given = 0;
	std::atomic<uint32_t> m_adsp_sent{ 0 }, m_adsp_done{ 0 };
	std::atomic<bool> m_adsp_resting{ false };
	/* The last member: its thread starts once the others are built */
	std::thread m_adsp_thread;
};
}

#endif
