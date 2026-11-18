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
#include <deque>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "odin/machine/chips/pmx1/pmx1_scene.hpp"
#include "odin/machine/cpu/adsp2115/adsp2115.hpp"
#include "odin/machine/cpu/m68000/m68000.hpp"
#include "odin/machine/cpu/m68020/m68020.hpp"
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
		uint8_t pad[1];

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
		bool kick_sensors = false;
		/* Speed Up and Surf Planet: a 68000 at 15 MHz and the TMS320C31
		 * at 60 MHz */
		bool m68000 = false;
		/* Speed Up's speakers, wheel motor and settings */
		bool speed_up = false;
		bool surf_planet = false;
	};

	static constexpr unsigned WIDTH = 576, HEIGHT = 432;

	static constexpr uint32_t SOUND_RATE = 16'000'000 / (2 * 6 * 16 * 4);

	explicit Gaelco3d(Roms roms);
	~Gaelco3d();

	void set_name(const std::string &name) { m_name = name; }
	void reset();
	void run_frame();

	void set_switches(const std::array<uint16_t, 4> &switches) { m_switches = switches; }
	void set_analog(const std::array<uint8_t, 4> &analog) { m_analog = analog; }
	/* Football Power: the middle, left and right sensors under each ball */
	using BallSensors = std::array<std::array<uint8_t, 3>, 2>;
	static constexpr std::array<uint8_t, 3> BALL_AT_REST = { 0x80, 0x80, 0x80 };
	void set_ball_sensors(const BallSensors &sensors) { m_ball = sensors; }

	const std::vector<uint32_t> &picture() const { return m_picture; }

	void set_gpu(bool on);
	PmxScene take_scene() { return std::exchange(m_ready, PmxScene{}); }

	const std::vector<int16_t> &sound() const { return m_sound; }
	/* Speed Up's rear speakers; its front and seat ones are sound() */
	const std::vector<int16_t> &rear_sound() const { return m_rear_sound; }

	uint8_t outputs() const { return m_outlatch; }
	/* Speed Up's wheel motor, -128 to 127, positive to the left */
	int16_t motor() const { return m_motor; }
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

	struct Vertex {
		float x, y;
	};
	struct Poly {
		uint32_t tex, color;
		float ooz_dx, ooz_dy, ooz_base, uoz_dx, uoz_dy, uoz_base, voz_dx, voz_dy, voz_base, z0;
	};
	enum class Span : uint8_t { FLAT, NORMAL, BLEND };

	uint32_t io_read(uint32_t addr, unsigned size);
	void io_write(uint32_t addr, uint32_t value, unsigned size);
	void palette_write(uint32_t index, uint16_t value);
	void main_latch(unsigned bit, bool on);
	void out_latch(unsigned bit, bool on);
	void kick_clock(bool on);
	void render_write(uint32_t data);
	void render_poly(const uint32_t *data);
	void triangle(const Poly &poly, Span span, const Vertex &a, const Vertex &b, const Vertex &c);
	void scan(const Poly &poly, Span span, int y, int x0, int x1);
	uint32_t texel(const Poly &poly, int u, int v) const;
	void scene_poly(const Poly &poly, Span span, const Vertex *vert, unsigned count, unsigned depth = 0);
	uint32_t image_of(uint32_t &base, unsigned &w, unsigned &h);
	void end_scene();
	void forget_images();
	void autobuffer();
	void run_adsp_to(uint64_t to);
	void start_autobuffer();
	void stop_autobuffer();
	void expand_textures(const std::vector<uint8_t> &texture, const std::vector<uint8_t> &texmask);
	void make_picture();
	void tell_once(const char *what, uint32_t addr, uint32_t value);
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
	Roms m_roms;
	std::string m_name;

	std::array<uint32_t, 0x4000> m_ram{};
	std::array<uint32_t, 0x4000> m_palette_ram{};
	std::vector<uint32_t> m_palette;
	std::vector<uint8_t> m_texture, m_texmask;
	std::vector<uint16_t> m_screen, m_zbuffer;
	std::vector<uint32_t> m_polydata;
	uint32_t m_polydata_count = 0;
	std::vector<uint32_t> m_picture;

	struct ImageKey {
		uint32_t base, size;
		bool operator==(const ImageKey &) const = default;
	};
	struct ImageKeyHash {
		size_t operator()(const ImageKey &k) const { return std::hash<uint64_t>()(uint64_t{ k.base } << 32 | k.size); }
	};
	struct Image {
		uint32_t id;
		unsigned used;
	};
	bool m_gpu = false;
	PmxScene m_scene, m_ready, m_carry;
	float m_most_w = 0.0f;
	std::unordered_map<ImageKey, Image, ImageKeyHash> m_images;
	std::unordered_map<ImageKey, ImageKey, ImageKeyHash> m_aliases;
	std::vector<uint32_t> m_gone;
	uint32_t m_next_image = 1;
	bool m_palette_changed = true;

	Eeprom93c66 m_eeprom;
	uint8_t m_mainlatch = 0, m_outlatch = 0;
	int16_t m_motor = 0;
	bool m_tms_reset = true;
	bool m_trap1_rte = false;
	uint32_t m_wait_counter = 0;
	unsigned m_adsp_wait_at = 4;
	bool m_vblank_irq = false;

	void serial_write(uint8_t byte);
	uint8_t serial_read();
	void serve_link();
	void drain_link();
	void pace_link();
	void link_send(uint16_t word);
	void set_ipl();
	uint64_t main_cycles() const { return m_main16 ? m_main16->cycles() : m_main->cycles(); }
	void main_run(uint32_t cycles);
	void end_main_run();
	uint64_t m_main_clock = 0, m_tms_cycles = 0, m_serial_byte = 0;
	uint64_t m_serial_ready_at = 0;
	bool m_serial_ready = true, m_serial_irq = false, m_sending = false;
	bool m_rts = false, m_other_rts = true;
	std::array<bool, 16> m_rts_of{};
	std::array<std::vector<uint16_t>, 16> m_held;
	unsigned m_cabinet = 0;
	uint8_t m_serial_in = 0;
	std::deque<uint16_t> m_rx;
	uint64_t m_rx_at = 0;
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
	std::array<uint32_t, 2> m_kick_ports{}, m_kick_length{};
	BallSensors m_ball{ BALL_AT_REST, BALL_AT_REST };
	uint8_t m_kick_clock = 27;
	bool m_kick_state = false;

	std::vector<int16_t> m_sound, m_rear_sound;

	std::array<uint64_t, 3> m_base{};
	uint64_t m_slices = 0;
	unsigned m_frame = 0;
	bool m_verbose = false;
	std::unordered_set<uint64_t> m_told;
};
}

#endif
