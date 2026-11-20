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

#include "odin/host/app/gaelco3d_play.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "odin/host/app/ball_kick.hpp"
#include "odin/host/app/gaelco_play.hpp"
#include "odin/host/app/kept_ranges.hpp"
#include "odin/host/app/options.hpp"
#include "odin/host/app/user_files.hpp"
#include "odin/host/audio/audio_out.hpp"
#include "odin/host/input/controls.hpp"
#include "odin/machine/boards/atvtrack/atvtrack.hpp"
#include "odin/machine/boards/gaelco3d/gaelco3d.hpp"
#include "odin/machine/cabinet/drive_board.hpp"
#include "odin/machine/chips/pmx1/pmx1_scene.hpp"

namespace odin {

namespace {

constexpr PortSwitch COIN_PIN{ 2, 0 }, SERVICE_PIN{ 2, 2 }, TEST_PIN{ 2, 3 };
/* Speed Up's SERVICE and TEST are on 0x51003C; its cabinet wires one of the
 * board's two coin inputs (sheet 7, manual p. 22) */
constexpr PortSwitch SPEEDUP_SERVICE_PIN{ 3, 8 }, SPEEDUP_TEST_PIN{ 3, 9 };

constexpr uint16_t SERVICE = 0x4000;
constexpr unsigned COIN_FRAMES = 6;	/* HEURISTIC: the length of a coin pulse */
constexpr float BALL_LIFT = 120.0f;	/* HEURISTIC */
/* HEURISTIC: the softest kick its PIC still accepts: a side sensor 12 above
 * rest (pm 0x0076-0x007F) */
constexpr float BALL_SOFT = 21.0f;
/* Speed Up's wheel motor reaches its full voltage from about 74: its latch
 * drives a 5 V R2R DAC (sheet 9), amplified by 390K / 47K (8.3) between
 * rails of 24 V (sheet 6), and 24 / (8.3 x 5 / 128) = 74. PROVEN; HEURISTIC:
 * the force follows the voltage (a motor at stall), the output transistors'
 * drop ignored */
constexpr int MOTOR_FULL = 74;
/* Speed Up's buzz: at every frame its wheel's spring is on, the effects get
 * the next of -5, 5, -5, 5, -4, 4, -2, 3... (2.2: 0x15E3E-0x15E54, table
 * 0x15ED4; the same in all six versions). PROVEN */
constexpr int BUZZ_MOST = 5;
/* HEURISTIC, Speed Up's gamepad (Gaelco3dGame::shake_pad): its effects
 * twice over against MOTOR_FULL; the thud of a hit; that of a landing, at
 * least LANDING_LEAST and more the longer the car flew (one more each
 * LANDING_FRAMES); no landing after LANDING_LONGEST frames let go: jumps
 * last 6 to 20 frames (preuves/ffb-20261117), the power-on's sweep 648 */
constexpr double HIT_THUD = 0.7, LANDING_LEAST = 0.4, LANDING_FRAMES = 60.0;
constexpr unsigned LANDING_LONGEST = 120;

class Gaelco3dGame final : public Board {
public:
	Gaelco3dGame(const Game &game, Controls &controls, Display &display, const std::string &kept, bool verbose);
	~Gaelco3dGame() override;

	const std::string &game_name() const override { return m_name; }
	unsigned frame() const override { return m_board->frame(); }
	uint64_t frame_nanoseconds() const override { return m_board->frame_nanoseconds(); }
	void run_frame() override;
	bool restart_asked() const override { return false; }
	bool save_state(const std::string &path) override { return m_board->save_state(path); }
	bool load_state(const std::string &path) override { return m_board->load_state(path); }
	std::span<const uint8_t> io_outputs() const override { return m_outputs; }
	std::span<const uint8_t> output_levels() const override { return m_levels; }
	const std::vector<int16_t> &sound(bool optional) const override
	{
		return optional ? (m_speedup ? m_seat : m_board->option_sound()) : m_board->sound();
	}
	std::optional<int> drive_torque() const override
	{
		if (!m_wheel_motor)
			return std::nullopt;
		const int most = DriveBoard::MOST;

		return std::clamp(-m_board->motor() * most / MOTOR_FULL, -most, most);
	}
	std::optional<PadShake> pad_shake() const override { return m_pad; }
	bool set_link(const NetLinkSettings &link) { return m_board->set_link(link); }
	void set_seat_woofer(double hz) { m_woofer_low = LowPass(hz, static_cast<int>(Gaelco3d::SOUND_RATE)); }
	KeptRanges &ranges() { return m_ranges; }

	bool set_eeprom_setting(uint32_t byte, unsigned shift, unsigned bits, uint8_t value)
	{
		return m_board->set_eeprom_setting(byte, shift, bits, value);
	}
	LinkStatus link_status() const override { return link_status_of(m_board->link(), m_board->link_failed()); }

private:
	void read_controls();
	void shake_pad();

	std::string m_name, m_kept;
	std::unique_ptr<Gaelco3d> m_board;
	Controls &m_controls;
	Display &m_display;
	std::vector<GameControl> m_wiring;
	std::vector<PortSwitch> m_grounded;
	bool m_speedup, m_wheel_motor;
	BallSticks m_balls;
	BallKick m_kick[2]{ { Gaelco3d::BALL_AT_REST, BALL_LIFT, BALL_SOFT }, { Gaelco3d::BALL_AT_REST, BALL_LIFT, BALL_SOFT } };
	unsigned m_coin = 0;

	std::array<uint8_t, 1> m_outputs{};
	std::array<uint8_t, 8> m_levels{};
	/* Speed Up's seat speakers with the low end of its woofer, which sits
	 * in the seat; filtered like the woofers of the other Gaelco games
	 * ([sound.speedup] woofer_cutoff) */
	LowPass m_woofer_low{ std::strtod(WOOFER_CUTOFFS[1], nullptr), static_cast<int>(Gaelco3d::SOUND_RATE) };
	std::vector<int16_t> m_seat;
	KeptRanges m_ranges;
	/* The texture ROMs in rows of 4096 texels, as the GPU takes them
	 * (Gaelco3dRoms); the card was found too small for them */
	unsigned m_texture_rows;
	bool m_too_large = false;
	/* Speed Up's gamepad (shake_pad): its two motors; how long the program
	 * has let go of the wheel's spring, and the hit it played then */
	std::optional<PadShake> m_pad;
	unsigned m_let_go = 0;
	int m_let_go_hit = 0;
};

/* The texture ROMs' slots of 4 MB: 8 when the game fills one of the upper
 * four, else 4 */
unsigned
texture_slots(const Game &game)
{
	for (unsigned slot = 4; slot < 8; slot++)
		if (!game.section("texture" + std::to_string(slot)).empty())
			return 8;
	return 4;
}

Gaelco3d::Roms
roms_of(const Game &game)
{
	Gaelco3d::Roms roms;
	const auto take = [&game](const char *section) {
		const std::span<const uint8_t> bytes = game.section(section);

		return std::vector<uint8_t>(bytes.begin(), bytes.end());
	};

	const unsigned lanes = game.section("program2").empty() ? 2 : 4;
	const unsigned slots = texture_slots(game);

	roms.program.assign(0x200000, 0);
	for (unsigned lane = 0; lane < lanes; lane++) {
		const std::span<const uint8_t> bytes = game.section("program" + std::to_string(lane));

		for (size_t i = 0; i < bytes.size() && i * lanes + lane < roms.program.size(); i++)
			roms.program[i * lanes + lane] = bytes[i];
	}
	roms.tms = take("tms");
	roms.adsp = take("adsp");
	roms.texture.assign(slots * 0x400000, 0);
	for (unsigned slot = 0; slot < slots; slot++) {
		const std::span<const uint8_t> bytes = game.section("texture" + std::to_string(slot));

		std::copy_n(bytes.begin(), std::min<size_t>(bytes.size(), 0x400000), roms.texture.begin() + slot * 0x400000);
	}
	roms.texmask = take("texmask");
	roms.texmask.resize(std::max<size_t>(roms.texmask.size(), 0x80000), 0);
	roms.boot = take("boot");
	roms.pic = take("pic");
	roms.kick_sensors = game.machine == "footbpow";
	roms.m68000 = game.machine == "speedup" || game.machine == "surfplnt";
	roms.speed_up = game.machine == "speedup";
	roms.surf_planet = game.machine == "surfplnt";
	return roms;
}

Gaelco3dGame::Gaelco3dGame(const Game &game, Controls &controls, Display &display, const std::string &kept,
                           bool verbose)
	: m_name(game.name), m_kept(kept), m_board(std::make_unique<Gaelco3d>(roms_of(game))), m_controls(controls),
	  m_display(display), m_wiring(game.controls), m_grounded(game.grounded_pins),
	  m_speedup(game.machine == "speedup"), m_wheel_motor(game.wheel_motor), m_balls(ball_sticks(game.controls)),
	  m_texture_rows(texture_slots(game) * (0x400000 / 4096))
{
	m_board->set_name(m_name);
	m_board->set_verbose(verbose);
	if (!m_kept.empty())
		m_board->load_kept(m_kept);
	m_board->reset();
}

Gaelco3dGame::~Gaelco3dGame()
{
	if (!m_kept.empty())
		m_board->save_kept(m_kept);
}

void
Gaelco3dGame::read_controls()
{
	CabinetInputs in{};
	std::array<uint16_t, 4> switches{};
	std::array<uint8_t, 4> analog{ 0x80, 0x80, 0x80, 0x80 };
	const auto close = [&](PortSwitch pin) {
		switches[pin.group] = static_cast<uint16_t>(switches[pin.group] | 1u << pin.bit);
	};

	m_controls.read_inputs(in);
	if (in.jvs_coin[0])
		m_coin = COIN_FRAMES;
	if (m_coin > 0) {
		close(COIN_PIN);
		m_coin--;
	}
	if (in.jvs_system & 0x80)
		close(m_speedup ? SPEEDUP_TEST_PIN : TEST_PIN);
	if (in.jvs_player[0] & SERVICE)
		close(m_speedup ? SPEEDUP_SERVICE_PIN : SERVICE_PIN);
	for (const GameControl &control : m_wiring) {
		if (control.port && in.jvs_player[control.button.player & 1] & control.button.mask)
			close(*control.port);
		for (const GameControl::Position &position : control.positions)
			for (const JvsSwitch &sw : position.switches)
				if (control.port && in.jvs_player[sw.player & 1] & sw.mask)
					close(*control.port);
		if (control.span)
			analog[control.analog] = adc_value(*control.span, in.jvs_analog[control.analog]);
	}
	for (const PortSwitch pin : m_grounded)
		close(pin);
	for (unsigned player = 0; player < 2; player++)
		if (m_balls[player])
			m_kick[player].read(in, *m_balls[player]);
	/* HEURISTIC: a kick holds the accelerometers at these levels while it
	 * lasts; the PIC sends the peaks of the side ones and twice the mean of
	 * the middle one over its 32 samples (pm 0x00DA-0x0144) */
	const auto levels = [](const BallKick::Sensors &sensors) {
		return std::array<uint8_t, 3>{ static_cast<uint8_t>(0x80 + (sensors[0] - 0x80) / 2), sensors[1], sensors[2] };
	};

	m_board->set_ball_sensors({ levels(m_kick[0].sensors()), levels(m_kick[1].sensors()) });
	m_board->set_switches(switches);
	m_board->set_analog(analog);
}

/* Speed Up's gamepad, from what its program does to the wheel
 * (preuves/ffb-manette-etude-20261118): the light motor shakes with the
 * effects beyond the buzz while no hit plays (its skids), the heavy one with
 * them while one plays (a wall's push), and thuds when the program takes the
 * spring back after letting go of it: after a hit (a car's: one frame; a
 * wall's: six), or as the car lands (as long as it flew). Both die away as a
 * shake does. HEURISTIC: the strengths (HIT_THUD...) and which motor each
 * goes to, the heavy for blows, the light for the road (the owner's choice) */
void
Gaelco3dGame::shake_pad()
{
	const std::optional<Gaelco3d::MotorParts> parts = m_board->motor_parts();

	if (!m_wheel_motor || !parts) {
		m_pad.reset();
		return;
	}

	const auto level = [](double share) { return static_cast<uint16_t>(std::lround(std::min(1.0, share) * 0xFFFF)); };
	const int felt = std::max(std::abs(m_board->motor() - std::clamp(parts->spring, -128, 127)) - BUZZ_MOST, 0);
	const uint16_t effects = level(2.0 * felt / MOTOR_FULL);
	uint16_t thud = 0;

	if (!parts->spring_on) {
		if (m_let_go++ == 0)
			m_let_go_hit = parts->hit;
	} else if (m_let_go) {
		if (m_let_go_hit)
			thud = level(HIT_THUD);
		else if (m_let_go <= LANDING_LONGEST)
			thud = level(LANDING_LEAST + m_let_go / LANDING_FRAMES);
		m_let_go = 0;
	}

	PadShake pad = m_pad.value_or(PadShake{});
	const uint16_t heavy = std::max(thud, parts->hit ? effects : uint16_t{ 0 });
	const uint16_t light = parts->hit ? uint16_t{ 0 } : effects;

	pad.heavy = std::max(heavy, AudioOut::smoothed(pad.heavy, heavy));
	pad.light = std::max(light, AudioOut::smoothed(pad.light, light));
	m_pad = pad;
}

void
Gaelco3dGame::run_frame()
{
	read_controls();
	/* Above the board's resolution, the GPU draws each field (note 124);
	 * on a card whose textures cannot hold the texture ROMs, the board
	 * draws its own picture, as at 1x, and the user is told once */
	const bool fits = m_texture_rows <= m_display.largest_texture();
	const bool fine = m_display.scale() > 1 && fits;

	if (m_display.scale() > 1 && !fits && !m_too_large) {
		std::fprintf(stderr, "renderer: the graphics card's textures are too small for the Gaelco 3D's "
		                     "(%u rows): its own picture is shown, as at 1x\n", m_texture_rows);
		m_too_large = true;
	}

	m_board->set_one_picture(!m_display.board_fields());
	m_board->set_gpu(fine, m_display.upscale_mode() == DisplayUpscaleMode::NATIVE, m_display.wide_edges());
	m_board->run_frame();
	m_ranges.follow(*m_board);
	shake_pad();
	if (m_speedup) {
		const std::vector<int16_t> &seat = m_board->option_sound(), &woofer = m_board->woofer_sound();

		m_seat.clear();
		for (size_t i = 0; i < woofer.size() && 2 * i + 1 < seat.size(); i++) {
			const int low = m_woofer_low(woofer[i]);

			for (unsigned side = 0; side < 2; side++)
				m_seat.push_back(static_cast<int16_t>(std::clamp(seat[2 * i + side] + low, -32768, 32767)));
		}
	}
	m_outputs.fill(0);
	m_levels.fill(0);
	for (unsigned bit = 0; bit < 8; bit++)
		if (m_board->outputs() & 1u << bit) {
			m_outputs[0] = static_cast<uint8_t>(m_outputs[0] | 0x80u >> bit);
			m_levels[bit] = 255;
		}
	if (fine)
		m_display.set_scene(m_board->take_scene());
	else
		m_display.set_scene(picture_scene(m_board->picture(), Gaelco3d::WIDTH, Gaelco3d::HEIGHT));
}

} /* namespace */

Ended
play_gaelco3d(const Options &options, const Game &game, const GameList &games, Settings &settings,
              const std::string &settings_file, Display &display, Keyboard &keys, Pointers *pointers, Opened &opened)
{
	GaelcoPlay play(options, game, games, settings, settings_file, display, keys, pointers, opened,
	                static_cast<int>(Gaelco3d::SOUND_RATE));
	Gaelco3dGame running(game, play.controls, display,
	                     SaveFolder::of_game(game.name, options.saves).path("kept.bin"), options.verbose);

	if (game.seat_woofer())
		running.set_seat_woofer(play.sound.woofer_hz);
	play.attach(running.ranges());

	const NetLinkSettings link = link_settings(settings);

	if (game.link.cabinets && !running.set_link(link))
		std::fprintf(stderr, "ERROR: no link cable\n");
	/* Its cabinet number, as for ATV Track: cabinet 0 leaves it as it is */
	if (game.link.seat_in == Game::Link::In::EEPROM) {
		const bool off = link.board != NetLinkSettings::Board::RING;

		if (!off && link.cabinet > game.link.cabinets)
			std::fprintf(stderr, "ERROR: link: %s links %u cabinets at most: cabinet %u keeps its link type\n",
			             game.title.c_str(), game.link.cabinets, link.cabinet);
		else if ((off || link.cabinet) &&
		         !running.set_eeprom_setting(game.link.seat_byte, game.link.seat_shift, game.link.seat_bits,
		                                     static_cast<uint8_t>(off ? 0 : game.link.seat_first + link.cabinet - 1)) &&
		         !off)
			std::fprintf(stderr, "link: %s keeps no settings yet: it gets its cabinet number at its next start\n",
			             game.name.c_str());
	}
	return play.run(running, running.ranges());
}

} /* namespace odin */
