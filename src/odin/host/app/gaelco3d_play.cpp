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
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "odin/host/app/ball_kick.hpp"
#include "odin/host/app/options.hpp"
#include "odin/host/app/user_files.hpp"
#include "odin/host/audio/audio_out.hpp"
#include "odin/host/input/controls.hpp"
#include "odin/host/input/outputs_network.hpp"
#include "odin/machine/boards/atvtrack/atvtrack.hpp"
#include "odin/machine/boards/gaelco3d/gaelco3d.hpp"
#include "odin/machine/cabinet/drive_board.hpp"

namespace odin {
namespace {
constexpr PortSwitch COIN_PIN{ 2, 0 }, SERVICE_PIN{ 2, 2 }, TEST_PIN{ 2, 3 };
constexpr PortSwitch SPEEDUP_COIN2_PIN{ 3, 8 }, SPEEDUP_TEST_PIN{ 3, 9 };

constexpr uint16_t SERVICE = 0x4000;
constexpr unsigned COIN_FRAMES = 6;	/* HEURISTIC: the length of a coin pulse */
constexpr float BALL_LIFT = 120.0f;	/* HEURISTIC */

class Gaelco3dGame final : public Board {
public:
	Gaelco3dGame(const Game &game, Controls &controls, Display &display, const std::string &kept, bool verbose);
	~Gaelco3dGame() override;

	const std::string &game_name() const override { return m_name; }
	unsigned frame() const override { return m_board->frame(); }
	void run_frame() override;
	bool restart_asked() const override { return false; }
	bool save_state(const std::string &path) override { return m_board->save_state(path); }
	bool load_state(const std::string &path) override { return m_board->load_state(path); }
	std::span<const uint8_t> io_outputs() const override { return m_outputs; }
	std::span<const uint8_t> output_levels() const override { return m_levels; }
	const std::vector<int16_t> &sound(bool optional) const override
	{
		return optional ? (m_speedup ? m_board->rear_sound() : m_no_sound) : m_board->sound();
	}
	std::optional<int> drive_torque() const override
	{
		if (!m_wheel_motor)
			return std::nullopt;
		return -m_board->motor() * DriveBoard::MOST / 128;
	}
	bool set_link(const NetLinkSettings &link) { return m_board->set_link(link); }
	bool set_eeprom_setting(uint32_t byte, unsigned shift, unsigned bits, uint8_t value)
	{
		return m_board->set_eeprom_setting(byte, shift, bits, value);
	}
	LinkStatus link_status() const override
	{
		const GaelcoLink *const link = m_board->link();
		LinkStatus status;

		status.failed = m_board->link_failed();
		status.board = link || status.failed;
		status.linked = link != nullptr;
		status.previous = link && link->previous_linked();
		status.next = link && link->next_linked();
		return status;
	}

private:
	void read_controls();

	std::string m_name, m_kept;
	std::unique_ptr<Gaelco3d> m_board;
	Controls &m_controls;
	Display &m_display;
	std::vector<GameControl> m_wiring;
	std::vector<PortSwitch> m_grounded;
	bool m_speedup, m_wheel_motor;
	std::array<std::optional<std::array<uint8_t, 2>>, 2> m_balls;
	BallKick m_kick[2]{ { Gaelco3d::BALL_AT_REST, BALL_LIFT }, { Gaelco3d::BALL_AT_REST, BALL_LIFT } };
	std::array<unsigned, 2> m_coin{};

	std::array<uint8_t, 1> m_outputs{};
	std::array<uint8_t, 8> m_levels{};
	std::vector<int16_t> m_no_sound;
};

Gaelco3d::Roms
roms_of(const Game &game)
{
	Gaelco3d::Roms roms;
	const auto take = [&game](const char *section) {
		const std::span<const uint8_t> bytes = game.section(section);

		return std::vector<uint8_t>(bytes.begin(), bytes.end());
	};

	const unsigned lanes = game.section("program2").empty() ? 2 : 4;
	unsigned slots = 4;

	roms.program.assign(0x200000, 0);
	for (unsigned lane = 0; lane < lanes; lane++) {
		const std::span<const uint8_t> bytes = game.section("program" + std::to_string(lane));

		for (size_t i = 0; i < bytes.size() && i * lanes + lane < roms.program.size(); i++)
			roms.program[i * lanes + lane] = bytes[i];
	}
	roms.tms = take("tms");
	roms.adsp = take("adsp");
	for (unsigned slot = 4; slot < 8; slot++)
		if (!game.section("texture" + std::to_string(slot)).empty())
			slots = 8;
	roms.texture.assign(slots * 0x400000, 0);
	for (unsigned slot = 0; slot < slots; slot++) {
		const std::span<const uint8_t> bytes = game.section("texture" + std::to_string(slot));

		std::copy_n(bytes.begin(), std::min<size_t>(bytes.size(), 0x400000), roms.texture.begin() + slot * 0x400000);
	}
	roms.texmask = take("texmask");
	roms.texmask.resize(std::max<size_t>(roms.texmask.size(), 0x80000), 0);
	roms.boot = take("boot");
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
	  m_speedup(game.machine == "speedup"), m_wheel_motor(game.wheel_motor)
{
	for (const GameControl &control : m_wiring)
		if (control.ball != GameControl::Ball::NONE) {
			std::optional<std::array<uint8_t, 2>> &stick = m_balls[control.player];

			if (!stick)
				stick.emplace();
			(*stick)[control.ball == GameControl::Ball::FORWARD] = control.analog;
		}
	m_board->set_name(m_name);
	m_board->set_verbose(verbose);
	m_board->set_gpu(true);
	if (!m_kept.empty())
		m_board->load_kept(m_kept);
	m_board->reset();
}

Gaelco3dGame::~Gaelco3dGame()
{
	if (!m_kept.empty())
		m_board->save_kept(m_kept);
}

uint8_t
adc_value(const std::array<uint8_t, 2> &span, uint16_t channel)
{
	return static_cast<uint8_t>(std::lround(span[0] + (span[1] - span[0]) * (channel / 65535.0)));
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
	for (unsigned slot = 0; slot < (m_speedup ? 2u : 1u); slot++) {
		if (in.jvs_coin[slot])
			m_coin[slot] = COIN_FRAMES;
		if (m_coin[slot] > 0) {
			close(slot ? SPEEDUP_COIN2_PIN : COIN_PIN);
			m_coin[slot]--;
		}
	}
	if (in.jvs_system & 0x80)
		close(m_speedup ? SPEEDUP_TEST_PIN : TEST_PIN);
	if (in.jvs_player[0] & SERVICE && !m_speedup)
		close(SERVICE_PIN);
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
		if (m_balls[player]) {
			const auto axis = [&](unsigned half) {
				return (static_cast<float>(in.jvs_analog[(*m_balls[player])[half]]) - 32768.0f) / 32768.0f;
			};

			m_kick[player].read(axis(0), -axis(1));
		}
	m_board->set_ball_sensors({ m_kick[0].sensors(), m_kick[1].sensors() });
	m_board->set_switches(switches);
	m_board->set_analog(analog);
}

void
Gaelco3dGame::run_frame()
{
	read_controls();
	m_board->run_frame();
	m_outputs.fill(0);
	m_levels.fill(0);
	for (unsigned bit = 0; bit < 8; bit++)
		if (m_board->outputs() & 1u << bit) {
			m_outputs[0] = static_cast<uint8_t>(m_outputs[0] | 0x80u >> bit);
			m_levels[bit] = 255;
		}
	if (PmxScene scene = m_board->take_scene(); scene.rendered)
		m_display.set_scene(std::move(scene));
}
}

Ended
play_gaelco3d(const Options &options, const Game &game, const GameList &games, Settings &settings,
              const std::string &settings_file, Display &display, Keyboard &keys, Pointers *pointers, Opened &opened)
{
	Controls controls(keys, game.controls, game.outputs, settings, game.name);

	controls.set_pointers(pointers);
	controls.set_wheel_rotation(wheel_rotation_of(settings, game.name));
	controls.feel_game(game, vibration_on(settings, game.name));
	controls.set_wheel_force(wheel_force_of(settings, game.name));
	controls.set_vibration_force(vibration_force_of(settings, game.name));

	const bool network = network_on(settings);
	std::unique_ptr<OutputsNetwork> outputs = network ? OutputsNetwork::open(game.name, game.outputs) : nullptr;
	Options sound_options = options;
	const GameSound sound = game_sound(sound_options, game, settings);
	std::unique_ptr<AudioOut> audio;

	if (sound_options.sound && options.unthrottled && options.sound_asked)
		std::fprintf(stderr, "sound: not with -u\n");
	else if (sound_options.sound && !options.unthrottled)
		audio = AudioOut::make(*sound_options.sound, sound.level, options.sound_device,
		                       static_cast<int>(Gaelco3d::SOUND_RATE));
	if (audio && options.volume != 100)
		audio->set_volume(static_cast<float>(options.volume) / 100.0f);

	Gaelco3dGame running(game, controls, display, SaveFolder::of_game(game.name, options.saves).path("kept.bin"),
	                     options.verbose);

	display.attach_scenes(game.name, game.title);

	bool started = true;

	const NetLinkSettings link = link_settings(settings);

	if (game.link.cabinets && !running.set_link(link))
		std::fprintf(stderr, "ERROR: no link cable\n");
	if (game.link.seat_in == Game::Link::In::EEPROM) {
		const bool off = link.board != NetLinkSettings::Board::RING;

		if (!off && link.cabinet > game.link.cabinets)
			std::fprintf(stderr, "ERROR: link: %s links %u cabinets at most: cabinet %u keeps its link type\n",
			             game.title.c_str(), game.link.cabinets, link.cabinet);
		else if (!running.set_eeprom_setting(game.link.seat_byte, game.link.seat_shift, game.link.seat_bits,
		                                     static_cast<uint8_t>(off ? 0 : game.link.seat_first + link.cabinet - 1)) &&
		         !off)
			std::fprintf(stderr, "link: %s keeps no settings yet: %s\n", game.name.c_str(),
			             game.machine == "speedup" ? "it takes its cabinet when it next starts"
			                                       : "set its link type once in its test menu");
	}

	if (!options.state_path.empty()) {
		started = running.load_state(options.state_path);
		if (started)
			std::printf("loaded state '%s', frame %u\n", options.state_path.c_str(), running.frame());
	}

	Ended ended = Ended::FAILED;

	if (started) {
		Session session(sound_options, game, running, nullptr, display, keys, controls, audio.get(),
		                std::move(outputs), network, settings, settings_file, games);

		std::printf("Running\n");
		ended = session.run();
		opened = session.opened();
	}
	std::printf("Finalizing\n");
	display.detach();
	return ended;
}
}
