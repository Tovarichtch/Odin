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

#include "odin/host/app/rollext_play.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "odin/common/env.hpp"
#include "odin/host/app/gaelco_play.hpp"
#include "odin/host/app/kept_ranges.hpp"
#include "odin/host/app/user_files.hpp"
#include "odin/host/audio/audio_out.hpp"
#include "odin/host/audio/dac_stage.hpp"
#include "odin/host/input/controls.hpp"
#include "odin/machine/boards/rollext/rollext.hpp"
#include "odin/machine/cabinet/drive_board.hpp"
#include "odin/machine/chips/pmx1/pmx1_scene.hpp"

namespace odin {

namespace {

/* The DACs' rate (Rollext::SOUND_DIVIDER, HEURISTIC) */
constexpr int SOUND_RATE = static_cast<int>(Rollext::SOUND_CLOCK_HZ / Rollext::SOUND_DIVIDER);

/* The four channels' analog stages on the board (Gaelco's schematic, sheet
 * INT_IO), each channel through its own, as
 * the cabinet's amplifiers get them. Left and right (IC12 to CANAL1-2:
 * R8 3.3 k || C8 1 nF, R7 56 k, C6 330 pF, R6 56 k, C4 680 pF, R4 56 k,
 * C7 100 pF): about 8 kHz. The woofer and the seat (IC11 to CANAL3-4: R5
 * 4.7 k || C5 1 nF, R3 56 k, C2 8.2 nF, R1 56 k, C1 15 nF, R2 56 k, C3
 * 2.2 nF): about 350 Hz, 3 dB louder (4.7 k against 3.3 k) */
constexpr DacStage::Parts FRONT{ 3300, 1e-9, 56e3, 330e-12, 56e3, 680e-12, 56e3, 100e-12 };
constexpr DacStage::Parts LOWS{ 4700, 1e-9, 56e3, 8.2e-9, 56e3, 15e-9, 56e3, 2.2e-9 };
constexpr double DAC_RATE = static_cast<double>(Rollext::SOUND_CLOCK_HZ) / Rollext::SOUND_DIVIDER;
/* Each channel into its TDA1552 (sheet AEREO IN/OUT, the interface board:
 * CANAL1 through R4 2.2 k, R3 2.2 k to ground, C2 220 nF; the same parts
 * for CANAL2-4) and its input's 60 k (Philips TDA1552Q data sheet, 1994:
 * typical, 50 to 75 k): a high-pass at about 12 Hz. The divider's 6 dB,
 * the same on all four, is left to Odin's volume */
constexpr double COUPLING = 220e-9 * (2200.0 * 2200.0 / (2200.0 + 2200.0) + 60e3);

/* The service panel (JP10, UR p. 34): COIN on 0xA0000006 bit 0, SERVICE
 * and TEST on 0xA0000000 bits 1 and 3 (Rollext::set_switches) */
constexpr PortSwitch COIN_PIN{ 3, 0 }, SERVICE_PIN{ 0, 1 }, TEST_PIN{ 0, 3 };
/* SERVICE of player 1 in CabinetInputs (control_panel.cpp) */
constexpr uint16_t SERVICE = 0x4000;
constexpr unsigned COIN_FRAMES = 6;	/* HEURISTIC: the length of a coin pulse */
/* The outputs of Rollext::outputs */
constexpr unsigned OUTPUTS = 4;
/* HEURISTIC, the gamepad (RollextGame::shake_pad): the jolts twice over
 * against the motor's whole byte, as Speed Up's effects against its full
 * force */
constexpr double JOLT_FULL = 127.0 / 2.0;

class RollextGame final : public Board {
public:
	RollextGame(const Game &game, Controls &controls, Display &display, const std::string &kept, bool verbose);
	~RollextGame() override;

	const std::string &game_name() const override { return m_name; }
	unsigned frame() const override { return m_board->frame(); }
	uint64_t frame_nanoseconds() const override { return Rollext::FRAME_NANOSECONDS; }
	void run_frame() override;
	bool restart_asked() const override { return false; }
	bool save_state(const std::string &path) override { return m_board->save_state(path); }
	bool load_state(const std::string &path) override { return m_board->load_state(path); }
	std::span<const uint8_t> io_outputs() const override { return m_outputs; }
	std::span<const uint8_t> output_levels() const override { return m_levels; }
	/* Left and right; the woofer's channel and the fourth: out of the
	 * board's analog stages */
	const std::vector<int16_t> &sound(bool optional) const override { return optional ? m_lows : m_front; }
	/* The handlebar's motor as a drive board's torque, the right positive
	 * (Rollext::motor: positive to the left) */
	std::optional<int> drive_torque() const override
	{
		if (!m_wheel_motor)
			return std::nullopt;
		const int most = DriveBoard::MOST;

		return std::clamp(-m_board->motor() * most / 127, -most, most);
	}
	std::optional<PadShake> pad_shake() const override { return m_pad; }
	bool set_eeprom_setting(uint32_t byte, uint8_t value, std::optional<uint8_t> only_from)
	{
		return m_board->set_eeprom_setting(byte, value, only_from);
	}
	KeptRanges &ranges() { return m_ranges; }

private:
	void read_controls();
	void shake_pad();
	void filter_sound();

	std::string m_name, m_kept;
	std::unique_ptr<Rollext> m_board;
	Controls &m_controls;
	Display &m_display;
	/* The controls of the cabinet played and the pins its wiring grounds
	 * (game list) */
	std::vector<GameControl> m_wiring;
	std::vector<PortSwitch> m_grounded;
	bool m_wheel_motor, m_verbose;
	unsigned m_coin = 0;
	/* For the tests: ODIN_ROLLEXT_OUTPUTS=1, a line each time the outputs
	 * or the motor change */
	bool m_show_outputs;
	unsigned m_shown_outputs = 0;
	int m_shown_motor = 0;
	std::array<uint8_t, 1> m_outputs{};
	std::array<uint8_t, OUTPUTS> m_levels{};
	KeptRanges m_ranges;
	/* The gamepad of the upright (shake_pad) */
	std::optional<PadShake> m_pad;
	/* Left, right, the woofer and the seat (filter_sound) */
	std::array<DacStage, 4> m_stages{ DacStage(FRONT, DAC_RATE, FRONT.r_iv, COUPLING),
		                          DacStage(FRONT, DAC_RATE, FRONT.r_iv, COUPLING),
		                          DacStage(LOWS, DAC_RATE, FRONT.r_iv, COUPLING),
		                          DacStage(LOWS, DAC_RATE, FRONT.r_iv, COUPLING) };
	std::vector<int16_t> m_front, m_lows;
};

Rollext::Roms
roms_of(const Game &game)
{
	const auto take = [&game](const char *section) {
		const std::span<const uint8_t> bytes = game.section(section);

		return std::vector<uint8_t>(bytes.begin(), bytes.end());
	};
	Rollext::Roms roms;

	roms.rom0 = take("rom0");
	roms.flash = take("flash");
	for (unsigned n = 0; n < roms.texture.size(); n++)
		roms.texture[n] = take(("texture" + std::to_string(n)).c_str());
	return roms;
}

RollextGame::RollextGame(const Game &game, Controls &controls, Display &display, const std::string &kept,
                         bool verbose)
	: m_name(game.name), m_kept(kept), m_board(std::make_unique<Rollext>(roms_of(game))), m_controls(controls),
	  m_display(display), m_wiring(game.controls), m_grounded(game.grounded_pins), m_wheel_motor(game.wheel_motor),
	  m_verbose(verbose), m_show_outputs(env_flag("ODIN_ROLLEXT_OUTPUTS"))
{
	m_board->set_name(m_name);
	m_board->set_verbose(verbose);
	if (!m_kept.empty())
		m_board->load_kept(m_kept);
	m_board->reset();
	if (const std::string dump = env_text("ODIN_D3PLUS_DUMP"); !dump.empty())
		m_board->d3plus().open_dump(dump);
	if (const std::string frames = env_text("ODIN_D3PLUS_FRAMES"); !frames.empty())
		m_board->d3plus().set_frames(frames);
	if (const std::string dump = env_text("ODIN_SOUND_DUMP"); !dump.empty())
		m_board->open_sound_dump(dump);
	/* No link cable yet: the board's link device would ask XPT1 and XPT3 */
	std::fprintf(stderr, "rollext: XPT1 and XPT3 of the link: not emulated yet, no request\n");
}

/* With -v, where the MP stands; with ODIN_DUMPS=1, the memories */
RollextGame::~RollextGame()
{
	if (m_verbose)
		std::printf("rollext: frame %u, MP at 0x%08X, PPERROR 0x%08X\n", m_board->frame(), m_board->chip().mp().ip(),
		            m_board->chip().pperror());
	if (env_flag("ODIN_DUMPS"))
		m_board->dump_memories();
	if (!m_kept.empty())
		m_board->save_kept(m_kept);
}

void
RollextGame::read_controls()
{
	CabinetInputs in{};
	std::array<uint16_t, 4> switches{};
	std::array<uint8_t, Rollext::ADCS> analog{ 0x80, 0x80 };
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
		close(TEST_PIN);
	if (in.jvs_player[0] & SERVICE)
		close(SERVICE_PIN);
	for (const GameControl &control : m_wiring) {
		if (control.port && in.jvs_player[control.button.player & 1] & control.button.mask)
			close(*control.port);
		if (control.span && size_t{ control.analog } < analog.size())
			analog[control.analog] = adc_value(*control.span, in.jvs_analog[control.analog]);
	}
	for (const PortSwitch pin : m_grounded)
		close(pin);
	m_board->set_switches(switches);
	m_board->set_analog(analog);
}

/* The gamepad, while the cabinet has the handlebar's motor: both its motors
 * as hard as the jolts the program adds to the spring (Rollext::motor_jolt),
 * which the handlebar would give the hands, dying away as a shake does; the
 * spring, which the player's own moves make, none */
void
RollextGame::shake_pad()
{
	if (!m_wheel_motor) {
		m_pad.reset();
		return;
	}

	const double share = std::min(1.0, std::abs(m_board->motor_jolt()) / JOLT_FULL);
	const auto level = static_cast<uint16_t>(std::lround(share * 0xFFFF));
	PadShake pad = m_pad.value_or(PadShake{});

	pad.heavy = std::max(level, AudioOut::smoothed(pad.heavy, level));
	pad.light = pad.heavy;
	m_pad = pad;
}

/* Each channel of the frame through its stage */
void
RollextGame::filter_sound()
{
	const auto through = [](const std::vector<int16_t> &in, DacStage &even, DacStage &odd, std::vector<int16_t> &out) {
		out.resize(in.size());
		for (size_t i = 0; i + 1 < in.size(); i += 2) {
			out[i] = even(in[i]);
			out[i + 1] = odd(in[i + 1]);
		}
	};

	through(m_board->sound(), m_stages[0], m_stages[1], m_front);
	through(m_board->option_sound(), m_stages[2], m_stages[3], m_lows);
}

void
RollextGame::run_frame()
{
	read_controls();
	m_board->run_frame();
	filter_sound();
	m_ranges.follow(*m_board);
	shake_pad();
	m_outputs.fill(0);
	m_levels.fill(0);
	for (unsigned bit = 0; bit < OUTPUTS; bit++)
		if (m_board->outputs() & 1u << bit) {
			m_outputs[0] = static_cast<uint8_t>(m_outputs[0] | 0x80u >> bit);
			m_levels[bit] = 255;
		}
	if (m_show_outputs && (m_board->outputs() != m_shown_outputs || m_board->motor() != m_shown_motor)) {
		m_shown_outputs = m_board->outputs();
		m_shown_motor = m_board->motor();
		std::printf("rollext: frame %u outputs %X motor %d jolt %.1f pad %04X\n", m_board->frame(), m_shown_outputs,
		            m_shown_motor, m_board->motor_jolt(), m_pad ? m_pad->heavy : 0);
	}
	m_display.set_scene(picture_scene(m_board->picture(), Rollext::WIDTH, Rollext::HEIGHT));
}

} /* namespace */

Ended
play_rollext(const Options &options, const Game &game, const GameList &games, Settings &settings,
             const std::string &settings_file, Display &display, Keyboard &keys, Pointers *pointers, Opened &opened)
{
	GaelcoPlay play(options, game, games, settings, settings_file, display, keys, pointers, opened, SOUND_RATE);
	RollextGame running(game, play.controls, display,
	                    SaveFolder::of_game(game.name, options.saves).path("kept.bin"), options.verbose);

	/* What a cabinet asks of the game's settings, as its operator sets them
	 * in the test menu (its cabinet mode): the cabinet played, its value;
	 * another one, the standard cabinet's value back, only from its own */
	for (const Game::Cabinet &model : game.cabinets) {
		if (!model.eeprom_setting)
			continue;

		const Game::Cabinet::Setting &setting = *model.eeprom_setting;
		const bool played = game.cabinet == model.id;

		if (!running.set_eeprom_setting(setting.byte, played ? setting.value : setting.standard,
		                                played ? std::nullopt : std::optional<uint8_t>(setting.value)) &&
		    played)
			std::fprintf(stderr, "cabinet: %s keeps no settings yet: it gets its %s cabinet at its next start\n",
			             game.name.c_str(), model.name.c_str());
	}
	play.attach(running.ranges());
	return play.run(running, running.ranges());
}

} /* namespace odin */
