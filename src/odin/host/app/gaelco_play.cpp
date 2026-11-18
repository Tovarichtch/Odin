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

#include "odin/host/app/gaelco_play.hpp"

#include <cmath>
#include <cstdio>

#include "odin/host/app/user_files.hpp"
#include "odin/machine/boards/atvtrack/atvtrack.hpp"

namespace odin {

uint8_t
adc_value(const std::array<uint8_t, 2> &span, uint16_t channel)
{
	return static_cast<uint8_t>(std::lround(span[0] + (span[1] - span[0]) * (channel / 65535.0)));
}

Board::LinkStatus
link_status_of(const GaelcoLink *link, bool failed)
{
	Board::LinkStatus status;

	status.failed = failed;
	status.board = link || status.failed;
	status.linked = link != nullptr;
	status.previous = link && link->previous_linked();
	status.next = link && link->next_linked();
	return status;
}

GaelcoPlay::GaelcoPlay(const Options &options, const Game &game, const GameList &games, Settings &settings,
                       const std::string &settings_file, Display &display, Keyboard &keys, Pointers *pointers,
                       Opened &opened, int rate)
	: m_options(options), m_game(game), m_games(games), m_settings(settings), m_settings_file(settings_file),
	  m_display(display), m_keys(keys), m_opened(opened),
	  m_kept_ranges(SaveFolder::of_game(game.name, options.saves).path(KEPT_RANGES)),
	  controls(keys, game.controls, game.outputs, settings, game.name), sound_options(options)
{
	controls.set_pointers(pointers);
	controls.set_wheel_rotation(wheel_rotation_of(settings, game.name));
	/* The pads shake and the wheel turns unless [players.<game>] says off */
	controls.feel_game(game, vibration_on(settings, game.name));
	controls.set_wheel_force(wheel_force_of(settings, game.name));
	controls.set_vibration_force(vibration_force_of(settings, game.name));

	network = network_on(settings);
	outputs = network ? OutputsNetwork::open(game.name, game.outputs) : nullptr;

	/* The board's rate, its woofer as game_sound says. None at unthrottled
	 * frames, which do not keep time (-u). */
	sound = game_sound(sound_options, game, settings);
	if (sound_options.sound && options.unthrottled && options.sound_asked)
		std::fprintf(stderr, "sound: not with -u\n");
	else if (sound_options.sound && !options.unthrottled)
		audio = AudioOut::make(*sound_options.sound, sound.level, options.sound_device, rate);
	if (audio && game.woofer)
		audio->set_woofer(sound.woofer_hz);
	if (audio && options.volume != 100)
		audio->set_volume(static_cast<float>(options.volume) / 100.0f);
}

void
GaelcoPlay::attach(KeptRanges &ranges)
{
	if (m_game.learns_range && keeps_calibration_of(m_settings, m_game.name))
		ranges.restore(m_kept_ranges);
	m_display.attach_scenes(m_game.name, m_game.title);
}

Ended
GaelcoPlay::run(Board &running, const KeptRanges &ranges)
{
	bool started = true;

	if (!m_options.state_path.empty()) {
		started = running.load_state(m_options.state_path);
		if (started)
			std::printf("loaded state '%s', frame %u\n", m_options.state_path.c_str(), running.frame());
	}

	Ended ended = Ended::FAILED;

	if (started) {
		Session session(sound_options, m_game, running, nullptr, m_display, m_keys, controls, audio.get(),
		                std::move(outputs), network, m_settings, m_settings_file, m_games);

		std::printf("Running\n");
		ended = session.run();
		m_opened = session.opened();
	}
	if (m_game.learns_range && keeps_calibration_of(m_settings, m_game.name))
		ranges.save(m_kept_ranges);
	std::printf("Finalizing\n");
	m_display.detach();
	return ended;
}

} /* namespace odin */
