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

#ifndef ODIN_GAELCO_PLAY_HPP
#define ODIN_GAELCO_PLAY_HPP

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "odin/host/app/games.hpp"
#include "odin/host/app/options.hpp"
#include "odin/host/app/session.hpp"
#include "odin/host/audio/audio_out.hpp"
#include "odin/host/input/controls.hpp"
#include "odin/host/input/outputs_network.hpp"
#include "odin/machine/boards/board.hpp"

namespace odin {

/* What the games of Gaelco's boards share on the host's side
 * (atvtrack_play.cpp, gaelco3d_play.cpp) */

class GaelcoLink;

/* The value of a channel of the ADC for a channel of CabinetInputs, from
 * span[0] at the low end of the control to span[1] at its high end */
uint8_t adc_value(const std::array<uint8_t, 2> &span, uint16_t channel);
/* The board's link cable as the page Link shows it: `link`, none without
 * the cable; `failed`, its port could not be opened */
Board::LinkStatus link_status_of(const GaelcoLink *link, bool failed);

/* A game of a Gaelco board in Odin's window, with the Hikaru menu, bar and
 * controls: what it plays with around its board, then the board the caller
 * makes, attached to the window and run */
class GaelcoPlay {
public:
	/* The controls as [players.<game>] sets them, the outputs sent to other
	 * programs, and the sound at the board's `rate` */
	GaelcoPlay(const Options &options, const Game &game, const GameList &games, Settings &settings,
	           const std::string &settings_file, Display &display, Keyboard &keys, Pointers *pointers, Opened &opened,
	           int rate);

	/* The window's scenes for the game */
	void attach();
	/* The game from its state (-s) or its start, until it ends; then the
	 * window let go */
	Ended run(Board &running);

private:
	const Options &m_options;
	const Game &m_game;
	const GameList &m_games;
	Settings &m_settings;
	const std::string &m_settings_file;
	Display &m_display;
	Keyboard &m_keys;
	Opened &m_opened;

public:
	/* Made in this order */
	Controls controls;
	bool network = false;
	std::unique_ptr<OutputsNetwork> outputs;
	Options sound_options;
	GameSound sound;
	std::unique_ptr<AudioOut> audio;
};

} /* namespace odin */

#endif
