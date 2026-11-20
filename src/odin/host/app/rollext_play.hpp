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

#ifndef ODIN_ROLLEXT_PLAY_HPP
#define ODIN_ROLLEXT_PLAY_HPP

#include "odin/host/app/games.hpp"
#include "odin/host/app/options.hpp"
#include "odin/host/app/session.hpp"

namespace odin {

/* Runs Rolling Extreme, Gaelco's TMS320C82 board, in Odin's window with
 * the Hikaru menu and bar: its 3D chip's picture, its four channels of
 * sound, the controls and outputs of its cabinet, its handlebar's motor
 * and its EEPROM kept between two sessions. */
Ended play_rollext(const Options &options, const Game &game, const GameList &games, Settings &settings,
                   const std::string &settings_file, Display &display, Keyboard &keys, Pointers *pointers,
                   Opened &opened);

} /* namespace odin */

#endif
