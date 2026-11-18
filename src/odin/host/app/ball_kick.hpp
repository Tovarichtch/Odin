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

#ifndef ODIN_BALL_KICK_HPP
#define ODIN_BALL_KICK_HPP

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace odin {

struct CabinetInputs;
struct GameControl;

/* The channels of CabinetInputs of the stick that kicks each ball, side
 * then forward, from the game's controls; none for a game without balls */
using BallSticks = std::array<std::optional<std::array<uint8_t, 2>>, 2>;
BallSticks ball_sticks(const std::vector<GameControl> &controls);

/* A Gaelco ball cabinet's kick, played on a stick (Gaelco Football,
 * Football Power): the middle, left and right sensors under the ball */
class BallKick {
public:
	using Sensors = std::array<uint8_t, 3>;

	/* `soft`: the power of the softest kick */
	BallKick(Sensors rest, float lift, float soft = 10.0f) : m_rest(rest), m_lift(lift), m_soft(soft), m_sensors(rest) {}

	/* The stick, -1 to 1: x to the right, forward away from the player */
	void read(float x, float forward);
	/* The same from the channels of that stick (BallSticks) */
	void read(const CabinetInputs &in, const std::array<uint8_t, 2> &stick);
	const Sensors &sensors() const { return m_sensors; }
	bool kicked() const { return m_frames > 0; }

private:
	Sensors m_rest;
	float m_lift, m_soft;
	float m_x = 0.0f, m_forward = 0.0f, m_back = 0.0f;
	unsigned m_rise = 0, m_frames = 0, m_rested = 0;
	bool m_spent = false;
	Sensors m_sensors;
};

} /* namespace odin */

#endif
