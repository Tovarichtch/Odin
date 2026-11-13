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

#include "odin/host/app/ball_kick.hpp"

#include <algorithm>
#include <cmath>

namespace odin {

namespace {

/* A kick on the right stick: once past KICK_FROM, its power is the peak
 * within KICK_RISE frames, and the stick must fall back under KICK_REST to
 * rearm. Pulling back first lifts the ball, until the stick has rested for
 * BACKSWING_FRAMES. HEURISTIC: all of these, and the ball's height below
 * (the sensors' PIC is not dumped). */
constexpr unsigned KICK_FRAMES = 3, KICK_RISE = 3, BACKSWING_FRAMES = 20;
constexpr float KICK_FROM = 0.3f, KICK_REST = 0.2f;
/* The program's kick power, up to 127: the root of the sum of both side
 * sensors squared, times the gain (0x8C02F5E0); Odin gives it 10 to 127.
 * HEURISTIC: a real foot's values are unknown. Up to 50 passes and past 80
 * clears the field, so the stick's travel is cubed. */
constexpr float KICK_SOFT = 10.0f, KICK_HARD = 127.0f, KICK_GAIN = 1.2f;

} /* namespace */

/* Forward sets the power, sideways the direction and pulling back the
 * height. The game reads the power from the side sensors, the height from
 * the middle one and the direction from right minus left. */
void
BallKick::read(float x, float forward)
{
	const float far = std::hypot(x, forward);

	m_back = std::max(m_back, -forward);
	m_rested = far < KICK_REST ? m_rested + 1 : 0;
	if (m_rested >= BACKSWING_FRAMES)
		m_back = 0.0f;

	if (m_frames > 0) {
		if (--m_frames == 0)
			m_sensors = m_rest;
	} else if (m_spent) {
		m_spent = forward > KICK_REST;
	} else if (forward > KICK_FROM) {
		if (forward >= m_forward) {
			m_forward = forward;
			m_x = x;
		}
		if (++m_rise >= KICK_RISE) {
			const float travel = std::clamp((std::hypot(m_x, m_forward) - KICK_FROM) / (1.0f - KICK_FROM), 0.0f, 1.0f);
			const float power = KICK_SOFT + (KICK_HARD - KICK_SOFT) * travel * travel * travel;
			const float force = power / KICK_GAIN / std::sqrt(2.0f * (1.0f + m_x * m_x));
			const auto side = [&](float turned) { return static_cast<uint8_t>(128.0f + force * (1.0f + turned)); };

			m_sensors = { static_cast<uint8_t>(std::min(255, m_rest[0] + static_cast<int>(m_back * m_lift))), side(-m_x),
				      side(m_x) };
			m_frames = KICK_FRAMES;
			m_spent = true;
			m_back = m_forward = 0.0f;
			m_rise = 0;
		}
	} else {
		m_forward = 0.0f;
		m_rise = 0;
	}
}

} /* namespace odin */
