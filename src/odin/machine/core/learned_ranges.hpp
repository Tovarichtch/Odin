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

#ifndef ODIN_LEARNED_RANGES_HPP
#define ODIN_LEARNED_RANGES_HPP

#include <algorithm>
#include <array>

namespace odin {

/* The ranges a game learns for its analog channels, the lowest and the
 * highest reading of each, as Speed Up, ATV Track and Smashing Drive keep
 * them in RAM from power-on: their cabinets learn them again each time
 * they are switched on. Odin can keep them from one start to the next
 * ([players.<game>] keep_calibration, experimental). A range whose low end
 * is above its high one has learned nothing yet */
struct LearnedRange {
	int low = 1, high = 0;

	bool learned() const { return low <= high; }
};

using LearnedRanges = std::array<LearnedRange, 4>;

/* Each range of `a` widened to cover that of `b`, as the game would have
 * learned it: its rule only ever widens */
inline LearnedRanges
widened(LearnedRanges a, const LearnedRanges &b)
{
	for (size_t i = 0; i < a.size(); i++) {
		if (!b[i].learned())
			continue;
		if (!a[i].learned())
			a[i] = b[i];
		a[i].low = std::min(a[i].low, b[i].low);
		a[i].high = std::max(a[i].high, b[i].high);
	}
	return a;
}

} /* namespace odin */

#endif
