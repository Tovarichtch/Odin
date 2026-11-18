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

#ifndef ODIN_KEPT_RANGES_HPP
#define ODIN_KEPT_RANGES_HPP

#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include "odin/host/app/user_files.hpp"
#include "odin/machine/core/learned_ranges.hpp"

namespace odin {

/* The ranges a game learned, kept in its save folder for its next start
 * ([players.<game>] keep_calibration): the low and high ends of each
 * channel, 32 bits each */
inline constexpr char KEPT_RANGES[] = "calibration.bin";

inline std::optional<LearnedRanges>
read_kept_ranges(const std::string &path)
{
	if (path.empty())
		return std::nullopt;

	const std::optional<std::vector<uint8_t>> bytes = read_kept(path, sizeof(int32_t) * 2 * LearnedRanges{}.size());

	if (!bytes)
		return std::nullopt;

	LearnedRanges ranges;

	for (size_t i = 0; i < ranges.size(); i++) {
		int32_t ends[2];

		std::memcpy(ends, bytes->data() + sizeof(ends) * i, sizeof(ends));
		ranges[i] = { ends[0], ends[1] };
	}
	return ranges;
}

inline bool
write_kept_ranges(const std::string &path, const LearnedRanges &ranges)
{
	std::vector<uint8_t> bytes(sizeof(int32_t) * 2 * ranges.size());

	for (size_t i = 0; i < ranges.size(); i++) {
		const int32_t ends[2] = { ranges[i].low, ranges[i].high };

		std::memcpy(bytes.data() + sizeof(ends) * i, ends, sizeof(ends));
	}
	return !path.empty() && write_kept(path, bytes);
}

/* The ranges a board's game learns (AtvTrack, Gaelco3d), widened each frame
 * to those kept from its last start ([players.<game>] keep_calibration,
 * experimental), and those to keep for the next one */
class KeptRanges {
public:
	void restore(const std::string &path) { m_kept = read_kept_ranges(path); }
	/* After each frame of the board */
	template <typename LearningBoard>
	void follow(LearningBoard &board)
	{
		if (m_kept)
			board.widen_ranges(*m_kept);
		if (const std::optional<LearnedRanges> learned = board.learned_ranges())
			m_learned = learned;
	}
	void save(const std::string &path) const
	{
		if (m_learned)
			write_kept_ranges(path, m_kept ? widened(*m_kept, *m_learned) : *m_learned);
	}

private:
	std::optional<LearnedRanges> m_kept, m_learned;
};

} /* namespace odin */

#endif
