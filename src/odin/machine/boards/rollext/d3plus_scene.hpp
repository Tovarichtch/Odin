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

#ifndef ODIN_D3PLUS_SCENE_HPP
#define ODIN_D3PLUS_SCENE_HPP

#include <cstdint>
#include <memory>
#include <vector>

namespace odin {

/* A picture of Rolling Extreme's 3D chip (D3PLUS-4), drawn by the host GPU
 * above the board's resolution (scale 2 and up): its packets in the order
 * the chip took them, each point worked out as the chip's drawing does it
 * at 1x (d3plus.cpp, draw), at its own finer place (d3plus_draw.frag) */

struct D3plusVertex {
	/* A corner, in the board's coordinates: x to the right, y up, (0, 0)
	 * at the centre (d3plus.hpp) */
	float x, y;
	/* The planes Q, U and V (a x + b y + c), and the depth w4 >> 7 */
	float q[3], depth;
	float u[3], v[3];
	/* The texture's origin: X0, and Y0 << 11 */
	uint32_t texture;
	/* The bank (bits 0-6), L (8-11), the piece's palette (12-14), NATIVE */
	uint32_t mode;
	/* The clipping rectangle: x from, x to, y from, y to; NATIVE: the
	 * points of the board the piece covers there */
	int16_t rect[4];
	static constexpr uint32_t NATIVE = 1u << 16;
};

/* Vertices drawn alike, three per triangle: the translucent ones (L not 0)
 * leave the depth as it is; the untested ones (bit 29) skip the depth test */
struct D3plusRun {
	uint32_t first, count;
	bool translucent, tested;
};

struct D3plusScene {
	/* The most pieces a picture has, the palette written between two of
	 * its packets */
	static constexpr unsigned PALETTES = 8;
	/* The texture sheet's 2048 x 16384 texels as 4096 x 8192: texel (x, y)
	 * at (x + 2048 (y & 1), y >> 1), the same bytes */
	static constexpr unsigned SHEET_W = 4096, SHEET_H = 8192;

	std::vector<D3plusVertex> vertices;
	std::vector<D3plusRun> runs;
	std::shared_ptr<const std::vector<uint8_t>> sheet;
	/* The table of 0x90000000, 256 bytes a texel row */
	std::shared_ptr<const std::vector<uint8_t>> table;
	/* The palette of each piece, 32,768 words */
	std::vector<std::shared_ptr<const std::vector<uint16_t>>> palettes;
};

} /* namespace odin */

#endif
