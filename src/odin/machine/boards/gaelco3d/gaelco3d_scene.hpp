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

#ifndef ODIN_GAELCO3D_SCENE_HPP
#define ODIN_GAELCO3D_SCENE_HPP

#include <cstdint>
#include <memory>
#include <vector>

namespace odin {

/* A field of the Gaelco 3D boards, drawn by the host GPU above the board's
 * resolution (scale 2 and up). The GPU does the board's two steps: each
 * pixel stores a texel address, a palette bank and two fractions, computed
 * from its polygon's terms; then the texture ROMs, the palette and CUS-COL's
 * filter make the colours (gaelco3d_draw.frag, gaelco3d_show.frag) */

/* The texture ROMs as one array of 4096-texel rows (pages of 1024 rows),
 * and the mask, one byte per texel of the first page (1 for transparent) */
struct Gaelco3dRoms {
	std::vector<uint8_t> texture, mask;
	uint32_t texture_end = 0, mask_off = 0;
};

struct Gaelco3dVertex {
	float x, y;		/* in the board's pixels from its top left corner */
	float k[10];		/* the polygon's terms K0-K9 */
	uint32_t texture;	/* W11 */
	uint32_t bank;		/* W10 bits 0-6; NATIVE: drawn on the board's pixels */
	static constexpr uint32_t NATIVE = 0x100;
};

/* Polygons drawn the same way, three vertices per triangle: those of bank
 * 0x7F (kept apart, half of what is seen), and those whose depth always
 * passes */
struct Gaelco3dRun {
	uint32_t first, count;
	bool see_through, always;
};

struct Gaelco3dScene {
	std::vector<Gaelco3dVertex> vertices;
	std::vector<Gaelco3dRun> runs;
	std::shared_ptr<const Gaelco3dRoms> roms;
	/* The 68000's palette words, when they have changed */
	std::vector<uint16_t> palette;
	/* Each field drawn on the lines of its own parity only (both fields
	 * kept), and that parity */
	bool both_fields = false;
	unsigned field = 0;
};

} /* namespace odin */

#endif
