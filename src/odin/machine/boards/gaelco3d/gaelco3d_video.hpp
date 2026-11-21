// license:BSD-3-Clause
// copyright-holders:Aaron Giles
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

#ifndef ODIN_GAELCO3D_VIDEO_HPP
#define ODIN_GAELCO3D_VIDEO_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "odin/machine/boards/gaelco3d/gaelco3d_scene.hpp"

namespace odin {
class StateFile;
class Worker;

/* The 3D of the Gaelco 3D boards, as Speed Up's schematic wires it (sheets 2,
 * 4 and 7). CUS-RASTER takes the DSP's polygons and walks their edges, one
 * field's lines at a time; CUS-GAEL4 gives the left pixel of each pair its
 * texel address, fractions and depth, and adders give the right pixel the
 * mean of its neighbours; the PAL IC88 gates the writes. The frame buffer
 * holds texel addresses, not colours: the texture ROMs, the palette and
 * CUS-COL's bilinear filter make the colours as the picture is scanned out. */
class Gaelco3dVideo {
public:
	static constexpr unsigned WIDTH = 576, HEIGHT = 432, PAIRS = WIDTH / 2;

	/* The texture ROMs in the order of the four parities (MAME's layout), and
	 * the texture mask ROMs */
	Gaelco3dVideo(const std::vector<uint8_t> &rom, std::vector<uint8_t> texmask);
	~Gaelco3dVideo();

	void reset();
	/* A word the DSP writes at 0xC00000 */
	void word(uint32_t data)
	{
		m_queued.push_back(data);
		if (m_queued.size() == BATCH || (m_queued.size() >= SOME && raster_idle()))
			send();
	}
	/* The DSP's IACK, just before the first word of its frame. HEURISTIC:
	 * a new polygon starts there */
	void frame_start() { m_queued.push_back(IACK); }
	/* A palette entry as the 68000 writes it */
	void set_palette(uint32_t index, uint16_t value);
	/* The field the screen shows now, H/L = NOT OH/L (the PAL IC88): the
	 * lines whose words or palette changed are scanned out again */
	void scan_out();
	/* RGBA bytes, WIDTH x HEIGHT, top line first */
	const std::vector<uint32_t> &picture() const { return m_picture; }
	/* The end of a field: the other field's lines are drawn from now on */
	void next_field()
	{
		settle();
		m_scene.field = m_field;
		m_field ^= 1;
		m_drawn = true;
	}
	/* Above the board's resolution (note 124): the polygons go to the host
	 * GPU as a scene, and the CPU draws none; with `native`, the flat ones
	 * are drawn on the board's pixels; with `wide`, each polygon is half a
	 * board pixel wider on every side, as the board's line ends make it on
	 * average. When the CPU draws again, the frame is the one from the last
	 * field it drew */
	void set_gpu(bool on, bool native, bool wide);
	/* The scene of the field drawn last (next_field) */
	Gaelco3dScene take_scene();
	/* What a flat screen shows: true, one picture per field (the older
	 * field only where it stood still); false, both fields as the board
	 * keeps them. The board itself does not depend on it */
	void set_one_picture(bool on) { m_one_picture = on; }

	bool save(StateFile &state);
	bool load(StateFile &state);

private:
	struct Plane;
	struct Texel;
	struct Edge {
		int64_t x, step;
		int high, low;
	};

	void polygon();
	void outline(const Plane &plane, const Edge *edges, unsigned count, int top, int bottom);
	static bool span(const Edge *edges, unsigned count, int y, int bottom, int &start, int &stop);
	void line(const Plane &plane, unsigned row, int y, int from, int to);
	Texel texel(const Plane &plane, float k23, float k89, float k56, int x) const;
	bool masked(uint32_t address) const;
	uint8_t texture(uint32_t address) const { return m_roms->texture[address & m_texture_end]; }

	/* Shared with the host GPU's scenes, which take the mask once one is
	 * drawn */
	std::shared_ptr<Gaelco3dRoms> m_roms;
	std::vector<uint8_t> m_texmask;
	uint32_t m_texture_end, m_mask_off;
	/* The palette, with each entry's red, green and blue 16 bits apart,
	 * and the words as written */
	std::vector<uint64_t> m_palette;
	std::vector<uint16_t> m_words_of_palette;

	/* 40 bits per pixel: texel address (0-24), palette bank (25-31), u and
	 * v fractions (32-35, 36-39) */
	std::vector<uint64_t> m_frame;
	/* One depth per pair of pixels */
	std::vector<uint16_t> m_depth;

	/* The colours of the frame's lines as last scanned out */
	std::vector<uint32_t> m_colours;
	std::vector<uint32_t> m_picture;
	std::array<bool, HEIGHT> m_line_changed;
	bool m_palette_changed = true;
	/* Each pixel's word when its field was last shown, and whether it
	 * differed from the time before (unknown after a reset or a load, so
	 * moved) */
	std::vector<uint64_t> m_seen;
	std::vector<uint8_t> m_moved;
	/* A field has been drawn since the last scan-out */
	bool m_drawn = false;
	bool m_one_picture = true;

	bool m_gpu = false, m_native = false, m_wide = false, m_gpu_palette_changed = true;
	Gaelco3dScene m_scene;

	void forget_motion();
	void colour(unsigned shown, bool drawn, size_t from, size_t to);
	void present(unsigned shown, size_t from, size_t to);

	static constexpr unsigned MOST_CORNERS = 32, MOST_WORDS = 13 + 2 * MOST_CORNERS;
	std::array<uint32_t, MOST_WORDS> m_words{};
	uint32_t m_count = 0;
	/* The parity of the lines drawn now (OH/L) */
	uint32_t m_field = 0;

	/* The raster draws on its own thread while the DSP computes, as
	 * CUS-RASTER draws behind its FIFO (the designer's talk, 1:41:06).
	 * Nothing goes back to the DSP, so the frame is the same; anything that
	 * reads the frame, and the end of a field, waits for the words sent.
	 * Words are sent as soon as the raster is idle, at least a few at a
	 * time, and the DSP waits for the raster once a batch is full */
	static constexpr size_t SOME = 64, BATCH = 4096;
	static constexpr uint64_t IACK = uint64_t{ 1 } << 32;
	void take(uint32_t data);
	bool raster_idle() const;
	void send();
	void settle();
	std::vector<uint64_t> m_queued, m_sent;
	/* The last member: its thread ends first */
	std::unique_ptr<Worker> m_raster;
};
}

#endif
