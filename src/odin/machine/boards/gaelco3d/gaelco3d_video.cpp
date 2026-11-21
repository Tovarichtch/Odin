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

/* Based on MAME's gaelco3d_v.cpp (BSD-3-Clause, Aaron Giles); those parts
 * keep their licence */

/* Sources: Speed Up's schematic (sheets 2, 4, 7) and the dump of its PAL
 * IC88 (m1_c_2af3); the C31 programs of the four games, which give the words
 * (Ghidra); the slides of the chips' designer, Javier Valero (RetroManía
 * 2026 talk). What none of them settles is marked HEURISTIC. */

#include "odin/machine/boards/gaelco3d/gaelco3d_video.hpp"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "odin/machine/core/state_file.hpp"
#include "odin/machine/core/worker.hpp"
#include "odin/machine/cpu/tms32031/tms32031.hpp"

namespace odin {
namespace {
constexpr unsigned HEADER = 12;
/* The texel address (ROMA0-24): u in bits 0-11, v in 12-21, the page in 22-24 */
constexpr uint32_t ADDRESS = 0x1FFFFFF, U_FIELD = 0xFFF, V_FIELD = 0x3FF000, PAGE = 0x1C00000;
/* The palette bank for which the PAL IC88 writes only one pixel of each pair */
constexpr uint32_t SEE_THROUGH = 0x7F;

/* HEURISTIC: the top line is Y = 215 and the left column X = -288. The C31
 * rounds the corners down and culls the view at 288 by 216 around the
 * centre; MAME's flip agrees */
constexpr int TOP = 215, LEFT = -288;

/* HEURISTIC: the chip keeps the slope words from bit 7 up, with 10 bits of
 * fraction (the designer's slide of the polygon buffer) */
constexpr unsigned SLOPE_DROP = 7, EDGE_FRACTION = 10;

/* In the last corner, bits 14 and 15 differ (the C31 programs) */
constexpr bool
last_corner(uint32_t word)
{
	return ((word ^ (word >> 1)) >> 14) & 1;
}

struct Corner {
	int x, y;
};

Corner
corner(uint32_t word)
{
	return { static_cast<int16_t>(word >> 16), static_cast<int32_t>(word << 18) >> 18 };
}

int32_t
saturated(float t)
{
	constexpr float MOST = 1073741824.0f;

	if (std::isnan(t))
		return 0;
	return static_cast<int32_t>(std::clamp(t, -MOST, MOST));
}

/* HEURISTIC: GAEL4 gives u and v rounded down to a 16th of a texel (4-bit
 * fractions, sheet 4); MAME rounds toward zero */
int32_t
sixteenths(float t)
{
	return saturated(std::floor(t * 16.0f));
}

/* HEURISTIC (MAME is the only source): the depth is rounded toward zero */
int32_t
whole(float t)
{
	return saturated(std::trunc(t));
}

/* HEURISTIC: the raster draws a line from its first pixel to its last, and a
 * polygon from its top line to its last (the designer's flowchart); both ends
 * of a line are the whole part of the edge, and so are the lines of the top and
 * bottom corners. Abutting polygons then overlap by one column, so each pair of
 * pixels on their seam is whole in one of them and gets its depth: a real Surf
 * Planet board shows no seams in its snow, where rounding the left end up
 * leaves dotted ones */
int
pixel(int64_t x)
{
	return static_cast<int>(x >> EDGE_FRACTION) - LEFT;
}
}

/* The calculator's terms (designer's slide of the raster): u = (K1 x + K2 y +
 * K3) / D, v = (K7 x + K8 y + K9) / D, depth = K0 / D, D = K4 x + K5 y + K6 */
struct Gaelco3dVideo::Plane {
	float k0, k1, k2, k3, k4, k5, k6, k7, k8, k9;
	uint32_t texture, bank;
};

struct Gaelco3dVideo::Texel {
	uint32_t address, fu, fv;
	int32_t depth;
	bool seen;
};

Gaelco3dVideo::Gaelco3dVideo(const std::vector<uint8_t> &rom, std::vector<uint8_t> texmask)
	: m_roms(std::make_shared<Gaelco3dRoms>()), m_texmask(std::move(texmask)),
	  m_texture_end(static_cast<uint32_t>(rom.size() - 1)), m_palette(32768), m_words_of_palette(32768),
	  m_frame(size_t{ WIDTH } * HEIGHT), m_depth(size_t{ PAIRS } * HEIGHT),
	  m_colours(size_t{ WIDTH } * HEIGHT, 0xFF000000u), m_picture(size_t{ WIDTH } * HEIGHT, 0xFF000000u),
	  m_seen(size_t{ WIDTH } * HEIGHT), m_moved(size_t{ WIDTH } * HEIGHT, 1), m_raster(std::make_unique<Worker>())
{
	m_queued.reserve(BATCH);
	m_sent.reserve(BATCH);
	m_line_changed.fill(true);
	/* Each ROM holds one parity of u and v (sheet 7, IC11-14), in MAME's
	 * order: odd u even v, odd u odd v, even u even v, even u odd v */
	const size_t quarter = rom.size() / 4;
	std::vector<uint8_t> &texels = m_roms->texture;

	texels.resize(rom.size());
	for (size_t row = 0; row < rom.size() / 4096; row += 2)
		for (size_t u = 0; u < 4096; u += 2) {
			const size_t from = row / 2 * 2048 + u / 2, at = row * 4096 + u;

			texels[at + 1] = rom[from];
			texels[at + 4097] = rom[quarter + from];
			texels[at] = rom[2 * quarter + from];
			texels[at + 4096] = rom[3 * quarter + from];
		}
	m_texmask.resize(std::max<size_t>(m_texmask.size(), 4 << 17), 0);
	/* HEURISTIC: with one bank of texture ROMs, address bit 24 reads the same
	 * bank again (MAME); on Speed Up's board no ROM answers there, an address
	 * its data never uses */
	m_mask_off = rom.size() > (size_t{ 1 } << 24) ? 0x1C00000 : 0xC00000;
	m_roms->texture_end = m_texture_end;
	m_roms->mask_off = m_mask_off;
}

Gaelco3dVideo::~Gaelco3dVideo() = default;

void
Gaelco3dVideo::reset()
{
	settle();
	m_count = 0;
	/* HEURISTIC: the even lines first */
	m_field = 0;
	forget_motion();
	m_scene = {};
}

void
Gaelco3dVideo::forget_motion()
{
	m_seen = m_frame;
	std::fill(m_moved.begin(), m_moved.end(), 1);
	m_drawn = false;
}

void
Gaelco3dVideo::set_palette(uint32_t index, uint16_t value)
{
	/* 15 bits reach CUS-COL (sheet 2); red, green, blue as in MAME */
	const uint64_t lanes = uint64_t{ value >> 10 & 31u } | uint64_t{ value >> 5 & 31u } << 16 | uint64_t{ value & 31u } << 32;

	m_palette_changed = m_palette_changed || m_palette[index] != lanes;
	m_gpu_palette_changed = m_gpu_palette_changed || m_words_of_palette[index] != value;
	m_palette[index] = lanes;
	m_words_of_palette[index] = value;
}

void
Gaelco3dVideo::send()
{
	if (m_queued.empty())
		return;
	m_raster->wait();
	std::swap(m_queued, m_sent);
	m_queued.clear();
	m_raster->post([this] {
		for (const uint64_t entry : m_sent)
			if (entry == IACK)
				m_count = 0;
			else
				take(static_cast<uint32_t>(entry));
	});
}

bool
Gaelco3dVideo::raster_idle() const
{
	return m_raster->idle();
}

void
Gaelco3dVideo::settle()
{
	send();
	m_raster->wait();
}

void
Gaelco3dVideo::take(uint32_t data)
{
	/* HEURISTIC: the words of a polygon that never ends are dropped */
	if (m_count == MOST_WORDS)
		m_count = 0;
	m_words[m_count++] = data;
	/* 12 words, then the first corner, then each corner with the slope of
	 * the edge that ends there (the C31 programs) */
	if (m_count > HEADER + 2 && (m_count & 1) && last_corner(m_words[m_count - 2])) {
		polygon();
		m_count = 0;
	}
}

void
Gaelco3dVideo::polygon()
{
	const auto k = [this](unsigned i) { return Tms32031::to_float(m_words[i]); };
	/* The order of the designer's slide: K0, K8, K7, K5, K4, K2, K1, K9, K6, K3;
	 * then the palette bank in bits 0-6 (IC86, IC87; the slide leaves bits
	 * 7-31 unused) and the texture's start */
	const Plane plane{ k(0), k(6), k(5), k(9), k(4), k(3), k(8), k(2), k(1), k(7), m_words[11], m_words[10] & 0x7F };

	/* Each edge starts at its upper corner; its slope is added at each line
	 * down */
	std::array<Edge, MOST_CORNERS> edges;
	unsigned count = 0;
	int top = INT_MIN, bottom = INT_MAX;
	Corner from = corner(m_words[HEADER]);

	for (unsigned at = HEADER + 1; at + 1 < m_count; at += 2) {
		const Corner to = corner(m_words[at]);

		if (from.y != to.y) {
			const Corner &upper = from.y > to.y ? from : to;
			const int low = std::min(from.y, to.y);

			edges[count++] = { int64_t{ upper.x } << EDGE_FRACTION,
				           static_cast<int32_t>(m_words[at + 1]) >> SLOPE_DROP, upper.y, low };
			top = std::max(top, upper.y);
			bottom = std::min(bottom, low);
		}
		from = to;
	}
	if (count < 2)
		return;
	if (m_gpu) {
		outline(plane, edges.data(), count, top, bottom);
		return;
	}

	const int first = std::max(TOP - top, 0), end = std::min(TOP - bottom + 1, static_cast<int>(HEIGHT));

	/* The lines of this field, two apart */
	for (int row = first + ((first ^ static_cast<int>(m_field)) & 1); row < end; row += 2) {
		const int y = TOP - row;
		int start, stop;

		if (span(edges.data(), count, y, bottom, start, stop))
			line(plane, static_cast<unsigned>(row), y, start, stop);
	}
}

/* The pixels the raster fills on line y: an edge counts from its top line
 * down to the line above its bottom corner, where the next edge takes over;
 * the polygon's bottom line comes from the edges that end on it */
bool
Gaelco3dVideo::span(const Edge *edges, unsigned count, int y, int bottom, int &start, int &stop)
{
	int64_t left = 0, right = 0;
	bool crossed = false;

	for (unsigned i = 0; i < count; i++) {
		const Edge &e = edges[i];

		if (y <= e.high && (y > e.low || (y == e.low && y == bottom))) {
			const int64_t x = e.x + (e.high - y) * e.step;

			left = crossed ? std::min(left, x) : x;
			right = crossed ? std::max(right, x) : x;
			crossed = true;
		}
	}
	start = std::clamp(pixel(left), 0, static_cast<int>(WIDTH));
	stop = std::clamp(pixel(right) + 1, 0, static_cast<int>(WIDTH));
	return crossed && start < stop;
}

/* Above the board's resolution: the outline the raster fills, each line from
 * the leftmost of its edges to the rightmost, with the edges stepped from their
 * upper corners by their slopes, as the raster steps them; cut into pieces
 * between two straight edges, so the GPU's own rule decides which of its pixels
 * a triangle covers. A flat polygon on the board's pixels (Native 2D): the
 * board's own spans, with lines of equal span joined */
void
Gaelco3dVideo::outline(const Plane &p, const Edge *edges, unsigned count, int top, int bottom)
{
	const bool native = m_native && p.k4 == 0.0f && p.k5 == 0.0f;
	Gaelco3dVertex corner{};
	const size_t first = m_scene.vertices.size();

	corner.k[0] = p.k0;
	corner.k[1] = p.k1;
	corner.k[2] = p.k2;
	corner.k[3] = p.k3;
	corner.k[4] = p.k4;
	corner.k[5] = p.k5;
	corner.k[6] = p.k6;
	corner.k[7] = p.k7;
	corner.k[8] = p.k8;
	corner.k[9] = p.k9;
	corner.texture = p.texture;
	corner.bank = p.bank | (native ? Gaelco3dVertex::NATIVE : 0);

	/* A piece from its upper line to its lower one, in the board's pixels */
	const auto piece = [&](double up, double up_left, double up_right, double down, double down_left,
	                       double down_right) {
		const auto put = [&](double x, double y) {
			corner.x = static_cast<float>(x);
			corner.y = static_cast<float>(y);
			m_scene.vertices.push_back(corner);
		};

		put(up_left, up);
		put(up_right, up);
		put(down_right, down);
		put(up_left, up);
		put(down_right, down);
		put(down_left, down);
	};

	if (native) {
		const int from = std::max(TOP - top, 0), end = std::min(TOP - bottom + 1, static_cast<int>(HEIGHT));
		int kept = -1, kept_start = 0, kept_stop = 0;
		const auto flush = [&](int row) {
			if (kept >= 0)
				piece(kept, kept_start, kept_stop, row, kept_start, kept_stop);
			kept = -1;
		};

		for (int row = from; row < end; row++) {
			int start, stop;

			if (!span(edges, count, TOP - row, bottom, start, stop)) {
				flush(row);
				continue;
			}
			if (kept >= 0 && start == kept_start && stop == kept_stop)
				continue;
			flush(row);
			kept = row;
			kept_start = start;
			kept_stop = stop;
		}
		flush(end);
	} else {
		/* Where the GPU samples the board's terms (gaelco3d_draw.frag): a
		 * board pixel takes them at its whole x and y, so x is half a
		 * pixel left of its middle */
		const auto across = [](double x) { return x - LEFT + 0.5; };
		const auto down = [](double y) { return TOP - y + 0.5; };
		const double grow = m_wide ? 0.5 : 0.0;
		const auto x_of = [](const Edge &e, double y) {
			return (static_cast<double>(e.x) + (e.high - y) * static_cast<double>(e.step)) / (1 << EDGE_FRACTION);
		};
		/* The leftmost and rightmost edges over the lines ya to yb */
		const auto ends = [&](double y, double ya, double yb, double &left, double &right) {
			bool crossed = false;

			for (unsigned i = 0; i < count; i++)
				if (edges[i].low <= ya && edges[i].high >= yb) {
					const double x = x_of(edges[i], y);

					left = crossed ? std::min(left, x) : x;
					right = crossed ? std::max(right, x) : x;
					crossed = true;
				}
			return crossed;
		};
		std::array<int, 2 * MOST_CORNERS> lines;
		unsigned n = 0;

		for (unsigned i = 0; i < count; i++) {
			lines[n++] = edges[i].high;
			lines[n++] = edges[i].low;
		}
		std::sort(lines.begin(), lines.begin() + n);
		n = static_cast<unsigned>(std::unique(lines.begin(), lines.begin() + n) - lines.begin());

		const double lowest = TOP - static_cast<int>(HEIGHT), highest = TOP + 1;

		for (unsigned k = 0; k + 1 < n; k++) {
			const int ya = lines[k], yb = lines[k + 1];
			const double a = std::max<double>(ya, lowest), b = std::min<double>(yb, highest);

			if (a >= b)
				continue;

			/* Where two of the edges cross, the leftmost or rightmost
			 * may change */
			std::array<double, MOST_CORNERS * MOST_CORNERS / 2 + 2> cuts;
			unsigned m = 0;

			cuts[m++] = a;
			for (unsigned i = 0; i < count; i++)
				for (unsigned j = i + 1; j < count; j++) {
					const Edge &e = edges[i], &f = edges[j];

					if (e.low > ya || e.high < yb || f.low > ya || f.high < yb)
						continue;

					const double ea = x_of(e, a), eb = x_of(e, b), fa = x_of(f, a), fb = x_of(f, b);

					if ((ea - fa) * (eb - fb) < 0.0)
						cuts[m++] = a + (b - a) * (ea - fa) / ((ea - fa) - (eb - fb));
				}
			cuts[m++] = b;
			std::sort(cuts.begin(), cuts.begin() + m);
			for (unsigned c = 0; c + 1 < m; c++) {
				const double y0 = cuts[c], y1 = cuts[c + 1];
				double l0, r0, l1, r1;

				if (y1 <= y0 || !ends(y0, ya, yb, l0, r0) || !ends(y1, ya, yb, l1, r1))
					continue;
				piece(down(y1), across(l1) - grow, across(r1) + grow, down(y0), across(l0) - grow,
				      across(r0) + grow);
			}
		}
		/* Wide: half a board line more above the top line and below the
		 * bottom one, as the board fills both */
		if (m_wide)
			for (const int y : { top, bottom }) {
				double left, right;

				if (y < lowest || y > highest || !ends(y, y, y, left, right))
					continue;
				if (y == top)
					piece(down(y) - grow, across(left) - grow, across(right) + grow, down(y), across(left) - grow,
					      across(right) + grow);
				else
					piece(down(y), across(left) - grow, across(right) + grow, down(y) + grow, across(left) - grow,
					      across(right) + grow);
			}
	}

	const uint32_t added = static_cast<uint32_t>(m_scene.vertices.size() - first);

	if (!added)
		return;

	const bool see_through = p.bank == SEE_THROUGH, always = p.k0 < 0.0f;

	if (!m_scene.runs.empty()) {
		Gaelco3dRun &last = m_scene.runs.back();

		if (last.see_through == see_through && last.always == always && last.first + last.count == first) {
			last.count += added;
			return;
		}
	}
	m_scene.runs.push_back({ static_cast<uint32_t>(first), added, see_through, always });
}

void
Gaelco3dVideo::set_gpu(bool on, bool native, bool wide)
{
	if (on == m_gpu && native == m_native && wide == m_wide)
		return;
	settle();
	if (on && m_roms->mask.empty()) {
		/* The mask of each texel of the first page; the others have none
		 * (masked()) */
		std::vector<uint8_t> mask(size_t{ 4096 } * 1024);

		for (uint32_t a = 0; a < mask.size(); a++)
			mask[a] = masked(a) ? 1 : 0;
		m_roms->mask = std::move(mask);
	}
	if (m_gpu && !on) {
		m_line_changed.fill(true);
		m_palette_changed = true;
		forget_motion();
	}
	m_gpu = on;
	m_native = native;
	m_wide = wide;
	m_scene = {};
	m_gpu_palette_changed = true;
}

Gaelco3dScene
Gaelco3dVideo::take_scene()
{
	settle();

	Gaelco3dScene scene = std::move(m_scene);

	m_scene = {};
	m_scene.vertices.reserve(scene.vertices.size());
	m_scene.runs.reserve(scene.runs.size());
	scene.roms = m_roms;
	scene.both_fields = !m_one_picture;
	if (m_gpu_palette_changed) {
		scene.palette = m_words_of_palette;
		m_gpu_palette_changed = false;
	}
	return scene;
}

Gaelco3dVideo::Texel
Gaelco3dVideo::texel(const Plane &p, float k23, float k89, float k56, int x) const
{
	const float fx = static_cast<float>(x);
	const float d = fx * p.k4 + k56;
	const int32_t u = sixteenths((fx * p.k1 + k23) / d), v = sixteenths((fx * p.k7 + k89) / d);
	Texel t;

	/* HEURISTIC: u and v are added to the texture's start as one address
	 * (MAME); the C31 adds rows of 4096 texels to it the same way */
	t.address = (p.texture + static_cast<uint32_t>(v >> 4) * 4096u + static_cast<uint32_t>(u >> 4)) & ADDRESS;
	t.fu = static_cast<uint32_t>(u) & 15;
	t.fv = static_cast<uint32_t>(v) & 15;
	t.depth = whole(p.k0 / d);
	/* HEURISTIC (MAME): nothing is drawn where the plane is behind the eye */
	t.seen = d > 0.0f;
	return t;
}

/* The mask EPROMs (sheet 7): one bit per texel, 1 for transparent; each
 * holds 1024 columns of 1024 rows, picked by u bits 10-11; none answers when
 * address bit 22 or 23 is set, and on Speed Up's board bit 24 is not decoded.
 * HEURISTIC for the games other than Speed Up (no schematic): MAME's layout
 * of four EPROMs; with a second bank of texture ROMs (Radikal Bikers), none
 * answers when bit 24 is set either, as its test card shows */
bool
Gaelco3dVideo::masked(uint32_t a) const
{
	if (a & m_mask_off)
		return false;
	return (m_texmask[(a >> 10 & 3) << 17 | (a >> 12 & 0x3FF) << 7 | (a & 0x3FF) >> 3] >> (a & 7)) & 1;
}

void
Gaelco3dVideo::line(const Plane &p, unsigned row, int y, int from, int to)
{
	/* The terms the calculator keeps for the line */
	const float fy = static_cast<float>(y);
	const float k23 = fy * p.k2 + p.k3, k89 = fy * p.k8 + p.k9, k56 = fy * p.k5 + p.k6;
	uint64_t *const frame = &m_frame[size_t{ row } * WIDTH];
	uint16_t *const depth = &m_depth[size_t{ row } * PAIRS];

	m_line_changed[row] = true;
	const uint64_t bank = uint64_t{ p.bank } << 25;
	const bool see_through = p.bank == SEE_THROUGH;
	/* HEURISTIC: OH/L is 1 while the odd lines are drawn (H/L = NOT OH/L is
	 * the field shown, the lowest line bit of the fixed text layer, sheet 2) */
	const bool odd = m_field != 0;
	/* HEURISTIC: the pairs start on even columns, and GAEL4 computes the
	 * left pixel of each */
	int x = from & ~1;
	Texel left = texel(p, k23, k89, k56, x + LEFT);

	for (; x < to; x += 2) {
		const Texel next = texel(p, k23, k89, k56, x + 2 + LEFT);
		/* The right pixel: the mean of its two neighbours' u (12.4) and v
		 * (10.4), rounded down (74F283 adders, sheet 4); the page and
		 * bank are those of the left pixel */
		const uint32_t u = (((left.address & U_FIELD) << 4 | left.fu) + ((next.address & U_FIELD) << 4 | next.fu)) >> 1;
		const uint32_t v = (((left.address & V_FIELD) >> 8 | left.fv) + ((next.address & V_FIELD) >> 8 | next.fv)) >> 1;
		const uint32_t right = (left.address & PAGE) | (v >> 4) << 12 | u >> 4;
		/* HEURISTIC (MAME is the only source): the pair passes when its
		 * depth is below the stored one, and always when it is negative
		 * (the C31 gives the first object of each frame a negative W0) */
		uint16_t &z = depth[x >> 1];
		const bool wrb = left.seen && (left.depth < 0 || left.depth < z);
		const bool wr0 = x >= from, wr1 = x + 1 < to;
		/* HEURISTIC: the second mask lookup (ROMB) is at the right
		 * pixel's texel */
		const bool ina = masked(left.address), inb = masked(right);

		/* The PAL IC88 */
		if (wrb && wr0 && !ina && (!see_through || odd))
			frame[x] = left.address | bank | uint64_t{ left.fu } << 32 | uint64_t{ left.fv } << 36;
		if (wrb && wr1 && !inb && (!see_through || !odd))
			frame[x + 1] = right | bank | uint64_t{ u & 15 } << 32 | uint64_t{ v & 15 } << 36;
		if (wrb && wr0 && wr1 && !ina && !inb)
			z = static_cast<uint16_t>(left.depth < 0 ? -left.depth : left.depth);
		left = next;
	}
}

void
Gaelco3dVideo::scan_out()
{
	settle();
	if (m_gpu)
		return;

	const unsigned shown = m_field ^ 1;

	if (m_palette_changed)
		m_line_changed.fill(true);
	m_palette_changed = false;
	/* Half the lines are done on the raster's thread, which is idle: each
	 * line is made from its own words, then the picture from the lines */
	const bool drawn = std::exchange(m_drawn, false);

	m_raster->post([this, shown, drawn] { colour(shown, drawn, HEIGHT / 2, HEIGHT); });
	colour(shown, drawn, 0, HEIGHT / 2);
	m_raster->wait();
	if (!m_one_picture) {
		m_picture = m_colours;
		return;
	}
	m_raster->post([this, shown] { present(shown, HEIGHT / 2, HEIGHT); });
	present(shown, 0, HEIGHT / 2);
	m_raster->wait();
}

void
Gaelco3dVideo::colour(unsigned shown, bool drawn, size_t from, size_t to)
{
	for (size_t row = from; row < to; row++) {
		/* The words of the field just drawn, compared with those they
		 * replaced */
		if (drawn && (row & 1) == shown)
			for (size_t i = row * WIDTH; i < (row + 1) * WIDTH; i++) {
				m_moved[i] = m_frame[i] != m_seen[i];
				m_seen[i] = m_frame[i];
			}
		if (!m_line_changed[row])
			continue;
		m_line_changed[row] = false;
		for (size_t i = row * WIDTH; i < (row + 1) * WIDTH; i++) {
			const uint64_t w = m_frame[i];
			const uint32_t a = static_cast<uint32_t>(w) & ADDRESS;
			const uint32_t fu = w >> 32 & 15, fv = w >> 36 & 15;
			const uint64_t *const palette = &m_palette[(w >> 25 & 0x7F) << 8];
			/* The neighbours wrap in their row of 4096 texels and their page
			 * of 1024 rows (adders IC10, IC7, IC57, IC34, IC40, sheet 2) */
			const uint32_t r = (a & ~U_FIELD) | ((a + 1) & U_FIELD);
			const uint32_t b = (a & ~V_FIELD) | ((a + 0x1000) & V_FIELD);
			const uint32_t br = (b & ~U_FIELD) | ((b + 1) & U_FIELD);
			/* HEURISTIC (MAME is the only source): CUS-COL weights the
			 * four texels by the 4-bit fractions and keeps 8 of the 13
			 * bits of each channel */
			const uint64_t sum = palette[texture(a)] * ((16 - fu) * (16 - fv)) +
			                     palette[texture(r)] * (fu * (16 - fv)) + palette[texture(b)] * ((16 - fu) * fv) +
			                     palette[texture(br)] * (fu * fv);

			m_colours[i] = 0xFF000000u | static_cast<uint32_t>(sum >> 37 & 0xFF) << 16 |
			               static_cast<uint32_t>(sum >> 21 & 0xFF) << 8 | static_cast<uint32_t>(sum >> 5 & 0xFF);
		}
	}
}

/* The monitor puts the 216 lines of a field between those of the previous
 * field, 16 ms older (432 lines, interlaced); its phosphor goes dark again
 * within a few ms, and the eye, following what moves, merges the two fields. A
 * flat screen holds each picture for a whole frame: with two instants in one
 * picture, whatever moves is seen twice. HEURISTIC (one picture): the field
 * just drawn; between its lines, the older field where it stood still (its
 * word, and the word above or below it, unchanged since they were last drawn:
 * there both fields show the same thing); elsewhere, the new line next to it
 * that moved, or the mean of the two (the beam of a line also lights the rows
 * beside it) */
void
Gaelco3dVideo::present(unsigned shown, size_t from, size_t to)
{
	for (size_t row = from; row < to; row++) {
		const size_t at = row * WIDTH;

		if ((row & 1) == shown) {
			std::copy_n(m_colours.begin() + static_cast<std::ptrdiff_t>(at), WIDTH,
			            m_picture.begin() + static_cast<std::ptrdiff_t>(at));
			continue;
		}
		const size_t up = row > 0 ? at - WIDTH : at + WIDTH, down = row + 1 < HEIGHT ? at + WIDTH : at - WIDTH;

		for (size_t x = 0; x < WIDTH; x++) {
			const bool moved_up = m_moved[up + x], moved_down = m_moved[down + x];
			uint32_t c;

			/* The PAL IC88 writes a see-through pixel in its own field only:
			 * it is shown from the older field when nothing moved above or
			 * below */
			const bool alone = (m_frame[at + x] >> 25 & 0x7F) == SEE_THROUGH;

			if (!m_moved[at + x] && (alone ? !moved_up && !moved_down : !(moved_up && moved_down)))
				c = m_colours[at + x];
			else if (moved_up != moved_down)
				c = m_colours[(moved_up ? up : down) + x];
			else {
				const uint32_t a = m_colours[up + x], b = m_colours[down + x];

				c = (a & b) + (((a ^ b) & 0xFEFEFEFEu) >> 1);
			}
			m_picture[at + x] = c;
		}
	}
}

bool
Gaelco3dVideo::save(StateFile &state)
{
	settle();

	const uint32_t counts[] = { m_count, m_field };

	return state.put(m_frame.data(), static_cast<uint32_t>(m_frame.size() * sizeof(m_frame[0]))) &&
	       state.put(m_depth.data(), static_cast<uint32_t>(m_depth.size() * sizeof(m_depth[0]))) &&
	       state.put(m_words) && state.put(counts);
}

bool
Gaelco3dVideo::load(StateFile &state)
{
	uint32_t counts[2] = {};

	settle();
	if (!state.get(m_frame.data(), static_cast<uint32_t>(m_frame.size() * sizeof(m_frame[0]))) ||
	    !state.get(m_depth.data(), static_cast<uint32_t>(m_depth.size() * sizeof(m_depth[0]))) ||
	    !state.get(m_words) || !state.get(counts) || counts[0] > MOST_WORDS || counts[1] > 1)
		return false;
	m_count = counts[0];
	m_field = counts[1];
	m_line_changed.fill(true);
	forget_motion();
	m_scene = {};
	m_gpu_palette_changed = true;
	return true;
}
}
