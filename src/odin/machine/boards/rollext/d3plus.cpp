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

#include "odin/machine/boards/rollext/d3plus.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#if defined(__AVX__) || defined(__SSE4_1__)
#include <immintrin.h>
#endif

#include "odin/common/repeats.hpp"
#include "odin/machine/core/state_file.hpp"
#include "odin/machine/core/worker.hpp"

namespace odin {

namespace {

/* The texture sheet: 2048 x 16384 texels */
constexpr uint32_t SHEET_W = 2048, SHEET_H = 16384;
/* The ROMs' bytes: 8 MB each, (y >> 1) * 1024 + (x >> 1) */
constexpr uint32_t ROM_SIZE = SHEET_W * SHEET_H / 4;
constexpr uint32_t BLACK = 0xFF000000u;
/* The palette: 32,768 big-endian words */
constexpr uint32_t PALETTE_BYTES = 0x10000;
/* A depth that any pixel passes: the depth buffer as cleared */
constexpr double FAR = std::numeric_limits<double>::infinity();

/* The 8-bit level of a 5-bit one, as the reference renderer makes it
 * (c * 255 // 31). HEURISTIC: the DACs' own ramp is not known */
constexpr std::array<uint32_t, 32> LEVEL = [] {
	std::array<uint32_t, 32> level{};

	for (uint32_t c = 0; c < 32; c++)
		level[c] = c * 255 / 31;
	return level;
}();

/* A 9-bit coordinate of the setup packet */
constexpr int32_t
signed9(uint32_t field)
{
	return field & 256 ? static_cast<int32_t>(field) - 512 : static_cast<int32_t>(field);
}

/* whole() past 2^52: never taken by the game, kept out of the drawing's
 * loop */
[[gnu::noinline, gnu::cold]] int64_t
whole_far(double f)
{
	return static_cast<int64_t>(std::fmod(f, 16384.0));
}

/* floor(x), within a multiple of 16384 (the sheet wraps at 2048 and 16384):
 * exact for any finite x, as the reference renderer's integers are */
inline int64_t
whole(double x)
{
	const double f = std::floor(x);

	if (std::fabs(f) < 0x1p52) [[likely]]
		return static_cast<int64_t>(f);
	return whole_far(f);
}

/* Points of a line taken together, a double each: four with AVX, else two
 * (SSE2 has 16 registers of two doubles; ARM's NEON the same) */
#if defined(__AVX__)
constexpr int LANES = 4;
using Lanes = double __attribute__((vector_size(32)));
#else
constexpr int LANES = 2;
using Lanes = double __attribute__((vector_size(16)));
#endif

/* A double in every lane, as it is (-0.0 stays -0.0) */
inline Lanes
splat(double d)
{
	Lanes v;

	for (int k = 0; k < LANES; k++)
		v[k] = d;
	return v;
}

#if defined(__AVX__) || defined(__SSE4_1__)
/* The texels of the points, when floor(u) and floor(v) of all of them are
 * within 2^31 (then u and v are finite, and whole() of each is that floor
 * as a 32-bit integer, the texel its low bits plus the texture's origin):
 * the x of each in tx, the y in ty. Otherwise false: the caller takes them
 * one by one */
inline bool
texels(const Lanes &u, const Lanes &v, uint32_t x0, uint32_t y0, uint32_t *tx, uint32_t *ty)
{
#if defined(__AVX__)
	const __m256d fu = _mm256_round_pd(u, _MM_FROUND_TO_NEG_INF | _MM_FROUND_NO_EXC);
	const __m256d fv = _mm256_round_pd(v, _MM_FROUND_TO_NEG_INF | _MM_FROUND_NO_EXC);
	const __m256d sign = _mm256_set1_pd(-0.0), limit = _mm256_set1_pd(0x1p31);

	if (_mm256_movemask_pd(_mm256_and_pd(_mm256_cmp_pd(_mm256_andnot_pd(sign, fu), limit, _CMP_LT_OQ),
	                                     _mm256_cmp_pd(_mm256_andnot_pd(sign, fv), limit, _CMP_LT_OQ))) != 15)
		return false;
	_mm_storeu_si128(reinterpret_cast<__m128i *>(tx),
	                 _mm_and_si128(_mm_add_epi32(_mm256_cvttpd_epi32(fu), _mm_set1_epi32(static_cast<int>(x0))),
	                               _mm_set1_epi32(SHEET_W - 1)));
	_mm_storeu_si128(reinterpret_cast<__m128i *>(ty),
	                 _mm_and_si128(_mm_add_epi32(_mm256_cvttpd_epi32(fv), _mm_set1_epi32(static_cast<int>(y0))),
	                               _mm_set1_epi32(SHEET_H - 1)));
#else
	/* SSE4.1 (x86-64-v2) */
	const __m128d fu = _mm_round_pd(u, _MM_FROUND_TO_NEG_INF | _MM_FROUND_NO_EXC);
	const __m128d fv = _mm_round_pd(v, _MM_FROUND_TO_NEG_INF | _MM_FROUND_NO_EXC);
	const __m128d sign = _mm_set1_pd(-0.0), limit = _mm_set1_pd(0x1p31);

	/* cmplt: false for a NaN, as _CMP_LT_OQ */
	if (_mm_movemask_pd(_mm_and_pd(_mm_cmplt_pd(_mm_andnot_pd(sign, fu), limit),
	                               _mm_cmplt_pd(_mm_andnot_pd(sign, fv), limit))) != 3)
		return false;
	_mm_storel_epi64(reinterpret_cast<__m128i *>(tx),
	                 _mm_and_si128(_mm_add_epi32(_mm_cvttpd_epi32(fu), _mm_set1_epi32(static_cast<int>(x0))),
	                               _mm_set1_epi32(SHEET_W - 1)));
	_mm_storel_epi64(reinterpret_cast<__m128i *>(ty),
	                 _mm_and_si128(_mm_add_epi32(_mm_cvttpd_epi32(fv), _mm_set1_epi32(static_cast<int>(y0))),
	                               _mm_set1_epi32(SHEET_H - 1)));
#endif
	return true;
}
#endif

} /* namespace */

D3plus::D3plus()
	: m_sheet(size_t{ SHEET_W } * SHEET_H, 0), m_drawn(size_t{ WIDTH } * HEIGHT, BLACK),
	  m_shown(size_t{ WIDTH } * HEIGHT, BLACK), m_depth(size_t{ WIDTH } * HEIGHT, FAR),
	  m_drawing(std::make_unique<Worker>())
{
	m_s.clip = { -256, 255, -192, 192 };
}

D3plus::~D3plus()
{
	settle();
	if (m_dump)
		std::fclose(m_dump);
}

void
D3plus::attach(const uint8_t *palette, const uint8_t *table)
{
	m_palette = palette;
	m_table = table;
	palette_written(0, PALETTE_BYTES);
}

void
D3plus::palette_written(uint32_t offset, uint32_t bytes)
{
	if (!m_palette)
		return;
	m_colours.resize(PALETTE_BYTES / 2);

	const uint32_t end = std::min(offset + bytes, PALETTE_BYTES);

	for (uint32_t entry = offset / 2; entry < (end + 1) / 2; entry++) {
		/* § 3, PROVEN: x RRRRR GGGGG BBBBB */
		const uint32_t colour = uint32_t{ m_palette[2 * entry] } << 8 | m_palette[2 * entry + 1];

		m_colours[entry] = BLACK | LEVEL[colour & 31] << 16 | LEVEL[colour >> 5 & 31] << 8 | LEVEL[colour >> 10 & 31];
	}
}

/* Texel (x, y): ic47 for x and y even, ic60 for x odd and y even, ic45 for
 * x even and y odd, ic58 for both odd (the sections
 * texture0-3 of odin-games.json in that order). A ROM missing reads 0 */
void
D3plus::set_textures(const std::array<std::vector<uint8_t>, 4> &roms)
{
	for (uint32_t y = 0; y < SHEET_H; y++)
		for (uint32_t odd = 0; odd < 2; odd++) {
			const std::vector<uint8_t> &rom = roms[(y & 1) * 2 + odd];
			const size_t at = size_t{ y >> 1 } * (SHEET_W / 2);
			uint8_t *const row = &m_sheet[size_t{ y } * SHEET_W + odd];

			for (size_t x = 0; x < SHEET_W / 2; x++)
				row[2 * x] = at + x < std::min<size_t>(rom.size(), ROM_SIZE) ? rom[at + x] : 0;
		}
}

/* HEURISTIC: empty at power-up, the chip idle, both frames black, the
 * clipping rectangle the whole picture */
void
D3plus::reset()
{
	settle();
	m_s = State{};
	m_s.clip = { -256, 255, -192, 192 };
	m_polls = m_refused = 0;
	std::fill(m_drawn.begin(), m_drawn.end(), BLACK);
	std::fill(m_shown.begin(), m_shown.end(), BLACK);
	std::fill(m_depth.begin(), m_depth.end(), FAR);
}

/* The chip takes the packet at the head of the FIFO once it is whole and
 * the previous one is done, draws it, then works PACKET_CYCLES on it.
 * Taking all of them at the end of a frame would not do: PP0 drops the
 * packets it has not sent when its task restarts (0x400001A0), and with ~30
 * a frame half of them would be lost. A marker is reached once the packets
 * before it are drawn. */
void
D3plus::take_due(uint64_t now)
{
	for (;;) {
		finish_pictures();
		if (m_s.count < PACKET || m_s.next_take > now)
			break;
		for (unsigned n = 0; n < PACKET; n++)
			m_s.current[n] = m_s.words[(m_s.head + n) % CAPACITY];
		m_s.head = (m_s.head + PACKET) % CAPACITY;
		m_s.count -= PACKET;
		m_s.taken++;
		m_s.next_take += PACKET_CYCLES;
		take(m_s.current);
	}
}

void
D3plus::finish_pictures()
{
	while (m_s.waiting_markers && m_s.marker_place[0] <= m_s.taken * PACKET) {
		next_picture();
		std::copy(m_s.marker_place.begin() + 1, m_s.marker_place.end(), m_s.marker_place.begin());
		std::copy(m_s.marker_frame.begin() + 1, m_s.marker_frame.end(), m_s.marker_frame.begin());
		m_s.waiting_markers--;
	}
}

/* HEURISTIC: the marker ends a picture;
 * the chip shows it and draws the next one into its other frame, black,
 * the depth buffer cleared. Two frames, as on the Gaelco 3D; what the
 * marker does on the board is not known */
void
D3plus::next_picture()
{
	queue(Job{ {}, {}, m_s.drawn_marker, m_s.drawn_frame, true });
	m_s.drawn_frame = m_s.marker_frame[0];
	m_s.drawn_marker = m_s.markers - m_s.waiting_markers + 1;
}

void
D3plus::end_picture(uint32_t frame, uint64_t marker)
{
	if (!m_frames.empty()) {
		char path[1024];

		std::snprintf(path, sizeof(path), "%s-f%06u-m%06llu.ppm", m_frames.c_str(), frame,
		              static_cast<unsigned long long>(marker));
		if (std::FILE *const file = std::fopen(path, "wb")) {
			std::vector<uint8_t> rgb(size_t{ WIDTH } * HEIGHT * 3);

			for (size_t i = 0; i < m_drawn.size(); i++)
				for (unsigned c = 0; c < 3; c++)
					rgb[3 * i + c] = static_cast<uint8_t>(m_drawn[i] >> (8 * c));
			std::fprintf(file, "P6\n%u %u\n255\n", WIDTH, HEIGHT);
			std::fwrite(rgb.data(), 1, rgb.size(), file);
			std::fclose(file);
		}
	}
	std::swap(m_drawn, m_shown);
	std::fill(m_drawn.begin(), m_drawn.end(), BLACK);
	std::fill(m_depth.begin(), m_depth.end(), FAR);
}

/* The setup packet sets the clipping rectangle at once; the others are
 * drawn with the rectangle in force when they are taken */
void
D3plus::take(const std::array<uint64_t, PACKET> &packet)
{
	const uint32_t w0 = static_cast<uint32_t>(packet[0] >> 32), w1 = static_cast<uint32_t>(packet[0]),
	               w3 = static_cast<uint32_t>(packet[1]);

	if (w0 >> 30 == 2) {
		/* w1 and w3 the clipping rectangle (PROVEN); HEURISTIC, its
		 * edges included */
		m_s.clip = { signed9(w1 >> 23 & 511), signed9(w1 >> 11 & 511), signed9(w3 >> 11 & 511),
		             signed9(w3 >> 23 & 511) };
		return;
	}
	m_queued.emplace_back(packet, m_s.clip, 0, 0, false);
	queued();
}

void
D3plus::queue(const Job &job)
{
	m_queued.push_back(job);
	m_end_queued |= job.end;
	queued();
}

void
D3plus::queued()
{
	if (m_queued.size() >= BATCH || ((m_queued.size() >= SOME || m_end_queued) && m_drawing->idle()))
		send();
}

void
D3plus::send()
{
	if (m_queued.empty())
		return;
	m_drawing->wait();
	std::swap(m_queued, m_sent);
	m_queued.clear();
	m_end_sent = m_end_queued;
	m_end_queued = false;
	m_drawing->post([this] {
		for (const Job &job : m_sent)
			if (job.end)
				end_picture(job.frame, job.marker);
			else
				draw(job.packet, job.clip);
	});
}

void
D3plus::settle()
{
	send();
	m_drawing->wait();
	m_end_sent = false;
}

/* The picture shown is read once the frame is run: the ends of pictures
 * sent are drawn first. The packets queued after the last end are only
 * drawn into the next picture's buffers, which nothing reads before the
 * next settle */
void
D3plus::finished()
{
	if (m_end_queued)
		send();
	if (m_end_sent) {
		m_drawing->wait();
		m_end_sent = false;
	}
}

/* A packet: w0 bits 31-30 the type (10 setup, 01 and 00
 * a triangle, 11 a quad), vertices (w1, w3), (w5, w7), (w9, w11), (w13,
 * w15), (w17, w19) as floats in the picture's coordinates; planes U = w6 x +
 * w8 y + w18, V = w14 x + w16 y + w22, Q = w10 x + w12 y + w20; w2 >> 8 the
 * texture's address, w4 its depth */
void
D3plus::draw(const std::array<uint64_t, PACKET> &packet, const std::array<int32_t, 4> &clip)
{
	const auto word = [&packet](unsigned i) {
		return static_cast<uint32_t>(i & 1 ? packet[i / 2] : packet[i / 2] >> 32);
	};
	const auto real = [&word](unsigned i) { return static_cast<double>(std::bit_cast<float>(word(i))); };
	const uint32_t w0 = word(0);
	const unsigned type = w0 >> 30;

	if (!m_palette || !m_table)
		return;

	/* HEURISTIC: type 00 (never seen) drawn as a triangle like 01 */
	const unsigned corners = type == 3 ? 4 : 3;
	std::array<double, 4> vx{}, vy{};

	for (unsigned k = 0; k < corners; k++) {
		vx[k] = real(1 + 4 * k);
		vy[k] = real(3 + 4 * k);
	}

	const double ua = real(6), ub = real(8), uc = real(18);
	const double va = real(14), vb = real(16), vc = real(22);
	const double qa = real(10), qb = real(12), qc = real(20);
	/* § 5, PROVEN: z = (w4 >> 7) / Q, w4 unsigned (the far backdrop has
	 * bit 31 set); the nearer wins, DEDUCED: the later on a tie */
	const double depth = static_cast<double>(word(4) >> 7);
	/* § 2, PROVEN: X0 = A & 0x7FF, Y0 = A >> 11, + 8192 with bit 24 of w0 */
	const uint32_t address = word(2) >> 8;
	const int64_t x0 = address & 0x7FF, y0 = (address >> 11) + (w0 >> 24 & 1 ? 8192 : 0);
	/* § 3, PROVEN: the bank in bits 14-8 of w0. UNKNOWN: bit 15 of the
	 * colours, ignored */
	const uint32_t bank = (w0 >> 8 & 0x7F) << 8;
	/* § 7, HEURISTIC: bits 23-20 of w0 an opacity L (0: opaque); the
	 * pixel becomes dst + (src - dst) * L / 16, its depth not written */
	const int level = static_cast<int>(w0 >> 20 & 15);
	/* HEURISTIC (§ 5): bit 29 of w0 (the backdrop, the 2D text) changes
	 * nothing; with the depth buffer cleared at each picture, reading it
	 * as "no depth test" gives the same pictures */
	const auto [xmin, xmax, ymin, ymax] = clip;

	double top = vy[0], bottom = vy[0];

	for (unsigned k = 1; k < corners; k++) {
		if (vy[k] > top)
			top = vy[k];
		if (vy[k] < bottom)
			bottom = vy[k];
	}

	/* The edges that are not level, in order, their differences taken
	 * once: the same doubles as taken on each line */
	struct Edge {
		double ax, ay, dx, dy, low, high;
	};
	std::array<Edge, 4> edges{};
	unsigned sides = 0;

	for (unsigned k = 0; k < corners; k++) {
		const double ax = vx[k], ay = vy[k], bx = vx[(k + 1) % corners], by = vy[(k + 1) % corners];

		if (ay == by)
			continue;
		edges[sides++] = { ax, ay, bx - ax, by - ay, ay < by ? ay : by, ay < by ? by : ay };
	}

	/* The lines the tests below can let through: in the rectangle and
	 * the picture, at or under top and at or over bottom (no bound from
	 * a NaN, which fails no test) */
	int high = std::min(ymax, 192), low = std::max(ymin, -191);

	if (top < 192.0)
		high = top >= -191.0 ? std::min(high, static_cast<int>(std::floor(top))) : -192;
	if (bottom > -191.0)
		low = bottom <= 192.0 ? std::max(low, static_cast<int>(std::ceil(bottom))) : 193;

	/* The buffers, in locals: the stores of points do not reload them */
	double *const depths = m_depth.data();
	uint32_t *const drawn = m_drawn.data();
	const uint8_t *const sheet = m_sheet.data(), *const table = m_table;
	const uint32_t *const colours = m_colours.data();

	const Lanes qa_ = splat(qa), ua_ = splat(ua), va_ = splat(va), qc_ = splat(qc), uc_ = splat(uc), vc_ = splat(vc);
	const Lanes depth_ = splat(depth), step = splat(LANES);

	for (int sy = high; sy >= low; sy--) {
		/* § 6: the pixel's sample point at whole coordinates (DEDUCED) */
		const int py = 192 - sy;
		const double y = sy;

		if (sy > ymax || sy < ymin || y > top || y < bottom)
			continue;

		/* HEURISTIC (§ 6): the edges crossing the line, a line from an
		 * edge's lower end included, its upper end not */
		double left = 0.0, right = 0.0;
		unsigned crossed = 0;

		for (unsigned k = 0; k < sides; k++) {
			const Edge &e = edges[k];

			if (!(e.low <= y && y < e.high))
				continue;

			const double x = e.ax + (y - e.ay) * e.dx / e.dy;

			if (crossed == 0) {
				left = right = x;
			} else {
				if (x < left)
					left = x;
				if (x > right)
					right = x;
			}
			crossed++;
		}
		if (crossed < 2 || !std::isfinite(left) || !std::isfinite(right))
			continue;

		/* HEURISTIC (§ 6): pixels ceil(left) to ceil(right) - 1, within
		 * the clipping rectangle */
		const double from = std::max(std::ceil(left), static_cast<double>(xmin));
		const double to = std::min(std::ceil(right) - 1.0, static_cast<double>(xmax));

		if (from > to)
			continue;

		/* Within the rectangle, so within the picture (x -256 to 255) */
		const int first = std::max(static_cast<int>(from), -256), last = std::min(static_cast<int>(to), 255);
		const double qrow = qb * y, urow = ub * y, vrow = vb * y;
		const Lanes qrow_ = splat(qrow), urow_ = splat(urow), vrow_ = splat(vrow);
		const size_t row = static_cast<size_t>(py) * WIDTH + 256;
		/* A point that passed the depth test, u and v finite, at its
		 * texel */
		const auto put = [&](size_t i, uint32_t tx, uint32_t ty, double z) {
			/* § 4, PROVEN: the table's bit of the texel, rows below
			 * 2048 only; 1, not written (the game's copy has its bytes
			 * swapped in pairs, 0x400403E0) */
			if (ty < 2048 && (table[(ty * 256 + (tx >> 3)) ^ 1] >> (tx & 7) & 1))
				return;

			const uint32_t colour = colours[bank + sheet[size_t{ ty } * SHEET_W + tx]];

			if (level) {
				const uint32_t old = drawn[i];
				const int r0 = static_cast<int>(old & 0xFF), g0 = static_cast<int>(old >> 8 & 0xFF),
				          b0 = static_cast<int>(old >> 16 & 0xFF);
				const int r = static_cast<int>(colour & 0xFF), g = static_cast<int>(colour >> 8 & 0xFF),
				          b = static_cast<int>(colour >> 16 & 0xFF);

				/* Rounded down, as the reference renderer does */
				drawn[i] = BLACK | static_cast<uint32_t>(b0 + ((b - b0) * level >> 4)) << 16 |
				           static_cast<uint32_t>(g0 + ((g - g0) * level >> 4)) << 8 |
				           static_cast<uint32_t>(r0 + ((r - r0) * level >> 4));
			} else {
				depths[i] = z;
				drawn[i] = colour;
			}
		};

		/* LANES points at a time: each lane the same operations, in
		 * the same order, as for one point (the divisions too, rounded
		 * alike); the lanes past the span are left unused. x stays
		 * whole, so adding LANES is exact */
		Lanes x;

		for (int k = 0; k < LANES; k++)
			x[k] = static_cast<double>(first + k);
		for (int sx = first; sx <= last; sx += LANES, x += step) {
			const Lanes q = qa_ * x + qrow_ + qc_;
			const Lanes z = depth_ / q;
			const Lanes u = (ua_ * x + urow_ + uc_) / q, v = (va_ * x + vrow_ + vc_) / q;
			const int lanes = std::min(LANES, last - sx + 1);
			const size_t at = row + static_cast<size_t>(sx);

#if defined(__AVX__) || defined(__SSE4_1__)
			uint32_t txs[LANES], tys[LANES];

			if (texels(u, v, static_cast<uint32_t>(x0), static_cast<uint32_t>(y0), txs, tys)) {
				for (int k = 0; k < lanes; k++)
					if (q[k] != 0.0 && !(z[k] > depths[at + k]))
						put(at + k, txs[k], tys[k], z[k]);
				continue;
			}
#endif
			for (int k = 0; k < lanes; k++) {
				if (q[k] == 0.0)
					continue;

				const size_t i = at + static_cast<size_t>(k);

				if (z[k] > depths[i])
					continue;
				if (!std::isfinite(u[k]) || !std::isfinite(v[k]))
					continue;

				/* § 2: texel floor(u), floor(v) from the texture's
				 * origin (HEURISTIC: floor); HEURISTIC, wrapped on the
				 * sheet */
				put(i, static_cast<uint32_t>(x0 + whole(u[k])) & (SHEET_W - 1),
				    static_cast<uint32_t>(y0 + whole(v[k])) & (SHEET_H - 1), z[k]);
			}
		}
	}
}

void
D3plus::receive(uint64_t word, uint32_t addr, uint64_t now)
{
	update(now);
	/* A packet that ends now cannot be taken before now */
	if (m_s.count < PACKET)
		m_s.next_take = std::max(m_s.next_take, now);
	if (m_s.count >= DEPTH)
		/* HEURISTIC: kept; the board would make the TC wait (SPRU261
		 * § 8.3.4), which is not emulated */
		repeats::print(stderr, "d3plus: a transfer into the full FIFO\n");
	if (m_s.count < CAPACITY) {
		m_s.words[(m_s.head + m_s.count) % CAPACITY] = word;
		m_s.count++;
	} else {
		repeats::print(stderr, "d3plus: FIFO overflow, a doubleword lost\n");
	}
	if (m_dump) {
		const unsigned n = static_cast<unsigned>(m_s.received % PACKET);

		if (n == 0)
			m_line_addr = addr;
		m_line[n] = word;
		if (n == PACKET - 1) {
			std::fprintf(m_dump, "P %llu %08X", static_cast<unsigned long long>(now), m_line_addr);
			for (const uint64_t w : m_line)
				std::fprintf(m_dump, " %08X %08X", static_cast<unsigned>(w >> 32), static_cast<unsigned>(w));
			std::fputc('\n', m_dump);
		}
	}
	m_s.received++;
}

/* Kept at its place in the stream, and HEURISTIC, takes no room in the
 * FIFO: the chip reaches it once it has drawn the packets before it */
void
D3plus::marker(uint64_t now)
{
	update(now);
	m_s.markers++;
	m_s.marker_at = m_s.received;
	if (m_dump)
		std::fprintf(m_dump, "M %llu %llu\n", static_cast<unsigned long long>(now),
		             static_cast<unsigned long long>(m_s.received));
	if (m_s.waiting_markers == MARKERS) {
		/* Never seen: the chip would be 16 pictures behind */
		repeats::print(stderr, "d3plus: too many markers waiting, a picture ended early\n");
		m_s.marker_place[0] = 0;
		finish_pictures();
	}
	m_s.marker_place[m_s.waiting_markers] = m_s.received;
	m_s.marker_frame[m_s.waiting_markers] = m_frame;
	m_s.waiting_markers++;
	finish_pictures();
}

void
D3plus::polled(uint64_t now, bool room, uint64_t count)
{
	update(now);
	if ((m_s.count + BURST <= DEPTH) != room)
		/* Never: the last poll would have answered otherwise */
		repeats::print(stderr, "d3plus: polls counted ahead answer otherwise\n");
	m_polls += count;
	m_refused += room ? 0 : count;
}

bool
D3plus::open_dump(const std::string &path)
{
	if (m_dump)
		std::fclose(m_dump);
	m_dump = std::fopen(path.c_str(), "w");
	if (!m_dump)
		std::fprintf(stderr, "ERROR: d3plus: cannot write '%s'\n", path.c_str());
	return m_dump != nullptr;
}

void
D3plus::frame_start(unsigned frame)
{
	m_frame = frame;
	if (m_dump)
		std::fprintf(m_dump, "F %u %llu %llu\n", frame, static_cast<unsigned long long>(m_polls),
		             static_cast<unsigned long long>(m_refused));
}

bool
D3plus::save_buffers(StateFile &state)
{
	settle();
	return state.put(m_drawn.data(), static_cast<uint32_t>(m_drawn.size() * sizeof(m_drawn[0]))) &&
	       state.put(m_shown.data(), static_cast<uint32_t>(m_shown.size() * sizeof(m_shown[0]))) &&
	       state.put(m_depth.data(), static_cast<uint32_t>(m_depth.size() * sizeof(m_depth[0])));
}

bool
D3plus::load_buffers(StateFile &state)
{
	settle();
	return state.get(m_drawn.data(), static_cast<uint32_t>(m_drawn.size() * sizeof(m_drawn[0]))) &&
	       state.get(m_shown.data(), static_cast<uint32_t>(m_shown.size() * sizeof(m_shown[0]))) &&
	       state.get(m_depth.data(), static_cast<uint32_t>(m_depth.size() * sizeof(m_depth[0])));
}

} /* namespace odin */
