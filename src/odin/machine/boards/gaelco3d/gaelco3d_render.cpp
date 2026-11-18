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

/* From MAME's gaelco3d_v.cpp and poly.h (BSD-3-Clause, Aaron Giles) */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

#include "odin/machine/boards/gaelco3d/gaelco3d.hpp"

namespace odin {
namespace {
constexpr unsigned MAX_VERTICES = 32;

constexpr bool
poly_end(uint32_t x)
{
	return ((x ^ (x >> 1)) >> 14) & 1;
}

int32_t
round_coordinate(float value)
{
	if (value >= static_cast<float>(std::numeric_limits<int32_t>::max()))
		return std::numeric_limits<int32_t>::max();

	const float whole = std::floor(value);

	if (whole < static_cast<float>(std::numeric_limits<int32_t>::min()))
		return std::numeric_limits<int32_t>::min();
	return static_cast<int32_t>(whole) + (value - whole > 0.5f ? 1 : 0);
}

uint32_t
bilinear(uint32_t c00, uint32_t c01, uint32_t c10, uint32_t c11, int u, int v)
{
	const uint32_t fu = static_cast<uint32_t>(u) & 0xFF, fv = static_cast<uint32_t>(v) & 0xFF;
	uint32_t out = 0;

	for (unsigned shift = 0; shift < 24; shift += 8) {
		const auto lane = [shift](uint32_t c) { return static_cast<int32_t>((c >> shift) & 0xFF); };
		const int32_t top = lane(c00) + (((lane(c01) - lane(c00)) * static_cast<int32_t>(fu)) >> 8);
		const int32_t bottom = lane(c10) + (((lane(c11) - lane(c10)) * static_cast<int32_t>(fu)) >> 8);
		const int32_t mixed = top + (((bottom - top) * static_cast<int32_t>(fv)) >> 8);

		out |= static_cast<uint32_t>(mixed & 0xFF) << shift;
	}
	return out;
}

float
dsp_float(uint32_t v)
{
	return Tms32031::to_float(v);
}
}

void
Gaelco3d::expand_textures(const std::vector<uint8_t> &texture, const std::vector<uint8_t> &texmask)
{
	const size_t size = texture.size(), quarter = size / 4;

	m_texture.assign(size, 0);
	for (size_t y = 0; y < size / 4096; y += 2)
		for (size_t x = 0; x < 4096; x += 2) {
			const size_t from = (y / 2) * 2048 + x / 2;

			m_texture[y * 4096 + x + 1] = texture[from];
			m_texture[(y + 1) * 4096 + x + 1] = texture[quarter + from];
			m_texture[y * 4096 + x] = texture[2 * quarter + from];
			m_texture[(y + 1) * 4096 + x] = texture[3 * quarter + from];
		}

	const size_t bits = texmask.size() * 8;

	m_texmask.assign(bits, 0);
	for (size_t y = 0; y < bits / 4096; y++)
		for (size_t x = 0; x < 4096; x++)
			m_texmask[y * 4096 + x] =
				(texmask[(x / 1024) * (bits / 8 / 4) + (y * 1024 + x % 1024) / 8] >> (x % 8)) & 1;
}

void
Gaelco3d::render_write(uint32_t data)
{
	if (m_polydata_count >= m_polydata.size()) {
		tell_once("polygon buffer full", 0xC00000, data);
		m_polydata_count = 0;
	}
	m_polydata[m_polydata_count++] = data;
	if (m_polydata_count >= 18 && (m_polydata_count % 2) == 1 && poly_end(m_polydata[m_polydata_count - 2])) {
		render_poly(m_polydata.data());
		m_polydata_count = 0;
	}
}

void
Gaelco3d::render_poly(const uint32_t *data)
{
	const float midx = WIDTH / 2.0f, midy = HEIGHT / 2.0f;
	Poly poly;

	poly.z0 = dsp_float(data[0]);
	poly.voz_dy = dsp_float(data[1]) * 256.0f;
	poly.voz_dx = dsp_float(data[2]) * 256.0f;
	poly.ooz_dy = dsp_float(data[3]);
	poly.ooz_dx = dsp_float(data[4]);
	poly.uoz_dy = dsp_float(data[5]) * 256.0f;
	poly.uoz_dx = dsp_float(data[6]) * 256.0f;
	poly.voz_base = dsp_float(data[7]) * 256.0f - midx * poly.voz_dx - midy * poly.voz_dy;
	poly.ooz_base = dsp_float(data[8]) - midx * poly.ooz_dx - midy * poly.ooz_dy;
	poly.uoz_base = dsp_float(data[9]) * 256.0f - midx * poly.uoz_dx - midy * poly.uoz_dy;
	poly.color = (data[10] & 0x7F) << 8;
	poly.tex = data[11];

	Vertex vert[MAX_VERTICES];
	unsigned count = 0;

	for (uint32_t word = 0; count < MAX_VERTICES && !poly_end(word); count++) {
		word = data[13 + count * 2];
		vert[count].x = midx + static_cast<float>(static_cast<int32_t>(word) >> 16) + 0.5f;
		vert[count].y = midy + static_cast<float>(static_cast<int32_t>(word << 18) >> 18) + 0.5f;
	}
	if (count < 3)
		return;

	for (unsigned i = 0; i < count; i++) {
		if (vert[i].x > WIDTH - 1.0f && vert[i].x < WIDTH)
			vert[i].x = WIDTH;
		if (vert[i].y > HEIGHT - 1.0f && vert[i].y < HEIGHT)
			vert[i].y = HEIGHT;
	}

	Span span = Span::NORMAL;

	if (poly.color == 0x7F00)
		span = Span::BLEND;
	else if (poly.z0 < 0 && poly.ooz_dx == 0 && poly.ooz_dy == 0)
		span = Span::FLAT;
	if (m_gpu) {
		scene_poly(poly, span, vert, count);
		return;
	}
	for (unsigned i = 2; i < count; i++)
		triangle(poly, span, vert[0], vert[i - 1], vert[i]);
}

void
Gaelco3d::triangle(const Poly &poly, Span span, const Vertex &a, const Vertex &b, const Vertex &c)
{
	const Vertex *v1 = &a, *v2 = &b, *v3 = &c;

	if (v2->y < v1->y)
		std::swap(v1, v2);
	if (v3->y < v2->y) {
		std::swap(v2, v3);
		if (v2->y < v1->y)
			std::swap(v1, v2);
	}

	const int32_t top = std::max(round_coordinate(v1->y), 0);
	const int32_t bottom = std::min(round_coordinate(v3->y), static_cast<int32_t>(HEIGHT));

	if (bottom - top <= 0)
		return;

	const float d13 = v3->y == v1->y ? 0.0f : (v3->x - v1->x) / (v3->y - v1->y);
	const float d12 = v2->y == v1->y ? 0.0f : (v2->x - v1->x) / (v2->y - v1->y);
	const float d23 = v3->y == v2->y ? 0.0f : (v3->x - v2->x) / (v3->y - v2->y);

	for (int32_t y = top; y < bottom; y++) {
		const float fy = static_cast<float>(y) + 0.5f;
		const float start = v1->x + (fy - v1->y) * d13;
		const float stop = fy < v2->y ? v1->x + (fy - v1->y) * d12 : v2->x + (fy - v2->y) * d23;
		int32_t x0 = round_coordinate(start), x1 = round_coordinate(stop);

		if (x0 > x1)
			std::swap(x0, x1);
		x0 = std::max(x0, 0);
		x1 = std::min(x1, static_cast<int32_t>(WIDTH));
		if (x0 < x1)
			scan(poly, span, y, x0, x1);
	}
}

uint32_t
Gaelco3d::texel(const Poly &poly, int u, int v) const
{
	const uint32_t end = static_cast<uint32_t>(m_texture.size() - 1);
	const uint32_t at = (poly.tex + static_cast<uint32_t>((v >> 8) * 4096 + (u >> 8))) & end;

	if (at < m_texmask.size() && m_texmask[at])
		return 0xFFFFFFFFu;

	const uint32_t *const pal = &m_palette[poly.color];

	return bilinear(pal[m_texture[at]], pal[m_texture[(at + 1) & end]], pal[m_texture[(at + 4096) & end]],
	                pal[m_texture[(at + 4097) & end]], u, v);
}

void
Gaelco3d::scan(const Poly &poly, Span span, int y, int x0, int x1)
{
	uint16_t *const dest = &m_screen[static_cast<size_t>(y) * WIDTH];
	uint16_t *const zbuf = &m_zbuffer[static_cast<size_t>(y) * WIDTH];
	const float fy = static_cast<float>(y), fx = static_cast<float>(x0);

	if (span == Span::FLAT) {
		const float zbase = 1.0f / poly.ooz_base;
		const float uoz_step = poly.uoz_dx * zbase, voz_step = poly.voz_dx * zbase;
		const auto zval = static_cast<uint16_t>(static_cast<int>(-poly.z0 * zbase));
		float uoz = (poly.uoz_base + fy * poly.uoz_dy + fx * poly.uoz_dx) * zbase;
		float voz = (poly.voz_base + fy * poly.voz_dy + fx * poly.voz_dx) * zbase;

		for (int x = x0; x < x1; x++) {
			const uint32_t rgb = texel(poly, static_cast<int>(uoz), static_cast<int>(voz));

			if (rgb != 0xFFFFFFFFu) {
				dest[x] = static_cast<uint16_t>((rgb & 0x1F) | ((rgb & 0x1FF800) >> 6));
				zbuf[x] = zval;
			}
			uoz += uoz_step;
			voz += voz_step;
		}
		return;
	}
	float ooz = poly.ooz_base + fy * poly.ooz_dy + fx * poly.ooz_dx;
	float uoz = poly.uoz_base + fy * poly.uoz_dy + fx * poly.uoz_dx;
	float voz = poly.voz_base + fy * poly.voz_dy + fx * poly.voz_dx;

	for (int x = x0; x < x1; x++) {
		if (ooz > 0) {
			const float z = 1.0f / ooz;
			const int zval = static_cast<int>(poly.z0 * z);

			if (zval < zbuf[x]) {
				const uint32_t rgb = texel(poly, static_cast<int>(uoz * z), static_cast<int>(voz * z));

				if (rgb != 0xFFFFFFFFu) {
					if (span == Span::BLEND) {
						const uint32_t half = rgb >> 1;

						dest[x] = static_cast<uint16_t>(((half & 0x0F) | ((half & 0x0F7800) >> 6)) +
						                                ((dest[x] >> 1) & 0x3DEF));
					} else {
						dest[x] = static_cast<uint16_t>((rgb & 0x1F) | ((rgb & 0x1FF800) >> 6));
					}
					zbuf[x] = static_cast<uint16_t>(zval < 0 ? -zval : zval);
				}
			}
		}
		ooz += poly.ooz_dx;
		uoz += poly.uoz_dx;
		voz += poly.voz_dx;
	}
}
}
