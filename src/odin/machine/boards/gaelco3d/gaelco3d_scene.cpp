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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>

#include "odin/machine/boards/gaelco3d/gaelco3d.hpp"

namespace odin {
namespace {
constexpr float SCENE_X = 640.0f / Gaelco3d::WIDTH, SCENE_Y = 480.0f / Gaelco3d::HEIGHT;

constexpr unsigned BLOCK = 64, TILE = 256;
constexpr unsigned CELL = 512, MOST_SIDE = CELL + 8, FAR_SIDE = 4096;
constexpr unsigned MOST_CORNERS = 40, MOST_SPLITS = 10;
constexpr unsigned TRIMMED_SIDE = CELL + 8;
/* HEURISTIC: polygons are clipped to the screen and at a horizon a thousand
 * times as far as their nearest corner, brought closer while their texels
 * do not fit in an image; beyond it texels are smaller than a pixel */
constexpr float NEAREST = 1e-3f;

constexpr unsigned KEPT_FRAMES = 300;
constexpr uint8_t COMPARE_GREATER = 4, COMPARE_ALWAYS = 7;

constexpr uint8_t SRC_ALPHA = 4, INV_SRC_ALPHA = 5;
constexpr uint32_t BLEND_BANK = 0x7F;
}

void
Gaelco3d::set_gpu(bool on)
{
	m_gpu = on;
	m_palette_changed = true;
}

void
Gaelco3d::forget_images()
{
	for (const auto &[key, image] : m_images)
		m_gone.push_back(image.id);
	m_images.clear();
	m_aliases.clear();
	m_scene = {};
	m_ready = {};
	m_carry = {};
	m_most_w = 0.0f;
	m_palette_changed = true;
}

uint32_t
Gaelco3d::image_of(uint32_t &base, unsigned &w, unsigned &h)
{
	const ImageKey asked{ base, w << 16 | h };
	const auto take = [&](const ImageKey &key, Image &kept) {
		kept.used = m_frame;
		base = key.base;
		w = key.size >> 16;
		h = key.size & 0xFFFF;
		return kept.id;
	};

	if (const auto found = m_images.find(asked); found != m_images.end())
		return take(found->first, found->second);
	if (const auto alias = m_aliases.find(asked); alias != m_aliases.end())
		if (const auto found = m_images.find(alias->second); found != m_images.end())
			return take(found->first, found->second);

	if (const uint32_t col = base & 4095, top = base >> 12; base < m_texture.size() && col + w <= 4096)
		for (auto &[key, kept] : m_images) {
			const uint32_t kcol = key.base & 4095, krow = key.base >> 12, kw = key.size >> 16, kh = key.size & 0xFFFF;

			if (kcol <= col && col + w <= kcol + kw && krow <= top &&
			    top + h <= krow + kh && kcol + kw <= 4096) {
				m_aliases.emplace(asked, key);
				return take(key, kept);
			}
		}

	auto image = std::make_shared<PmxImage>();
	const uint32_t end = static_cast<uint32_t>(m_texture.size() - 1);
	std::vector<uint32_t> pixels(size_t{ w } * h);

	for (unsigned j = 0; j < h; j++) {
		const uint32_t row = (base + j * 4096) & end;
		uint32_t *const out = &pixels[size_t{ j } * w];

		for (unsigned i = 0; i < w; i++) {
			const uint32_t at = (row + i) & end;
			const bool masked = at < m_texmask.size() && m_texmask[at];

			out[i] = 0xFF000000u | (masked ? 0xFF00u : 0) | m_texture[at];
		}
	}
	image->id = m_next_image++;
	if (m_next_image == 0)
		m_next_image = 1;
	image->width = w;
	image->height = h;
	image->levels.push_back(std::move(pixels));
	m_scene.new_images.push_back(image);
	m_images.emplace(asked, Image{ image->id, m_frame });
	return image->id;
}

void
Gaelco3d::scene_poly(const Poly &poly, Span span, const Vertex *vert, unsigned count, unsigned depth)
{
	struct Corner {
		float x, y, u, v, w;
	};
	Corner c[MOST_CORNERS];
	Vertex ring[2][MOST_CORNERS];
	unsigned side = 0;
	bool trimmed = depth > 0;
	float umin = 1e30f, umax = -1e30f, vmin = 1e30f, vmax = -1e30f;

	count = std::min(count, MOST_CORNERS);
	std::copy_n(vert, count, ring[0]);

	const auto cut = [&](auto inside) {
		const Vertex *const from = ring[side];
		Vertex *const to = ring[side ^ 1];
		unsigned n = 0;

		for (unsigned i = 0; i < count; i++) {
			const Vertex &a = from[(i + count - 1) % count], &b = from[i];
			const float da = inside(a), db = inside(b);

			if ((da >= 0.0f) != (db >= 0.0f) && n < MOST_CORNERS) {
				const float t = da / (da - db);

				to[n++] = { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t };
				trimmed = true;
			}
			if (db >= 0.0f && n < MOST_CORNERS)
				to[n++] = b;
		}
		trimmed = trimmed || n != count;
		count = n;
		side ^= 1;
	};

	cut([](const Vertex &k) { return k.x; });
	cut([](const Vertex &k) { return WIDTH - k.x; });
	cut([](const Vertex &k) { return k.y; });
	cut([](const Vertex &k) { return HEIGHT - k.y; });

	const unsigned on_screen = side, screen_count = count;
	const auto ooz_at = [&poly](const Vertex &k) {
		return poly.ooz_base + (k.y - 0.5f) * poly.ooz_dy + (k.x - 0.5f) * poly.ooz_dx;
	};
	float most = 0.0f;

	for (unsigned i = 0; i < count; i++)
		most = std::max(most, ooz_at(ring[side][i]));
	if (span != Span::FLAT && most <= 0.0f)
		return;
	for (float nearest = NEAREST;; nearest *= 4.0f) {
		side = on_screen;
		count = screen_count;
		if (span != Span::FLAT && count >= 3) {
			const float near = most * nearest;

			cut([&](const Vertex &k) { return ooz_at(k) - near; });
		}
		if (count < 3)
			return;
		vert = ring[side];
		umin = vmin = 1e30f;
		umax = vmax = -1e30f;
		for (unsigned i = 0; i < count; i++) {
			const float px = vert[i].x - 0.5f, py = vert[i].y - 0.5f;
			float u, v, zval;

			if (span == Span::FLAT) {
				const float zbase = 1.0f / poly.ooz_base;

				u = (poly.uoz_base + py * poly.uoz_dy + px * poly.uoz_dx) * zbase;
				v = (poly.voz_base + py * poly.voz_dy + px * poly.voz_dx) * zbase;
				zval = -poly.z0 * zbase;
			} else {
				const float ooz = ooz_at(vert[i]);

				u = (poly.uoz_base + py * poly.uoz_dy + px * poly.uoz_dx) / ooz;
				v = (poly.voz_base + py * poly.voz_dy + px * poly.voz_dx) / ooz;
				zval = poly.z0 / ooz;
			}
			c[i] = { vert[i].x * SCENE_X, (HEIGHT - vert[i].y) * SCENE_Y, u / 256.0f, v / 256.0f,
			         1.0f / std::max(std::fabs(zval), 0.5f) };
			umin = std::min(umin, c[i].u);
			umax = std::max(umax, c[i].u);
			vmin = std::min(vmin, c[i].v);
			vmax = std::max(vmax, c[i].v);
		}
		if (span == Span::FLAT || nearest > 0.25f ||
		    std::max(umax - umin, vmax - vmin) <= static_cast<float>(FAR_SIDE))
			break;
	}

	const int64_t fu0 = static_cast<int64_t>(std::floor(umin)), fv0 = static_cast<int64_t>(std::floor(vmin));
	const int64_t fu1 = static_cast<int64_t>(std::floor(umax)) + 1, fv1 = static_cast<int64_t>(std::floor(vmax)) + 1;
	const uint32_t end = static_cast<uint32_t>(m_texture.size() - 1);
	const uint32_t start = static_cast<uint32_t>(static_cast<int64_t>(poly.tex) + fv0 * 4096 + fu0) & end;
	const uint32_t col = start & 4095, row = start >> 12;
	const uint32_t bank = poly.color >> 8;
	int64_t u0, v0;
	unsigned w, h;
	uint32_t base;

	const uint32_t slack = trimmed ? (TRIMMED_SIDE - CELL) / 2 : 0, unit = trimmed ? CELL : TILE;
	const uint32_t tile_col = (col + slack) / unit * unit, tile_row = (row + slack) / unit * unit;
	const uint32_t left = tile_col - std::min(tile_col, slack), top = tile_row - std::min(tile_row, slack);
	const auto round_up = [](int64_t t) { return static_cast<unsigned>((t + TILE - 1) / TILE * TILE); };
	const int64_t need_w = col - left + fu1 - fu0 + 1, need_h = row - top + fv1 - fv0 + 1;
	const unsigned tw = trimmed ? std::min(TRIMMED_SIDE, 4096 - left) : round_up(need_w);
	const unsigned th = trimmed ? TRIMMED_SIDE : round_up(need_h);

	if (need_w <= tw && need_h <= th && left + tw <= 4096 && tw <= MOST_SIDE && th <= MOST_SIDE) {
		u0 = fu0 - (col - left);
		v0 = fv0 - (row - top);
		w = tw;
		h = th;
		base = top * 4096 + left;
	} else {
		const auto down = [](int64_t t) { return t >= 0 ? t / BLOCK * BLOCK : -((-t + BLOCK - 1) / BLOCK) * BLOCK; };

		u0 = down(fu0);
		v0 = down(fv0);
		if (fu1 + 1 - u0 > MOST_SIDE || fv1 + 1 - v0 > MOST_SIDE || (trimmed && depth < MOST_SPLITS)) {
			if (depth == MOST_SPLITS) {
				tell_once("polygon of too many texels", poly.tex, static_cast<uint32_t>(fu1 - u0));
				return;
			}

			const auto edge_in = [](float low, float high, uint32_t off) {
				float edge = std::round(((low + high) / 2.0f + static_cast<float>(off)) / CELL) * CELL - static_cast<float>(off);

				if (edge <= low + 0.5f)
					edge += CELL;
				else if (edge >= high - 0.5f)
					edge -= CELL;
				return edge > low + 0.5f && edge < high - 0.5f ? std::optional<float>(edge) : std::nullopt;
			};
			const std::optional<float> eu = edge_in(umin, umax, poly.tex & 4095), ev = edge_in(vmin, vmax, poly.tex >> 12);
			const bool wide = need_w > tw, tall = need_h > th;
			const bool across = eu && (wide || !ev || !tall) ? true : ev ? false : umax - umin >= vmax - vmin;
			const std::optional<float> edge = across ? eu : ev;
			const float mid = (edge ? *edge : across ? (umin + umax) / 2.0f : (vmin + vmax) / 2.0f) * 256.0f;
			const auto past_mid = [&](const Vertex &k) {
				const float px = k.x - 0.5f, py = k.y - 0.5f;
				const float oz = span == Span::FLAT ? poly.ooz_base : ooz_at(k);
				const float t = across ? poly.uoz_base + py * poly.uoz_dy + px * poly.uoz_dx
				                       : poly.voz_base + py * poly.voz_dy + px * poly.voz_dx;

				return t - mid * oz;
			};
			const unsigned whole = side, whole_count = count;

			for (const float sign : { 1.0f, -1.0f }) {
				Vertex piece[MOST_CORNERS];

				side = whole;
				count = whole_count;
				cut([&](const Vertex &k) { return sign * past_mid(k); });
				if (count >= 3) {
					std::copy_n(ring[side], count, piece);
					scene_poly(poly, span, piece, count, depth + 1);
				}
			}
			return;
		}
		w = static_cast<unsigned>(down(fu1) + BLOCK - u0);
		h = static_cast<unsigned>(down(fv1) + BLOCK - v0);
		base = static_cast<uint32_t>(static_cast<int64_t>(poly.tex) + v0 * 4096 + u0);
	}

	const uint32_t asked = base;
	const uint32_t image = image_of(base, w, h);

	u0 -= static_cast<int64_t>(asked & 4095) - static_cast<int64_t>(base & 4095);
	v0 -= static_cast<int64_t>(asked >> 12) - static_cast<int64_t>(base >> 12);

	PmxDraw draw{};

	draw.image = image;
	draw.compare = span == Span::FLAT || poly.z0 < 0 ? COMPARE_ALWAYS : COMPARE_GREATER;
	draw.write_z = true;
	draw.shading = 0;
	draw.fog = 2;
	draw.wrap_u = draw.wrap_v = PmxWrap::CLAMP;
	draw.smooth = true;
	draw.alpha_test = true;
	draw.indexed = true;
	draw.bank = static_cast<uint8_t>(bank);
	draw.use_alpha = span == Span::BLEND;
	draw.ignore_texture_alpha = span != Span::BLEND;
	draw.blend = span == Span::BLEND;
	draw.src = SRC_ALPHA;
	draw.dst = INV_SRC_ALPHA;

	const uint32_t first = static_cast<uint32_t>(m_scene.vertices.size());
	const auto put = [&](const Corner &k) {
		m_scene.vertices.push_back({ k.x, k.y, k.w, (k.u - static_cast<float>(u0) + 0.5f) / static_cast<float>(w),
		                             (k.v - static_cast<float>(v0) + 0.5f) / static_cast<float>(h), 0xFFFFFFFFu, 0 });
		m_most_w = std::max(m_most_w, k.w);
	};

	for (unsigned i = 2; i < count; i++) {
		put(c[0]);
		put(c[i - 1]);
		put(c[i]);
	}

	const uint32_t added = static_cast<uint32_t>(m_scene.vertices.size()) - first;

	if (!m_scene.draws.empty()) {
		PmxDraw &last = m_scene.draws.back();

		if (last.image == draw.image && last.bank == draw.bank && last.compare == draw.compare && last.blend == draw.blend &&
		    last.first + last.count == first) {
			last.count += added;
			m_scene.rendered = true;
			return;
		}
	}
	draw.first = first;
	draw.count = added;
	m_scene.draws.push_back(draw);
	m_scene.rendered = true;
}

void
Gaelco3d::end_scene()
{
	const size_t had = m_gone.size();

	for (auto it = m_images.begin(); it != m_images.end();)
		if (m_frame - it->second.used > KEPT_FRAMES) {
			m_gone.push_back(it->second.id);
			it = m_images.erase(it);
		} else {
			++it;
		}
	if (m_gone.size() != had)
		std::erase_if(m_aliases, [this](const auto &alias) { return !m_images.contains(alias.second); });
	m_scene.gone_images.insert(m_scene.gone_images.end(), m_gone.begin(), m_gone.end());
	m_gone.clear();
	if (!m_scene.rendered) {
		m_carry.new_images.insert(m_carry.new_images.end(), m_scene.new_images.begin(), m_scene.new_images.end());
		m_carry.gone_images.insert(m_carry.gone_images.end(), m_scene.gone_images.begin(), m_scene.gone_images.end());
	} else {
		m_scene.new_images.insert(m_scene.new_images.begin(), m_carry.new_images.begin(), m_carry.new_images.end());
		m_scene.gone_images.insert(m_scene.gone_images.begin(), m_carry.gone_images.begin(), m_carry.gone_images.end());
		m_carry = {};
		m_scene.most_w = std::max(m_most_w, 1e-6f);
		if (m_palette_changed) {
			m_scene.palette.resize(m_palette.size());
			for (size_t i = 0; i < m_palette.size(); i++) {
				const uint32_t c = m_palette[i];

				m_scene.palette[i] = static_cast<uint16_t>(((c >> 16) & 31) << 10 | ((c >> 11) & 31) << 5 | (c & 31) |
				                                           ((i >> 8) == BLEND_BANK ? 0x8000 : 0));
			}
			m_palette_changed = false;
		}
		m_ready = std::move(m_scene);
	}
	m_scene = {};
	m_most_w = 0.0f;
}
}
