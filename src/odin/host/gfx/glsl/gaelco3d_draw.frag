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

/* Gaelco 3D above the board's resolution: what CUS-GAEL4 and the PAL IC88
 * give a board pixel (gaelco3d_video.cpp, texel() and line()), with each
 * pixel here computed at its own position. It stores a texel address, a
 * palette bank and two fractions, with those of bank 0x7F kept apart; its
 * depth is the board's 16 bits, compared by the depth test
 * (gaelco3d_gl.cpp) */

#ifdef GL_ES
precision highp usampler2D;
#endif

#ifdef VULKAN
layout(set = 0, binding = 0) uniform usampler2D u_mask;
layout(push_constant) uniform Draw {
	uint mask_off;
	int fields;
} draw;
#define MASK_OFF draw.mask_off
#define FIELDS draw.fields
#else
/* u_mask: one byte per texel of the first page, 1 for transparent;
 * u_mask_off: the address bits for which no mask answers; u_fields: 0
 * for every line, else 2 | the parity of the lines drawn */
uniform usampler2D u_mask;
uniform uint u_mask_off;
uniform int u_fields;
#define MASK_OFF u_mask_off
#define FIELDS u_fields
#endif

layout(location = 0) in vec2 v_at;
layout(location = 1) flat in vec4 v_k0;
layout(location = 2) flat in vec4 v_k4;
layout(location = 3) flat in vec2 v_k8;
layout(location = 4) flat in uvec2 v_word;

layout(location = 0) out uvec2 o_word;
layout(location = 1) out uvec2 o_see;

const float TOP = 215.0, LEFT = -288.0;
const uint ADDRESS = 0x1FFFFFFu, U_FIELD = 0xFFFu, V_FIELD = 0x3FF000u, PAGE = 0x1C00000u;
const uint SEE_THROUGH = 0x7Fu, NATIVE = 0x100u, SEEN = 0x100u;

int
saturated(float t)
{
	if (isnan(t))
		return 0;
	return int(clamp(t, -1073741824.0, 1073741824.0));
}

/* An address, and the fractions u (bits 0-3) and v (4-7) */
uvec2
texel(float x, float k23, float k89, float d)
{
	int u = saturated(floor((x * v_k0.y + k23) / d * 16.0));
	int v = saturated(floor((x * v_k4.w + k89) / d * 16.0));

	return uvec2((v_word.x + uint(v >> 4) * 4096u + uint(u >> 4)) & ADDRESS, (uint(u) & 15u) | (uint(v) & 15u) << 4);
}

bool
masked(uint a)
{
	if ((a & MASK_OFF) != 0u)
		return false;
	return texelFetch(u_mask, ivec2(int(a & U_FIELD), int((a >> 12) & 0x3FFu)), 0).r != 0u;
}

void
main()
{
	bool native = (v_word.y & NATIVE) != 0u;

	if (FIELDS != 0 && (int(floor(v_at.y)) & 1) != (FIELDS & 1))
		discard;

	/* A board pixel takes the terms at its whole x and y; at a finer
	 * resolution, each pixel takes them at its own position, shifted half a
	 * board pixel up and left so the picture lands where the board's
	 * does */
	float x, y;

	if (native) {
		x = floor(v_at.x) + LEFT;
		y = TOP - floor(v_at.y);
	} else {
		x = v_at.x - 0.5 + LEFT;
		y = TOP - (v_at.y - 0.5);
	}

	float k23 = y * v_k0.z + v_k0.w, k89 = y * v_k8.x + v_k8.y, k56 = y * v_k4.y + v_k4.z;
	float d = x * v_k4.x + k56;

	/* HEURISTIC (MAME): no pixel where the plane is behind the eye */
	if (!(d > 0.0))
		discard;

	uvec2 t;

	if (native && (int(floor(v_at.x)) & 1) == 1) {
		/* On the board's pixels, the right pixel of a pair: the mean of
		 * its neighbours' u (12.4) and v (10.4), with the page of the
		 * left one */
		float dl = (x - 1.0) * v_k4.x + k56, dn = (x + 1.0) * v_k4.x + k56;
		uvec2 l = texel(x - 1.0, k23, k89, dl), n = texel(x + 1.0, k23, k89, dn);
		uint u = (((l.x & U_FIELD) << 4 | (l.y & 15u)) + ((n.x & U_FIELD) << 4 | (n.y & 15u))) >> 1;
		uint v = (((l.x & V_FIELD) >> 8 | l.y >> 4) + ((n.x & V_FIELD) >> 8 | n.y >> 4)) >> 1;

		t = uvec2((l.x & PAGE) | (v >> 4) << 12 | u >> 4, (u & 15u) | (v & 15u) << 4);
	} else {
		t = texel(x, k23, k89, d);
	}
	if (masked(t.x))
		discard;

	/* The board's depth, 16 bits (HEURISTIC, MAME is the only source):
	 * rounded toward zero; a negative depth always passes and keeps its
	 * absolute value */
	int z = saturated(trunc(v_k0.x / d));

	if (v_k0.x < 0.0)
		gl_FragDepth = float(uint(abs(z)) & 0xFFFFu) / 65536.0;
	else
		gl_FragDepth = z > 0xFFFF ? 1.0 : float(z) / 65536.0;

	uint bank = v_word.y & 0x7Fu;

	o_word = uvec2(t.x | bank << 25, t.y);
	o_see = bank == SEE_THROUGH ? uvec2(t.x | bank << 25, t.y | SEEN) : uvec2(0u);
}
