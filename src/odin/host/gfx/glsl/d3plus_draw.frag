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

/* Rolling Extreme above the board's resolution: a point of a packet of its
 * 3D chip D3PLUS-4, worked out as the chip's drawing does at 1x (d3plus.cpp,
 * draw), at the point's own place. Its depth goes to
 * the depth test, "less or equal" (the later wins a tie) */

#ifdef GL_ES
precision highp usampler2D;
#endif

#ifdef VULKAN
layout(set = 0, binding = 0) uniform usampler2D u_sheet;
layout(set = 0, binding = 1) uniform usampler2D u_table;
layout(set = 0, binding = 2) uniform usampler2D u_palette;
layout(push_constant) uniform Draw {
	vec2 step;
} draw;
#define STEP draw.step
#else
/* u_sheet: the texture ROMs' 2048 x 16384 texels as 4096 x 8192; u_table:
 * the table of 0x90000000, 256 bytes a texel row; u_palette: 128 rows of
 * 256 words a piece; u_step: the board's points a pixel here */
uniform usampler2D u_sheet;
uniform usampler2D u_table;
uniform usampler2D u_palette;
uniform vec2 u_step;
#define STEP u_step
#endif

layout(location = 0) flat in vec4 v_q;
layout(location = 1) flat in vec3 v_u;
layout(location = 2) flat in vec3 v_v;
layout(location = 3) flat in uvec2 v_word;
layout(location = 4) flat in ivec4 v_rect;

layout(location = 0) out vec4 o_colour;

const uint NATIVE = 0x10000u;

/* floor(n / q) as a whole number of texels, within 2^30 (the sheet wraps
 * far below). The GPU's division need not be exact: where n / q is a whole
 * number k, as on the text's texels at the board's points, k * q == n says
 * so; the quotient is moved to the k with k <= n / q < k + 1 */
uint
whole(float n, float q)
{
	float k = floor(n / q);

	if (q > 0.0 ? (k + 1.0) * q <= n : (k + 1.0) * q >= n)
		k += 1.0;
	else if (q > 0.0 ? k * q > n : k * q < n)
		k -= 1.0;
	return uint(int(clamp(k, -1073741824.0, 1073741824.0)));
}

void
main()
{
	/* The point's place in the board's coordinates, from its own pixel:
	 * the same for every packet, so equal depths stay equal */
	float x = gl_FragCoord.x * STEP.x - 256.5;
	float y = gl_FragCoord.y * STEP.y - 191.5;
	vec4 r = vec4(v_rect);

	if ((v_word.y & NATIVE) != 0u) {
		/* Native 2D: the board's own point, as at 1x */
		x = clamp(floor(x + 0.5), r.x, r.y);
		y = clamp(ceil(y - 0.5), r.z, r.w);
	} else if (x < r.x - 0.5 || x >= r.y + 0.5 || y <= r.z - 0.5 || y > r.w + 0.5) {
		/* Off the clipping rectangle's points */
		discard;
	}

	float q = (v_q.x * x + v_q.y * y) + v_q.z;

	if (q == 0.0)
		discard;

	/* PROVEN: z = (w4 >> 7) / Q, HEURISTIC (as at 1x) its Q without the
	 * lowest bit of its three floats; u and v from the texture's origin */
	vec3 qz = uintBitsToFloat(floatBitsToUint(v_q.xyz) & uvec3(~1u));
	float z = v_q.w / ((qz.x * x + qz.y * y) + qz.z);
	float nu = (v_u.x * x + v_u.y * y) + v_u.z, nv = (v_v.x * x + v_v.y * y) + v_v.z;
	float u = nu / q, v = nv / q;

	if (isnan(u) || isinf(u) || isnan(v) || isinf(v))
		discard;

	uint tx = ((v_word.x & 0x7FFu) + whole(nu, q)) & 2047u;
	uint ty = ((v_word.x >> 11) + whole(nv, q)) & 16383u;

	/* PROVEN: the table's bit of the texel, rows below 2048 only; 1, not
	 * written */
	if (ty < 2048u && ((texelFetch(u_table, ivec2(int((tx >> 3) ^ 1u), int(ty)), 0).r >> (tx & 7u)) & 1u) != 0u)
		discard;

	uint mode = v_word.y;
	uint texel = texelFetch(u_sheet, ivec2(int(tx | (ty & 1u) << 11), int(ty >> 1)), 0).r;
	uint colour = texelFetch(u_palette, ivec2(int(texel), int((mode & 0x7Fu) + ((mode >> 12) & 7u) * 128u)), 0).r;
	/* x RRRRR GGGGG BBBBB, 8 bits as c * 255 / 31 (HEURISTIC, as at 1x);
	 * bit 15 unknown, ignored */
	uvec3 level = uvec3((colour >> 10) & 31u, (colour >> 5) & 31u, colour & 31u) * 255u / 31u;
	uint opacity = (mode >> 8) & 15u;

	/* HEURISTIC, as at 1x: L (not 0) the share of the point over what is
	 * there, out of 16 */
	o_colour = vec4(vec3(level) / 255.0, opacity == 0u ? 1.0 : float(opacity) / 16.0);
	/* MEASURED: z within 0.999 and 64,000 where it is seen; below 0 it
	 * passes any */
	gl_FragDepth = z < 0.0 ? 0.0 : min(z * (1.0 / 65536.0), 1.0);
}
