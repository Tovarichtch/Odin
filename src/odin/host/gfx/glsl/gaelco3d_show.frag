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

/* Gaelco 3D above the board's resolution: the colours as the board makes
 * them when it scans out its frame (gaelco3d_video.cpp, colour()): four
 * texels, the current palette, CUS-COL's filter, 8 bits per channel. Bank
 * 0x7F, which the board writes into one pixel of each pair per field, is
 * mixed half and half with what is under it (HEURISTIC: the eye's mean of
 * the two fields) */

#ifdef GL_ES
precision highp usampler2D;
#endif

#ifdef VULKAN
layout(set = 0, binding = 0) uniform usampler2D u_word;
layout(set = 0, binding = 1) uniform usampler2D u_see;
layout(set = 0, binding = 2) uniform usampler2D u_texture;
layout(set = 0, binding = 3) uniform usampler2D u_palette;
layout(push_constant) uniform Show {
	uint texture_end;
} show;
#define TEXTURE_END show.texture_end
#else
/* u_word, u_see: the words of the pixels (gaelco3d_draw.frag); u_texture:
 * the texture ROMs, rows of 4096 texels; u_palette: 128 banks of 256
 * words; u_texture_end: the last texel address */
uniform usampler2D u_word;
uniform usampler2D u_see;
uniform usampler2D u_texture;
uniform usampler2D u_palette;
uniform uint u_texture_end;
#define TEXTURE_END u_texture_end
#endif

layout(location = 0) out vec4 o_color;

const uint U_FIELD = 0xFFFu, V_FIELD = 0x3FF000u, SEEN = 0x100u;

uint
texel_at(uint a)
{
	a &= TEXTURE_END;
	return texelFetch(u_texture, ivec2(int(a & U_FIELD), int(a >> 12)), 0).r;
}

/* Red, green, blue as in MAME (15 bits reach CUS-COL) */
uvec3
lanes(uint bank, uint t)
{
	uint c = texelFetch(u_palette, ivec2(int(t), int(bank)), 0).r;

	return uvec3(c >> 10 & 31u, c >> 5 & 31u, c & 31u);
}

uvec3
colour(uvec2 w)
{
	uint a = w.x & 0x1FFFFFFu, bank = w.x >> 25, fu = w.y & 15u, fv = (w.y >> 4) & 15u;
	/* The neighbours wrap in their row of 4096 texels and their page of
	 * 1024 rows */
	uint r = (a & ~U_FIELD) | ((a + 1u) & U_FIELD);
	uint b = (a & ~V_FIELD) | ((a + 0x1000u) & V_FIELD);
	uint br = (b & ~U_FIELD) | ((b + 1u) & U_FIELD);
	/* HEURISTIC (MAME is the only source): the four texels weighted by the
	 * 4-bit fractions, keeping 8 of the 13 bits of each channel */
	uvec3 sum = lanes(bank, texel_at(a)) * ((16u - fu) * (16u - fv)) + lanes(bank, texel_at(r)) * (fu * (16u - fv)) +
	            lanes(bank, texel_at(b)) * ((16u - fu) * fv) + lanes(bank, texel_at(br)) * (fu * fv);

	return (sum >> 5u) & 0xFFu;
}

void
main()
{
	ivec2 at = ivec2(gl_FragCoord.xy);
	uvec3 c = colour(texelFetch(u_word, at, 0).xy);
	uvec2 see = texelFetch(u_see, at, 0).xy;

	if ((see.y & SEEN) != 0u)
		c = (c + colour(see)) >> 1u;
	o_color = vec4(vec3(c) / 255.0, 1.0);
}
