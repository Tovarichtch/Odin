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

/* Rolling Extreme above the board's resolution: a triangle of a packet of
 * its 3D chip (d3plus_scene.hpp), in the board's coordinates; the packet's
 * terms go to every point as they are (d3plus_draw.frag) */

layout(location = 0) in vec2 a_at;
layout(location = 1) in vec4 a_q;
layout(location = 2) in vec3 a_u;
layout(location = 3) in vec3 a_v;
layout(location = 4) in uvec2 a_word;
layout(location = 5) in ivec4 a_rect;

layout(location = 0) flat out vec4 v_q;
layout(location = 1) flat out vec3 v_u;
layout(location = 2) flat out vec3 v_v;
layout(location = 3) flat out uvec2 v_word;
layout(location = 4) flat out ivec4 v_rect;

void
main()
{
	/* The board's 512 x 384 over the whole picture, its point (px, py)
	 * centred on (px - 256, 192 - py), its top line at the top as
	 * pmx.vert has it */
	gl_Position = vec4((a_at.x + 256.5) / 256.0 - 1.0, (a_at.y - 192.5) / 192.0 + 1.0, 0.0, 1.0);
	v_q = a_q;
	v_u = a_u;
	v_v = a_v;
	v_word = a_word;
	v_rect = a_rect;
}
