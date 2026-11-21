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

/* Gaelco 3D above the board's resolution: a piece of a polygon, in the
 * board's pixels from its top left corner (Gaelco3dVertex); its terms pass
 * unchanged to each pixel (gaelco3d_draw.frag) */

layout(location = 0) in vec2 a_at;
layout(location = 1) in vec4 a_k0;
layout(location = 2) in vec4 a_k4;
layout(location = 3) in vec2 a_k8;
layout(location = 4) in uvec2 a_word;

layout(location = 0) out vec2 v_at;
layout(location = 1) flat out vec4 v_k0;
layout(location = 2) flat out vec4 v_k4;
layout(location = 3) flat out vec2 v_k8;
layout(location = 4) flat out uvec2 v_word;

void
main()
{
	/* The board's 576 x 432 over the whole picture, with its top line at
	 * the top, as pmx.vert does */
	gl_Position = vec4(a_at.x / 288.0 - 1.0, 1.0 - a_at.y / 216.0, 0.0, 1.0);
	v_at = a_at;
	v_k0 = a_k0;
	v_k4 = a_k4;
	v_k8 = a_k8;
	v_word = a_word;
}
