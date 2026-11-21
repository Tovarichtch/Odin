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

#include <cstddef>

#include "odin/host/gfx/glsl_texts.hpp"
#include "odin/host/renderer/hikaru/gl/drawing_gl.hpp"

/* Draws a field of a Gaelco 3D board above the board's resolution, in the
 * board's two steps: the pixels' words (gaelco3d_draw.frag), then their
 * colours (gaelco3d_show.frag) into the scene's picture */

namespace odin {

struct GlDrawing::Gaelco3d {
	GLuint draw = 0, show = 0;
	GLint mask_off = -1, fields = -1, texture_end = -1;
	GLuint vao = 0, vbo = 0, empty = 0;
	/* The ROMs (kept while loaded) and the palette */
	GLuint texture = 0, mask = 0, palette = 0;
	std::shared_ptr<const Gaelco3dRoms> roms;
	/* The pixels' words, with those of bank 0x7F kept apart, and the depth:
	 * kept from field to field, as the board keeps its frame */
	GLuint fb = 0, word = 0, see = 0, depth = 0;
	unsigned w = 0, h = 0;

	~Gaelco3d()
	{
		glDeleteFramebuffers(1, &fb);
		const GLuint textures[] = { word, see, depth, texture, mask, palette };

		glDeleteTextures(6, textures);
		glDeleteBuffers(1, &vbo);
		glDeleteVertexArrays(1, &vao);
		glDeleteVertexArrays(1, &empty);
		glDeleteProgram(draw);
		glDeleteProgram(show);
	}
};

namespace {

/* The attachments a run draws to: the words and those of bank 0x7F, or only
 * the latter (a run of bank 0x7F) */
constexpr GLenum BOTH[] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1 };
constexpr GLenum SEE_ONLY[] = { GL_NONE, GL_COLOR_ATTACHMENT1 };

GLuint
integer_texture(GLenum format, unsigned w, unsigned h)
{
	GLuint id;

	glGenTextures(1, &id);
	glBindTexture(GL_TEXTURE_2D, id);
	gl::storage(1, format, static_cast<GLsizei>(w), static_cast<GLsizei>(h));
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glBindTexture(GL_TEXTURE_2D, 0);
	return id;
}

} /* namespace */

void
GlDrawing::draw_gaelco3d(const Gaelco3dScene &scene)
{
	if (!m_gaelco3d) {
		m_gaelco3d = std::make_shared<Gaelco3d>();
		Gaelco3d &g = *m_gaelco3d;

		g.draw = m_programs.link("gaelco3d-draw", odin_glsl_gaelco3d_draw_vert, odin_glsl_gaelco3d_draw_frag, "");
		g.mask_off = gl::uniform(g.draw, "u_mask_off");
		g.fields = gl::uniform(g.draw, "u_fields");
		glUseProgram(g.draw);
		glUniform1i(gl::uniform(g.draw, "u_mask"), 0);
		/* Compiled each time, never read back from a kept binary: Intel's
		 * Windows driver (HD 520, 21.20.16.5174) recompiles the one read
		 * back once the menu bar has drawn, and it then writes no pixel */
		g.show = gl::compile_program(odin_glsl_whole_picture_vert, odin_glsl_gaelco3d_show_frag);
		g.texture_end = gl::uniform(g.show, "u_texture_end");
		glUseProgram(g.show);
		glUniform1i(gl::uniform(g.show, "u_word"), 0);
		glUniform1i(gl::uniform(g.show, "u_see"), 1);
		glUniform1i(gl::uniform(g.show, "u_texture"), 2);
		glUniform1i(gl::uniform(g.show, "u_palette"), 3);
		glUseProgram(0);

		glGenVertexArrays(1, &g.vao);
		glGenVertexArrays(1, &g.empty);
		glGenBuffers(1, &g.vbo);
		glBindVertexArray(g.vao);
		glBindBuffer(GL_ARRAY_BUFFER, g.vbo);
		for (GLuint i = 0; i < 5; i++)
			glEnableVertexAttribArray(i);

		const auto at = [](size_t offset) { return reinterpret_cast<const void *>(offset); };
		const GLsizei size = sizeof(Gaelco3dVertex);

		glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, size, at(offsetof(Gaelco3dVertex, x)));
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, size, at(offsetof(Gaelco3dVertex, k)));
		glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, size, at(offsetof(Gaelco3dVertex, k) + 4 * sizeof(float)));
		glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, size, at(offsetof(Gaelco3dVertex, k) + 8 * sizeof(float)));
		glVertexAttribIPointer(4, 2, GL_UNSIGNED_INT, size, at(offsetof(Gaelco3dVertex, texture)));
		glBindVertexArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		g.palette = integer_texture(GL_R16UI, 256, 128);
	}

	Gaelco3d &g = *m_gaelco3d;

	if (scene.roms && scene.roms != g.roms && !scene.roms->mask.empty()) {
		const Gaelco3dRoms &roms = *scene.roms;
		const unsigned rows = static_cast<unsigned>(roms.texture.size() / 4096);

		/* Never sent: the game shows the board's own picture then
		 * (Gaelco3dGame) */
		if (rows > largest_texture())
			return;
		glDeleteTextures(1, &g.texture);
		glDeleteTextures(1, &g.mask);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		g.texture = integer_texture(GL_R8UI, 4096, rows);
		glBindTexture(GL_TEXTURE_2D, g.texture);
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 4096, static_cast<GLsizei>(rows), GL_RED_INTEGER, GL_UNSIGNED_BYTE,
		                roms.texture.data());
		glBindTexture(GL_TEXTURE_2D, 0);
		g.mask = integer_texture(GL_R8UI, 4096, 1024);
		glBindTexture(GL_TEXTURE_2D, g.mask);
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 4096, 1024, GL_RED_INTEGER, GL_UNSIGNED_BYTE, roms.mask.data());
		glBindTexture(GL_TEXTURE_2D, 0);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
		g.roms = scene.roms;
	}
	if (!scene.palette.empty()) {
		glPixelStorei(GL_UNPACK_ALIGNMENT, 2);
		glBindTexture(GL_TEXTURE_2D, g.palette);
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 256, 128, GL_RED_INTEGER, GL_UNSIGNED_SHORT, scene.palette.data());
		glBindTexture(GL_TEXTURE_2D, 0);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
	}

	const GLsizei w = static_cast<GLsizei>(width()), h = static_cast<GLsizei>(height());

	if (g.w != width() || g.h != height()) {
		static const GLuint zero[4] = { 0, 0, 0, 0 };
		static const GLfloat nearest = 0.0f;

		glDeleteFramebuffers(1, &g.fb);
		const GLuint old[] = { g.word, g.see, g.depth };

		glDeleteTextures(3, old);
		g.word = integer_texture(GL_RG32UI, width(), height());
		g.see = integer_texture(GL_RG32UI, width(), height());
		g.depth = gl::new_texture(GL_DEPTH_COMPONENT32F, width(), height());
		glGenFramebuffers(1, &g.fb);
		glBindFramebuffer(GL_FRAMEBUFFER, g.fb);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, g.word, 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, g.see, 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, g.depth, 0);
		glDrawBuffers(2, BOTH);
		if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
			gl::fatal("the framebuffer of the Gaelco 3D is not complete");
		/* The board's frame and depth at power-on: zero */
		glClearBufferuiv(GL_COLOR, 0, zero);
		glClearBufferuiv(GL_COLOR, 1, zero);
		glDepthMask(GL_TRUE);
		glClearBufferfv(GL_DEPTH, 0, &nearest);
		g.w = width();
		g.h = height();
	}

	/* The words: the polygons in the board's order, with the depth compared
	 * as the board compares it; those of bank 0x7F leave the other words
	 * alone */
	glBindFramebuffer(GL_FRAMEBUFFER, g.fb);
	glViewport(0, 0, w, h);
	glDisable(GL_SCISSOR_TEST);
	glDisable(GL_CULL_FACE);
	glDisable(GL_BLEND);
	glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	glUseProgram(g.draw);
	glUniform1ui(g.mask_off, scene.roms ? scene.roms->mask_off : 0);
	glUniform1i(g.fields, scene.both_fields ? static_cast<GLint>(2 | (scene.field & 1)) : 0);
	glActiveTexture(GL_TEXTURE0);
	glBindSampler(0, 0);
	glBindTexture(GL_TEXTURE_2D, g.mask);
	glBindVertexArray(g.vao);
	glBindBuffer(GL_ARRAY_BUFFER, g.vbo);
	glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(scene.vertices.size() * sizeof(Gaelco3dVertex)),
	             scene.vertices.data(), GL_STREAM_DRAW);
	for (const Gaelco3dRun &run : scene.runs) {
		glDrawBuffers(2, run.see_through ? SEE_ONLY : BOTH);
		glDepthFunc(run.always ? GL_ALWAYS : GL_LESS);
		glDrawArrays(GL_TRIANGLES, static_cast<GLint>(run.first), static_cast<GLsizei>(run.count));
	}
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glDisable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	/* The colours, into the scene's picture */
	glBindFramebuffer(GL_FRAMEBUFFER, m_scene_fb);
	glViewport(0, 0, w, h);
	glUseProgram(g.show);
	glUniform1ui(g.texture_end, scene.roms ? scene.roms->texture_end : 0);
	const GLuint units[] = { g.word, g.see, g.texture, g.palette };

	for (GLuint unit = 0; unit < 4; unit++) {
		glActiveTexture(GL_TEXTURE0 + unit);
		glBindSampler(unit, 0);
		glBindTexture(GL_TEXTURE_2D, units[unit]);
	}
	glBindVertexArray(g.empty);
	glDrawArrays(GL_TRIANGLES, 0, 3);
	glBindVertexArray(0);
	for (GLuint unit = 0; unit < 4; unit++) {
		glActiveTexture(GL_TEXTURE0 + unit);
		glBindTexture(GL_TEXTURE_2D, 0);
	}
	glActiveTexture(GL_TEXTURE0);
	glUseProgram(0);

	glBindFramebuffer(GL_READ_FRAMEBUFFER, m_scene_fb);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_frame_fb);
	glBlitFramebuffer(0, 0, w, h, 0, 0, w, h, GL_COLOR_BUFFER_BIT, GL_NEAREST);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	gl::check("a field of the Gaelco 3D");
}

} /* namespace odin */
