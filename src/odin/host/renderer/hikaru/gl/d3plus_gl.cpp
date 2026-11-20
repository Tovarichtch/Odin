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

/* Draws a picture of Rolling Extreme's 3D chip above the board's resolution:
 * its packets in order, each point coloured as the chip colours it
 * (d3plus_draw.frag), with a depth buffer, into the scene's picture */

namespace odin {

struct GlDrawing::D3plus {
	GLuint program = 0;
	GLint step = -1;
	GLuint vao = 0, vbo = 0;
	/* The texture ROMs (kept while loaded), the table, the palettes of the
	 * pieces, and what each was made from */
	GLuint sheet = 0, table = 0, palettes = 0;
	std::shared_ptr<const std::vector<uint8_t>> sheet_of, table_of;
	std::shared_ptr<const std::vector<uint16_t>> palette_of[D3plusScene::PALETTES];
	/* The depth of the picture drawn into the scene's colours */
	GLuint fb = 0, depth = 0;
	unsigned w = 0, h = 0;

	~D3plus()
	{
		glDeleteFramebuffers(1, &fb);
		const GLuint textures[] = { depth, sheet, table, palettes };

		glDeleteTextures(4, textures);
		glDeleteBuffers(1, &vbo);
		glDeleteVertexArrays(1, &vao);
		glDeleteProgram(program);
	}
};

namespace {

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

void
fill_texture(GLuint texture, int y, unsigned w, unsigned h, GLenum type, const void *bytes)
{
	glBindTexture(GL_TEXTURE_2D, texture);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, y, static_cast<GLsizei>(w), static_cast<GLsizei>(h), GL_RED_INTEGER, type,
	                bytes);
	glBindTexture(GL_TEXTURE_2D, 0);
}

} /* namespace */

void
GlDrawing::draw_d3plus(const D3plusScene &scene)
{
	constexpr unsigned PALETTE_ROWS = 128, TABLE_W = 256, TABLE_H = 2048;

	if (!m_d3plus) {
		m_d3plus = std::make_shared<D3plus>();
		D3plus &d = *m_d3plus;

		/* Compiled each time, never read back from a kept binary, as the
		 * Gaelco 3D's (gaelco3d_gl.cpp) */
		d.program = gl::compile_program(odin_glsl_d3plus_draw_vert, odin_glsl_d3plus_draw_frag);
		d.step = gl::uniform(d.program, "u_step");
		glUseProgram(d.program);
		glUniform1i(gl::uniform(d.program, "u_sheet"), 0);
		glUniform1i(gl::uniform(d.program, "u_table"), 1);
		glUniform1i(gl::uniform(d.program, "u_palette"), 2);
		glUseProgram(0);

		glGenVertexArrays(1, &d.vao);
		glGenBuffers(1, &d.vbo);
		glBindVertexArray(d.vao);
		glBindBuffer(GL_ARRAY_BUFFER, d.vbo);
		for (GLuint i = 0; i < 6; i++)
			glEnableVertexAttribArray(i);

		const auto at = [](size_t offset) { return reinterpret_cast<const void *>(offset); };
		const GLsizei size = sizeof(D3plusVertex);

		glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, size, at(offsetof(D3plusVertex, x)));
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, size, at(offsetof(D3plusVertex, q)));
		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, size, at(offsetof(D3plusVertex, u)));
		glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, size, at(offsetof(D3plusVertex, v)));
		glVertexAttribIPointer(4, 2, GL_UNSIGNED_INT, size, at(offsetof(D3plusVertex, texture)));
		glVertexAttribIPointer(5, 4, GL_SHORT, size, at(offsetof(D3plusVertex, rect)));
		glBindVertexArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		d.table = integer_texture(GL_R8UI, TABLE_W, TABLE_H);
		d.palettes = integer_texture(GL_R16UI, 256, PALETTE_ROWS * D3plusScene::PALETTES);
	}

	D3plus &d = *m_d3plus;

	if (scene.sheet && scene.sheet != d.sheet_of) {
		/* Never sent: the game shows the board's own picture then
		 * (RollextGame) */
		if (D3plusScene::SHEET_H > largest_texture() || D3plusScene::SHEET_W > largest_texture())
			return;
		glDeleteTextures(1, &d.sheet);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		d.sheet = integer_texture(GL_R8UI, D3plusScene::SHEET_W, D3plusScene::SHEET_H);
		fill_texture(d.sheet, 0, D3plusScene::SHEET_W, D3plusScene::SHEET_H, GL_UNSIGNED_BYTE, scene.sheet->data());
		glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
		d.sheet_of = scene.sheet;
	}
	if (!d.sheet_of)
		return;
	if (scene.table && scene.table != d.table_of) {
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		fill_texture(d.table, 0, TABLE_W, TABLE_H, GL_UNSIGNED_BYTE, scene.table->data());
		glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
		d.table_of = scene.table;
	}
	for (size_t piece = 0; piece < scene.palettes.size() && piece < D3plusScene::PALETTES; piece++)
		if (scene.palettes[piece] && scene.palettes[piece] != d.palette_of[piece]) {
			glPixelStorei(GL_UNPACK_ALIGNMENT, 2);
			fill_texture(d.palettes, static_cast<int>(piece * PALETTE_ROWS), 256, PALETTE_ROWS, GL_UNSIGNED_SHORT,
			     scene.palettes[piece]->data());
			glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
			d.palette_of[piece] = scene.palettes[piece];
		}

	const GLsizei w = static_cast<GLsizei>(width()), h = static_cast<GLsizei>(height());

	if (d.w != width() || d.h != height()) {
		glDeleteFramebuffers(1, &d.fb);
		glDeleteTextures(1, &d.depth);
		d.depth = gl::new_texture(GL_DEPTH_COMPONENT32F, width(), height());
		d.fb = gl::new_framebuffer(m_scene_color);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, d.depth, 0);
		if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
			gl::fatal("the framebuffer of Rolling Extreme is not complete");
		d.w = width();
		d.h = height();
	}

	/* A picture starts black with its depth cleared (d3plus.cpp,
	 * end_picture); the packets in the chip's order */
	static const GLfloat black[4] = { 0.0f, 0.0f, 0.0f, 1.0f }, far = 1.0f;

	glBindFramebuffer(GL_FRAMEBUFFER, d.fb);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_scene_color, 0);
	glViewport(0, 0, w, h);
	glDisable(GL_SCISSOR_TEST);
	glDisable(GL_CULL_FACE);
	glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
	glDepthMask(GL_TRUE);
	glClearBufferfv(GL_COLOR, 0, black);
	glClearBufferfv(GL_DEPTH, 0, &far);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	/* The colour times its share (1 for an opaque one) over the rest; the
	 * picture stays opaque */
	glEnable(GL_BLEND);
	glBlendEquation(GL_FUNC_ADD);
	glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ZERO, GL_ONE);
	glUseProgram(d.program);
	glUniform2f(d.step, 512.0f / static_cast<float>(w), 384.0f / static_cast<float>(h));

	const GLuint units[] = { d.sheet, d.table, d.palettes };

	for (GLuint unit = 0; unit < 3; unit++) {
		glActiveTexture(GL_TEXTURE0 + unit);
		glBindSampler(unit, 0);
		glBindTexture(GL_TEXTURE_2D, units[unit]);
	}
	glBindVertexArray(d.vao);
	glBindBuffer(GL_ARRAY_BUFFER, d.vbo);
	glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(scene.vertices.size() * sizeof(D3plusVertex)),
	             scene.vertices.data(), GL_STREAM_DRAW);
	for (const D3plusRun &run : scene.runs) {
		/* As at 1x: a translucent packet leaves the depth (HEURISTIC); one
		 * with bit 29 skips the depth test, its depth written where nearer
		 * (HEURISTIC): its colours, then its depth alone */
		const GLint first = static_cast<GLint>(run.first);
		const GLsizei count = static_cast<GLsizei>(run.count);

		glDepthMask(run.translucent || !run.tested ? GL_FALSE : GL_TRUE);
		glDepthFunc(run.tested ? GL_LEQUAL : GL_ALWAYS);
		glDrawArrays(GL_TRIANGLES, first, count);
		if (!run.translucent && !run.tested) {
			glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
			glDepthMask(GL_TRUE);
			glDepthFunc(GL_LEQUAL);
			glDrawArrays(GL_TRIANGLES, first, count);
			glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
		}
	}
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	for (GLuint unit = 0; unit < 3; unit++) {
		glActiveTexture(GL_TEXTURE0 + unit);
		glBindTexture(GL_TEXTURE_2D, 0);
	}
	glActiveTexture(GL_TEXTURE0);
	glUseProgram(0);
	glDisable(GL_BLEND);
	glDisable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glDepthMask(GL_TRUE);

	glBindFramebuffer(GL_READ_FRAMEBUFFER, m_scene_fb);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_frame_fb);
	glBlitFramebuffer(0, 0, w, h, 0, 0, w, h, GL_COLOR_BUFFER_BIT, GL_NEAREST);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	gl::check("a picture of Rolling Extreme");
}

} /* namespace odin */
