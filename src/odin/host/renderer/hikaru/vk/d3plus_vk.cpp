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
#include <cstring>

#include "odin/host/gfx/spirv.hpp"
#include "odin/host/renderer/hikaru/vk/drawing_vk.hpp"

/* Draws a picture of Rolling Extreme's 3D chip above the board's resolution,
 * like d3plus_gl.cpp: its packets in order, each point coloured as the chip
 * colours it, with a depth buffer, into the scene's picture */

namespace odin {

namespace {

struct DrawPush {
	float step[2];
};

constexpr unsigned PALETTE_ROWS = 128, TABLE_W = 256, TABLE_H = 2048;

} /* namespace */

struct VkDrawing::D3plus {
	vk::Context &card;
	const vk::Functions &fn;
	VkDescriptorSetLayout set_layout = VK_NULL_HANDLE;
	VkPipelineLayout layout = VK_NULL_HANDLE;
	VkShaderModule vertex = VK_NULL_HANDLE, fragment = VK_NULL_HANDLE;
	/* For the opaque packets and the translucent ones, which leave the
	 * depth as it is (+1), each with the depth test or without it (+2);
	 * then the depth alone of the opaque ones without it (4) */
	VkPipeline pipelines[5] = { VK_NULL_HANDLE, VK_NULL_HANDLE, VK_NULL_HANDLE, VK_NULL_HANDLE, VK_NULL_HANDLE };
	VkFormat depth_format;
	/* The texture ROMs (kept while loaded), the table, the palettes of the
	 * pieces, and what each was made from; the depth of the picture */
	vk::Image sheet, table, palettes, depth;
	std::shared_ptr<const std::vector<uint8_t>> sheet_of, table_of;
	std::shared_ptr<const std::vector<uint16_t>> palette_of[D3plusScene::PALETTES];

	D3plus(vk::Context &c, const vk::Functions &f) : card(c), fn(f)
	{
		VkDescriptorSetLayoutBinding bindings[3];

		for (uint32_t i = 0; i < 3; i++)
			bindings[i] = { i, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr };

		const VkDescriptorSetLayoutCreateInfo set{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO, nullptr, 0, 3,
			                                   bindings };

		card.check(fn.vkCreateDescriptorSetLayout(card.device(), &set, nullptr, &set_layout),
		           "the pictures of Rolling Extreme");

		const VkPushConstantRange range{ VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(DrawPush) };
		const VkPipelineLayoutCreateInfo info{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO, nullptr, 0, 1,
			                               &set_layout, 1, &range };

		card.check(fn.vkCreatePipelineLayout(card.device(), &info, nullptr, &layout), "the layout of Rolling Extreme");

		const vk::Spirv vert = ODIN_SPIRV_OF(d3plus_draw_vert), frag = ODIN_SPIRV_OF(d3plus_draw_frag);
		const VkShaderModuleCreateInfo v{ VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO, nullptr, 0, vert.bytes, vert.code };
		const VkShaderModuleCreateInfo g{ VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO, nullptr, 0, frag.bytes, frag.code };

		card.check(fn.vkCreateShaderModule(card.device(), &v, nullptr, &vertex), "a shader");
		card.check(fn.vkCreateShaderModule(card.device(), &g, nullptr, &fragment), "a shader");

		/* A float depth format where the card has one, as the Gaelco 3D's */
		VkFormatProperties props;

		fn.vkGetPhysicalDeviceFormatProperties(card.gpu(), VK_FORMAT_D32_SFLOAT, &props);
		depth_format = (props.optimalTilingFeatures & VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT)
		                       ? VK_FORMAT_D32_SFLOAT
		                       : card.depth_format();
		for (unsigned k = 0; k < 4; k++)
			pipelines[k] = make_pipeline((k & 1) != 0, (k & 2) == 0, false);
		pipelines[4] = make_pipeline(false, true, true);
		table = card.make_image(VK_FORMAT_R8_UINT, TABLE_W, TABLE_H, 1,
		                        VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT);
		palettes = card.make_image(VK_FORMAT_R16_UINT, 256, PALETTE_ROWS * D3plusScene::PALETTES, 1,
		                           VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT);
	}

	~D3plus()
	{
		const VkDevice device = card.device();

		card.send_and_wait();
		for (vk::Image *image : { &sheet, &table, &palettes, &depth })
			if (image->image)
				card.destroy_image(*image);
		for (const VkPipeline pipeline : pipelines)
			fn.vkDestroyPipeline(device, pipeline, nullptr);
		fn.vkDestroyShaderModule(device, vertex, nullptr);
		fn.vkDestroyShaderModule(device, fragment, nullptr);
		fn.vkDestroyPipelineLayout(device, layout, nullptr);
		fn.vkDestroyDescriptorSetLayout(device, set_layout, nullptr);
	}

	VkPipeline make_pipeline(bool translucent, bool tested, bool depth_only)
	{
		const VkPipelineShaderStageCreateInfo stages[2] = {
			{ VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_VERTEX_BIT, vertex, "main", nullptr },
			{ VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_FRAGMENT_BIT, fragment, "main", nullptr },
		};
		const VkVertexInputBindingDescription binding{ 0, sizeof(D3plusVertex), VK_VERTEX_INPUT_RATE_VERTEX };
		const VkVertexInputAttributeDescription attributes[6] = {
			{ 0, 0, VK_FORMAT_R32G32_SFLOAT, offsetof(D3plusVertex, x) },
			{ 1, 0, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(D3plusVertex, q) },
			{ 2, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(D3plusVertex, u) },
			{ 3, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(D3plusVertex, v) },
			{ 4, 0, VK_FORMAT_R32G32_UINT, offsetof(D3plusVertex, texture) },
			{ 5, 0, VK_FORMAT_R16G16B16A16_SINT, offsetof(D3plusVertex, rect) },
		};
		const VkPipelineVertexInputStateCreateInfo input{ VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO, nullptr, 0,
			                                          1, &binding, 6, attributes };
		const VkPipelineInputAssemblyStateCreateInfo assembly{ VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO, nullptr, 0,
			                                               VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, VK_FALSE };
		const VkPipelineViewportStateCreateInfo viewport{ VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO, nullptr, 0, 1, nullptr, 1, nullptr };
		VkPipelineRasterizationStateCreateInfo raster{};
		raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
		raster.polygonMode = VK_POLYGON_MODE_FILL;
		raster.cullMode = VK_CULL_MODE_NONE;
		raster.lineWidth = 1.0f;
		VkPipelineMultisampleStateCreateInfo samples{};
		samples.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
		samples.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
		/* As at 1x: the later wins a tie, a translucent packet leaves the
		 * depth (HEURISTIC), one with bit 29 skips the depth test, its depth
		 * written where nearer by depth_only (HEURISTIC) */
		VkPipelineDepthStencilStateCreateInfo depth_state{};
		depth_state.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
		depth_state.depthTestEnable = VK_TRUE;
		depth_state.depthWriteEnable = depth_only || (!translucent && tested) ? VK_TRUE : VK_FALSE;
		depth_state.depthCompareOp = tested ? VK_COMPARE_OP_LESS_OR_EQUAL : VK_COMPARE_OP_ALWAYS;
		/* The colour times its share (1 for an opaque one) over the rest;
		 * the picture stays opaque */
		VkPipelineColorBlendAttachmentState blend{};
		blend.blendEnable = VK_TRUE;
		blend.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
		blend.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
		blend.colorBlendOp = VK_BLEND_OP_ADD;
		blend.srcAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
		blend.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
		blend.alphaBlendOp = VK_BLEND_OP_ADD;
		blend.colorWriteMask = depth_only ? 0
		                                  : VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
		                                            VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
		VkPipelineColorBlendStateCreateInfo blending{};
		blending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
		blending.attachmentCount = 1;
		blending.pAttachments = &blend;
		const VkDynamicState states[2] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
		const VkPipelineDynamicStateCreateInfo dynamic{ VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO, nullptr, 0, 2, states };
		const VkFormat colour = PICTURE_FORMAT;
		const VkPipelineRenderingCreateInfo rendering{ VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO, nullptr, 0, 1,
			                                       &colour, depth_format, VK_FORMAT_UNDEFINED };
		VkGraphicsPipelineCreateInfo info{};
		info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
		info.pNext = &rendering;
		info.stageCount = 2;
		info.pStages = stages;
		info.pVertexInputState = &input;
		info.pInputAssemblyState = &assembly;
		info.pViewportState = &viewport;
		info.pRasterizationState = &raster;
		info.pMultisampleState = &samples;
		info.pDepthStencilState = &depth_state;
		info.pColorBlendState = &blending;
		info.pDynamicState = &dynamic;
		info.layout = layout;

		VkPipeline made;

		card.check(fn.vkCreateGraphicsPipelines(card.device(), VK_NULL_HANDLE, 1, &info, nullptr, &made),
		           "a pipeline of Rolling Extreme");
		return made;
	}

	/* Copies rows of bytes into an image from row `y` on, which shaders then
	 * read */
	void load(VkCommandBuffer cb, vk::Image &image, unsigned y, const void *bytes, unsigned w, unsigned h,
	          unsigned size)
	{
		const VkDeviceSize count = VkDeviceSize{ w } * h * size;
		const vk::Slice in = card.stage(count, 16);
		const VkBufferImageCopy region{ in.offset, 0, 0, { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 },
			                        { 0, static_cast<int32_t>(y), 0 }, { w, h, 1 } };

		std::memcpy(in.bytes, bytes, count);
		card.barrier(cb, image, vk::COPIED_TO);
		fn.vkCmdCopyBufferToImage(cb, in.buffer, image.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
		card.barrier(cb, image, vk::SAMPLED);
	}
};

void
VkDrawing::draw_d3plus(const D3plusScene &scene)
{
	if (!m_d3plus)
		m_d3plus = std::make_shared<D3plus>(m_card, m_fn);

	D3plus &d = *m_d3plus;
	const VkCommandBuffer cb = m_card.commands();

	if (scene.sheet && scene.sheet != d.sheet_of) {
		/* Never sent: the game shows the board's own picture then
		 * (RollextGame) */
		if (D3plusScene::SHEET_H > largest_texture() || D3plusScene::SHEET_W > largest_texture())
			return;
		if (d.sheet.image)
			m_card.later([card = &m_card, old = d.sheet]() mutable { card->destroy_image(old); });
		d.sheet = m_card.make_image(VK_FORMAT_R8_UINT, D3plusScene::SHEET_W, D3plusScene::SHEET_H, 1,
		                            VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT);
		d.load(cb, d.sheet, 0, scene.sheet->data(), D3plusScene::SHEET_W, D3plusScene::SHEET_H, 1);
		d.sheet_of = scene.sheet;
	}
	if (!d.sheet_of || !scene.table || scene.palettes.empty())
		return;
	if (scene.table != d.table_of) {
		d.load(cb, d.table, 0, scene.table->data(), TABLE_W, TABLE_H, 1);
		d.table_of = scene.table;
	}
	for (size_t piece = 0; piece < scene.palettes.size() && piece < D3plusScene::PALETTES; piece++)
		if (scene.palettes[piece] && scene.palettes[piece] != d.palette_of[piece]) {
			d.load(cb, d.palettes, static_cast<unsigned>(piece * PALETTE_ROWS), scene.palettes[piece]->data(), 256,
			       PALETTE_ROWS, 2);
			d.palette_of[piece] = scene.palettes[piece];
		}
	if (d.depth.w != width() || d.depth.h != height()) {
		if (d.depth.image)
			m_card.later([card = &m_card, old = d.depth]() mutable { card->destroy_image(old); });
		d.depth = m_card.make_image(d.depth_format, width(), height(), 1, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT);
	}

	const VkDeviceSize vertex_bytes = std::max<size_t>(scene.vertices.size(), 1) * sizeof(D3plusVertex);
	const vk::Slice vertices = m_card.stage(vertex_bytes, 16);

	if (!scene.vertices.empty())
		std::memcpy(vertices.bytes, scene.vertices.data(), scene.vertices.size() * sizeof(D3plusVertex));

	/* A picture starts black with its depth cleared (d3plus.cpp,
	 * end_picture); the packets in the chip's order */
	m_card.barrier(cb, m_scene, vk::DRAWN_IN);
	m_card.barrier(cb, d.depth, vk::DEPTH_IN);

	VkRenderingAttachmentInfo colour{};
	colour.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
	colour.imageView = m_scene.view;
	colour.imageLayout = m_scene.layout;
	colour.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	colour.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	colour.clearValue.color = { { 0.0f, 0.0f, 0.0f, 1.0f } };
	VkRenderingAttachmentInfo depth{};
	depth.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
	depth.imageView = d.depth.view;
	depth.imageLayout = d.depth.layout;
	depth.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	depth.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	depth.clearValue.depthStencil = { 1.0f, 0 };

	const VkRect2D area{ { 0, 0 }, { width(), height() } };
	VkRenderingInfo rendering{};
	rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
	rendering.renderArea = area;
	rendering.layerCount = 1;
	rendering.colorAttachmentCount = 1;
	rendering.pColorAttachments = &colour;
	rendering.pDepthAttachment = &depth;
	m_fn.vkCmdBeginRendering(cb, &rendering);

	const VkViewport viewport{ 0.0f, 0.0f, static_cast<float>(width()), static_cast<float>(height()), 0.0f, 1.0f };
	const VkDescriptorSet set = m_card.frame_set(d.set_layout);
	const VkDescriptorImageInfo images[3] = { { m_nearest, d.sheet.view, d.sheet.layout },
		                                  { m_nearest, d.table.view, d.table.layout },
		                                  { m_nearest, d.palettes.view, d.palettes.layout } };
	VkWriteDescriptorSet writes[3];

	for (uint32_t i = 0; i < 3; i++)
		writes[i] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, set, i, 0, 1,
			      VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, &images[i], nullptr, nullptr };

	const DrawPush push{ { 512.0f / static_cast<float>(width()), 384.0f / static_cast<float>(height()) } };

	m_fn.vkUpdateDescriptorSets(m_card.device(), 3, writes, 0, nullptr);
	m_fn.vkCmdSetViewport(cb, 0, 1, &viewport);
	m_fn.vkCmdSetScissor(cb, 0, 1, &area);
	m_fn.vkCmdBindVertexBuffers(cb, 0, 1, &vertices.buffer, &vertices.offset);
	m_fn.vkCmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, d.layout, 0, 1, &set, 0, nullptr);
	m_fn.vkCmdPushConstants(cb, d.layout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(push), &push);

	VkPipeline bound = VK_NULL_HANDLE;

	for (const D3plusRun &run : scene.runs) {
		const VkPipeline pipeline = d.pipelines[(run.translucent ? 1 : 0) | (run.tested ? 0 : 2)];

		if (pipeline != bound) {
			m_fn.vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
			bound = pipeline;
		}
		m_fn.vkCmdDraw(cb, run.count, 1, run.first, 0);
		if (!run.translucent && !run.tested) {
			m_fn.vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, d.pipelines[4]);
			bound = d.pipelines[4];
			m_fn.vkCmdDraw(cb, run.count, 1, run.first, 0);
		}
	}
	m_card.end_drawing(cb);

	const VkImageCopy copy{ { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 }, { 0, 0, 0 }, { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 },
		                { 0, 0, 0 }, { width(), height(), 1 } };

	m_card.barrier(cb, m_scene, vk::COPIED_FROM);
	m_card.barrier(cb, m_frame, vk::COPIED_TO);
	m_fn.vkCmdCopyImage(cb, m_scene.image, m_scene.layout, m_frame.image, m_frame.layout, 1, &copy);
}

} /* namespace odin */
