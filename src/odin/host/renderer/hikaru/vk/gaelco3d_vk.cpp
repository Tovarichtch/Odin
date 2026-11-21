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

/* Draws a field of a Gaelco 3D board above the board's resolution, in the
 * board's two steps, like gaelco3d_gl.cpp: the pixels' words, then their
 * colours into the scene's picture */

namespace odin {

namespace {

struct DrawPush {
	uint32_t mask_off;
	int32_t fields;
};

struct ShowPush {
	uint32_t texture_end;
};

constexpr VkFormat WORD_FORMAT = VK_FORMAT_R32G32_UINT;

} /* namespace */

struct VkDrawing::Gaelco3d {
	vk::Context &card;
	const vk::Functions &fn;
	VkDescriptorSetLayout mask_layout = VK_NULL_HANDLE;
	VkPipelineLayout layout = VK_NULL_HANDLE;
	VkShaderModule vertex = VK_NULL_HANDLE, fragment = VK_NULL_HANDLE;
	/* For the polygons' words, and for those of bank 0x7F */
	VkPipeline pipelines[2] = { VK_NULL_HANDLE, VK_NULL_HANDLE };
	VkFormat depth_format;
	vk::Pass show;
	/* The ROMs (kept while loaded) and the palette */
	vk::Image texture, mask, palette;
	std::shared_ptr<const Gaelco3dRoms> roms;
	/* The pixels' words, with those of bank 0x7F kept apart, and the depth:
	 * kept from field to field, as the board keeps its frame; zero when
	 * created, as at power-on */
	vk::Image word, see, depth;
	bool fresh = true;

	Gaelco3d(vk::Context &c, const vk::Functions &f) : card(c), fn(f)
	{
		const VkDescriptorSetLayoutBinding binding{ 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
			                                    VK_SHADER_STAGE_FRAGMENT_BIT, nullptr };
		const VkDescriptorSetLayoutCreateInfo set{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO, nullptr, 0, 1,
			                                   &binding };

		card.check(fn.vkCreateDescriptorSetLayout(card.device(), &set, nullptr, &mask_layout),
		           "the mask of the Gaelco 3D");

		const VkPushConstantRange range{ VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(DrawPush) };
		const VkPipelineLayoutCreateInfo info{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO, nullptr, 0, 1,
			                               &mask_layout, 1, &range };

		card.check(fn.vkCreatePipelineLayout(card.device(), &info, nullptr, &layout), "the layout of the Gaelco 3D");

		const vk::Spirv vert = ODIN_SPIRV_OF(gaelco3d_draw_vert), frag = ODIN_SPIRV_OF(gaelco3d_draw_frag);
		const VkShaderModuleCreateInfo v{ VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO, nullptr, 0, vert.bytes, vert.code };
		const VkShaderModuleCreateInfo g{ VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO, nullptr, 0, frag.bytes, frag.code };

		card.check(fn.vkCreateShaderModule(card.device(), &v, nullptr, &vertex), "a shader");
		card.check(fn.vkCreateShaderModule(card.device(), &g, nullptr, &fragment), "a shader");

		/* A float depth format where the card has one; the board's 16
		 * bits fit in either */
		VkFormatProperties props;

		fn.vkGetPhysicalDeviceFormatProperties(card.gpu(), VK_FORMAT_D32_SFLOAT, &props);
		depth_format = (props.optimalTilingFeatures & VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT)
		                       ? VK_FORMAT_D32_SFLOAT
		                       : card.depth_format();
		for (unsigned see_through = 0; see_through < 2; see_through++)
			pipelines[see_through] = make_pipeline(see_through != 0);
		show = card.make_pass(ODIN_SPIRV_OF(whole_picture_vert), ODIN_SPIRV_OF(gaelco3d_show_frag), 4,
		                      sizeof(ShowPush), VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, PICTURE_FORMAT);
		palette = card.make_image(VK_FORMAT_R16_UINT, 256, 128, 1,
		                          VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT);
	}

	~Gaelco3d()
	{
		const VkDevice device = card.device();

		card.send_and_wait();
		for (vk::Image *image : { &texture, &mask, &palette, &word, &see, &depth })
			if (image->image)
				card.destroy_image(*image);
		card.destroy_pass(show);
		for (const VkPipeline pipeline : pipelines)
			fn.vkDestroyPipeline(device, pipeline, nullptr);
		fn.vkDestroyShaderModule(device, vertex, nullptr);
		fn.vkDestroyShaderModule(device, fragment, nullptr);
		fn.vkDestroyPipelineLayout(device, layout, nullptr);
		fn.vkDestroyDescriptorSetLayout(device, mask_layout, nullptr);
	}

	/* The polygons' words, with a dynamic depth compare: those of bank 0x7F
	 * write only their own words */
	VkPipeline make_pipeline(bool see_through)
	{
		const VkPipelineShaderStageCreateInfo stages[2] = {
			{ VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_VERTEX_BIT, vertex, "main", nullptr },
			{ VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_FRAGMENT_BIT, fragment, "main", nullptr },
		};
		const VkVertexInputBindingDescription binding{ 0, sizeof(Gaelco3dVertex), VK_VERTEX_INPUT_RATE_VERTEX };
		const VkVertexInputAttributeDescription attributes[5] = {
			{ 0, 0, VK_FORMAT_R32G32_SFLOAT, offsetof(Gaelco3dVertex, x) },
			{ 1, 0, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(Gaelco3dVertex, k) },
			{ 2, 0, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(Gaelco3dVertex, k) + 4 * sizeof(float) },
			{ 3, 0, VK_FORMAT_R32G32_SFLOAT, offsetof(Gaelco3dVertex, k) + 8 * sizeof(float) },
			{ 4, 0, VK_FORMAT_R32G32_UINT, offsetof(Gaelco3dVertex, texture) },
		};
		const VkPipelineVertexInputStateCreateInfo input{ VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO, nullptr, 0,
			                                          1, &binding, 5, attributes };
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
		VkPipelineDepthStencilStateCreateInfo depth_state{};
		depth_state.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
		depth_state.depthTestEnable = VK_TRUE;
		depth_state.depthWriteEnable = VK_TRUE;
		depth_state.depthCompareOp = VK_COMPARE_OP_LESS;
		constexpr VkColorComponentFlags RG = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT;
		VkPipelineColorBlendAttachmentState writes[2]{};
		writes[0].colorWriteMask = see_through ? 0 : RG;
		writes[1].colorWriteMask = RG;
		VkPipelineColorBlendStateCreateInfo blending{};
		blending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
		blending.attachmentCount = 2;
		blending.pAttachments = writes;
		const VkDynamicState states[3] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR,
			                           VK_DYNAMIC_STATE_DEPTH_COMPARE_OP };
		const VkPipelineDynamicStateCreateInfo dynamic{ VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO, nullptr, 0, 3, states };
		const VkFormat colours[2] = { WORD_FORMAT, WORD_FORMAT };
		const VkPipelineRenderingCreateInfo rendering{ VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO, nullptr, 0, 2,
			                                       colours, depth_format, VK_FORMAT_UNDEFINED };
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
		           "a pipeline of the Gaelco 3D");
		return made;
	}

	/* Copies bytes into an image, which shaders then read */
	void load(VkCommandBuffer cb, vk::Image &image, const void *bytes, unsigned w, unsigned h, unsigned size)
	{
		const VkDeviceSize count = VkDeviceSize{ w } * h * size;
		const vk::Slice in = card.stage(count, 16);
		const VkBufferImageCopy region{ in.offset, 0, 0, { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 }, { 0, 0, 0 }, { w, h, 1 } };

		std::memcpy(in.bytes, bytes, count);
		card.barrier(cb, image, vk::COPIED_TO);
		fn.vkCmdCopyBufferToImage(cb, in.buffer, image.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
		card.barrier(cb, image, vk::SAMPLED);
	}
};

void
VkDrawing::draw_gaelco3d(const Gaelco3dScene &scene)
{
	if (!m_gaelco3d)
		m_gaelco3d = std::make_shared<Gaelco3d>(m_card, m_fn);

	Gaelco3d &g = *m_gaelco3d;
	const VkCommandBuffer cb = m_card.commands();

	if (scene.roms && scene.roms != g.roms && !scene.roms->mask.empty()) {
		const Gaelco3dRoms &roms = *scene.roms;
		const unsigned rows = static_cast<unsigned>(roms.texture.size() / 4096);

		/* Never sent: the game shows the board's own picture then
		 * (Gaelco3dGame) */
		if (rows > largest_texture())
			return;
		for (vk::Image *image : { &g.texture, &g.mask })
			if (image->image)
				m_card.later([card = &m_card, old = *image]() mutable { card->destroy_image(old); });
		g.texture = m_card.make_image(VK_FORMAT_R8_UINT, 4096, rows, 1,
		                              VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT);
		g.mask = m_card.make_image(VK_FORMAT_R8_UINT, 4096, 1024, 1,
		                           VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT);
		g.load(cb, g.texture, roms.texture.data(), 4096, rows, 1);
		g.load(cb, g.mask, roms.mask.data(), 4096, 1024, 1);
		g.roms = scene.roms;
	}
	if (!scene.palette.empty())
		g.load(cb, g.palette, scene.palette.data(), 256, 128, 2);

	if (g.word.w != width() || g.word.h != height()) {
		for (vk::Image *image : { &g.word, &g.see, &g.depth })
			if (image->image)
				m_card.later([card = &m_card, old = *image]() mutable { card->destroy_image(old); });
		g.word = m_card.make_image(WORD_FORMAT, width(), height(), 1,
		                           VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
		g.see = m_card.make_image(WORD_FORMAT, width(), height(), 1,
		                          VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
		g.depth = m_card.make_image(g.depth_format, width(), height(), 1, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT);
		g.fresh = true;
	}
	if (!g.roms)
		return;

	const VkDeviceSize vertex_bytes = std::max<size_t>(scene.vertices.size(), 1) * sizeof(Gaelco3dVertex);
	const vk::Slice vertices = m_card.stage(vertex_bytes, 16);

	if (!scene.vertices.empty())
		std::memcpy(vertices.bytes, scene.vertices.data(), scene.vertices.size() * sizeof(Gaelco3dVertex));

	/* The words: the polygons in the board's order, with the depth compared
	 * as the board compares it; those of bank 0x7F leave the other words
	 * alone */
	m_card.barrier(cb, g.word, vk::DRAWN_IN);
	m_card.barrier(cb, g.see, vk::DRAWN_IN);
	m_card.barrier(cb, g.depth, vk::DEPTH_IN);

	const VkAttachmentLoadOp load = g.fresh ? VK_ATTACHMENT_LOAD_OP_CLEAR : VK_ATTACHMENT_LOAD_OP_LOAD;
	VkRenderingAttachmentInfo colours[2]{};

	for (unsigned i = 0; i < 2; i++) {
		colours[i].sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
		colours[i].imageView = (i ? g.see : g.word).view;
		colours[i].imageLayout = (i ? g.see : g.word).layout;
		colours[i].loadOp = load;
		colours[i].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	}
	VkRenderingAttachmentInfo depth{};
	depth.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
	depth.imageView = g.depth.view;
	depth.imageLayout = g.depth.layout;
	depth.loadOp = load;
	depth.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	depth.clearValue.depthStencil = { 0.0f, 0 };
	g.fresh = false;

	const VkRect2D area{ { 0, 0 }, { width(), height() } };
	VkRenderingInfo rendering{};
	rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
	rendering.renderArea = area;
	rendering.layerCount = 1;
	rendering.colorAttachmentCount = 2;
	rendering.pColorAttachments = colours;
	rendering.pDepthAttachment = &depth;
	m_fn.vkCmdBeginRendering(cb, &rendering);

	const VkViewport viewport{ 0.0f, 0.0f, static_cast<float>(width()), static_cast<float>(height()), 0.0f, 1.0f };
	const VkDescriptorSet mask_set = m_card.frame_set(g.mask_layout);
	const VkDescriptorImageInfo masked{ m_nearest, g.mask.view, g.mask.layout };
	const VkWriteDescriptorSet write{ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, mask_set, 0, 0, 1,
		                          VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, &masked, nullptr, nullptr };
	const DrawPush push{ scene.roms ? scene.roms->mask_off : 0,
		             scene.both_fields ? static_cast<int32_t>(2 | (scene.field & 1)) : 0 };

	m_fn.vkUpdateDescriptorSets(m_card.device(), 1, &write, 0, nullptr);
	m_fn.vkCmdSetViewport(cb, 0, 1, &viewport);
	m_fn.vkCmdSetScissor(cb, 0, 1, &area);
	m_fn.vkCmdBindVertexBuffers(cb, 0, 1, &vertices.buffer, &vertices.offset);
	m_fn.vkCmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, g.layout, 0, 1, &mask_set, 0, nullptr);
	m_fn.vkCmdPushConstants(cb, g.layout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(push), &push);

	VkPipeline bound = VK_NULL_HANDLE;

	for (const Gaelco3dRun &run : scene.runs) {
		const VkPipeline pipeline = g.pipelines[run.see_through ? 1 : 0];

		if (pipeline != bound) {
			m_fn.vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
			bound = pipeline;
		}
		m_fn.vkCmdSetDepthCompareOp(cb, run.always ? VK_COMPARE_OP_ALWAYS : VK_COMPARE_OP_LESS);
		m_fn.vkCmdDraw(cb, run.count, 1, run.first, 0);
	}
	m_card.end_drawing(cb);

	/* The colours, into the scene's picture */
	const ShowPush shown{ scene.roms ? scene.roms->texture_end : 0 };

	m_card.barrier(cb, g.word, vk::SAMPLED);
	m_card.barrier(cb, g.see, vk::SAMPLED);
	m_card.barrier(cb, m_scene, vk::DRAWN_IN);
	m_card.begin_drawing(cb, m_scene, area, VK_ATTACHMENT_LOAD_OP_DONT_CARE);
	m_card.bind_images(cb, g.show, { &g.word, &g.see, &g.texture, &g.palette }, m_nearest);
	m_fn.vkCmdPushConstants(cb, g.show.layout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(shown), &shown);
	m_fn.vkCmdDraw(cb, 3, 1, 0, 0);
	m_card.end_drawing(cb);

	const VkImageCopy copy{ { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 }, { 0, 0, 0 }, { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 },
		                { 0, 0, 0 }, { width(), height(), 1 } };

	m_card.barrier(cb, m_scene, vk::COPIED_FROM);
	m_card.barrier(cb, m_frame, vk::COPIED_TO);
	m_fn.vkCmdCopyImage(cb, m_scene.image, m_scene.layout, m_frame.image, m_frame.layout, 1, &copy);
}

} /* namespace odin */
