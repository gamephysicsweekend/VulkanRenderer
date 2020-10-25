//
//  Pipeline.cpp
//
#include "Pipeline.h"
#include "DeviceContext.h"
#include "shader.h"
#include "FrameBuffer.h"
#include "Descriptor.h"
#include "model.h"
#include <assert.h>

/*
========================================================================================================

Pipeline

========================================================================================================
*/

/*
====================================================
Pipeline::Create
====================================================
*/
bool Pipeline::Create( DeviceContext * device, const CreateParms_t & parms ) {
	VkResult result;

	m_parms = parms;

	const int width = parms.width;
	const int height = parms.height;

	std::vector< VkPipelineShaderStageCreateInfo > shaderStages;
	for ( int i = 0; i < Shader::SHADER_STAGE_NUM; i++ ) {
		if ( NULL == parms.shader->m_vkShaderModules[ i ] ) {
			continue;
		}

		VkShaderStageFlagBits stage;
		switch ( i ) {
			default:
			case 0: { stage = VkShaderStageFlagBits::VK_SHADER_STAGE_VERTEX_BIT; } break;
			case 1: { stage = VkShaderStageFlagBits::VK_SHADER_STAGE_TESSELLATION_CONTROL_BIT; } break;
			case 2: { stage = VkShaderStageFlagBits::VK_SHADER_STAGE_TESSELLATION_EVALUATION_BIT; } break;
			case 3: { stage = VkShaderStageFlagBits::VK_SHADER_STAGE_GEOMETRY_BIT; } break;
			case 4: { stage = VkShaderStageFlagBits::VK_SHADER_STAGE_FRAGMENT_BIT; } break;
			case 5: { stage = VkShaderStageFlagBits::VK_SHADER_STAGE_COMPUTE_BIT; } break;
			case 6: { stage = VkShaderStageFlagBits::VK_SHADER_STAGE_RAYGEN_BIT_NV; } break;
			case 7: { stage = VkShaderStageFlagBits::VK_SHADER_STAGE_ANY_HIT_BIT_NV; } break;
			case 8: { stage = VkShaderStageFlagBits::VK_SHADER_STAGE_CLOSEST_HIT_BIT_NV; } break;
			case 9: { stage = VkShaderStageFlagBits::VK_SHADER_STAGE_MISS_BIT_NV; } break;
			case 10: { stage = VkShaderStageFlagBits::VK_SHADER_STAGE_INTERSECTION_BIT_NV; } break;
			case 11: { stage = VkShaderStageFlagBits::VK_SHADER_STAGE_CALLABLE_BIT_NV; } break;
			case 12: { stage = VkShaderStageFlagBits::VK_SHADER_STAGE_TASK_BIT_NV; } break;
			case 13: { stage = VkShaderStageFlagBits::VK_SHADER_STAGE_MESH_BIT_NV; } break;
		}

		VkPipelineShaderStageCreateInfo shaderStageInfo = {};
		shaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
		shaderStageInfo.stage = stage;
		shaderStageInfo.module = parms.shader->m_vkShaderModules[ i ];
		shaderStageInfo.pName = "main";

		shaderStages.push_back( shaderStageInfo );
	}

	VkPipelineVertexInputStateCreateInfo vertexInputInfo = {};
	vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

	VkVertexInputBindingDescription bindingDescription = vert_t::GetBindingDescription();
	std::array< VkVertexInputAttributeDescription, 5 > attributeDescriptions = vert_t::GetAttributeDescriptions();

	vertexInputInfo.vertexBindingDescriptionCount = 1;
	vertexInputInfo.vertexAttributeDescriptionCount = (uint32_t)attributeDescriptions.size();
	vertexInputInfo.pVertexBindingDescriptions = &bindingDescription;
	vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();

	VkPipelineInputAssemblyStateCreateInfo inputAssembly = {};
	inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
	inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
	inputAssembly.primitiveRestartEnable = VK_FALSE;

	VkViewport viewport = {};
	viewport.x = 0.0f;
	viewport.y = 0.0f;
	viewport.width = (float)width;
	viewport.height = (float)height;
	viewport.minDepth = 0.0f;
	viewport.maxDepth = 1.0f;

	VkRect2D scissor = {};
	scissor.offset = { 0, 0 };
	scissor.extent = { (uint32_t)width, (uint32_t)height };

	VkPipelineViewportStateCreateInfo viewportState = {};
	viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
	viewportState.viewportCount = 1;
	viewportState.pViewports = &viewport;
	viewportState.scissorCount = 1;
	viewportState.pScissors = &scissor;

	VkPipelineRasterizationStateCreateInfo rasterizer = {};
	rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
	rasterizer.depthClampEnable = VK_FALSE;
	rasterizer.rasterizerDiscardEnable = VK_FALSE;
	rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
	rasterizer.lineWidth = 1.0f;

	if ( CULL_MODE_FRONT == parms.cullMode ) {
		rasterizer.cullMode = VK_CULL_MODE_FRONT_BIT;
	} else if ( CULL_MODE_BACK == parms.cullMode ) {
		rasterizer.cullMode = VK_CULL_MODE_BACK_BIT;
	} else {
		rasterizer.cullMode = VK_CULL_MODE_NONE;
	}

	rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
	rasterizer.depthBiasEnable = VK_FALSE;

	VkPipelineMultisampleStateCreateInfo multisampling = {};
	multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
	multisampling.sampleShadingEnable = VK_FALSE;
	multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

	VkPipelineDepthStencilStateCreateInfo depthStencil = {};
	depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
	depthStencil.depthTestEnable = parms.depthTest ? VK_TRUE : VK_FALSE;
	depthStencil.depthWriteEnable = parms.depthWrite ? VK_TRUE : VK_FALSE;
	depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;
	depthStencil.depthBoundsTestEnable = VK_FALSE;
	depthStencil.stencilTestEnable = VK_FALSE;

	VkPipelineColorBlendAttachmentState colorBlendAttachment = {};
	colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
	colorBlendAttachment.blendEnable = VK_FALSE;

	VkPipelineColorBlendStateCreateInfo colorBlending = {};
	colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
	colorBlending.logicOpEnable = VK_FALSE;
	colorBlending.logicOp = VK_LOGIC_OP_COPY;
	colorBlending.attachmentCount = 1;
	colorBlending.pAttachments = &colorBlendAttachment;
	colorBlending.blendConstants[ 0 ] = 0.0f;
	colorBlending.blendConstants[ 1 ] = 0.0f;
	colorBlending.blendConstants[ 2 ] = 0.0f;
	colorBlending.blendConstants[ 3 ] = 0.0f;

	// Check for push constants
	VkPushConstantRange pushConstantRange = {};
	if ( parms.pushConstantSize > 0 ) {
		pushConstantRange.stageFlags = parms.pushConstantShaderStages;
		pushConstantRange.size = parms.pushConstantSize;
		pushConstantRange.offset = 0;
	}

	VkPipelineLayoutCreateInfo pipelineLayoutInfo = {};
	pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
	pipelineLayoutInfo.setLayoutCount = 1;
	pipelineLayoutInfo.pSetLayouts = &parms.descriptors->m_vkDescriptorSetLayout;
	if ( parms.pushConstantSize > 0 ) {
		pipelineLayoutInfo.pushConstantRangeCount = 1;
		pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;
	}


	result = vkCreatePipelineLayout( device->m_vkDevice, &pipelineLayoutInfo, nullptr, &m_vkPipelineLayout );
	if ( VK_SUCCESS != result ) {
		printf( "ERROR: Failed to create pipeline layout\n" );
		assert( 0 );
		return false;
	}

	VkDynamicState dynamicSate[ 3 ] = {
		VK_DYNAMIC_STATE_VIEWPORT,
		VK_DYNAMIC_STATE_SCISSOR,
		VK_DYNAMIC_STATE_DEPTH_BIAS
	};

	VkPipelineDynamicStateCreateInfo dynamicState = {};
	dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
	dynamicState.flags = 0;
	dynamicState.dynamicStateCount = 3;
	dynamicState.pDynamicStates = dynamicSate;

	VkGraphicsPipelineCreateInfo pipelineInfo = {};
	pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
	pipelineInfo.stageCount = (uint32_t)shaderStages.size();
	pipelineInfo.pStages = shaderStages.data();
	pipelineInfo.pVertexInputState = &vertexInputInfo;
	pipelineInfo.pInputAssemblyState = &inputAssembly;
	pipelineInfo.pViewportState = &viewportState;
	pipelineInfo.pRasterizationState = &rasterizer;
	pipelineInfo.pMultisampleState = &multisampling;
	pipelineInfo.pDepthStencilState = &depthStencil;
	pipelineInfo.pColorBlendState = &colorBlending;
	pipelineInfo.pDynamicState = &dynamicState;
	pipelineInfo.layout = m_vkPipelineLayout;
	if ( NULL == parms.framebuffer ) {
		pipelineInfo.renderPass = parms.renderPass;
	} else {
		pipelineInfo.renderPass = parms.framebuffer->m_vkRenderPass;
	}
	pipelineInfo.subpass = 0;
	pipelineInfo.basePipelineHandle = VK_NULL_HANDLE;

	VkPipelineCache pipelineCache = VK_NULL_HANDLE;

	result = vkCreateGraphicsPipelines( device->m_vkDevice, pipelineCache, 1, &pipelineInfo, nullptr, &m_vkPipeline );
	if ( VK_SUCCESS != result ) {
		printf( "ERROR: Failed to create pipeline\n" );
		assert( 0 );
		return false;
	}

	return true;
}

/*
====================================================
Pipeline::CreateCompute
====================================================
*/
bool Pipeline::CreateCompute( DeviceContext * device, const CreateParms_t & parms ) {
	VkResult result;

	m_parms = parms;

	VkPipelineShaderStageCreateInfo shaderStageInfo = {};
	shaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	shaderStageInfo.stage = VkShaderStageFlagBits::VK_SHADER_STAGE_COMPUTE_BIT;
	shaderStageInfo.module = parms.shader->m_vkShaderModules[ Shader::SHADER_STAGE_COMPUTE ];
	shaderStageInfo.pName = "main";

	VkPushConstantRange pushConstantRange = {};
	if ( parms.pushConstantSize > 0 ) {
		pushConstantRange.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
		pushConstantRange.size = parms.pushConstantSize;
		pushConstantRange.offset = 0;
	}

	VkPipelineLayoutCreateInfo pipelineLayoutInfo = {};
	pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
	pipelineLayoutInfo.setLayoutCount = 1;
	pipelineLayoutInfo.pSetLayouts = &parms.descriptors->m_vkDescriptorSetLayout;
	if ( parms.pushConstantSize > 0 ) {
		pipelineLayoutInfo.pushConstantRangeCount = 1;
		pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;
	}

	result = vkCreatePipelineLayout( device->m_vkDevice, &pipelineLayoutInfo, nullptr, &m_vkPipelineLayout );
	if ( VK_SUCCESS != result ) {
		printf( "ERROR: Failed to create pipeline layout\n" );
		assert( 0 );
		return false;
	}

	VkComputePipelineCreateInfo pipelineInfo = {};
	pipelineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
	pipelineInfo.stage = shaderStageInfo;
	pipelineInfo.layout = m_vkPipelineLayout;
	pipelineInfo.basePipelineHandle = VK_NULL_HANDLE;	
	

	result = vkCreateComputePipelines( device->m_vkDevice, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &m_vkPipeline );
	if ( VK_SUCCESS != result ) {
		printf( "ERROR: Failed to create pipeline\n" );
		assert( 0 );
		return false;
	}

	return true;
}

/*
====================================================
Pipeline::Cleanup
====================================================
*/
void Pipeline::Cleanup( DeviceContext * device ) {
	vkDestroyPipeline( device->m_vkDevice, m_vkPipeline, nullptr );
	vkDestroyPipelineLayout( device->m_vkDevice, m_vkPipelineLayout, nullptr );
}

/*
====================================================
Pipeline::BindPipeline
====================================================
*/
void Pipeline::BindPipeline( VkCommandBuffer cmdBuffer ) {
	vkCmdBindPipeline( cmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_vkPipeline );
}

/*
====================================================
Pipeline::BindPipelineCompute
====================================================
*/
void Pipeline::BindPipelineCompute( VkCommandBuffer cmdBuffer ) {
	vkCmdBindPipeline( cmdBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_vkPipeline );
}

/*
====================================================
Pipeline::DispatchCompute
====================================================
*/
void Pipeline::DispatchCompute( VkCommandBuffer cmdBuffer, int groupCountX, int groupCountY, int groupCountZ ) {
	vkCmdDispatch( cmdBuffer, groupCountX, groupCountY, groupCountZ );
}
