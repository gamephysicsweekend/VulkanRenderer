//
//  Pipeline.h
//
#pragma once
#include <vulkan/vulkan.h>
#include "Descriptor.h"
#include "Buffer.h"

class DeviceContext;
class FrameBuffer;
class Descriptors;
class Shader;

/*
====================================================
Pipeline

Think of this as all of the state that's used to draw
====================================================
*/
class Pipeline {
public:
	Pipeline() {}
	~Pipeline() {}

	enum CullMode_t {
		CULL_MODE_FRONT,
		CULL_MODE_BACK,
		CULL_MODE_NONE
	};

	struct CreateParms_t {
		CreateParms_t() {
			memset( this, 0, sizeof( CreateParms_t ) );
		}
		VkRenderPass	renderPass;
		FrameBuffer * framebuffer;
		Descriptors * descriptors;
		Shader * shader;

		int width;
		int height;

		CullMode_t cullMode;

		bool depthTest;
		bool depthWrite;

		int pushConstantSize;
		VkShaderStageFlagBits pushConstantShaderStages;
	};
	bool Create( DeviceContext * device, const CreateParms_t & parms );
	bool CreateCompute( DeviceContext * device, const CreateParms_t & parms );
	void Cleanup( DeviceContext * device );

	Descriptor GetFreeDescriptor() { return m_parms.descriptors->GetFreeDescriptor(); }

	void BindPipeline( VkCommandBuffer cmdBuffer );
	void BindPipelineCompute( VkCommandBuffer cmdBuffer );
	void DispatchCompute( VkCommandBuffer cmdBuffer, int groupCountX, int groupCountY, int groupCountZ );

	CreateParms_t m_parms;

	//
	//	PipelineState
	//
	VkPipelineLayout m_vkPipelineLayout;
	VkPipeline m_vkPipeline;
};