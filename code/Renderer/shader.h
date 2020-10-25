//
//  shader.h
//
#pragma once
#include "DeviceContext.h"
#include "Pipeline.h"
#include "Descriptor.h"
#include <vulkan/vulkan.h>

/*
====================================================
Shader
====================================================
*/
class Shader {
public:
	enum ShaderStage_t {
		SHADER_STAGE_VERTEX = 0,
		SHADER_STAGE_TESSELLATION_CONTROL,
		SHADER_STAGE_TESSELLATION_EVALUATION,
		SHADER_STAGE_GEOMETRY,
		SHADER_STAGE_FRAGMENT,
		SHADER_STAGE_COMPUTE,
		SHADER_STAGE_RAYGEN,
		SHADER_STAGE_ANY_HIT,
		SHADER_STAGE_CLOSEST_HIT,
		SHADER_STAGE_MISS,
		SHADER_STAGE_INTERSECTION,
		SHADER_STAGE_CALLABLE,
		SHADER_STAGE_TASK,
		SHADER_STAGE_MESH,

		SHADER_STAGE_NUM,
	};

public:
	Shader();
	~Shader() {}

	bool Load( DeviceContext * device, const char * name );
	void Cleanup( DeviceContext * device );

private:
	static VkShaderModule CreateShaderModule( VkDevice vkDevice, const char * code, const int size );

public:
	//
	//	Shader Modules
	//
	VkShaderModule m_vkShaderModules[ SHADER_STAGE_NUM ];
};