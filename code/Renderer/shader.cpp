//
//  shader.cpp
//
#include "shader.h"
#include "../Fileio.h"
#include <assert.h>

#include "model.h"

#include "FrameBuffer.h"

/*
========================================================================================================

Shader

========================================================================================================
*/

/*
====================================================
Shader::Shader
====================================================
*/
Shader::Shader() {
	memset( m_vkShaderModules, 0, sizeof( VkShaderModule ) * SHADER_STAGE_NUM );
}

/*
====================================================
Shader::Load
====================================================
*/
bool Shader::Load( DeviceContext * device, const char * name ) {
	const char * fileExtensions[ SHADER_STAGE_NUM ];
	fileExtensions[ SHADER_STAGE_VERTEX ]					= "vert";
	fileExtensions[ SHADER_STAGE_TESSELLATION_CONTROL ]		= "tess";
	fileExtensions[ SHADER_STAGE_TESSELLATION_EVALUATION ]	= "tval";
	fileExtensions[ SHADER_STAGE_GEOMETRY ]					= "geom";
	fileExtensions[ SHADER_STAGE_FRAGMENT ]					= "frag";
	fileExtensions[ SHADER_STAGE_COMPUTE ]					= "comp";
	fileExtensions[ SHADER_STAGE_RAYGEN ]					= "rgen";
	fileExtensions[ SHADER_STAGE_ANY_HIT ]					= "ahit";
	fileExtensions[ SHADER_STAGE_CLOSEST_HIT ]				= "chit";
	fileExtensions[ SHADER_STAGE_MISS ]						= "miss";
	fileExtensions[ SHADER_STAGE_INTERSECTION ]				= "rint";
	fileExtensions[ SHADER_STAGE_CALLABLE ]					= "call";
	fileExtensions[ SHADER_STAGE_TASK ]						= "task";
	fileExtensions[ SHADER_STAGE_MESH ]						= "mesh";

	for ( int i = 0; i < SHADER_STAGE_NUM; i++ ) {
		unsigned char * code = NULL;
		unsigned int size = 0;

		// Try loading the spirv code first
		char nameSpirv[ 1024 ];
		sprintf_s( nameSpirv, 1024, "data/shaders/spirv/%s.%s.spirv", name, fileExtensions[ i ] );
		if ( GetFileData( nameSpirv, &code, size ) ) {
			m_vkShaderModules[ i ] = Shader::CreateShaderModule( device->m_vkDevice, (char*)code, size );
			continue;
		}
	}

	return true;
}

/*
====================================================
Shader::Cleanup
====================================================
*/
void Shader::Cleanup( DeviceContext * device ) {
	for ( int i = 0; i < SHADER_STAGE_NUM; i++ ) {
		if ( NULL != m_vkShaderModules[ i ] ) {
			vkDestroyShaderModule( device->m_vkDevice, m_vkShaderModules[ i ], nullptr );
		}
		m_vkShaderModules[ i ] = NULL;
	}
}

/*
====================================================
Shader::CreateShaderModule
====================================================
*/
VkShaderModule Shader::CreateShaderModule( VkDevice vkDevice, const char * code, const int size ) {
	VkShaderModuleCreateInfo createInfo = {};
	createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
	createInfo.codeSize = size;
	createInfo.pCode = (const uint32_t *)code;

	VkShaderModule shaderModule;
	VkResult result = vkCreateShaderModule( vkDevice, &createInfo, nullptr, &shaderModule );
	if ( VK_SUCCESS != result ) {
		printf( "Failed to create shader module\n" );
		assert( 0 );
	}

	return shaderModule;
}