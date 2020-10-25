//
//  Samplers.cpp
//
#include "Samplers.h"
#include <assert.h>
#include <stdio.h>

VkSampler Samplers::m_samplerStandard;
VkSampler Samplers::m_samplerDepth;

/*
====================================================
Samplers::InitializeSamplers
====================================================
*/
bool Samplers::InitializeSamplers( DeviceContext * device ) {
	VkResult result;
	VkSamplerCreateInfo samplerInfo = {};

	// Create Sampler
	memset( &samplerInfo, 0, sizeof( samplerInfo ) );
	samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
	samplerInfo.magFilter = VK_FILTER_LINEAR;
	samplerInfo.minFilter = VK_FILTER_LINEAR;
	samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	samplerInfo.anisotropyEnable = VK_TRUE;
	samplerInfo.maxAnisotropy = 16;
	samplerInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
	samplerInfo.unnormalizedCoordinates = VK_FALSE;
	samplerInfo.compareEnable = VK_FALSE;
	samplerInfo.compareOp = VK_COMPARE_OP_ALWAYS;
	samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;

	result = vkCreateSampler( device->m_vkDevice, &samplerInfo, nullptr, &m_samplerStandard );
	if ( VK_SUCCESS != result ) {
		printf( "failed to create m_samplerStandard!\n" );
		assert( 0 );
		return false;
	}


	// Create sampler to sample from to depth attachment 
	// Used to sample in the fragment shader for shadowed rendering
	memset( &samplerInfo, 0, sizeof( samplerInfo ) );
	samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
	samplerInfo.magFilter = VK_FILTER_LINEAR;
	samplerInfo.minFilter = VK_FILTER_LINEAR;
	samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
	samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	samplerInfo.mipLodBias = 0.0f;
	samplerInfo.maxAnisotropy = 1.0f;
	samplerInfo.minLod = 0.0f;
	samplerInfo.maxLod = 1.0f;
	samplerInfo.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;

	result = vkCreateSampler( device->m_vkDevice, &samplerInfo, nullptr, &m_samplerDepth );
	if ( VK_SUCCESS != result ) {
		printf( "failed to create m_samplerDepth!\n" );
		assert( 0 );
		return false;
	}

	return true;
}

/*
====================================================
Samplers::Cleanup
====================================================
*/
void Samplers::Cleanup( DeviceContext * device ) {
	vkDestroySampler( device->m_vkDevice, m_samplerStandard, nullptr );
	vkDestroySampler( device->m_vkDevice, m_samplerDepth, nullptr );
}