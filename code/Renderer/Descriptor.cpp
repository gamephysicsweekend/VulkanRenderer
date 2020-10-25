//
//  Descriptor.cpp
//
#include "Descriptor.h"
#include "DeviceContext.h"
#include "Samplers.h"
#include "Pipeline.h"
#include "Image.h"
#include "model.h"
#include <vector>
#include <assert.h>

/*
========================================================================================================

Descriptors

========================================================================================================
*/

/*
====================================================
Descriptors::Create
====================================================
*/
bool Descriptors::Create( DeviceContext * device, const CreateParms_t & parms ) {
	VkResult result;

	m_parms = parms;

	//
	//	Create the non-global pool
	//
	std::vector< VkDescriptorPoolSize > poolSizes;
	const int numUniforms = parms.numUniformsFragment + parms.numUniformsVertex;
	if ( numUniforms > 0 ) {
		VkDescriptorPoolSize poolSize;
		poolSize.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		poolSize.descriptorCount = numUniforms * MAX_DESCRIPTOR_SETS;
		poolSizes.push_back( poolSize );
	}
	if ( parms.numImageSamplers > 0 ) {
		VkDescriptorPoolSize poolSize;
		poolSize.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
		poolSize.descriptorCount = parms.numImageSamplers * MAX_DESCRIPTOR_SETS;
		poolSizes.push_back( poolSize );
	}

	VkDescriptorPoolCreateInfo poolInfo = {};
	poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
	poolInfo.poolSizeCount = (uint32_t)poolSizes.size();
	poolInfo.pPoolSizes = poolSizes.data();
	poolInfo.maxSets = MAX_DESCRIPTOR_SETS;
	poolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;

	result = vkCreateDescriptorPool( device->m_vkDevice, &poolInfo, nullptr, &m_vkDescriptorPool );
	if ( VK_SUCCESS != result ) {
		printf( "ERROR: Failed to create descriptor pool\n" );
		assert( 0 );
		return false;
	}

	//
	// Create Descriptor Set Layout
	//
	VkDescriptorSetLayoutBinding * uniformBindings = (VkDescriptorSetLayoutBinding *)alloca( sizeof( VkDescriptorSetLayoutBinding ) * ( numUniforms ) );
	memset( uniformBindings, 0, sizeof( VkDescriptorSetLayoutBinding ) * numUniforms );

	int idx = 0;

	const int numVertexUniforms = parms.numUniformsVertex;
	for ( int i = 0; i < parms.numUniformsVertex; i++ ) {
		uniformBindings[ idx ].binding = idx;
		uniformBindings[ idx ].descriptorCount = 1;
		uniformBindings[ idx ].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		uniformBindings[ idx ].pImmutableSamplers = nullptr;
		uniformBindings[ idx ].stageFlags = VK_SHADER_STAGE_VERTEX_BIT;

		idx++;
	}

	for ( int i = 0; i < parms.numUniformsFragment; i++ ) {
		uniformBindings[ idx ].binding = idx;
		uniformBindings[ idx ].descriptorCount = 1;
		uniformBindings[ idx ].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
		uniformBindings[ idx ].pImmutableSamplers = nullptr;
		uniformBindings[ idx ].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

		idx++;
	}

	VkDescriptorSetLayoutCreateInfo layoutInfo = {};
	layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	layoutInfo.bindingCount = numUniforms;
	layoutInfo.pBindings = uniformBindings;

	result = vkCreateDescriptorSetLayout( device->m_vkDevice, &layoutInfo, nullptr, &m_vkDescriptorSetLayout );
	if ( VK_SUCCESS != result ) {
		printf( "ERROR: Failed to create descriptor set layout\n" );
		assert( 0 );
		return false;
	}

	//
	//	Create Descriptor Sets
	//
	VkDescriptorSetLayout layouts[ MAX_DESCRIPTOR_SETS ];
	for ( int i = 0; i < MAX_DESCRIPTOR_SETS; i++ ) {
		layouts[ i ] = m_vkDescriptorSetLayout;
	}
	VkDescriptorSetAllocateInfo allocInfo = {};
	allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	allocInfo.descriptorPool = m_vkDescriptorPool;
	allocInfo.descriptorSetCount = MAX_DESCRIPTOR_SETS;
	allocInfo.pSetLayouts = layouts;

	result = vkAllocateDescriptorSets( device->m_vkDevice, &allocInfo, m_vkDescriptorSets );
	if ( VK_SUCCESS != result ) {
		printf( "ERROR: Failed to allocate descriptor set\n" );
		assert( 0 );
		return false;
	}

	return true;
}

/*
====================================================
Descriptors::Create
====================================================
*/
void Descriptors::Cleanup( DeviceContext * device ) {
	// Free the descriptor sets
	vkFreeDescriptorSets( device->m_vkDevice, m_vkDescriptorPool, MAX_DESCRIPTOR_SETS, m_vkDescriptorSets );

	// Destroy descriptor set layout
	vkDestroyDescriptorSetLayout( device->m_vkDevice, m_vkDescriptorSetLayout, nullptr );

	// Destroy Descriptor Pool
	vkDestroyDescriptorPool( device->m_vkDevice, m_vkDescriptorPool, nullptr );
}






/*
========================================================================================================

Descriptor

========================================================================================================
*/

/*
====================================================
Descriptor::Descriptor
====================================================
*/
Descriptor::Descriptor() {
	m_parent = NULL;
	m_id = -1;
	m_numImages = 0;
	m_numBuffers = 0;
	memset( m_imageInfo, 0, sizeof( VkDescriptorImageInfo ) * MAX_IMAGEINFO );
	memset( m_bufferInfo, 0, sizeof( VkDescriptorBufferInfo ) * MAX_BUFFERS );
}

/*
====================================================
Descriptor::BindImage
====================================================
*/
void Descriptor::BindImage( VkImageLayout imageLayout, VkImageView imageView, VkSampler sampler, int slot ) {
	assert( slot < MAX_IMAGEINFO );
	assert( m_numImages < MAX_IMAGEINFO );
	m_imageInfo[ slot ].imageLayout = imageLayout;
	m_imageInfo[ slot ].imageView = imageView;
	m_imageInfo[ slot ].sampler = sampler;
	m_numImages++;
}

/*
====================================================
Descriptor::BindBuffer
====================================================
*/
void Descriptor::BindBuffer( Buffer * uniformBuffer, int offset, int size, int slot ) {
	assert( slot < MAX_BUFFERS );
	assert( m_numBuffers < MAX_BUFFERS );
	m_bufferInfo[ slot ].buffer	= uniformBuffer->m_vkBuffer;
	m_bufferInfo[ slot ].offset	= offset;
	m_bufferInfo[ slot ].range	= size;
	m_numBuffers++;
}

/*
====================================================
Descriptor::BindDescriptor
====================================================
*/
void Descriptor::BindDescriptor( DeviceContext * device, VkCommandBuffer vkCommandBuffer, Pipeline * pso ) {
	const int numDescriptors = m_numImages + m_numBuffers;
	const int allocationSize = sizeof( VkWriteDescriptorSet ) * numDescriptors;
	VkWriteDescriptorSet * descriptorWrites = (VkWriteDescriptorSet *)alloca( allocationSize );
	memset( descriptorWrites, 0, allocationSize );

	int idx = 0;
	for ( int i = 0; i < m_numBuffers; i++ ) {
		descriptorWrites[ idx ].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		descriptorWrites[ idx ].dstSet = m_parent->m_vkDescriptorSets[ m_id ];
		descriptorWrites[ idx ].dstBinding = idx;
		descriptorWrites[ idx ].dstArrayElement = 0;
		descriptorWrites[ idx ].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		descriptorWrites[ idx ].descriptorCount = 1;
		descriptorWrites[ idx ].pBufferInfo = &m_bufferInfo[ i ];

		idx++;
	}

	for ( int i = 0; i < m_numImages; i++ ) {
		descriptorWrites[ idx ].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		descriptorWrites[ idx ].dstSet = m_parent->m_vkDescriptorSets[ m_id ];
		descriptorWrites[ idx ].dstBinding = idx;
		descriptorWrites[ idx ].dstArrayElement = 0;
		descriptorWrites[ idx ].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
		descriptorWrites[ idx ].descriptorCount = 1;
		descriptorWrites[ idx ].pImageInfo = &m_imageInfo[ i ];

		idx++;
	}

	vkUpdateDescriptorSets( device->m_vkDevice, (uint32_t)numDescriptors, descriptorWrites, 0, nullptr );
	vkCmdBindDescriptorSets( vkCommandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pso->m_vkPipelineLayout, 0, 1, &m_parent->m_vkDescriptorSets[ m_id ], 0, nullptr );
}