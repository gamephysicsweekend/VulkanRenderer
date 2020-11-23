//
//  Image.cpp
//
#include "Image.h"
#include <assert.h>
#include <stdio.h>

/*
========================================================================================================

Image

========================================================================================================
*/

/*
====================================================
Image::Cleanup
====================================================
*/
bool Image::Create( DeviceContext * device, const CreateParms_t & parms ) {
	VkResult result;

	m_parms = parms;

	//
	//	Create the Image
	//

	VkImageCreateInfo image = {};
	image.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
	image.imageType = VK_IMAGE_TYPE_1D;
	if ( m_parms.height > 1 ) {
		image.imageType = VK_IMAGE_TYPE_2D;
	}
	if ( m_parms.depth > 1 ) {
		image.imageType = VK_IMAGE_TYPE_3D;
	}
	
	image.extent.width = m_parms.width;
	image.extent.height = m_parms.height;
	image.extent.depth = m_parms.depth;
	image.mipLevels = 1;
	image.arrayLayers = 1;
	image.samples = VK_SAMPLE_COUNT_1_BIT;
	image.tiling = VK_IMAGE_TILING_OPTIMAL;
	image.format = m_parms.format;
	if ( VK_FORMAT_D32_SFLOAT == m_parms.format ) {
		image.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
	} else {
		image.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
	}
	image.usage = parms.usageFlags;

	result = vkCreateImage( device->m_vkDevice, &image, nullptr, &m_vkImage );
	if ( VK_SUCCESS != result ) {
		printf( "ERROR: Failed to create image\n" );
		assert( 0 );
		return false;
	}

	//
	//	Allocate memory on the GPU and attach it to the 
	//

	VkMemoryAllocateInfo memAlloc = {};
	memAlloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	VkMemoryRequirements memReqs;
	vkGetImageMemoryRequirements( device->m_vkDevice, m_vkImage, &memReqs );
	memAlloc.allocationSize = memReqs.size;
	memAlloc.memoryTypeIndex = device->FindMemoryTypeIndex( memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT );

	result = vkAllocateMemory( device->m_vkDevice, &memAlloc, nullptr, &m_vkDeviceMemory );
	if ( VK_SUCCESS != result ) {
		printf( "ERROR: Failed to allocate memory\n" );
		assert( 0 );
		return false;
	}

	result = vkBindImageMemory( device->m_vkDevice, m_vkImage, m_vkDeviceMemory, 0 );
	if ( VK_SUCCESS != result ) {
		printf( "ERROR: Failed to bind image memory\n" );
		assert( 0 );
		return false;
	}

	//
	//	Create the image view
	//

	VkImageViewCreateInfo imageView = {};
	imageView.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
	imageView.viewType = VK_IMAGE_VIEW_TYPE_1D;
	if ( m_parms.height > 1 ) {
		imageView.viewType = VK_IMAGE_VIEW_TYPE_2D;
	}
	if ( m_parms.depth > 1 ) {
		imageView.viewType = VK_IMAGE_VIEW_TYPE_3D;
	}

	imageView.format = m_parms.format;
	imageView.subresourceRange = {};
	if ( VK_FORMAT_D32_SFLOAT == m_parms.format ) {
		imageView.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
	} else {
		imageView.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	}

	imageView.subresourceRange.baseMipLevel = 0;
	imageView.subresourceRange.levelCount = 1;
	imageView.subresourceRange.baseArrayLayer = 0;
	imageView.subresourceRange.layerCount = 1;
	imageView.image = m_vkImage;

	result = vkCreateImageView( device->m_vkDevice, &imageView, nullptr, &m_vkImageView );
	if ( VK_SUCCESS != result ) {
		printf( "ERROR: Failed to create image view\n" );
		assert( 0 );
		return false;
	}

	return true;
}

/*
====================================================
Image::Cleanup
====================================================
*/
void Image::Cleanup( DeviceContext * device ) {
	vkDestroyImageView( device->m_vkDevice, m_vkImageView, nullptr );
	vkDestroyImage( device->m_vkDevice, m_vkImage, nullptr );
	vkFreeMemory( device->m_vkDevice, m_vkDeviceMemory, nullptr );
}

/*
====================================================
Image::TransitionLayout
====================================================
*/
void Image::TransitionLayout( DeviceContext * device ) {
	// Transition the image layout
	VkCommandBuffer vkCommandBuffer = device->CreateCommandBuffer( VK_COMMAND_BUFFER_LEVEL_PRIMARY );

	VkImageMemoryBarrier barrier = {};
	barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
	barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	barrier.newLayout = VK_IMAGE_LAYOUT_GENERAL;
	barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	barrier.image = m_vkImage;
	if ( VK_FORMAT_D32_SFLOAT == m_parms.format ) {
		barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
	} else {
		barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	}
	barrier.subresourceRange.baseMipLevel = 0;
	barrier.subresourceRange.levelCount = 1;
	barrier.subresourceRange.baseArrayLayer = 0;
	barrier.subresourceRange.layerCount = 1;
	barrier.srcAccessMask = 0;
	barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

	VkPipelineStageFlags sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
	VkPipelineStageFlags destinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;				

	vkCmdPipelineBarrier(
		vkCommandBuffer,
		sourceStage,
		destinationStage,
		0,
		0,
		nullptr,
		0,
		nullptr,
		1,
		&barrier
	);

	device->FlushCommandBuffer( vkCommandBuffer, device->m_vkGraphicsQueue );

	m_vkImageLayout = VK_IMAGE_LAYOUT_GENERAL;
}

/*
====================================================
Image::TransitionLayout
====================================================
*/
void Image::TransitionLayout( VkCommandBuffer cmdBuffer, VkImageLayout newLayout ) {
	if ( m_vkImageLayout == newLayout ) {
		return;
	}

	VkImageMemoryBarrier barrier = {};
	barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
	barrier.oldLayout = m_vkImageLayout;
	barrier.newLayout = newLayout;
	barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	barrier.image = m_vkImage;
	if ( VK_FORMAT_D32_SFLOAT == m_parms.format ) {
		barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
	} else {
		barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	}
	barrier.subresourceRange.baseMipLevel = 0;
	barrier.subresourceRange.levelCount = 1;
	barrier.subresourceRange.baseArrayLayer = 0;
	barrier.subresourceRange.layerCount = 1;
	barrier.srcAccessMask = 0;
	barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

	VkPipelineStageFlags sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
	VkPipelineStageFlags destinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;				

	vkCmdPipelineBarrier(
		cmdBuffer,
		sourceStage,
		destinationStage,
		0,
		0,
		nullptr,
		0,
		nullptr,
		1,
		&barrier
	);

	m_vkImageLayout = newLayout;
}