//
//  SwapChain.cpp
//
#include "SwapChain.h"
#include "DeviceContext.h"
#include "../Fileio.h"
#include <assert.h>

/*
====================================================
SwapChain::Resize
====================================================
*/
void SwapChain::Resize( DeviceContext * device, int width, int height ) {
	vkDeviceWaitIdle( device->m_vkDevice );

	// Delete swapchain
	Cleanup( device );

	// Recreate swap chain
	Create( device, width, height );
}

/*
====================================================
SwapChain::Cleanup
====================================================
*/
void SwapChain::Cleanup( DeviceContext * device ) {
	// semaphores
	vkDestroySemaphore( device->m_vkDevice, m_vkRenderFinishedSemaphore, nullptr );
	vkDestroySemaphore( device->m_vkDevice, m_vkImageAvailableSemaphore, nullptr );

	// depth buffer
	vkDestroyImageView( device->m_vkDevice, m_vkDepthImageView, nullptr );
	vkDestroyImage( device->m_vkDevice, m_vkDepthImage, nullptr );
	vkFreeMemory( device->m_vkDevice, m_vkDepthImageMemory, nullptr );

	// frame buffer
	for ( int i = 0; i < m_vkFramebuffers.size(); i++ ) {
		vkDestroyFramebuffer( device->m_vkDevice, m_vkFramebuffers[ i ], nullptr );
	}

	// render pass
	vkDestroyRenderPass( device->m_vkDevice, m_vkRenderPass, nullptr );

	// color buffers
	for ( int i = 0; i < m_vkImageViews.size(); i++ ) {
		vkDestroyImageView( device->m_vkDevice, m_vkImageViews[ i ], nullptr );
	}

	// swapchain
	vkDestroySwapchainKHR( device->m_vkDevice, m_vkSwapChain, nullptr );
}

/*
====================================================
SwapChain::Create
====================================================
*/
bool SwapChain::Create( DeviceContext * device, int width, int height ) {
	VkResult result;

	m_windowWidth = width;
	m_windowHeight = height;

	m_vkExtent.width = width;
	m_vkExtent.height = height;

	//
	//	Create Semaphores
	//
	{
		VkSemaphoreCreateInfo semaphoreInfo = {};
		semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

		result = vkCreateSemaphore( device->m_vkDevice, &semaphoreInfo, nullptr, &m_vkImageAvailableSemaphore );
		if ( VK_SUCCESS != result ) {
			printf( "ERROR: Failed to create semaphores\n" );
			assert( 0 );
			return false;
		}

		result = vkCreateSemaphore( device->m_vkDevice, &semaphoreInfo, nullptr, &m_vkRenderFinishedSemaphore );
		if ( VK_SUCCESS != result ) {
			printf( "ERROR: Failed to create semaphores\n" );
			assert( 0 );
			return false;
		}
	}

	//
	//	Create Swapchain
	//
	{
		const int deviceIndex = device->m_deviceIndex;
		PhysicalDeviceProperties physicalDeviceInfo = device->m_physicalDevices[ deviceIndex ];

		VkSurfaceFormatKHR surfaceFormat;
		surfaceFormat.format = VK_FORMAT_B8G8R8A8_UNORM;
		surfaceFormat.colorSpace = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR;

		VkPresentModeKHR presentMode = VK_PRESENT_MODE_FIFO_KHR;
		for ( int i = 0; i < physicalDeviceInfo.m_vkPresentModes.size(); i++ ) {
			if ( VK_PRESENT_MODE_MAILBOX_KHR == physicalDeviceInfo.m_vkPresentModes[ i ] ) {
				presentMode = VK_PRESENT_MODE_MAILBOX_KHR;
				break;
			}
		}

		uint32_t imageCount = physicalDeviceInfo.m_vkSurfaceCapabilities.minImageCount + 1;
		if ( physicalDeviceInfo.m_vkSurfaceCapabilities.maxImageCount > 0 && imageCount > physicalDeviceInfo.m_vkSurfaceCapabilities.maxImageCount ) {
			imageCount = physicalDeviceInfo.m_vkSurfaceCapabilities.maxImageCount;
		}

		uint32_t queueFamilyIndices[] = {
			(uint32_t)device->m_graphicsFamilyIdx,
			(uint32_t)device->m_presentFamilyIdx
		};

		VkSwapchainCreateInfoKHR createInfo = {};
		createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
		createInfo.surface = device->m_vkSurface;
		createInfo.minImageCount = imageCount;
		createInfo.imageFormat = surfaceFormat.format;
		createInfo.imageColorSpace = surfaceFormat.colorSpace;
		createInfo.imageExtent = m_vkExtent;
		createInfo.imageArrayLayers = 1;
		createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
		createInfo.queueFamilyIndexCount = 0;
		createInfo.pQueueFamilyIndices = NULL;
		createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
		if ( device->m_graphicsFamilyIdx != device->m_presentFamilyIdx ) {
			createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
			createInfo.queueFamilyIndexCount = 2;
			createInfo.pQueueFamilyIndices = queueFamilyIndices;
		}
		createInfo.preTransform = physicalDeviceInfo.m_vkSurfaceCapabilities.currentTransform;
		createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
		createInfo.presentMode = presentMode;
		createInfo.clipped = VK_TRUE;

		result = vkCreateSwapchainKHR( device->m_vkDevice, &createInfo, nullptr, &m_vkSwapChain );
		if ( VK_SUCCESS != result ) {
			printf( "ERROR: Failed to create swap chain\n" );
			assert( 0 );
			return false;
		}

		vkGetSwapchainImagesKHR( device->m_vkDevice, m_vkSwapChain, &imageCount, nullptr );
		m_vkColorImages.resize( imageCount );
		vkGetSwapchainImagesKHR( device->m_vkDevice, m_vkSwapChain, &imageCount, m_vkColorImages.data() );

		m_vkColorImageFormat = surfaceFormat.format;
	}

	//
	//	Create Image Views for swap chain
	//
	{
		m_vkImageViews.resize( m_vkColorImages.size() );

		for ( uint32_t i = 0; i < m_vkColorImages.size(); i++ ) {
			VkImageViewCreateInfo viewInfo = {};
			viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
			viewInfo.image = m_vkColorImages[ i ];
			viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
			viewInfo.format = m_vkColorImageFormat;
			viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
			viewInfo.subresourceRange.baseMipLevel = 0;
			viewInfo.subresourceRange.levelCount = 1;
			viewInfo.subresourceRange.baseArrayLayer = 0;
			viewInfo.subresourceRange.layerCount = 1;

			result = vkCreateImageView( device->m_vkDevice, &viewInfo, nullptr, &m_vkImageViews[ i ] );
			if ( VK_SUCCESS != result ) {
				printf( "ERROR: Failed to create texture image view\n" );
				assert( 0 );
				return false;
			}
		}
	}

	//
	//	Create Depth Image and Depth Image View for swap chain
	//
	{
		// Choose Depth Image Format
		m_vkDepthFormat = VK_FORMAT_D24_UNORM_S8_UINT;
		{
			VkFormatProperties props;
			vkGetPhysicalDeviceFormatProperties( device->m_vkPhysicalDevice, VK_FORMAT_D32_SFLOAT, &props );
			if ( 0 != ( props.optimalTilingFeatures & VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT ) ) {
				m_vkDepthFormat = VK_FORMAT_D32_SFLOAT;
			}
		}

		VkImageCreateInfo imageInfo = {};
		imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
		imageInfo.imageType = VK_IMAGE_TYPE_2D;
		imageInfo.extent.width = width;
		imageInfo.extent.height = height;
		imageInfo.extent.depth = 1;
		imageInfo.mipLevels = 1;
		imageInfo.arrayLayers = 1;
		imageInfo.format = m_vkDepthFormat;
		imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
		imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		imageInfo.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
		imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
		imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

		result = vkCreateImage( device->m_vkDevice, &imageInfo, nullptr, &m_vkDepthImage );
		if ( VK_SUCCESS != result ) {
			printf( "ERROR: Failed to create image\n" );
			assert( 0 );
			return false;
		}

		VkMemoryRequirements memRequirements;
		vkGetImageMemoryRequirements( device->m_vkDevice, m_vkDepthImage, &memRequirements );

		VkMemoryAllocateInfo allocInfo = {};
		allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
		allocInfo.allocationSize = memRequirements.size;
		allocInfo.memoryTypeIndex = device->FindMemoryTypeIndex( memRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT );

		result = vkAllocateMemory( device->m_vkDevice, &allocInfo, nullptr, &m_vkDepthImageMemory );
		if ( VK_SUCCESS != result ) {
			printf( "ERROR: Failed to allocate image memory\n" );
			assert( 0 );
			return false;
		}

		vkBindImageMemory( device->m_vkDevice, m_vkDepthImage, m_vkDepthImageMemory, 0 );

		//
		//	Create depth image view
		//
		VkImageViewCreateInfo viewInfo = {};
		viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
		viewInfo.image = m_vkDepthImage;
		viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
		viewInfo.format = m_vkDepthFormat;
		viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
		viewInfo.subresourceRange.baseMipLevel = 0;
		viewInfo.subresourceRange.levelCount = 1;
		viewInfo.subresourceRange.baseArrayLayer = 0;
		viewInfo.subresourceRange.layerCount = 1;

		result = vkCreateImageView( device->m_vkDevice, &viewInfo, nullptr, &m_vkDepthImageView );
		if ( VK_SUCCESS != result ) {
			printf( "ERROR: Failed to create texture image view\n" );
			assert( 0 );
			return false;
		}
	}

	//
	//	Create the render pass for the swap chain
	//
	{
		VkAttachmentDescription colorAttachment = {};
		colorAttachment.format = m_vkColorImageFormat;
		colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
		colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
		colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
		colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
		colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
		colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

		VkAttachmentDescription depthAttachment = {};
		depthAttachment.format = m_vkDepthFormat;
		depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
		depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
		depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
		depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
		depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
		depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		depthAttachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

		VkAttachmentReference colorAttachmentRef = {};
		colorAttachmentRef.attachment = 0;
		colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

		VkAttachmentReference depthAttachmentRef = {};
		depthAttachmentRef.attachment = 1;
		depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

		VkSubpassDescription subpass = {};
		subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
		subpass.colorAttachmentCount = 1;
		subpass.pColorAttachments = &colorAttachmentRef;
		subpass.pDepthStencilAttachment = &depthAttachmentRef;

		VkSubpassDependency dependency = {};
		dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
		dependency.dstSubpass = 0;
		dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		dependency.srcAccessMask = 0;
		dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

		VkAttachmentDescription attachments[ 2 ] = {
			colorAttachment,
			depthAttachment
		};
		VkRenderPassCreateInfo renderPassInfo = {};
		renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
		renderPassInfo.attachmentCount = 2;
		renderPassInfo.pAttachments = attachments;
		renderPassInfo.subpassCount = 1;
		renderPassInfo.pSubpasses = &subpass;
		renderPassInfo.dependencyCount = 1;
		renderPassInfo.pDependencies = &dependency;

		result = vkCreateRenderPass( device->m_vkDevice, &renderPassInfo, nullptr, &m_vkRenderPass );
		if ( VK_SUCCESS != result ) {
			printf( "ERROR: Failed to create the render pass\n" );
			assert( 0 );
			return false;
		}
	}

	//
	//	Create Frame Buffers for SwapChain
	//
	{
		m_vkFramebuffers.resize( m_vkImageViews.size() );

		for ( size_t i = 0; i < m_vkImageViews.size(); i++ ) {
			VkImageView attachments[ 2 ] = {
				m_vkImageViews[ i ],
				m_vkDepthImageView
			};

			VkFramebufferCreateInfo framebufferInfo = {};
			framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
			framebufferInfo.renderPass = m_vkRenderPass;
			framebufferInfo.attachmentCount = 2;
			framebufferInfo.pAttachments = attachments;
			framebufferInfo.width = m_vkExtent.width;
			framebufferInfo.height = m_vkExtent.height;
			framebufferInfo.layers = 1;

			result = vkCreateFramebuffer( device->m_vkDevice, &framebufferInfo, nullptr, &m_vkFramebuffers[ i ] );
			if ( VK_SUCCESS != result ) {
				printf( "ERROR: Failed to create the frame buffer\n" );
				assert( 0 );
				return false;
			}
		}
	}

	return true;
}

/*
====================================================
SwapChain::BeginFrame
====================================================
*/
uint32_t SwapChain::BeginFrame( DeviceContext * device ) {
	VkResult result;

	m_currentImageIndex = 0;

	result = vkAcquireNextImageKHR( device->m_vkDevice, m_vkSwapChain, std::numeric_limits< uint64_t >::max(), m_vkImageAvailableSemaphore, VK_NULL_HANDLE, &m_currentImageIndex );
	if ( VK_SUCCESS != result && VK_SUBOPTIMAL_KHR != result ) {
		printf( "ERROR: Failed to acquire swap chain image\n" );
		assert( 0 );
	}

	// Reset the command buffer
	vkResetCommandBuffer( device->m_vkCommandBuffers[ m_currentImageIndex ], VK_COMMAND_BUFFER_RESET_RELEASE_RESOURCES_BIT );

	// Begin recording draw commands
	VkCommandBufferBeginInfo beginInfo = {};
	beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	beginInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
	vkBeginCommandBuffer( device->m_vkCommandBuffers[ m_currentImageIndex ], &beginInfo );

	return m_currentImageIndex;
}

/*
====================================================
SwapChain::EndFrame
====================================================
*/
void SwapChain::EndFrame( DeviceContext * device ) {
	VkResult result;

	result = vkEndCommandBuffer( device->m_vkCommandBuffers[ m_currentImageIndex ] );
	if ( VK_SUCCESS != result ) {
		printf( "ERROR: Failed to record command buffer\n" );
		assert( 0 );
	}

	VkPipelineStageFlags waitStages[] = {
		VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT
	};

	VkSubmitInfo submitInfo = {};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submitInfo.waitSemaphoreCount = 1;
	submitInfo.pWaitSemaphores = &m_vkImageAvailableSemaphore;
	submitInfo.pWaitDstStageMask = waitStages;
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &device->m_vkCommandBuffers[ m_currentImageIndex ] ;
	submitInfo.signalSemaphoreCount = 1;
	submitInfo.pSignalSemaphores = &m_vkRenderFinishedSemaphore;

	result = vkQueueSubmit( device->m_vkGraphicsQueue, 1, &submitInfo, VK_NULL_HANDLE );
	if ( VK_SUCCESS != result ) {
		printf( "ERROR: Failed to submit queue\n" );
		assert( 0 );
	}

	VkPresentInfoKHR presentInfo = {};
	presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
	presentInfo.waitSemaphoreCount = 1;
	presentInfo.pWaitSemaphores = &m_vkRenderFinishedSemaphore;
	presentInfo.swapchainCount = 1;
	presentInfo.pSwapchains = &m_vkSwapChain;
	presentInfo.pImageIndices = &m_currentImageIndex;

	result = vkQueuePresentKHR( device->m_vkPresentQueue, &presentInfo );
	if ( VK_SUCCESS != result && VK_SUBOPTIMAL_KHR != result ) {
		printf( "ERROR: Failed to present\n" );
		assert( 0 );
	}

	vkQueueWaitIdle( device->m_vkPresentQueue );
}

/*
====================================================
SwapChain::BeginRenderPass
====================================================
*/
void SwapChain::BeginRenderPass( DeviceContext * device ) {
	//
	//	Set the renderpass
	//
	VkClearValue clearValues[ 2 ];
	clearValues[ 0 ].color = { 0.0f, 0.0f, 0.0f, 1.0f };
	clearValues[ 1 ].depthStencil = { 1.0f, 0 };

	VkRenderPassBeginInfo renderPassInfo = {};
	renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
	renderPassInfo.renderPass = m_vkRenderPass;
	renderPassInfo.framebuffer = m_vkFramebuffers[ m_currentImageIndex ];
	renderPassInfo.renderArea.offset = { 0, 0 };
	renderPassInfo.renderArea.extent = m_vkExtent;
	renderPassInfo.clearValueCount = 2;
	renderPassInfo.pClearValues = clearValues;

	vkCmdBeginRenderPass( device->m_vkCommandBuffers[ m_currentImageIndex ], &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE );

	//
	//	Set the viewport
	//
	VkViewport viewport = {};
	viewport.x = 0.0f;
	viewport.y = 0.0f;
	viewport.width = (float)m_windowWidth;
	viewport.height = (float)m_windowHeight;
	viewport.minDepth = 0.0f;
	viewport.maxDepth = 1.0f;
	vkCmdSetViewport( device->m_vkCommandBuffers[ m_currentImageIndex ], 0, 1, &viewport );

	VkRect2D scissor = {};
	scissor.offset.x = 0;
	scissor.offset.y = 0;
	scissor.extent.width = m_windowWidth;
	scissor.extent.height = m_windowHeight;
	vkCmdSetScissor( device->m_vkCommandBuffers[ m_currentImageIndex ], 0, 1, &scissor );
}

/*
====================================================
SwapChain::EndRenderPass
====================================================
*/
void SwapChain::EndRenderPass( DeviceContext * device ) {
	vkCmdEndRenderPass( device->m_vkCommandBuffers[ m_currentImageIndex ] );
}