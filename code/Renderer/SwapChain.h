//
//  SwapChain.h
//
#pragma once
#include <vulkan/vulkan.h>
#include <vector>

class DeviceContext;

/*
====================================================
SwapChain
====================================================
*/
class SwapChain {
public:
	bool Create( DeviceContext * device, int width, int height );
	void Cleanup( DeviceContext * device );
	void Resize( DeviceContext * device, int width, int height );

	uint32_t BeginFrame( DeviceContext * device );
	void EndFrame( DeviceContext * device );

	void BeginRenderPass( DeviceContext * device );
	void EndRenderPass( DeviceContext * device );

	VkSemaphore m_vkImageAvailableSemaphore;
	VkSemaphore m_vkRenderFinishedSemaphore;

	int m_windowWidth;
	int m_windowHeight;

	VkSwapchainKHR m_vkSwapChain;
	VkExtent2D m_vkExtent;

	uint32_t					m_currentImageIndex;
	VkFormat					m_vkColorImageFormat;
	std::vector< VkImage >		m_vkColorImages;
	std::vector< VkImageView >	m_vkImageViews;

	VkFormat m_vkDepthFormat;
	VkImage m_vkDepthImage;
	VkImageView m_vkDepthImageView;
	VkDeviceMemory m_vkDepthImageMemory;

	std::vector< VkFramebuffer > m_vkFramebuffers;

	VkRenderPass m_vkRenderPass;
};