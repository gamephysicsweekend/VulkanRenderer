//
//  Image.h
//
#pragma once
#include <vulkan/vulkan.h>
#include "DeviceContext.h"

/*
====================================================
Image
====================================================
*/
class Image {
public:
	Image() {}
	~Image() {}

	struct CreateParms_t {
		VkImageUsageFlags usageFlags;
		VkFormat format;
		int width;
		int height;
		int depth;
	};

	bool Create( DeviceContext * device, const CreateParms_t & parms );
	void Cleanup( DeviceContext * device );
	void TransitionLayout( DeviceContext * device );
	void TransitionLayout( VkCommandBuffer cmdBuffer, VkImageLayout newLayout );

	CreateParms_t	m_parms;
	VkImage			m_vkImage;
	VkImageView		m_vkImageView;
	VkDeviceMemory	m_vkDeviceMemory;

	VkImageLayout	m_vkImageLayout;
};