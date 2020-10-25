//
//  Buffer.h
//
#pragma once
#include "DeviceContext.h"

/*
================================================================================================

Buffer

A general buffer

================================================================================================
*/

class Buffer {
public:
	Buffer();

	bool Allocate( DeviceContext * device, const void * data, int size, VkBufferUsageFlagBits usageFlags );
	void Cleanup( DeviceContext * device );
	void * MapBuffer( DeviceContext * device );
	void UnmapBuffer( DeviceContext * device );

	VkBuffer		m_vkBuffer;
	VkDeviceMemory	m_vkBufferMemory;
	VkDeviceSize	m_vkBufferSize;
	VkMemoryPropertyFlags m_vkMemoryPropertyFlags;
};