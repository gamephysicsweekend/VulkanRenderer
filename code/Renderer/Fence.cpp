//
//  Fence.cpp
//
#include "Fence.h"
#include <assert.h>
#include <stdio.h>

/*
========================================================================================================

Fence

========================================================================================================
*/

/*
====================================================
Fence::Create
====================================================
*/
bool Fence::Create( DeviceContext * device ) {
	VkFenceCreateInfo fenceCreateInfo {};
	fenceCreateInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
	fenceCreateInfo.flags = 0;

	VkResult result = vkCreateFence( device->m_vkDevice, &fenceCreateInfo, nullptr, &m_vkFence );
	if ( VK_SUCCESS != result ) {
		printf( "failed to create fence\n" );
		assert( 0 );
		return false;
	}

	return true;
}

/*
====================================================
Fence::Wait
====================================================
*/
bool Fence::Wait( DeviceContext * device ) {
	uint64_t timeoutns = (uint64_t)1e7;
	VkResult result = vkWaitForFences( device->m_vkDevice, 1, &m_vkFence, VK_TRUE, timeoutns );

	vkDestroyFence( device->m_vkDevice, m_vkFence, nullptr );

	if ( VK_SUCCESS != result ) {
		printf( "failed to wait for fence\n" );
		assert( 0 );
		return false;
	}
	return true;
}
