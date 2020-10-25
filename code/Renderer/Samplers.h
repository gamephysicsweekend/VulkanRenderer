//
//  Samplers.h
//
#pragma once
#include <vulkan/vulkan.h>
#include "DeviceContext.h"

/*
====================================================
Samplers
====================================================
*/
class Samplers {
public:
	static bool InitializeSamplers( DeviceContext * device );
	static void Cleanup( DeviceContext * device );

	static VkSampler m_samplerStandard;
	static VkSampler m_samplerDepth;
};
