//
//  Descriptor.h
//
#pragma once
#include <vulkan/vulkan.h>

class DeviceContext;
class Buffer;
class Pipeline;
class Image;
struct RenderModel;

/*
====================================================
Descriptor
====================================================
*/
class Descriptor {
public:
	Descriptor();
	~Descriptor() {}

	void BindImage( VkImageLayout imageLayout, VkImageView imageView, VkSampler sampler, int slot );
	void BindBuffer( Buffer * uniformBuffer, int offset, int size, int slot );
	void BindDescriptor( DeviceContext * device, VkCommandBuffer vkCommandBuffer, Pipeline * pso );

	friend class Descriptors;
private:
	Descriptors * m_parent;

	int m_id;	// the id of the descriptor set to be used

	int m_numBuffers;
	static const int MAX_BUFFERS = 16;
	VkDescriptorBufferInfo m_bufferInfo[ MAX_BUFFERS ];

	int m_numImages;
	static const int MAX_IMAGEINFO = 16;
	VkDescriptorImageInfo m_imageInfo[ MAX_IMAGEINFO ];
};

/*
====================================================
DescriptorSets
====================================================
*/
class Descriptors {
public:
	Descriptors() : m_numDescriptorUsed( 0 ) {}
	~Descriptors() {}

	// This structure creates the layout
	struct CreateParms_t {
		int numUniformsVertex;
		int numUniformsFragment;
		int numImageSamplers;
	};
	CreateParms_t m_parms;

	bool Create( DeviceContext * device, const CreateParms_t & parms );
	void Cleanup( DeviceContext * device );

	static const int MAX_DESCRIPTOR_SETS = 256;

	VkDescriptorPool m_vkDescriptorPool;
	VkDescriptorSetLayout m_vkDescriptorSetLayout;

	// The individual descriptor sets
	int m_numDescriptorUsed;
	VkDescriptorSet m_vkDescriptorSets[ MAX_DESCRIPTOR_SETS ];


	Descriptor GetFreeDescriptor() {
		Descriptor descriptor;
		descriptor.m_parent = this;
		descriptor.m_id = m_numDescriptorUsed % MAX_DESCRIPTOR_SETS;
		m_numDescriptorUsed++;
		return descriptor;
	}
};