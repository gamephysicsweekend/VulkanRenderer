//
//  model.h
//
#pragma once
#include <vulkan/vulkan.h>
#include <vector>
#include <array>
#include "DeviceContext.h"
#include "Buffer.h"
#include "../Math/Vector.h"
#include "../Math/Quat.h"

/*
====================================================
vert_t
// 8 * 4 = 32 bytes - data structure for drawable verts... this should be good for most things
====================================================
*/
struct vert_t {
	float			xyz[ 3 ];	// 12 bytes
	float			st[ 2 ];	// 8 bytes
	unsigned char	norm[ 4 ];	// 4 bytes
	unsigned char	tang[ 4 ];	// 4 bytes
	unsigned char	buff[ 4 ];	// 4 bytes

	static VkVertexInputBindingDescription GetBindingDescription() {
		VkVertexInputBindingDescription bindingDescription = {};
		bindingDescription.binding = 0;
		bindingDescription.stride = sizeof( vert_t );
		bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

		return bindingDescription;
	}

	static std::array< VkVertexInputAttributeDescription, 5 > GetAttributeDescriptions() {
		std::array< VkVertexInputAttributeDescription, 5 > attributeDescriptions = {};

		attributeDescriptions[ 0 ].binding = 0;
		attributeDescriptions[ 0 ].location = 0;
		attributeDescriptions[ 0 ].format = VK_FORMAT_R32G32B32_SFLOAT;
		attributeDescriptions[ 0 ].offset = offsetof( vert_t, xyz );

		attributeDescriptions[ 1 ].binding = 0;
		attributeDescriptions[ 1 ].location = 1;
		attributeDescriptions[ 1 ].format = VK_FORMAT_R32G32_SFLOAT;
		attributeDescriptions[ 1 ].offset = offsetof( vert_t, st );

		attributeDescriptions[ 2 ].binding = 0;
		attributeDescriptions[ 2 ].location = 2;
		attributeDescriptions[ 2 ].format = VK_FORMAT_R8G8B8A8_UNORM;
		attributeDescriptions[ 2 ].offset = offsetof( vert_t, norm );

		attributeDescriptions[ 3 ].binding = 0;
		attributeDescriptions[ 3 ].location = 3;
		attributeDescriptions[ 3 ].format = VK_FORMAT_R8G8B8A8_UNORM;
		attributeDescriptions[ 3 ].offset = offsetof( vert_t, tang );

		attributeDescriptions[ 4 ].binding = 0;
		attributeDescriptions[ 4 ].location = 4;
		attributeDescriptions[ 4 ].format = VK_FORMAT_R8G8B8A8_UNORM;
		attributeDescriptions[ 4 ].offset = offsetof( vert_t, buff );

		return attributeDescriptions;
	}
};

class Shape;

/*
====================================================
Model
====================================================
*/
class Model {
public:
	Model() : m_isVBO( false ) {}
	~Model() {}

	std::vector< vert_t > m_vertices;
	std::vector< unsigned int > m_indices;

	bool BuildFromShape( const Shape * shape );
	bool MakeVBO( DeviceContext * device );

	// GPU Data
	bool m_isVBO;
	Buffer	m_vertexBuffer;
	Buffer	m_indexBuffer;

	void Cleanup( DeviceContext & deviceContext );

	void DrawIndexed( VkCommandBuffer vkCommandBUffer );
};

void FillCube( Model & model );
void FillFullScreenQuad( Model & model );




struct RenderModel {
	Model * model;			// The vao buffer to draw
	uint32_t uboByteOffset;	// The byte offset into the uniform buffer
	uint32_t uboByteSize;	// how much space we consume in the uniform buffer

	Vec3 pos;
	Quat orient;
};