//
//  OffscreenRenderer.h
//
#pragma once

class DeviceContext;
class Buffer;
struct RenderModel;

bool InitOffscreen( DeviceContext * device, int width, int height );
bool CleanupOffscreen( DeviceContext * device );

void DrawOffscreen( DeviceContext * device, int cmdBufferIndex, Buffer * uniforms, const RenderModel * renderModels, const int numModels );