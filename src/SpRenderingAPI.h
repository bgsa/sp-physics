#ifndef SP_RENDERING_API_HEADER
#define SP_RENDERING_API_HEADER

#include "SpectrumPhysics.h"
#include "SpGpuBuffer.h"
#include "SpShader.h"
#include "SpViewportData.h"
#include "SpRect.h"

namespace NAMESPACE_PHYSICS
{
	class SpRenderingAPI
	{
	public:

		API_INTERFACE virtual void enable(const sp_uint propertyId) = 0;

		API_INTERFACE virtual void disable(const sp_uint propertyId) = 0;

		API_INTERFACE virtual void disableDepthTest() = 0;

		API_INTERFACE virtual void enableLineSmooth() = 0;

		API_INTERFACE virtual void enableDepthTest() = 0;

		API_INTERFACE virtual void enableCullingFace() = 0;

		API_INTERFACE virtual void enablePolygonOffsetFill() = 0;

		API_INTERFACE virtual void setPolygonOffset(const sp_float x, const sp_float y) = 0;

		API_INTERFACE virtual void clearColor(const SpColorRGBA& rgba) = 0;

		API_INTERFACE virtual void clearFramebuffer() = 0;

		API_INTERFACE virtual void setViewport(const SpViewportData& viewport) = 0;

		API_INTERFACE virtual void setScissor(const SpRect<sp_int>& area) = 0;

		API_INTERFACE virtual sp_uint bufferUsageTypeStaticDraw() = 0;

		API_INTERFACE virtual sp_uint bufferUsageTypeDynamicDraw() = 0;

		API_INTERFACE virtual sp_uint typeFloatId() = 0;

		API_INTERFACE virtual sp_uint typeUIntId() = 0;

		API_INTERFACE virtual sp_uint typeTriangleId() = 0;

		API_INTERFACE virtual sp_uint typeLinesId() = 0;
		
		API_INTERFACE virtual SpGpuBuffer* createBuffer(const sp_int type) = 0;

		API_INTERFACE virtual SpGpuTextureBuffer* createTextureBuffer() = 0;

		API_INTERFACE virtual SpGpuBuffer* createArrayBuffer() = 0;

		API_INTERFACE virtual SpGpuBuffer* createElementArrayBuffer() = 0;

		API_INTERFACE virtual SpGpuBuffer* createShaderStorageBufferObject() = 0;

		API_INTERFACE virtual SpShader* createShader() = 0;

		API_INTERFACE virtual SpShader* createPrimitiveShader() = 0;

		API_INTERFACE virtual SpShader* createLinesShader() = 0;

		API_INTERFACE virtual SpShader* createManipulatorShader() = 0;

	};

}

#endif // SP_RENDERING_API_HEADER