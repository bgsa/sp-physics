#ifndef SP_RENDERING_FACTORY_HEADER
#define SP_RENDERING_FACTORY_HEADER

#include "SpectrumPhysics.h"
#include "SpShader.h"

namespace NAMESPACE_PHYSICS
{
	class SpRenderingFactory
	{
	public:

		API_INTERFACE virtual sp_int bufferUsageTypeStaticDraw() = 0;

		API_INTERFACE virtual sp_int typeFloatId() = 0;

		API_INTERFACE virtual sp_int typeUIntId() = 0;

		API_INTERFACE virtual sp_int typeTriangleId() = 0;
		
		API_INTERFACE virtual SpGpuBuffer* createBuffer(const sp_int type) = 0;

		API_INTERFACE virtual SpGpuBuffer* createArrayBuffer() = 0;

		API_INTERFACE virtual SpGpuBuffer* createElementArrayBuffer() = 0;

		API_INTERFACE virtual SpShader* createShader() = 0;

		API_INTERFACE virtual SpShader* createPrimitiveShader() = 0;

	};

}

#endif // SP_RENDERING_FACTORY_HEADER