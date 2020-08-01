#ifndef SP_GPU_RENDERING_FACTORY_HEADER
#define SP_GPU_RENDERING_FACTORY_HEADER

#include "SpectrumPhysics.h"
#include "SpGpuBuffer.h"
#include "SpGpuTextureBuffer.h"

namespace NAMESPACE_PHYSICS
{
	class SpGpuRenderingFactory
		: public Object
	{
	public:

		API_INTERFACE virtual SpGpuBuffer* createBuffer(const sp_int type) = 0;

		API_INTERFACE virtual SpGpuBuffer* createBuffer(const sp_size size, const void* data, sp_int bufferType, sp_int usageType) = 0;
		
		API_INTERFACE virtual SpGpuTextureBuffer* createTextureBuffer() = 0;

	};

	extern SpGpuRenderingFactory* instanceGpuRendering;
}

#endif // SP_GPU_RENDERING_FACTORY_HEADER