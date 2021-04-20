#ifndef SP_RENDERER_DATA_HEADER
#define SP_RENDERER_DATA_HEADER

#include "SpectrumPhysics.h"
#include "SpViewportData.h"
#include "SpVector.h"

namespace NAMESPACE_PHYSICS
{
	class SpRenderData
	{
	public:
		Mat4 projectionMatrix;
		Mat4 viewMatrix;
		SpViewportData* viewport;

		SpVector<void*> customProperties = SpVector<void*>();
	};
}

#endif // SP_RENDERER_DATA_HEADER