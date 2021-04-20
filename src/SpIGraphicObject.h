#ifndef SP_I_GRAPHIC_OBJECT_HEADER
#define SP_I_GRAPHIC_OBJECT_HEADER

#include "SpectrumPhysics.h"
#include "SpRenderData.h"

namespace NAMESPACE_PHYSICS
{
	class SpIGraphicObject
	{
	public:

		API_INTERFACE virtual void render(const SpRenderData& renderData) = 0;

	};

}

#endif // SP_I_GRAPHIC_OBJECT_HEADER