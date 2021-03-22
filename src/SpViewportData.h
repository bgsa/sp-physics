#ifndef SP_VIEWPORT_DATA_HEADER
#define SP_VIEWPORT_DATA_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
	class SpViewportData
	{
	public:
		sp_int x, y, width, height;
		
		API_INTERFACE SpViewportData()
		{
			x = ZERO_INT;
			y = ZERO_INT;
			width = 640;
			height = 480;
		}

		API_INTERFACE void move(sp_int x, sp_int y)
		{
			this->x = x;
			this->y = y;
		}

		API_INTERFACE void resize(sp_int width, sp_int height)
		{
			this->width = width;
			this->height = height;
		}

		API_INTERFACE sp_float aspectRatio() const
		{
			return (sp_float)((sp_float)width / (sp_float)height);
		}

	};
}

#endif // CAMERA_HEADER