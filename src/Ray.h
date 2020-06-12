#ifndef RAY_HEADER
#define RAY_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
	class Ray
	{
	public:
		Vec3 point;
		Vec3 direction;

		API_INTERFACE Ray()
		{
			point = Vec3(0.0f);
			direction = Vec3(0.0f, 1.0f, 0.0f);
		}

		API_INTERFACE Ray(const Vec3& point, const Vec3& direction)
		{
			this->point = point;
			this->direction = direction;
		}


	};

}

#endif // RAY_HEADER
