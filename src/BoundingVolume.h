#ifndef SP_BOUNDING_VOLUME_HEADER
#define SP_BOUNDING_VOLUME_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{

	enum class BoundingVolumeType
		: sp_int
	{
		Sphere = 1,
		OBB = 2,
		AABB = 3,
		DOP18 = 4,
	};

	class BoundingVolume 
		: public Object
	{
	public:

		API_INTERFACE virtual Vec3 centerOfBoundingVolume() const = 0;

		API_INTERFACE virtual void translate(const Vec3& translation) = 0;

		API_INTERFACE virtual void scale(const Vec3& factor) = 0;

		API_INTERFACE virtual void rotate(const Vec3& angles) = 0;

		API_INTERFACE virtual BoundingVolumeType type() const = 0;

	};

}

#endif // SP_BOUNDING_VOLUME_HEADER