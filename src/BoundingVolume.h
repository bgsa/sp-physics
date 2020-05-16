#ifndef BOUNDING_VOLUME_HEADER
#define BOUNDING_VOLUME_HEADER

#include "SpectrumPhysics.h"
#include "Vec3.h"
#include "ParticleSystem.h"

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

		API_INTERFACE virtual Vec3f centerOfBoundingVolume() const = 0;

		API_INTERFACE virtual BoundingVolumeType type() const = 0;

	};

}

#endif // BOUNDING_VOLUME_HEADER