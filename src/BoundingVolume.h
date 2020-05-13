#ifndef BOUNDING_VOLUME_HEADER
#define BOUNDING_VOLUME_HEADER

#include "SpectrumPhysics.h"
#include "Vec3.h"
#include "ParticleSystem.h"

namespace NAMESPACE_PHYSICS
{

	template <typename T>
	class BoundingVolume :
		public Object
	{
	public:

		ParticleSystem* particleSystem = NULL;
		
		API_INTERFACE virtual Vec3f centerOfBoundingVolume() const = 0;

		API_INTERFACE virtual T* translate(sp_float xAxis, sp_float yAxis, sp_float zAxis) = 0;
		API_INTERFACE virtual T* rotate(sp_float angleInRadians, sp_float xAxis, sp_float yAxis, sp_float zAxis) = 0;
		API_INTERFACE virtual T* scale(sp_float xAxis, sp_float yAxis, sp_float zAxis) = 0;

		API_INTERFACE virtual Mat3f modelView() = 0;

	};

	typedef BoundingVolume<Sphere> BoundingVolumeSphere;
	typedef BoundingVolume<AABB> BoundingVolumeAABB;
	typedef BoundingVolume<OBB> BoundingVolumeOBB;
	typedef BoundingVolume<DOP18> BoundingVolumeDOP18;

}

#endif // !BOUNDING_VOLUME_HEADER