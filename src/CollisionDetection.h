#ifndef COLLISION_DETECTION_HEADER
#define COLLISION_DETECTION_HEADER

#include "SpectrumPhysics.h"
#include "AABB.h"
#include "DOP18.h"

namespace NAMESPACE_PHYSICS
{

	class CollisionDetection
	{
	public:

		///<summary>
		/// Find the collision using brute force algorithm O(n^2)
		///</summary>
		API_INTERFACE std::vector<std::pair<AABB, AABB>> bruteForce(AABB* boundingVolumes, size_t count);

		///<summary>
		/// Find the collision using brute force algorithm O(n^2)
		///</summary>
		API_INTERFACE std::vector<std::pair<DOP18, DOP18>> bruteForce(DOP18* boundingVolumes, size_t count);

	};

}

#endif // COLLISION_DETECTION_HEADER