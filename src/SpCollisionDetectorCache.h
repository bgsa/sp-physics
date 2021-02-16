#ifndef SP_COLLISION_DETECTOR_CACHE_HEADER
#define SP_COLLISION_DETECTOR_CACHE_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
	class SpCollisionDetectorCache
	{
	public:
		sp_uint edgeIndex;
		sp_uint faceIndex;
		sp_bool edgeIndexOnObj1;
		sp_bool searchOnObj1;
		sp_float distance;

		API_INTERFACE SpCollisionDetectorCache()
		{
			clear();
		}

		API_INTERFACE inline void clear()
		{
			edgeIndex = SP_UINT_MAX;
			faceIndex = SP_UINT_MAX;
			edgeIndexOnObj1 = true;
			searchOnObj1 = false;
			distance = SP_FLOAT_MAX;
		}

		API_INTERFACE inline sp_bool hasCache() const
		{
			return edgeIndex != SP_UINT_MAX;
		}
	};

}

#endif // SP_COLLISION_DETECTOR_CACHE_HEADER