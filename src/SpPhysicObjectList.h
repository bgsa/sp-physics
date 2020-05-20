#ifndef SP_PHYSIC_OBJECT_LIST_HEADER
#define SP_PHYSIC_OBJECT_LIST_HEADER

#include "SpectrumPhysics.h"
#include "BoundingVolume.h"
#include "SpArray.h"

namespace NAMESPACE_PHYSICS
{
	class SpPhysicObjectList :
		public Object
	{
	public:

		API_INTERFACE SpPhysicObjectList()
		{
		}

		API_INTERFACE virtual BoundingVolume* boundingVolumes() const = 0;

		API_INTERFACE virtual sp_uint length() const = 0;

		API_INTERFACE void dispose() 
		{
		}

		API_INTERFACE inline const sp_char* toString()
		{
			return "SpPhysic Object List";
		}

		~SpPhysicObjectList()
		{
			dispose();
		}
	};
}

#endif // SP_PHYSIC_OBJECT_LIST_HEADER