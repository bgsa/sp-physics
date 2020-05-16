#ifndef SP_PHYSIC_OBJECT_HEADER
#define SP_PHYSIC_OBJECT_HEADER

#include "SpectrumPhysics.h"
#include "BoundingVolume.h"

namespace NAMESPACE_PHYSICS
{
	class SpPhysicObject :
		public Object
	{
	public:

		API_INTERFACE SpPhysicObject()
		{
		}

		API_INTERFACE virtual BoundingVolume* boundingVolume() const = 0;

		API_INTERFACE void dispose() 
		{
		}

		API_INTERFACE inline const sp_char* toString()
		{
			return "SpPhysic Object";
		}

		~SpPhysicObject()
		{
			dispose();
		}
	};
}

#endif // SP_PHYSIC_OBJECT_HEADER