#ifndef SP_COLLISION_GROUP_HEADER
#define SP_COLLISION_GROUP_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
#define SP_COLLISION_GROUP_MAX 10

	class SpCollisionGroup
	{
	public:
		sp_uint id;
		sp_uint elementsLength;
		sp_uint elements[SP_COLLISION_GROUP_MAX];

		API_INTERFACE SpCollisionGroup()
		{
			elementsLength = ZERO_UINT;
		}

		API_INTERFACE inline void addElement(sp_uint elementId)
		{
			sp_assert(elementsLength < SP_COLLISION_GROUP_MAX, "InvalidOperationException");

			elements[elementsLength++] = elementId;
		}
	};

	class SpCollisionGroups
	{
	public:
		sp_uint groupLength;
		SpCollisionGroup* groups;
		sp_uint* mapper;

		API_INTERFACE SpCollisionGroups(sp_uint maxObjects, sp_uint maxGroups)
		{
			groupLength = ZERO_UINT;

			mapper = ALLOC_ARRAY(sp_uint, maxObjects);
			std::memset(mapper, SP_UINT_MAX, maxObjects * SIZEOF_UINT);

			groups = ALLOC_NEW_ARRAY(SpCollisionGroup, maxGroups);
		}
	};

#undef SP_COLLISION_GROUP_MAX
}

#endif // SP_COLLISION_GROUP_HEADER