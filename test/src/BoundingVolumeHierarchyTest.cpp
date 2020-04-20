#include "SpectrumPhysicsTest.h"
#include <BoundingVolumeHierarchy.h>

#define CLASS_NAME BoundingVolumeHierarchyTest

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(BoundingVolumeHierarchy_constructor_empty_Test);

	};

	SP_TEST_METHOD(CLASS_NAME, BoundingVolumeHierarchy_constructor_empty_Test)
	{
		//BoundingVolumeHierarchy bvh = BoundingVolumeHierarchy();
	}

}

#undef CLASS_NAME