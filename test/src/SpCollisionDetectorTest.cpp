#include "SpectrumPhysicsTest.h"
#include "SpGpuRenderingFactoryOpenGL.h"
#include <SpCollisionDetector.h>

#define CLASS_NAME SpCollisionDetectorTest

namespace NAMESPACE_PHYSICS_TEST
{

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(areMovingAway);

	};

	SP_TEST_METHOD(CLASS_NAME, areMovingAway)
	{
		NAMESPACE_RENDERING::SpOpenGL::initOffScreenRendering();
		GpuContext::init();
		NAMESPACE_RENDERING::SpGpuRenderingFactoryOpenGL::init();
		SpPhysicSimulator::init(2u);
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		simulator->alloc(2u);
		SpCollisionDetector collisionDetector;

		simulator->physicProperties(0u)->position(Vec3(1.0f, 0.0f, 0.0f));
		simulator->physicProperties(1u)->position(Vec3(0.0f, 0.0f, 0.0f));

		simulator->physicProperties(0u)->velocity(Vec3(10.0f, 0.0f, 0.0f));
		simulator->physicProperties(1u)->velocity(Vec3(-10.0f, 0.0f, 0.0f));

		sp_bool result = collisionDetector.areMovingAway(0u, 1u);

		Assert::IsTrue(result, L"Wrong value.", LINE_INFO());

		simulator->physicProperties(0u)->velocity(Vec3(10.0f, 0.0f, 0.0f));
		simulator->physicProperties(1u)->velocity(Vec3(1.0f, 0.0f, 0.0f));

		result = collisionDetector.areMovingAway(0u, 1u);
		
		Assert::IsFalse(result, L"Wrong value.", LINE_INFO());

		NAMESPACE_RENDERING::SpOpenGL::dispose();
	}
}

#undef CLASS_NAME
