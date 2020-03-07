#include "TestHeader.h"
#include <SweepAndPruneKdop.h>
#include <CollisionDetection.h>

#define CLASS_NAME SweepAndPruneKdopTest

namespace SP_PHYSICS_TEST_NAMESPACE
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(SweepAndPruneKdop_findCollisions_Test);

#ifdef OPENCL_ENABLED
		SP_TEST_METHOD_DEF(SweepAndPruneKdop_findCollisionsGPU_Test);
#endif // !OPENCL_ENABLED

	};

	SP_TEST_METHOD(CLASS_NAME, SweepAndPruneKdop_findCollisions_Test)
	{
		CollisionDetection collisionDetection;
		size_t count = 5;
		DOP18 kdops[5] = { DOP18(), DOP18(), DOP18(), DOP18(), DOP18() };

		kdops[0].translate(0.0f, 0.0f, 2.0f);
		kdops[1].translate(10.0f, 1.0f, 0.0f);
		kdops[2].translate(11.0f, 2.0f, 1.0f);
		kdops[3].translate(-10.0f, 0.0f, 0.0f);
		kdops[4].translate(10.0f, 1.5f, 0.0f);

		std::vector<std::pair<DOP18, DOP18>> expected = collisionDetection.bruteForce(kdops, count);

		std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

		SweepAndPruneResult result = SweepAndPruneKdop::findCollisions(kdops, count);

		std::chrono::high_resolution_clock::time_point currentTime2 = std::chrono::high_resolution_clock::now();
		std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime2 - currentTime);

		Assert::AreEqual(expected.size(), result.count, L"wrong value", LINE_INFO());
	}

#ifdef OPENCL_ENABLED
	SP_TEST_METHOD(CLASS_NAME, SweepAndPruneKdop_findCollisionsGPU_Test)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;
		SweepAndPruneKdop* sapKdop = ALLOC_NEW(SweepAndPruneKdop)();
		sapKdop->init(gpu, NULL);

		CollisionDetection collisionDetection;
		size_t count = 5;
		DOP18 kdops[5] = { DOP18(), DOP18(), DOP18(), DOP18(), DOP18() };

		kdops[0].translate(0.0f, 0.0f, 2.0f);
		kdops[1].translate(10.0f, 1.0f, 0.0f);
		kdops[2].translate(11.0f, 2.0f, 1.0f);
		kdops[3].translate(-10.0f, 0.0f, 0.0f);
		kdops[4].translate(10.0f, 1.5f, 0.0f);

		std::vector<std::pair<DOP18, DOP18>> expected = collisionDetection.bruteForce(kdops, count);

		std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

		SweepAndPruneResult result = sapKdop->findCollisions(gpu, kdops, count);

		std::chrono::high_resolution_clock::time_point currentTime2 = std::chrono::high_resolution_clock::now();
		std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime2 - currentTime);

		Assert::AreEqual(expected.size(), result.count, L"wrong value", LINE_INFO());
	}
#endif // !OPENCL_ENABLED

}

#undef CLASS_NAME