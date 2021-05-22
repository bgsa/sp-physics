#ifdef OPENCL_ENABLED

#include "SpectrumPhysicsTest.h"
#include "Randomizer.h"
#include <GpuReverse.h>
#include "GpuContext.h"
#include <limits>
#include <AABB.h>

#define CLASS_NAME GpuReverseTest

namespace NAMESPACE_PHYSICS_TEST
{

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(GpuReverse_Test);

	};

	SP_TEST_METHOD(CLASS_NAME, GpuReverse_Test)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice();

		GpuReverse* commandReverse = ALLOC_NEW(GpuReverse)();

		const sp_uint count = 5;
		sp_uint input[count] = { 1u, 2u, 4u, 5u, 3u };
		sp_uint expected[count] = { 3u, 5u, 4u, 2u, 1u };

		cl_event evt;
		cl_mem output = commandReverse
			->init(gpu, NULL)
			->setParameters(input, count)
			->execute(ZERO_UINT, NULL, &evt);

		sp_uint* values = ALLOC_ARRAY(sp_uint, count);
		gpu->commandManager->readBuffer(output, count * sizeof(sp_uint), values, ONE_UINT, &evt);
		gpu->releaseEvent(evt);

		for (sp_uint i = 0; i < count; i++)
			Assert::AreEqual(expected[i], values[i], L"Wrong value.", LINE_INFO());

		gpu->releaseBuffer(output);
		ALLOC_DELETE(commandReverse, GpuReverse);
	}

}

#undef CLASS_NAME

#endif // OPENCL_ENABLED