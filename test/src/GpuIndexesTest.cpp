#ifdef OPENCL_ENABLED

#include "SpectrumPhysicsTest.h"
#include "Randomizer.h"
#include <GpuIndexes.h>
#include "GpuContext.h"
#include <limits>
#include <AABB.h>

#define CLASS_NAME GpuIndexesTest

#undef max
#undef min

namespace NAMESPACE_PHYSICS_TEST
{

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(GpuIndexes_createIndexes);

		SP_TEST_METHOD_DEF(GpuIndexes_createIndexes_Some);

		SP_TEST_METHOD_DEF(GpuIndexes_createIndexes_Many);

	};

	SP_TEST_METHOD(CLASS_NAME, GpuIndexes_createIndexes)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice();

		GpuIndexes* commandIndexes = ALLOC_NEW(GpuIndexes)();

		const sp_uint length = 10;

		commandIndexes->init(gpu, nullptr);
		commandIndexes->setParametersCreateIndexes(length);

		cl_event evt;
		cl_mem output = commandIndexes->execute(ZERO_UINT, NULL, &evt);

		sp_uint* values = ALLOC_ARRAY(sp_uint, length);
		gpu->commandManager->readBuffer(output, length * SIZEOF_UINT, values, ONE_UINT, &evt);
		gpu->releaseEvent(evt);

		for (sp_uint i = 0; i < length; i++)
			Assert::AreEqual(i, values[i], L"Wrong value.", LINE_INFO());

		gpu->releaseBuffer(output);
		ALLOC_DELETE(commandIndexes, GpuIndexes);
		ALLOC_RELEASE(commandIndexes);
	}

	SP_TEST_METHOD(CLASS_NAME, GpuIndexes_createIndexes_Many)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice();

		GpuIndexes* commandIndexes = ALLOC_NEW(GpuIndexes)();

		const sp_uint count = (sp_uint)powf(2.0f, 17.0f);

		commandIndexes->init(gpu, nullptr);
		commandIndexes->setParametersCreateIndexes(count);

		cl_event evt;
		cl_mem output = commandIndexes->execute(ZERO_UINT, NULL, &evt);

		sp_uint* values = ALLOC_ARRAY(sp_uint, count);
		gpu->commandManager->readBuffer(output, count * SIZEOF_UINT, values, ONE_UINT, &evt);
		gpu->releaseEvent(evt);

		for (sp_uint i = 0; i < count; i++)
			Assert::AreEqual(i, values[i], L"Wrong value.", LINE_INFO());

		gpu->releaseBuffer(output);
		ALLOC_DELETE(commandIndexes, GpuIndexes);
		ALLOC_RELEASE(commandIndexes);
	}

	SP_TEST_METHOD(CLASS_NAME, GpuIndexes_createIndexes_Some)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice();

		GpuIndexes* commandIndexes = ALLOC_NEW(GpuIndexes)();

		const sp_uint count = 100u;

		commandIndexes->init(gpu, nullptr);
		commandIndexes->setParametersCreateIndexes(count);

		cl_event evt;
		cl_mem output = commandIndexes->execute(ZERO_UINT, NULL, &evt);

		sp_uint* values = ALLOC_ARRAY(sp_uint, count);
		gpu->commandManager->readBuffer(output, count * SIZEOF_UINT, values, ONE_UINT, &evt);
		gpu->releaseEvent(evt);

		for (sp_uint i = 0; i < count; i++)
			Assert::AreEqual(i, values[i], L"Wrong value.", LINE_INFO());

		gpu->releaseBuffer(output);
		ALLOC_DELETE(commandIndexes, GpuIndexes);
		ALLOC_RELEASE(commandIndexes);
	}

}

#undef CLASS_NAME

#endif // OPENCL_ENABLED