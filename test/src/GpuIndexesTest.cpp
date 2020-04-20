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
		GpuDevice* gpu = context->defaultDevice;

		GpuIndexes* commandIndexes = ALLOC_NEW(GpuIndexes)();

		const sp_uint count = 10;

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << count;
		buildOptions << " -DINPUT_STRIDE=1";
		buildOptions << " -DINPUT_OFFSET=0";

		commandIndexes->init(gpu, buildOptions.str().c_str());
		commandIndexes->setParametersCreateIndexes(count);

		cl_mem output = commandIndexes->execute();

		sp_uint* values = ALLOC_ARRAY(sp_uint, count);
		gpu->commandManager->executeReadBuffer(output, count * SIZEOF_UINT, values, true);

		for (sp_uint i = 0; i < count; i++)
			Assert::AreEqual(i, values[i], L"Wrong value.", LINE_INFO());

		gpu->releaseBuffer(output);
		ALLOC_DELETE(commandIndexes, GpuIndexes);
		ALLOC_RELEASE(commandIndexes);
	}

	SP_TEST_METHOD(CLASS_NAME, GpuIndexes_createIndexes_Many)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;

		GpuIndexes* commandIndexes = ALLOC_NEW(GpuIndexes)();

		const sp_uint count = (sp_uint)powf(2.0f, 17.0f);

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << count;
		buildOptions << " -DINPUT_STRIDE=1";
		buildOptions << " -DINPUT_OFFSET=0";

		commandIndexes->init(gpu, buildOptions.str().c_str());
		commandIndexes->setParametersCreateIndexes(count);

		cl_mem output = commandIndexes->execute();

		sp_uint* values = ALLOC_ARRAY(sp_uint, count);
		gpu->commandManager->executeReadBuffer(output, count * SIZEOF_UINT, values, true);

		for (sp_uint i = 0; i < count; i++)
			Assert::AreEqual(i, values[i], L"Wrong value.", LINE_INFO());

		gpu->releaseBuffer(output);
		ALLOC_DELETE(commandIndexes, GpuIndexes);
		ALLOC_RELEASE(commandIndexes);
	}

	SP_TEST_METHOD(CLASS_NAME, GpuIndexes_createIndexes_Some)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;

		GpuIndexes* commandIndexes = ALLOC_NEW(GpuIndexes)();

		const sp_uint count = 100u;

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << count;
		buildOptions << " -DINPUT_STRIDE=1";
		buildOptions << " -DINPUT_OFFSET=0";

		commandIndexes->init(gpu, buildOptions.str().c_str());
		commandIndexes->setParametersCreateIndexes(count);

		cl_mem output = commandIndexes->execute();

		sp_uint* values = ALLOC_ARRAY(sp_uint, count);
		gpu->commandManager->executeReadBuffer(output, count * SIZEOF_UINT, values, true);

		for (sp_uint i = 0; i < count; i++)
			Assert::AreEqual(i, values[i], L"Wrong value.", LINE_INFO());

		gpu->releaseBuffer(output);
		ALLOC_DELETE(commandIndexes, GpuIndexes);
		ALLOC_RELEASE(commandIndexes);
	}

}

#undef CLASS_NAME

#endif // OPENCL_ENABLED