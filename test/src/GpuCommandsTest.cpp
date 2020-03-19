#ifdef OPENCL_ENABLED

#include "TestHeader.h"
#include "Randomizer.h"
#include <GpuCommands.h>
#include <GpuFindMinMax.h>
#include <limits>
#include <AABB.h>

#define CLASS_NAME GpuCommandsTest

#undef max
#undef min

namespace NAMESPACE_PHYSICS_TEST
{

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(GpuCommands_createIndexes);

		SP_TEST_METHOD_DEF(GpuCommands_createIndexes_Many);

	};

	SP_TEST_METHOD(CLASS_NAME, GpuCommands_createIndexes)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;

		const sp_size count = 10;

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << count;
		buildOptions << " -DINPUT_STRIDE=1";
		buildOptions << " -DINPUT_OFFSET=0";
		GpuCommands::init(gpu, buildOptions.str().c_str());

		cl_mem indexes = GpuCommands::creteIndexes(gpu, count);

		sp_size* values = ALLOC_ARRAY(sp_size, count);
		gpu->commandManager->executeReadBuffer(indexes, count * SIZEOF_SIZE, values, true);

		for (sp_size i = 0; i < count; i++)
			Assert::AreEqual(i, values[i], L"Wrong value.", LINE_INFO());

		gpu->releaseBuffer(indexes);
		ALLOC_RELEASE(values);
	}

	SP_TEST_METHOD(CLASS_NAME, GpuCommands_createIndexes_Many)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;

		const sp_size count = (sp_size)powf(2.0f, 17.0f);

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << count;
		buildOptions << " -DINPUT_STRIDE=1";
		buildOptions << " -DINPUT_OFFSET=0";
		GpuCommands::init(gpu, buildOptions.str().c_str());

		cl_mem indexes = GpuCommands::creteIndexes(gpu, count);

		sp_size* values = ALLOC_ARRAY(sp_size, count);
		gpu->commandManager->executeReadBuffer(indexes, count * SIZEOF_UINT, (void*)values, true);

		for (sp_size i = 0; i < count; i++)
			Assert::AreEqual(i, values[i], L"Wrong value.", LINE_INFO());

		gpu->releaseBuffer(indexes);
		ALLOC_RELEASE(values);
	}

}

#undef CLASS_NAME

#endif // OPENCL_ENABLED