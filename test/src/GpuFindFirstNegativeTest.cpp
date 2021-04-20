#ifdef OPENCL_ENABLED

#ifndef GPU_FIRST_NEGATIVE_TEST_HEADER
#define GPU_FIRST_NEGATIVE_TEST_HEADER

#include "SpectrumPhysicsTest.h"
#include "GpuDevice.h"
#include "GpuContext.h"
#include "Randomizer.h"
#include "FileSystem.h"

#define CLASS_NAME GpuFindFirstNegativeTest

namespace NAMESPACE_PHYSICS_TEST
{

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		SP_TEST_METHOD_DEF(execute_1);
		SP_TEST_METHOD_DEF(execute_2);
		SP_TEST_METHOD_DEF(execute_3);
	};

	SP_TEST_METHOD(CLASS_NAME, execute_1)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice();

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_STRIDE=1 -DINPUT_OFFSET=0";

		SpDirectory* filename = SpDirectory::currentDirectory()
			->add(SP_DIRECTORY_OPENCL_SOURCE)
			->add("FindFirstNegative.cl");

		SP_FILE file;
		SpString* source = file.readTextFile(filename->name()->data());
		const sp_uint programIndex = gpu->commandManager->cacheProgram(source->data(), SIZEOF_CHAR * source->length(), buildOptions.str().c_str());
		cl_program program = gpu->commandManager->cachedPrograms[programIndex];

		sp_mem_delete(source, SpString);
		sp_mem_delete(filename, SpDirectory);

		sp_uint length = 6u;
		sp_float input[] = { 1.0f, -4.0f, 2.0f, 3.0f, -5.0f, -6.0f };
		sp_uint indexes[] = { 0, 2, 3, 1, 4, 5 };
		sp_uint defaultOutput = SP_UINT_MAX;
		cl_mem outputGpu = gpu->createBuffer(&defaultOutput, SIZEOF_UINT, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);

		GpuCommand* command = gpu->commandManager->createCommand()
			->setInputParameter(input, length * SIZEOF_FLOAT)
			->setInputParameter(indexes, length * SIZEOF_UINT)
			->setInputParameter(&length, SIZEOF_UINT)
			->setInputParameter(outputGpu, SIZEOF_UINT)
			->buildFromProgram(program, "findFirstNegative");
		
		sp_uint globalWorkSize[3] = { length, 0u, 0u };
		sp_uint localWorkSize[3] = { 1u, 0u, 0u };

		cl_event evt;
		command->execute(1u, globalWorkSize, localWorkSize, ZERO_UINT, ZERO_UINT, NULL, &evt);
		gpu->releaseEvent(evt);

		sp_uint result;
		gpu->commandManager->readBuffer(outputGpu, SIZEOF_UINT, &result);
		Assert::AreEqual(3u, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, execute_2)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice();
		
		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_STRIDE=1 -DINPUT_OFFSET=0";

		SpDirectory* filename = SpDirectory::currentDirectory()
			->add(SP_DIRECTORY_OPENCL_SOURCE)
			->add("FindFirstNegative.cl");

		SP_FILE file;
		SpString* source = file.readTextFile(filename->name()->data());
		const sp_uint programIndex = gpu->commandManager->cacheProgram(source->data(), SIZEOF_CHAR * source->length(), buildOptions.str().c_str());
		cl_program program = gpu->commandManager->cachedPrograms[programIndex];

		sp_mem_delete(source, SpString);
		sp_mem_delete(filename, SpDirectory);

		sp_uint length = 6u;
		sp_float input[] = { -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f };
		sp_uint indexes[] = { 0, 1, 2, 3, 4, 5 };
		sp_uint defaultOutput = SP_UINT_MAX;
		cl_mem outputGpu = gpu->createBuffer(&defaultOutput, SIZEOF_UINT, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);

		GpuCommand* command = gpu->commandManager->createCommand()
			->setInputParameter(input, length * SIZEOF_FLOAT)
			->setInputParameter(indexes, length * SIZEOF_UINT)
			->setInputParameter(&length, SIZEOF_UINT)
			->setInputParameter(outputGpu, SIZEOF_UINT)
			->buildFromProgram(program, "findFirstNegative");

		sp_uint globalWorkSize[3] = { length - 1u, 0u, 0u };
		sp_uint localWorkSize[3] = { 1u, 0u, 0u };

		cl_event evt;
		command->execute(1u, globalWorkSize, localWorkSize, ZERO_UINT, ZERO_UINT, NULL, &evt);
		gpu->releaseEvent(evt);

		sp_uint result;
		gpu->commandManager->readBuffer(outputGpu, SIZEOF_UINT, &result);
		Assert::AreEqual(0u, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, execute_3)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice();

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_STRIDE=1 -DINPUT_OFFSET=0";

		SpDirectory* filename = SpDirectory::currentDirectory()
			->add(SP_DIRECTORY_OPENCL_SOURCE)
			->add("FindFirstNegative.cl");

		SP_FILE file;
		SpString* source = file.readTextFile(filename->name()->data());
		const sp_uint programIndex = gpu->commandManager->cacheProgram(source->data(), SIZEOF_CHAR * source->length(), buildOptions.str().c_str());
		cl_program program = gpu->commandManager->cachedPrograms[programIndex];

		sp_mem_delete(source, SpString);
		sp_mem_delete(filename, SpDirectory);

		sp_uint length = 6u;
		sp_float input[] = { 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f };
		sp_uint indexes[] = { 0, 1, 2, 3, 4, 5 };
		sp_uint defaultOutput = SP_UINT_MAX;
		cl_mem outputGpu = gpu->createBuffer(&defaultOutput, SIZEOF_UINT, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);

		GpuCommand* command = gpu->commandManager->createCommand()
			->setInputParameter(input, length * SIZEOF_FLOAT)
			->setInputParameter(indexes, length * SIZEOF_UINT)
			->setInputParameter(&length, SIZEOF_UINT)
			->setInputParameter(outputGpu, SIZEOF_UINT)
			->buildFromProgram(program, "findFirstNegative");

		sp_uint globalWorkSize[3] = { length - 1u, 0u, 0u };
		sp_uint localWorkSize[3] = { 1u, 0u, 0u };

		cl_event evt;
		command->execute(1u, globalWorkSize, localWorkSize, ZERO_UINT, ZERO_UINT, NULL, &evt);
		gpu->releaseEvent(evt);

		sp_uint result;
		gpu->commandManager->readBuffer(outputGpu, SIZEOF_UINT, &result);
		Assert::AreEqual(defaultOutput, result, L"Wrong value.", LINE_INFO());
	}

}

#endif // GPU_FIRST_NEGATIVE_TEST_HEADER

#endif // OPENCL