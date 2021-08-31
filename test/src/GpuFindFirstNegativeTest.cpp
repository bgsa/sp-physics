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
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_STRIDE=1 -DINPUT_OFFSET=0";

		sp_char filename[512];
		currentDirectory(filename, 512);
		directoryAddPath(filename, std::strlen(filename), SP_DIRECTORY_OPENCL_SOURCE, SP_DIRECTORY_OPENCL_SOURCE_LENGTH);
		directoryAddPath(filename, std::strlen(filename), "FindFirstNegative.cl", 20);

		SP_FILE file;
		SpString* source = file.readTextFile(filename);
		cl_program program;
		gpu->commandManager->buildProgram(source->data(), sizeof(sp_char) * source->length(), buildOptions.str().c_str(), &program);

		sp_mem_delete(source, SpString);

		sp_uint length = 6u;
		sp_float input[] = { 1.0f, -4.0f, 2.0f, 3.0f, -5.0f, -6.0f };
		sp_uint indexes[] = { 0, 2, 3, 1, 4, 5 };
		sp_uint defaultOutput = SP_UINT_MAX;
		cl_mem outputGpu = gpu->createBuffer(&defaultOutput, sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);

		GpuCommand* command = gpu->commandManager->createCommand()
			->setInputParameter(input, length * sizeof(sp_float))
			->setInputParameter(indexes, length * sizeof(sp_uint))
			->setInputParameter(&length, sizeof(sp_uint))
			->setInputParameter(outputGpu, sizeof(sp_uint))
			->buildFromProgram(program, "findFirstNegative");
		
		sp_size globalWorkSize[3] = { length, 0u, 0u };
		sp_size localWorkSize[3] = { 1u, 0u, 0u };

		cl_event evt;
		command->execute(1u, globalWorkSize, localWorkSize, ZERO_UINT, ZERO_UINT, NULL, &evt);
		gpu->releaseEvent(evt);

		sp_uint result;
		gpu->commandManager->readBuffer(outputGpu, sizeof(sp_uint), &result);
		Assert::AreEqual(3u, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, execute_2)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();
		
		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_STRIDE=1 -DINPUT_OFFSET=0";

		sp_char filename[512];
		currentDirectory(filename, 512);
		directoryAddPath(filename, std::strlen(filename), SP_DIRECTORY_OPENCL_SOURCE, SP_DIRECTORY_OPENCL_SOURCE_LENGTH);
		directoryAddPath(filename, std::strlen(filename), "FindFirstNegative.cl", 20);

		SP_FILE file;
		SpString* source = file.readTextFile(filename);
		cl_program program;
		gpu->commandManager->buildProgram(source->data(), sizeof(sp_char) * source->length(), buildOptions.str().c_str(), &program);
		
		sp_mem_delete(source, SpString);
		
		sp_uint length = 6u;
		sp_float input[] = { -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f };
		sp_uint indexes[] = { 0, 1, 2, 3, 4, 5 };
		sp_uint defaultOutput = SP_UINT_MAX;
		cl_mem outputGpu = gpu->createBuffer(&defaultOutput, sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);

		GpuCommand* command = gpu->commandManager->createCommand()
			->setInputParameter(input, length * sizeof(sp_float))
			->setInputParameter(indexes, length * sizeof(sp_uint))
			->setInputParameter(&length, sizeof(sp_uint))
			->setInputParameter(outputGpu, sizeof(sp_uint))
			->buildFromProgram(program, "findFirstNegative");

		sp_size globalWorkSize[3] = { length - 1u, 0u, 0u };
		sp_size localWorkSize[3] = { 1u, 0u, 0u };

		cl_event evt;
		command->execute(1u, globalWorkSize, localWorkSize, ZERO_UINT, ZERO_UINT, NULL, &evt);
		gpu->releaseEvent(evt);

		sp_uint result;
		gpu->commandManager->readBuffer(outputGpu, sizeof(sp_uint), &result);
		Assert::AreEqual(0u, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, execute_3)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_STRIDE=1 -DINPUT_OFFSET=0";

		sp_char filename[512];
		currentDirectory(filename, 512);
		directoryAddPath(filename, std::strlen(filename), SP_DIRECTORY_OPENCL_SOURCE, SP_DIRECTORY_OPENCL_SOURCE_LENGTH);
		directoryAddPath(filename, std::strlen(filename), "FindFirstNegative.cl", 20);

		SP_FILE file;
		SpString* source = file.readTextFile(filename);
		cl_program program;
		gpu->commandManager->buildProgram(source->data(), sizeof(sp_char) * source->length(), buildOptions.str().c_str(), &program);
		
		sp_mem_delete(source, SpString);

		sp_uint length = 6u;
		sp_float input[] = { 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f };
		sp_uint indexes[] = { 0, 1, 2, 3, 4, 5 };
		sp_uint defaultOutput = SP_UINT_MAX;
		cl_mem outputGpu = gpu->createBuffer(&defaultOutput, sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);

		GpuCommand* command = gpu->commandManager->createCommand()
			->setInputParameter(input, length * sizeof(sp_float))
			->setInputParameter(indexes, length * sizeof(sp_uint))
			->setInputParameter(&length, sizeof(sp_uint))
			->setInputParameter(outputGpu, sizeof(sp_uint))
			->buildFromProgram(program, "findFirstNegative");

		sp_size globalWorkSize[3] = { length - 1u, 0u, 0u };
		sp_size localWorkSize[3] = { 1u, 0u, 0u };

		cl_event evt;
		command->execute(1u, globalWorkSize, localWorkSize, ZERO_UINT, ZERO_UINT, NULL, &evt);
		gpu->releaseEvent(evt);

		sp_uint result;
		gpu->commandManager->readBuffer(outputGpu, sizeof(sp_uint), &result);
		Assert::AreEqual(defaultOutput, result, L"Wrong value.", LINE_INFO());
	}

}

#endif // GPU_FIRST_NEGATIVE_TEST_HEADER

#endif // OPENCL