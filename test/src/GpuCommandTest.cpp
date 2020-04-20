#ifdef OPENCL_ENABLED

#include "SpectrumPhysicsTest.h"
#include "GpuContext.h"
#include "GpuCommand.h"

#include <IFile.h>
#include <IFileManager.h>
#include <Factory.h>

#define CLASS_NAME GpuCommandTest

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(GpuCommand_execute_Test);

	};

	SP_TEST_METHOD(CLASS_NAME, GpuCommand_execute_Test)
	{
		const sp_int LIST_SIZE = 1024;
		sp_float* param1 = ALLOC_ARRAY(sp_float, LIST_SIZE);
		sp_float* param2 = ALLOC_ARRAY(sp_float, LIST_SIZE);
		
		for (sp_uint i = 0; i < LIST_SIZE; i++) {
			param1[i] = (sp_float) i;
			param2[i] = (sp_float) (LIST_SIZE - i);
		}

		sp_size globalWorkSize = LIST_SIZE;
		sp_size localWorkSize = 64;
		
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;

		IFileManager* fileManager = Factory::getFileManagerInstance();
		std::string source = fileManager->readTextFile("sumVector.cl");
		sp_size sumProgram = gpu->commandManager->cacheProgram(source.c_str(), SIZEOF_CHAR * source.length(), NULL);

		GpuCommand* command = gpu->commandManager->createCommand();
		sp_float* result = ALLOC_NEW_ARRAY(sp_float, 1);

		command
			->setInputParameter(param1, SIZEOF_FLOAT * LIST_SIZE)
			->setInputParameter(param2, SIZEOF_FLOAT * LIST_SIZE)
			->setOutputParameter(SIZEOF_FLOAT * LIST_SIZE)
			->buildFromProgram(gpu->commandManager->cachedPrograms[sumProgram], "sum")
			->execute(1, &globalWorkSize, &localWorkSize)
			->fetch(result);

		Assert::AreEqual(1024.0f, result[0], L"Wrong value.", LINE_INFO());
		
		delete fileManager;
		command->~GpuCommand();
		ALLOC_RELEASE(result);
		ALLOC_RELEASE(command);
		ALLOC_RELEASE(param2);
		ALLOC_RELEASE(param1);
	}

}

#undef CLASS_NAME

#endif // OPENCL_ENABLED