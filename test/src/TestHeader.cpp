#include "TestHeader.h"

namespace NAMESPACE_PHYSICS_TEST
{
	void setupModule()
	{
		MemoryAllocatorManager::init(ONE_MEGABYTE * 512);
		//	Logger::WriteMessage("TEST MODULE INITIALIZED");
	}

	void tearDownModule()
	{
		MemoryAllocatorManager::release();
		//	Logger::WriteMessage("TEST MODULE FINISHED");
	}

	#ifdef MSTEST_ENABLED

	TEST_MODULE_INITIALIZE(ModuleInitialize)
	{
		setupModule();
	}

	TEST_MODULE_CLEANUP(ModuleCleanup)
	{
		tearDownModule();
	}

	#endif // !MSTEST_ENABLED
}