#include "TestHeader.h"
#include "MemoryAllocatorManager.h"

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

#ifdef GOOGLETEST_ENABLED

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	setupModule();

	int testsResult = RUN_ALL_TESTS();

	tearDownModule();
	return testsResult;
}

#endif // !GOOGLETEST_ENABLED
