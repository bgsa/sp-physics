#include "TestHeader.h"

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
