#ifdef OPENCL_ENABLED

#include "TestHeader.h"
#include <GpuContext.h>

#define CLASS_NAME GpuContextTest

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(GPUContext_instance_Test);

		SP_TEST_METHOD_DEF(GPUContext_getPlatforms_Test);

	};

	SP_TEST_METHOD(CLASS_NAME, GPUContext_instance_Test)
	{
		std::vector<cl_platform_id> platforms = GpuContext::getPlatforms();

		GpuContext* gpu = GpuContext::init(platforms[0]);

		Assert::IsTrue(gpu != NULL, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, GPUContext_getPlatforms_Test)
	{
		std::vector<cl_platform_id> platforms = GpuContext::getPlatforms();

		Assert::IsTrue(platforms.size() > 0, L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME

#endif // OPENCL_ENABLED