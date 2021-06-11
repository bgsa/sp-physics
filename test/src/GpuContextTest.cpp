#ifdef OPENCL_ENABLED

#include "SpectrumPhysicsTest.h"
#include <GpuContext.h>

#define CLASS_NAME GpuContextTest

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		SP_TEST_METHOD_DEF(GPUContext_init);
	};

	SP_TEST_METHOD(CLASS_NAME, GPUContext_init)
	{
		GpuContext::init();
		Assert::IsTrue(GpuContextInstance != NULL, L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME

#endif // OPENCL_ENABLED