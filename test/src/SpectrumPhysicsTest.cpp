#include "SpectrumPhysicsTest.h"
#include "SpOpenGL.h"
#include "SpGpuRenderingFactoryOpenGL.h"
#include "GpuContext.h"
#include "SpPhysicSimulator.h"

#define CLASS_NAME SpectrumPhysicsTest

namespace NAMESPACE_PHYSICS_TEST
{
	static std::mutex locker;

	void TestPhysic::lock()
	{
		locker.lock();
	}
	void TestPhysic::unlock()
	{
		locker.unlock();
	}

#ifdef MSTEST_ENABLED
	
	TEST_MODULE_INITIALIZE(ModuleInitialize)
	{
		NAMESPACE_FOUNDATION::SpStackMemoryAllocator::main()->init(ONE_MEGABYTE * 512);

		SpLogMsTestProvider* logProvider = sp_mem_new(SpLogMsTestProvider);
		SpLogger::init();
		SpLogger::instance()->addProvider(logProvider);

		NAMESPACE_RENDERING::SpOpenGL::initOffScreenRendering();
		NAMESPACE_RENDERING::SpGpuRenderingFactoryOpenGL::init();
		GpuContext::init();

		SpWorldManagerInstance->init();
	}

	TEST_MODULE_CLEANUP(ModuleCleanup)
	{
		SpWorldManagerInstance->dispose();
		NAMESPACE_RENDERING::SpOpenGL::dispose();
		NAMESPACE_FOUNDATION::SpStackMemoryAllocator::main()->dispose();
		SpLogger::dispose();
	}

#endif // MSTEST_ENABLED

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		SP_TEST_METHOD_DEF(clamp_Test);
		SP_TEST_METHOD_DEF(sign_Test);
	};

	SP_TEST_METHOD(CLASS_NAME, clamp_Test)
	{
		sp_float result = clamp(1.0f, 0.0f, 2.0f);
		sp_float expected = 1.0f;
		Assert::AreEqual(expected, result, L"Wrong value.", LINE_INFO());

		result = clamp(-1.0f, 0.0f, 2.0f);
		expected = 0.0f;
		Assert::AreEqual(expected, result, L"Wrong value.", LINE_INFO());

		result = clamp(3.0f, 0.0f, 2.0f);
		expected = 2.0f;
		Assert::AreEqual(expected, result, L"Wrong value.", LINE_INFO());

		result = clamp(2.0f, 0.0f, 2.0f);
		expected = 2.0f;
		Assert::AreEqual(expected, result, L"Wrong value.", LINE_INFO());

		result = clamp(0.0f, 0.0f, 2.0f);
		expected = 0.0f;
		Assert::AreEqual(expected, result, L"Wrong value.", LINE_INFO());

		result = clamp(0.0001f, 0.0f, 2.0f);
		expected = 0.0001f;
		Assert::AreEqual(expected, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, sign_Test)
	{
		sp_int result = sign(0.1f);
		Assert::AreEqual(1, result, L"Wrong value.", LINE_INFO());

		result = sign(-0.1f);
		Assert::AreEqual(-1, result, L"Wrong value.", LINE_INFO());

		result = sign(0);
		Assert::AreEqual(0, result, L"Wrong value.", LINE_INFO());
	}
	
}

#undef CLASS_NAME