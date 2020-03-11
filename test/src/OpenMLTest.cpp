#include "TestHeader.h"
#include <OpenML.h>

#define CLASS_NAME OpenMLTest

namespace SP_PHYSICS_TEST_NAMESPACE
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
	
		SP_TEST_METHOD_DEF(OpenML_clamp_Test);

		SP_TEST_METHOD_DEF(sign_Test);
		
	};

	SP_TEST_METHOD(CLASS_NAME, OpenML_clamp_Test)
	{
		float result = clamp(1.0f, 0.0f, 2.0f);
		float expected = 1.0f;			
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
		int result = sign(0.1f);
		Assert::AreEqual(1, result, L"Wrong value.", LINE_INFO());

		result = sign(-0.1f);
		Assert::AreEqual(-1, result, L"Wrong value.", LINE_INFO());

		result = sign(0);
		Assert::AreEqual(0, result, L"Wrong value.", LINE_INFO());
	}
	
}

#undef CLASS_NAME