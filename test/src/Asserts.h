#include "SpectrumPhysicsTest.h"

namespace NAMESPACE_PHYSICS_TEST
{
	class Asserts 
	{
	public:

	#ifdef WINDOWS
		template<typename T>
		static void isCloseEnough(T value, T compare, T epsilon, const wchar_t* message, const __LineInfo* lineInfo = nullptr)
		{
			sp_bool result = NAMESPACE_FOUNDATION::isCloseEnough(value, compare, epsilon);

			Assert::IsTrue(result, message, lineInfo);
		}
	#endif

		static void isCloseEnough(const Vec3& value, const Vec3& compare, sp_float epsilon, const wchar_t* message = L"Error", const __LineInfo* lineInfo = nullptr)
		{
			sp_bool result = NAMESPACE_PHYSICS::isCloseEnough(value, compare, epsilon);

			Assert::IsTrue(result, message, lineInfo);
		}

	};
}