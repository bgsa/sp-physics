#include "TestHeader.h"
#include <OpenML.h>

namespace NAMESPACE_PHYSICS_TEST
{
	class Asserts 
	{
	public:

	#ifdef WINDOWS
		template<typename T>
		static void isCloseEnough(T value, T compare, T epsilon, const wchar_t* message, const __LineInfo* lineInfo = nullptr)
		{
			bool result = isCloseEnough(value, compare, epsilon);

			Assert::IsTrue(result, message, lineInfo);
		}
	#endif

		template<typename T>
		static void isCloseEnough(T value1, T value2, T epslon, const wchar_t* message, const char* filename, const char* functionName, const int line)
		{

		}

	};
}