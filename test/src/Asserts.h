#include "TestHeader.h"
#include <OpenML.h>

class Asserts 
{
public:

#ifdef WINDOWS
	template<typename T>
	static void isCloseEnough(T value, T compare, T epsilon, std::string message, const __LineInfo* lineInfo = nullptr)
	{
		const wchar_t* messageAswchar = std::wstring(message.begin(), message.end()).c_str();

		bool result = OpenML::isCloseEnough(value, compare, epsilon);

		Assert::IsTrue(result, messageAswchar, lineInfo);
	}
#endif

	template<typename T>
	static void isCloseEnough(T value1, T value2, T epslon, const wchar_t* message, const char* filename, const char* functionName, const int line)
	{

	}

};
