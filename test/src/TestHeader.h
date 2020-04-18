#ifndef TEST_HEADER_HEADER
#define TEST_HEADER_HEADER

#include <SpectrumPhysics.h>
#include <chrono>

#ifndef NAMESPACE_PHYSICS_TEST
	#define NAMESPACE_PHYSICS_TEST  SpPhysicsTest
#endif

using namespace NAMESPACE_PHYSICS;

namespace NAMESPACE_PHYSICS_TEST
{
	API_INTERFACE void setupModule();
	API_INTERFACE void tearDownModule();
}

#ifdef GOOGLETEST_ENABLED
	#include "gtest/gtest.h"

	#define SP_TEST_CLASS(className)                  class className : public ::testing::Test
	#define SP_TEST_METHOD(className, methodMame)     TEST_F(className, methodMame)
	#define SP_TEST_METHOD_DEF(methodMame)

	#define LINE_INFO() __FILE__ , __FUNCTION__, __LINE__

	namespace NAMESPACE_PHYSICS_TEST
	{
		class Assert
		{
			public:

				static void AreEqual(size_t value1, size_t value2, const wchar_t* message, const char* filename, const char* functionName, const int line)
				{

				}
				static void AreEqual(int value1, int value2, const wchar_t* message, const char* filename, const char* functionName, const int line)
				{

				}
				static void AreEqual(float value1, float value2, const wchar_t* message, const char* filename, const char* functionName, const int line)
				{

				}
				static void AreEqual(double value1, double value2, const wchar_t* message, const char* filename, const char* functionName, const int line)
				{

				}
				static void AreEqual(std::string value1, std::string value2, const wchar_t* message, const char* filename, const char* functionName, const int line)
				{

				}
				static void AreEqual(const Vec2f& value1, const Vec2f& value2, const wchar_t* message, const char* filename, const char* functionName, const int line)
				{

				}
				static void AreEqual(const Vec3f& value1, const Vec3f& value2, const wchar_t* message, const char* filename, const char* functionName, const int line)
				{

				}
				static void AreEqual(const CollisionStatus& value1, const CollisionStatus& value2, const wchar_t* message, const char* filename, const char* functionName, const int line)
				{

				}

				static void IsTrue(bool value, const wchar_t* message, const char* filename, const char* functionName, const int line)
				{

				}
				static void IsFalse(bool value, const wchar_t* message, const char* filename, const char* functionName, const int line)
				{

				}

				static void IsNull(void* value, const wchar_t* message, const char* filename, const char* functionName, const int line)
				{

				}
				static void IsNotNull(void* value, const wchar_t* message, const char* filename, const char* functionName, const int line)
				{

				}
				static void Fail(const wchar_t* message, const char* filename, const char* functionName, const int line)
				{

				}
		};
	}

#endif

#ifdef MSTEST_ENABLED

	#include "CppUnitTest.h"

	#define SP_TEST_CLASS(className)              TEST_CLASS(className)
	#define SP_TEST_METHOD(className, methodMame) void className::methodMame()
	#define SP_TEST_METHOD_DEF(methodName)            TEST_METHOD(methodName)
	
	using namespace Microsoft::VisualStudio::CppUnitTestFramework;

	namespace Microsoft
	{
		namespace VisualStudio
		{
			namespace CppUnitTestFramework
			{
				template<>
				static std::wstring ToString<Vec2f>(const Vec2f& vector)
				{
					return L"Some string representing Vector2D.";
				}

				template<>
				static std::wstring ToString<Vec3f>(const Vec3f& vector)
				{
					return L"Some string representing Vector3D.";
				}

				template<>
				static std::wstring ToString<Vec4f>(const Vec4f& vector)
				{
					return L"Some string representing Vector4D.";
				}

				template<>
				static std::wstring ToString<Mat2f>(const Mat2f& matrix)
				{
					return L"Some string representing Matrix2D.";
				}

				template<>
				static std::wstring ToString<Mat3f>(const Mat3f& matrix)
				{
					return L"Some string representing Matrix3D.";
				}

				template<>
				static std::wstring ToString<Mat4f>(const Mat4f& matrix)
				{
					return L"Some string representing Matrix4D.";
				}

				template<>
				static std::wstring ToString<Rectangle2Df>(const Rectangle2Df& rectangle)
				{
					return L"Some string representing Rectangle2D.";
				}

				template<>
				static std::wstring ToString<CollisionStatus>(const CollisionStatus& status)
				{
					return L"Some string representing CollisionStatus.";
				}
			}
		}
	}

#endif // !MSTEST_ENABLED

#endif // !TEST_HEADER_HEADER
