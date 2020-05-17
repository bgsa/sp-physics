#ifndef SPECTRUM_PHYSICS_TEST_HEADER
#define SPECTRUM_PHYSICS_TEST_HEADER

#include "SpectrumFoundationTest.h"
#include <SpectrumPhysics.h>

#ifndef NAMESPACE_PHYSICS_TEST
	#define NAMESPACE_PHYSICS_TEST  SpPhysicsTest
#endif

using namespace NAMESPACE_PHYSICS;

#ifdef MSTEST_ENABLED

	namespace Microsoft
	{
		namespace VisualStudio
		{
			namespace CppUnitTestFramework
			{
				template<>
				static std::wstring ToString<Vec2>(const Vec2& vector)
				{
					return L"Some string representing Vector2D.";
				}

				template<>
				static std::wstring ToString<Vec3>(const Vec3& vector)
				{
					return L"Some string representing Vector3D.";
				}

				template<>
				static std::wstring ToString<Vec4>(const Vec4& vector)
				{
					return L"Some string representing Vector4D.";
				}

				template<>
				static std::wstring ToString<Mat2>(const Mat2& matrix)
				{
					return L"Some string representing Matrix2D.";
				}

				template<>
				static std::wstring ToString<Mat3>(const Mat3& matrix)
				{
					return L"Some string representing Matrix3D.";
				}

				template<>
				static std::wstring ToString<Mat4>(const Mat4& matrix)
				{
					return L"Some string representing Matrix4D.";
				}

				template<>
				static std::wstring ToString<Rectangle2D>(const Rectangle2D& rectangle)
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

#endif // MSTEST_ENABLED

#endif // SPECTRUM_PHYSICS_TEST_HEADER
