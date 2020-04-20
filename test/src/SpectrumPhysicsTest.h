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

#endif // MSTEST_ENABLED

#endif // SPECTRUM_PHYSICS_TEST_HEADER
