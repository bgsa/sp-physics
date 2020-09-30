#include "Triangle3D.h"

namespace NAMESPACE_PHYSICS
{

	void Triangle3D::project(const Vec3& target, Vec3* output) const
	{
		Plane3D plane(point1, point2, point3);
		Ray ray(target, -plane.normalVector);

		plane.intersection(ray, output);
	}

	void Triangle3D::convert(Line3D* lines) const
	{
		lines[0] = Line3D(point1, point2);
		lines[1] = Line3D(point2, point3);
		lines[2] = Line3D(point3, point1);
	}
	
	sp_bool Triangle3D::isInside(const Vec3& target, const sp_float _epsilon) const
	{
#ifdef AVX_ENABLED

		const __m128 point1_simd = sp_vec3_convert_simd(point1);
		const __m128 point2_simd = sp_vec3_convert_simd(point2);
		const __m128 point3_simd = sp_vec3_convert_simd(point3);
		const __m128 target_simd = sp_vec3_convert_simd(target);
		
		sp_triangle3D_isInside_simd(point1_simd, point2_simd, point3_simd, target_simd, const sp_bool isIn);
		
		return isIn;
#else
		const sp_float total = area();
		const sp_float area1 = NAMESPACE_PHYSICS::area(point1, point2, target);
		const sp_float area2 = NAMESPACE_PHYSICS::area(point2, point3, target);
		const sp_float area3 = NAMESPACE_PHYSICS::area(point3, point1, target);

		return isCloseEnough(total, area1 + area2 + area3, total * _epsilon);
#endif
	}

}
