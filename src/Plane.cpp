#include "Plane.h"

namespace NAMESPACE_PHYSICS
{

	Plane::Plane(const Vec3& point1, const Vec3& point2, const Vec3& point3)
	{
		point = point1;
		normal(point1, point2, point3, &normalVector);
		distanceFromOrigin = normalVector.dot(point);
	}

	Plane::Plane(const Triangle3D& triangle)
	{
		point = triangle.point1;
		triangle.normalFace(&normalVector);
		distanceFromOrigin = normalVector.dot(point);
	}

	Plane::Plane(sp_float a, sp_float b, sp_float c, sp_float d)
	{
		point = Vec3(ZERO_FLOAT, ZERO_FLOAT, -d / c);

		normalize(Vec3(a, b, c), normalVector);
		distanceFromOrigin = normalVector.dot(point);
	}

	sp_bool Plane::intersection(const Line3D& line, Vec3* contactPoint, const sp_float _epsilon) const
	{
#ifdef AVX_ENABLED
		const __m128 line_point1_simd = sp_vec3_convert_simd(line.point1);
		const __m128 line_point2_simd = sp_vec3_convert_simd(line.point2);
		const __m128 lineAsVector_simd = sp_vec3_sub_simd(sp_vec3_convert_simd(line.point2), line_point1_simd);
		const __m128 normal_simd = sp_vec3_convert_simd(normalVector);
		__m128 contact_simd;
		sp_bool hasIntersection;

		sp_plane3D_intersection_line(normal_simd, sp_vec4_create_simd1f(distanceFromOrigin),
			line_point1_simd, line_point2_simd,
			contact_simd, hasIntersection);

		std::memcpy(contactPoint, contact_simd.m128_f32, SIZEOF_FLOAT * 3u);
		return hasIntersection;
#else
		const Vec3 lineAsVector = line.point2 - line.point1;
		const sp_float angle = normalVector.dot(lineAsVector);

		if (isCloseEnough(angle, ZERO_FLOAT))
			return false;

		const sp_float t = (distanceFromOrigin - normalVector.dot(line.point1)) / angle;

		if (t >= -_epsilon && t <= ONE_FLOAT + _epsilon)
		{
			contactPoint[0] = line.point1 + lineAsVector * t;
			return true;
		}
#endif
		return false;
	}

	void Plane::closestPoint(const Line3D& line, Vec3* closest) const
	{
		const Vec3 lineAsVector = line.point2 - line.point1;

		const sp_float t = -(normalVector.dot(line.point1) + distanceFromOrigin) 
			/ normalVector.dot(lineAsVector);

		closest->x = line.point1[0] + lineAsVector[0] * t;
		closest->y = line.point1[1] + lineAsVector[1] * t;
		closest->z = line.point1[2] + lineAsVector[2] * t;
	}

	sp_bool Plane::intersection(const Plane& plane, Line3D* line) const
	{
		// Compute direction of intersection line  
		Vec3 lineDirection;
		cross(plane.normalVector, normalVector, &lineDirection);

		// If d is (near) zero, the planes are parallel (and separated)  
		// or coincident, so they’re not considered intersecting  
		const sp_float denom = lineDirection.dot(lineDirection);

		if (denom < DefaultErrorMargin)
			return false;

		// Compute point on intersection line  
		cross(plane.normalVector * distanceFromOrigin - distanceFromOrigin * plane.distanceFromOrigin, lineDirection, &line->point1);
		line->point1 /= denom;
		line->point2 = line->point1 + lineDirection;

		return true;
	}

	sp_bool Plane::intersection(const Plane& plane, Ray* ray) const
	{
		// Compute direction of intersection line  
		cross(normalVector, plane.normalVector, &ray->direction);

		// If d is (near) zero, the planes are parallel (and separated)  
		// or coincident, so they’re not considered intersecting  
		const sp_float denom = ray->direction.dot(ray->direction);

		if (denom < DefaultErrorMargin)
			return false;

		// Compute point on intersection line  
		cross(plane.normalVector * distanceFromOrigin - distanceFromOrigin * plane.distanceFromOrigin, ray->direction, &ray->point);
		ray->point /= denom;
		
		return true;
	}

	sp_float Plane::distance(const Plane& plane) const
	{
#ifdef AVX_ENABLED
		const __m128 normal_simd = sp_vec3_convert_simd(normalVector);
		const __m128 plane_point_simd = sp_vec3_convert_simd(plane.point);
		const __m128 ray_direction_simd = sp_vec3_mult_simd(normal_simd, sp_vec4_create_simd1f(-ONE_FLOAT));
		__m128 projectedPoint;

		sp_plane3D_intersection_ray_simd(normal_simd, plane_point_simd, ray_direction_simd, projectedPoint, sp_bool hasIntersection);
		sp_vec3_distance_simd(plane_point_simd, projectedPoint, const __m128 output);

		return output.m128_f32[0];
#else
		Vec3 projectedPoint;
		project(plane.point, &projectedPoint);

		return NAMESPACE_PHYSICS::distance(plane.point, projectedPoint);

		/* It does not work when (distanceFromOrigin - plane.distanceFromOrigin) = 0
		return std::fabsf(distanceFromOrigin - plane.distanceFromOrigin)
		/ sp_sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
		*/
#endif
	}

	Vec3 Plane::closestPointOnThePlane(const Vec3& target) const
	{
		// t = ((n . p) - d) / (n.n)
		sp_float t = (normalVector.dot(target) - getDcomponent()) / normalVector.dot(normalVector); 

		return target - (normalVector * t); //result = point - tn
	}

	sp_float Plane::angle(const Plane& plane) const
	{
		sp_float angle = normalVector.dot(plane.normalVector);
		sp_float length = NAMESPACE_PHYSICS::length(normalVector) * NAMESPACE_PHYSICS::length(plane.normalVector);

		return angle / length;
	}

	void Plane::project(const Vec3& target, Vec3* output) const
	{
#ifdef AVX_ENABLED
		const __m128 normal_simd = sp_vec3_convert_simd(normalVector);
		const __m128 ray_point_simd = sp_vec3_convert_simd(target);
		const __m128 ray_direction_simd = sp_vec3_mult_simd(normal_simd, sp_vec4_create_simd1f(-ONE_FLOAT));
		__m128 contact;

		sp_plane3D_intersection_ray_simd(normal_simd, ray_point_simd, ray_direction_simd, contact, sp_bool hasIntersection);

		std::memcpy(output, contact.m128_f32, SIZEOF_FLOAT * 3u);
#else
		Ray ray(target, -normalVector);
		intersection(ray, output);
#endif
	}

}