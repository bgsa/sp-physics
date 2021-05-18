#ifndef PLANE_HEADER
#define PLANE_HEADER

#include "SpectrumPhysics.h"
#include "Ray.h"
#include "Line3D.h"
#include "Orientation.h"
#include "Triangle3D.h"

namespace NAMESPACE_PHYSICS
{
	class Plane
	{

	public:
		Vec3 point;
		Vec3 normalVector;
		sp_float distanceFromOrigin;
		
		API_INTERFACE Plane()
		{
			point = Vec3Zeros;
			normalVector = Vec3Up;
			distanceFromOrigin = normalVector.dot(point);
		}

		/// <summary>
		/// Build a plane from a point and normal vector (NORMALIZED!)
		/// </summary>
		API_INTERFACE Plane(const Vec3& point, const Vec3& normal)
		{
			//sp_assert(NAMESPACE_FOUNDATION::isCloseEnough(length(normal), ONE_FLOAT, 0.009f), "InvalidArgumentException");

			this->point = point;
			this->normalVector = normal;
			this->distanceFromOrigin = normalVector.dot(point);
		}

		/// <summary>
		/// Build a plane from 3 points (right-hand)
		/// </summary>
		API_INTERFACE Plane(const Vec3& point1, const Vec3& point2, const Vec3& point3);

		/// <summary>
		/// Build a plane from equation
		/// </summary>
		API_INTERFACE Plane(sp_float a, sp_float b, sp_float c, sp_float d);

		/// <summary>
		/// Build a plane from triangle
		/// </summary>
		API_INTERFACE Plane(const Triangle3D& triangle);

		/// <summary>
		/// Get "D" components from plane equation: ax + by + cz + D = 0
		/// </summary>
		API_INTERFACE inline sp_float getDcomponent() const
		{
			return -normalVector.dot(point);
		}

		/// <summary>
		/// Get the tangent vector of the plane
		/// </summary>
		/// <param name="tangentVector">Output parameter</param>
		/// <returns>tangentVector</returns>
		API_INTERFACE inline void tangent(Vec3& tangentVector) const
		{
			cross(point, normalVector, tangentVector);
			normalize(tangentVector);

			sp_assert(NAMESPACE_FOUNDATION::isCloseEnough(tangentVector.dot(normalVector - point), ZERO_FLOAT), "ApplicationException");
		}
		
		/// <summary>
		/// Get the equation of the plane
		/// </summary>
		API_INTERFACE inline void equation(Vec4* vector) const
		{
			vector->x = normalVector.x;
			vector->y = normalVector.y;
			vector->z = normalVector.z;
			vector->w = distanceFromOrigin;
		}

		/// <summary>
		/// Test if the ray cross the plane
		/// </summary>
		API_INTERFACE inline sp_bool intersection(const Ray& ray, Vec3* contactPoint) const
		{
#ifdef AVX_ENABLED
			const __m128 normal_simd = sp_vec3_convert_simd(normalVector);
			const __m128 ray_point_simd = sp_vec3_convert_simd(ray.point);
			const __m128 ray_direction_simd = sp_vec3_convert_simd(ray.direction);
			__m128 output;

			sp_plane3D_intersection_ray_simd(normal_simd, ray_point_simd, ray_direction_simd, output, sp_bool hasIntersection)
			
			std::memcpy(contactPoint, output.m128_f32, SIZEOF_FLOAT * 3u);

			/*
			const __m128 _angle = sp_vec3_dot_simd(normal_simd, ray_direction_simd);

			if (isCloseEnough(_angle.m128_f32[0], ZERO_FLOAT))
				return false;

			const __m128 ray_point_simd = sp_vec3_convert_simd(ray.point);

			__m128 temp1 = sp_vec3_dot_simd(normal_simd, ray_point_simd);
			temp1 = sp_vec3_sub_simd(sp_vec4_create_simd1f(distanceFromOrigin), temp1);

			temp1 = sp_vec3_div_simd(temp1, _angle);

			temp1 = sp_vec3_mult_simd(ray_direction_simd, temp1);
			temp1 = sp_vec3_add_simd(ray_point_simd, temp1);

			std::memcpy(contactPoint, temp1.m128_f32, SIZEOF_FLOAT * 3u);
			*/

#else
			const sp_float angle = normalVector.dot(ray.direction);

			if (NAMESPACE_FOUNDATION::isCloseEnough(angle, ZERO_FLOAT))
				return false;

			const sp_float numerator = distanceFromOrigin - normalVector.dot(ray.point);
			const sp_float t = numerator / angle;

			contactPoint[0] = ray.point + ray.direction * t;
#endif
			return true;
		}

		/// <summary>
		/// Test if the line cross the plane
		/// </summary>
		API_INTERFACE sp_bool intersection(const Line3D& line, Vec3* contactPoint, const sp_float _epsilon = ERROR_MARGIN_PHYSIC) const;

		/// <summary>
		/// Get the line intersection between the planes
		/// </summary>
		API_INTERFACE sp_bool intersection(const Plane& plane, Line3D* line) const;

		/// <summary>
		/// Find the plane intersection
		/// </summary>
		/// <param name="plane">Second plane</param>
		/// <param name="ray">Ray as output</param>
		/// <returns>void</returns>
		API_INTERFACE sp_bool intersection(const Plane& plane, Ray* ray) const;

		/// <summary>
		/// Get the angle of two planes
		/// </summary>
		API_INTERFACE sp_float angle(const Plane& plane) const;

		/// <summary>
		/// Get the distance from the plane to the point
		/// </summary>
		API_INTERFACE inline sp_float distance(const Vec3& target) const
		{
#ifdef AVX_ENABLED
			const __m128 normal_simd = sp_vec3_convert_simd(normalVector);
			const __m128 target_simd = sp_vec3_convert_simd(target);

			sp_plane3D_distance_simd(normal_simd, target_simd, const __m128 output);

			return output.m128_f32[0];
#else
			return (normalVector.dot(target) - distanceFromOrigin) / normalVector.dot(normalVector);
#endif
		}

		/// <summary>
		/// Get the distance from the plane to the other one
		/// </summary>
		API_INTERFACE sp_float distance(const Plane& plane) const;

		/// <summary>
		/// Indicate whether the point is on the left, right fo the plane OR the point relies on the plane
		/// </summary>
		/// <param name="point">Arbitrary point</param>
		/// <returns>0 if the point relies on the plane; 
		/// Greater than zero if the point is left of the plane; 
		/// Lesser than zero if the point is right of the plane 
		/// </returns>
		API_INTERFACE sp_float orientation(const Vec3& point) const
		{
			return distance(point);
		}

		/// <summary>
		/// Check if the planes are parallel each other
		/// </summary>
		API_INTERFACE inline sp_bool isParallel(const Plane& plane, const sp_float _epsilon = DefaultErrorMargin) const
		{
			Vec3 temp;
			cross(normalVector, plane.normalVector, temp);
			return isCloseEnough(temp, Vec3Zeros, _epsilon);
		}

		/// <summary>
		/// Check if the planes are perpendicular each other
		/// </summary>
		API_INTERFACE inline sp_bool isPerpendicular(const Plane& plane, const sp_float _epsilon = DefaultErrorMargin) const
		{
			return NAMESPACE_FOUNDATION::isCloseEnough(normalVector.dot(plane.normalVector), ZERO_FLOAT, _epsilon);
		}

		/// <summary>
		/// Given an arbitrary point, find the closest point on the plane
		/// </summary>
		API_INTERFACE Vec3 closestPointOnThePlane(const Vec3 &target) const;

		/// <summary>
		/// Find the closest point on the line
		/// </summary>
		/// <param name="line"></param>
		/// <param name="closest"></param>
		/// <returns></returns>
		API_INTERFACE void closestPoint(const Line3D& line, Vec3* closest) const;

		/// <summary>
		/// Check if the point parameter is back or front from the plane face
		/// </summary>
		/// <param name="point">Arbitrary point</param>
		/// <returns>True if the point is back-face, otherwise False</returns>
		API_INTERFACE inline sp_bool isBackFace(const Vec3& point) const
		{
			return distance(point) < ZERO_FLOAT;
		}

		/// <summary>
		/// Project the target on the plane
		/// </summary>
		/// <param name="target">Point to be projected</param>
		/// <param name="output">Projected point</param>
		/// <returns>void</returns>
		API_INTERFACE void project(const Vec3& target, Vec3* output) const;

	};

}

#endif // PLANE_HEADER