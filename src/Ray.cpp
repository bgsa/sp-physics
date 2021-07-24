#include "Ray.h"

namespace NAMESPACE_PHYSICS
{

	sp_bool Ray::intersection(const Plane& p, Vec3& contact) const
	{
#ifdef AVX_ENABLED
		const __m128 normal_simd = sp_vec3_convert_simd(p.normalVector);
		const __m128 ray_point_simd = sp_vec3_convert_simd(point);
		const __m128 ray_direction_simd = sp_vec3_convert_simd(direction);
		__m128 output;

		sp_plane3D_intersection_ray_simd(normal_simd, ray_point_simd, ray_direction_simd, output, sp_bool hasIntersection)

			std::memcpy(contact, output.m128_f32, sizeof(sp_float) * 3u);

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

		std::memcpy(contactPoint, temp1.m128_f32, sizeof(sp_float) * 3u);
		*/

#else
		const sp_float angle = p.normalVector.dot(direction);

		if (NAMESPACE_FOUNDATION::isCloseEnough(angle, ZERO_FLOAT))
			return false;

		const sp_float numerator = p.distanceFromOrigin - p.normalVector.dot(point);
		const sp_float t = numerator / angle;

		contact = point + direction * t;
#endif
		return true;
	}

	sp_bool Ray::intersection(const AABB& aabb, Vec3 contact[2], sp_float& distanceToFirstContact, sp_float& distanceToSecondContact, const sp_float _epsilon) const
	{
		distanceToFirstContact = 0.0f; // set to -FLT_MAX to get first hit on line
		distanceToSecondContact = SP_FLOAT_MAX; // set to max distance ray can travel (for segment)

		// For all three slabs
		for (sp_int i = 0; i < 3; i++)
		{  
			if (sp_abs(direction[i]) < _epsilon)
			{  
				// Ray is parallel to slab. No hit if origin not within slab
				if (point[i] < aabb.minPoint[i] || point[i] > aabb.maxPoint[i])
					return false;
			} 
			else 
			{
				// Compute intersection t value of ray with near and far plane of slab
				const sp_float ood = NAMESPACE_FOUNDATION::div(1.0f, direction[i]);
				sp_float t1 = (aabb.minPoint[i] - point[i]) * ood;
				sp_float t2 = (aabb.maxPoint[i] - point[i]) * ood;
				
				// Make t1 be intersection with near plane, t2 with far plane
				if (t1 > t2) 
					std::swap(t1, t2);  
				
				// Compute the intersection of slab intersection intervals
				distanceToFirstContact = sp_max(distanceToFirstContact, t1);
				distanceToSecondContact = sp_min(distanceToSecondContact, t2);
				
				// Exit with no collision as soon as slab intersection becomes empty
				if (distanceToFirstContact > distanceToSecondContact)
					return false;
			}  
		}  
		
		// Ray intersects all 3 slabs. Return point (q) and intersection t value (tmin)
		contact[0] = point + direction * distanceToFirstContact;
		contact[1] = point + direction * distanceToSecondContact;

		return true; 
	}

	sp_bool Ray::intersection(const Ray& ray, Vec3* contact, const sp_float _epsilon) const
	{
		Vec3 contact1, contact2;
		sp_float sqDistance;

		closestPoint(ray, &contact1, &contact2, &sqDistance, _epsilon);
	
		if (isCloseEnough(contact1, contact2, _epsilon))
		{
			contact[0] = (contact1 + contact2) * HALF_FLOAT;
			return true;
		}

		return false;
	}

	void Ray::closestPoint(const Ray& ray, Vec3* closestPointRay1, Vec3* closestPointRay2, sp_float* sqDistance, const sp_float _epsilon) const
	{
		const Vec3 d1 = direction; // Direction vector of segment S1  
		const Vec3 d2 = ray.direction; // Direction vector of segment S2  
		const Vec3 r = point - ray.point;
		const sp_float a = d1.dot(d1); // Squared length of segment S1, always nonnegative  
		const sp_float e = d2.dot(d2); // Squared length of segment S2, always nonnegative  
		const sp_float f = d2.dot(r);  // Check if either or both segments degenerate into points  

		sp_float s, t;

		if (a <= DefaultErrorMargin && e <= DefaultErrorMargin)
		{
			// Both segments degenerate into points
			closestPointRay1[0] = point;
			closestPointRay2[0] = ray.point;
			sqDistance[0] = (point - ray.point).dot(point - ray.point);
			return;
		}

		if (a <= DefaultErrorMargin)
		{
			// First segment degenerates into a point  
			s = ZERO_FLOAT;
			t = f / e;

			// s = 0 => t = (b*s + f) / e = f / e  
			t = clamp(t, ZERO_FLOAT, ONE_FLOAT);
		}
		else
		{
			sp_float c = d1.dot(r);

			if (e <= DefaultErrorMargin)
			{
				// Second segment degenerates into a point  
				t = ZERO_FLOAT;
				s = clamp(-c / a, ZERO_FLOAT, ONE_FLOAT); // t = 0 => s = (b*t - c) / a = -c / a  
			}
			else
			{  // The general nondegenerate case starts here  
				sp_float b = d1.dot(d2);
				sp_float denom = a * e - b * b; // Always nonnegative  

				// If segments not parallel, compute closest point on L1 to L2
				if (denom != ZERO_FLOAT)
					s = (b * f - c * e) / denom;
				else
					s = ZERO_FLOAT;

				// Compute point on L2 closest to S1(s) using  
				// t = Dot((P1 + D1*s) - P2,D2) / Dot(D2,D2) = (b*s + f) / e  
				t = (b * s + f) / e;
			}
		}

		const Vec3 c1 = point + d1 * s;
		const Vec3 c2 = ray.point + d2 * t;

		closestPointRay1[0] = c1;
		closestPointRay2[0] = c2;
		sqDistance[0] = (c1 - c2).dot(c1 - c2);
	}


}