#include "Ray.h"

namespace NAMESPACE_PHYSICS
{

	sp_bool Ray::intersection(const Ray& ray, Vec3* contact, const sp_float _epsilon) const
	{
		Vec3 contact1, contact2;
		sp_float sqDistance;

		closestPoint(ray, &contact1, &contact2, &sqDistance, _epsilon);
	
		if (isCloseEnough(sqDistance, ZERO_FLOAT, _epsilon))
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