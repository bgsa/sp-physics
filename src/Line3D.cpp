#include "Line3D.h"
#include "Vec3.h"

namespace NAMESPACE_PHYSICS
{
	Line3D::Line3D() {	};

	Line3D::Line3D(const Vec3& point1, const Vec3& point2)
	{
		sp_assert(point1 != point2, "InvalidArgumentException");

		this->point1 = point1;
		this->point2 = point2;
	}

	Line3D::Line3D(Vec3* points)
	{
		sp_assert(points[0] != points[1], "InvalidArgumentException");

		this->point1 = points[0];
		this->point2 = points[1];
	}

	Line3D::Line3D(sp_float* point1, sp_float* point2)
	{
		sp_assert(point1 != point2, "InvalidArgumentException");

		this->point1 = Vec3(point1[0], point1[1], point1[2]);
		this->point2 = Vec3(point2[0], point2[1], point2[2]);
	}

	sp_float Line3D::lengthOfSegment() const
	{
		return NAMESPACE_PHYSICS::distance(point1, point2);
	}

	sp_bool Line3D::isOnLine(const Vec3& point, const sp_float _epsilon) const
	{
		Vec3 dif;
		diff(point2, point1, dif);

		Vec3 temp;
		NAMESPACE_PHYSICS::cross(dif, point, &temp);

		return isCloseEnough(temp, Vec3Zeros, _epsilon);
	}

	sp_bool Line3D::isOnSegment(const Vec3& point) const
	{
		Vec3 lineDirection = point2 - point1;

		Vec3 crossedVector;
		NAMESPACE_PHYSICS::cross(lineDirection, point, &crossedVector);

		if (crossedVector != ZERO_FLOAT)
			return false;
		
		sp_float ab = lineDirection.dot(lineDirection);
		sp_float ac = lineDirection.dot(point - point1);
		
		if (ac < ZERO_FLOAT || ac > ab)
			return false;

		return (ZERO_FLOAT <= ac && ac <= ab);
	}

	sp_bool Line3D::intersection(const Line3D& line2, Vec3* point, const sp_float _epsilon) const
	{
		Vec3 p2;
		sp_float sqDistance;
		closestPoint(line2, point, &p2, &sqDistance);

		if (isCloseEnough(*point, p2, _epsilon))
			return true;

		return false;
	}

	sp_bool Line3D::intersection(const Triangle3D& triangle, Vec3* point, const sp_float _epsilon) const
	{
#ifdef AVX_ENABLED
		const __m128 triangle_point1 = sp_vec3_convert_simd(triangle.point1);
		const __m128 triangle_point2 = sp_vec3_convert_simd(triangle.point2);
		const __m128 triangle_point3 = sp_vec3_convert_simd(triangle.point3);
		const __m128 normal_simd = sp_vec3_normal_simd(triangle_point1, triangle_point2, triangle_point3);
		const __m128 d_simd = sp_vec3_dot_simd(normal_simd, triangle_point1);

		// if the normal plane is perpendicular. there is no way to cross the plane
		// but it can be a 2D intersection ...
		if (sp_line3D_isPerpendicular2_simd(point1, point2, normal_simd))
		{
			if (intersection(Line3D(triangle.point1, triangle.point2), point, _epsilon))
				return true;
			if (intersection(Line3D(triangle.point2, triangle.point3), point, _epsilon))
				return true;
			if (intersection(Line3D(triangle.point3, triangle.point1), point, _epsilon))
				return true;

			return false;
		}

		sp_bool hasIntersection;
		__m128 contact_simd;
		sp_plane3D_intersection_line(normal_simd, d_simd, sp_vec3_convert_simd(point1), sp_vec3_convert_simd(point2), contact_simd, hasIntersection);

		if (!hasIntersection)
			return false;

		sp_triangle3D_isInside_simd(triangle_point1, triangle_point2, triangle_point3, contact_simd, const sp_bool isIn);

		std::memcpy(point, contact_simd.m128_f32, SIZEOF_FLOAT * 3u);

		return isIn;
#else
		Vec3 triangleNormal;
		triangle.normalFace(&triangleNormal);
		
		// if the normal plane is perpendicular. there is no way to cross the plane
		// but it can be a 2D intersection ...
		if (isPerpendicular(triangleNormal))
		{
			if (intersection(Line3D(triangle.point1, triangle.point2), point, _epsilon))
				return true;
			if (intersection(Line3D(triangle.point2, triangle.point3), point, _epsilon))
				return true;
			if (intersection(Line3D(triangle.point3, triangle.point1), point, _epsilon))
				return true;

			return false;
		}

		const Plane plane(triangle.point1, triangleNormal);

		if (!plane.intersection(*this, point, _epsilon))
			return false;

		if (!triangle.isInside(*point, _epsilon))
			return false;

		return true;
#endif
	}

	void Line3D::closestPointOnTheLine(const Vec3& target, Vec3* output) const
	{
		Vec3 lineDirection;
		diff(point2, point1, lineDirection);
		
		// Project target onto lineDirection, computing parameterized position closestPoint(t) = point1 + t*(point2 � point1) 
		sp_float t = (target - point1).dot(lineDirection) / lineDirection.dot(lineDirection);

		// If outside segment, clamp t (and therefore d) to the closest endpoint 
		t = clamp(t, ZERO_FLOAT, ONE_FLOAT); // clamp t from 0.0 to 1.0
		
		output[0] = point1 + lineDirection * t;
	}

	sp_bool Line3D::hasIntersectionOnRay(const Sphere& sphere) const
	{	
		Vec3 m = point1 - sphere.center; 
		sp_float c = m.dot(m) - sphere.ray * sphere.ray;
		
		// If there is definitely at least one real root, there must be an intersection 
		if (c <= 0.0f) 
			return true; 
		
		Vec3 d = point2 - point1;

		sp_float b = m.dot(d);
		
		// Early exit if ray origin outside sphere and ray pointing away from sphere 
		if (b > ZERO_FLOAT)
			return false; 
		
		sp_float disc = b*b - c;
		
		// A negative discriminant corresponds to ray missing sphere 
		if (disc < 0.0f) 
			return false; // Now ray must hit sphere 
		
		return true;
	}

	Vec3* Line3D::findIntersectionOnSegment(const Plane& plane) const
	{
		Vec3 lineDirection = point2 - point1;
		sp_float d = plane.getDcomponent();

		// Segment = Poin1 + t . (Point2 - Point1)
		// Plane: (n . X) = d
		// put the line on the plane, Compute the t value for the directed line ab intersecting the plane.
		sp_float t = (d - plane.normalVector.dot(point1)) / plane.normalVector.dot(lineDirection);
		
		// If t in [0..1] compute and return intersection point 
		if (t >= ZERO_FLOAT && t <= ONE_FLOAT)
		{
			Vec3 intersectionPoint = point1 + lineDirection * t;
			return ALLOC_NEW(Vec3)(intersectionPoint);
		}

		return nullptr;
	}

	void Line3D::intersectionOnRay(const Plane& plane, Vec3* point) const
	{
		Vec3 lineDirection;
		normalize(point2 - point1, lineDirection);

		sp_float d = plane.getDcomponent();

		// Ray = Point1 + t . lineDirection
		// Plane: (n . X) = d
		// put the Ray on the Plane (X), Compute the t value for the directed line ab intersecting the plane.
		sp_float t = -(plane.normalVector.dot(point1) + d) / plane.normalVector.dot(lineDirection);
	
		*point = point1 + lineDirection * t;
	}

	DetailedCollisionStatus Line3D::findIntersectionOnRay(const Sphere& sphere, const sp_float _epsilon) const
	{
		Vec3 lineDirection;
		Vec3 point1ToSphere = point1 - sphere.center;

		direction(lineDirection);

		const sp_float b = point1ToSphere.dot(lineDirection);
		const sp_float c = point1ToSphere.dot(point1ToSphere) - (sphere.ray * sphere.ray);
		
		// Exit if r�s origin outside sphere (c > 0) and ray pointing away from sphere (b > 0) 
		if (c > ZERO_FLOAT && b > ZERO_FLOAT)
			return DetailedCollisionStatus(CollisionStatus::OUTSIDE);
		
		sp_float discriminant = b * b - c;    // d = b^2 - c
		
		// A negative discriminant corresponds to ray missing sphere 
		if (discriminant < -_epsilon) // the quadratic equation has not real root
			return DetailedCollisionStatus(CollisionStatus::OUTSIDE);

		sp_float sqrtDisctiminant; 

		if (discriminant > ZERO_FLOAT)
			sqrtDisctiminant = sqrtf(discriminant);
		else
		{
			discriminant = ZERO_FLOAT;
			sqrtDisctiminant = ZERO_FLOAT;
		}
		
		// Ray now found to intersect sphere, compute smallest t value of intersection 
		sp_float t1 = -b - sqrtDisctiminant;   // -b - sqrt(b^2 - c)

		// If t is negative, ray started inside sphere so clamp t to zero 	
		if (t1 < ZERO_FLOAT)
			t1 = ZERO_FLOAT;

		Vec3 intersectionPoint1 = point1 + lineDirection * t1;
		
		if (NAMESPACE_FOUNDATION::isCloseEnough(discriminant, ZERO_FLOAT))
			return DetailedCollisionStatus(CollisionStatus::INLINE, intersectionPoint1);

		// discriminant > T(0)  =>  the quadratic equation has 2 real root, then the ray intersect in 2 points

		// Ray now found to intersect sphere in 2 points, compute bigger t value of intersection 
		sp_float t2 = -b + sqrtDisctiminant;   // -b + sqrt(b^2 - c)
		
		Vec3 intersectionPoint2 = point1 + lineDirection * t2;

		return DetailedCollisionStatus(CollisionStatus::INSIDE, intersectionPoint1, intersectionPoint2);
	}

	DetailedCollisionStatus Line3D::findIntersectionOnSegment(const Sphere& sphere) const
	{
		DetailedCollisionStatus collision = findIntersectionOnRay(sphere);

		if (collision.status == CollisionStatus::OUTSIDE)
			return collision;

		if (!isOnSegment(collision.points[0]))
			return DetailedCollisionStatus(CollisionStatus::OUTSIDE);

		if (!isOnSegment(collision.points[1]))
			return DetailedCollisionStatus(CollisionStatus::INSIDE, collision.points[0]);

		return collision;
	}

	DetailedCollisionStatus Line3D::findIntersectionOnRay(const AABB& aabb) const
	{
		sp_float tmin = ZERO_FLOAT;
		sp_float tmax = SP_FLOAT_MAX;
		Vec3 lineDirection;
		direction(lineDirection);

		// For all three slabs (planes on AABB)
		for (int i = 0; i < 3; i++) 
		{ 
			if (std::abs(lineDirection[i]) < DBL_EPSILON)
			{ 
				// Ray is parallel to slab! No hit if origin not within slab 
				if (point1[i] < aabb.minPoint[i] || point1[i] > aabb.maxPoint[i])
					return DetailedCollisionStatus(CollisionStatus::OUTSIDE);
			} 
			else 
			{ 
				// Compute intersection t value of ray with near and far plane of slab 
				float ood = 1.0f / lineDirection[i];
				float t1 = (aabb.minPoint[i] - point1[i]) * ood;
				float t2 = (aabb.maxPoint[i] - point1[i]) * ood;
				
				// Make t1 be intersection with near plane, t2 with far plane 
				if (t1 > t2) 
					std::swap(t1, t2); 
				
				// Compute the intersection of slab intersection intervals 
				tmin = std::max(tmin, t1); 
				tmax = std::min(tmax, t2); 
				
				// Exit with no collision as soon as slab intersection becomes empty 
				if (tmin > tmax) 
					return DetailedCollisionStatus(CollisionStatus::OUTSIDE);
			} 	
		} 
		
		// Ray intersects all 3 slabs. Return point (q) and intersection t value (tmin) 
		return DetailedCollisionStatus(CollisionStatus::INSIDE, point1 + lineDirection * tmin, point1 + lineDirection * tmax);
	}

	CollisionStatus Line3D::hasIntersectionOnSegment(const AABB& aabb) const
	{
		float epsilon = std::numeric_limits<float>().epsilon();

		/*
		Vec3 c = (aabb.minPoint + aabb.maxPoint) * T(0.5);	// Box center-point
		Vec3 e = aabb.maxPoint - c; 	// Box halflength extents 
		Vec3 m = (point1 + point2) * T(0.5); 	// Segment midpoint 
		Vec3 d = point2 - m;	// Segment halflength vector 
		m = m - c; 
		*/
		
		Vec3 halfLengthExtends = aabb.maxPoint - aabb.minPoint;
		Vec3 halfLengthVector = point2 - point1;
		Vec3 lineCenterPoint = point1 + point2 - aabb.minPoint - aabb.maxPoint;


		// Translate box and segment to origin 	
		// Try world coordinate axes as separating axes 
		sp_float adx = std::abs(halfLengthVector[0]);
		if (std::abs(lineCenterPoint[0]) > halfLengthExtends[0] + adx)
			return CollisionStatus::OUTSIDE;
		
		sp_float ady = std::abs(halfLengthVector[1]);
		if (std::abs(lineCenterPoint[1]) > halfLengthExtends[1] + ady)
			return CollisionStatus::OUTSIDE;
		
		sp_float adz = std::abs(halfLengthVector[2]);
		if (std::abs(lineCenterPoint[2]) > halfLengthExtends[2] + adz)
			return CollisionStatus::OUTSIDE;
		
		// Add in an epsilon term to counteract arithmetic errors when segment is 
		// (near) parallel to a coordinate axis (see text for detail) 
		adx += epsilon;
		ady += epsilon;
		adz += epsilon;

		// Try cross products of segment direction vector with coordinate axes 
		if (std::abs(lineCenterPoint[1] * halfLengthVector[2] - lineCenterPoint[2] * halfLengthVector[1]) > halfLengthExtends[1] * adz + halfLengthExtends[2] * ady)
			return CollisionStatus::OUTSIDE;
		
		if (std::abs(lineCenterPoint[2] * halfLengthVector[0] - lineCenterPoint[0] * halfLengthVector[2]) > halfLengthExtends[0] * adz + halfLengthExtends[2] * adx)
			return CollisionStatus::OUTSIDE;
		
		if (std::abs(lineCenterPoint[0] * halfLengthVector[1] - lineCenterPoint[1] * halfLengthVector[0]) > halfLengthExtends[0] * ady + halfLengthExtends[1] * adx)
			return CollisionStatus::OUTSIDE;
		
		// No separating axis found; segment must be overlapping AABB 
		return CollisionStatus::INSIDE;
	}

	sp_float Line3D::squaredDistance(const Vec3& target) const
	{
		//Returns the squared distance between point and segment point1-point2

		const Vec3 ab = point2 - point1;
		const Vec3 ac = target - point1;
		const Vec3 bc = target - point2;
		
		const sp_float e = ac.dot(ab); // Handle cases where point projects outside the line segment
		
		if (e <= ZERO_FLOAT)
			return ac.dot(ac); 
		
		sp_float f = ab.dot(ab);

		if (e >= f) 		
			return bc.dot(bc); // Handle cases where point projects onto line segment
		
		return ac.dot(ac) - e * e / f;
	}

	sp_float Line3D::distance(const Vec3& target) const
	{
		return std::sqrtf(squaredDistance(target));
	}

	void Line3D::closestPoint(const Line3D& other, Vec3* closestPointOnLine1, Vec3* closestPointOnLine2, sp_float* squaredDistance) const
	{
		const Vec3 d1 = point2 - point1; // Direction vector of segment S1  
		const Vec3 d2 = other.point2 - other.point1; // Direction vector of segment S2  
		const Vec3 r = point1 - other.point1;
		const sp_float a = d1.dot(d1); // Squared length of segment S1, always nonnegative  
		const sp_float e = d2.dot(d2); // Squared length of segment S2, always nonnegative  
		const sp_float f = d2.dot(r);  // Check if either or both segments degenerate into points  

		sp_float s, t;
		
		if (a <= DefaultErrorMargin && e <= DefaultErrorMargin)
		{
			// Both segments degenerate into points
			closestPointOnLine1[0] = point1;
			closestPointOnLine2[0] = other.point1;
			squaredDistance[0] = (point1 - other.point1).dot(point1 - other.point1);
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

				// If segments not parallel, compute closest point on L1 to L2 and  
				// clamp to segment S1. Else pick arbitrary s (here 0)  
				if (denom != ZERO_FLOAT)
					s = clamp((b * f - c * e) / denom, ZERO_FLOAT, ONE_FLOAT);
				else
					s = ZERO_FLOAT;

				// Compute point on L2 closest to S1(s) using  
				// t = Dot((P1 + D1*s) - P2,D2) / Dot(D2,D2) = (b*s + f) / e  
				t = (b * s + f) / e;

				// If t in [0,1] done. Else clamp t, recompute s for the new value  
				// of t using s = Dot((P2 + D2*t) - P1,D1) / Dot(D1,D1)= (t*b - c) / a  
				// and clamp s to [0, 1]  
				if (t < ZERO_FLOAT)
				{
					t = ZERO_FLOAT;
					s = clamp(-c / a, ZERO_FLOAT, ONE_FLOAT);
				}
				else
					if (t > ONE_FLOAT)
					{
						t = ONE_FLOAT;
						s = clamp((b - c) / a, ZERO_FLOAT, ONE_FLOAT);
					}
			}
		}

		const Vec3 c1 = point1 + d1 * s;
		const Vec3 c2 = other.point1 + d2 * t;

		closestPointOnLine1[0] = c1;
		closestPointOnLine2[0] = c2;
		squaredDistance[0] = (c1 - c2).dot(c1 - c2);
	}

}
