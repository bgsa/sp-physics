#include "Sphere.h"

namespace NAMESPACE_PHYSICS
{
	Sphere WelzlSphere(Vec3* points, int numPts, Vec3 suportPoints[], sp_int suportPointsCount)
	{
		// if no input points, the recursion has bottomed out. Now compute an 
		// exact sphere based on points in set of support (zero through four points) 

		if (numPts == 0)
		{
			switch (suportPointsCount)
			{
			case 0:
				return Sphere();
			case 1:
				return Sphere(suportPoints[0]);
			case 2:
				return Sphere(suportPoints[0], suportPoints[1]);
			case 3:
				return Sphere(suportPoints[0], suportPoints[1], suportPoints[2]);
			case 4:
				return Sphere(suportPoints[0], suportPoints[1], suportPoints[2], suportPoints[3]);
			}
		}

		// Pick a point at "random" (here just the last point of the input set) 	
		int index = numPts - 1;

		// Recursively compute the smallest bounding sphere of the remaining points 
		Sphere smallestSphere = WelzlSphere(points, numPts - 1, suportPoints, suportPointsCount); // (*) 

		bool isPointInsideTheSphere = smallestSphere.collisionStatus(points[index]) == CollisionStatus::INSIDE;
		if (isPointInsideTheSphere)
			return smallestSphere;

		// Otherwise, update set of support to additionally contain the new point 
		suportPoints[suportPointsCount] = points[index];

		// Recursively compute the smallest sphere of remaining points with new s.o.s. 
		return WelzlSphere(points, numPts - 1, suportPoints, suportPointsCount + 1);
	}

	Sphere::Sphere()
	{
		this->center = Vec3Zeros;
		this->ray = ONE_FLOAT;
	}

	Sphere::Sphere(const Vec3 &center, sp_float ray)
	{
		this->center = center;
		this->ray = ray;
	}

	Sphere::Sphere(const Vec3 &point1)
	{
		this->center = point1;
		this->ray = 1.0f;
	}

	Sphere::Sphere(const Vec3 &point1, const Vec3 &point2)
	{
		Line3D line = Line3D(point1, point2);
		line.center(&center);
		ray = line.lengthOfSegment() * HALF_FLOAT;
	}

	Sphere::Sphere(const Vec3 &point1, const Vec3 &point2, const Vec3 &point3)
	{
		Vec3 ac = point3 - point1;
		Vec3 ab = point2 - point1;
		Vec3 abXac, temp1, temp2;
		cross(ab, ac, abXac);
		cross(abXac, ab, temp1);
		cross(ac, abXac, temp2);

		// this is the vector from a TO the circumsphere center
		const Vec3 toCircumsphereCenter = (temp1 * ac.squaredLength() + temp2 *ab.squaredLength()) / (TWO_FLOAT * abXac.squaredLength());
		
		// The 3 space coords of the circumsphere center then:
		center = point1 + toCircumsphereCenter; // now this is the actual 3space location
		ray = length(toCircumsphereCenter);
	}

	Sphere::Sphere(const Vec3 &point1, const Vec3 &point2, const Vec3 &point3, const Vec3 &point4)
	{
		Mat4 m = Mat4(
			Vec4(point1, ONE_FLOAT),
			Vec4(point2, ONE_FLOAT),
			Vec4(point3, ONE_FLOAT),
			Vec4(point4, ONE_FLOAT)
			);

		sp_float invertedDeterminant = 1.0f / m.determinant();

		sp_float t1 = -(point1.dot(point1));
		sp_float t2 = -(point2.dot(point2));
		sp_float t3 = -(point3.dot(point3));
		sp_float t4 = -(point4.dot(point4));

		m = Mat4(
			Vec4(t1, point1[1], point1[2], 1.0f),
			Vec4(t2, point2[1], point2[2], 1.0f),
			Vec4(t3, point3[1], point3[2], 1.0f),
			Vec4(t4, point4[1], point4[2], 1.0f)
			);
		sp_float a = m.determinant() * invertedDeterminant;
		sp_float x = a * -0.5f;

		m = Mat4(
			Vec4(point1[0], t1, point1[2], 1.0f),
			Vec4(point2[0], t2, point2[2], 1.0f),
			Vec4(point3[0], t3, point3[2], 1.0f),
			Vec4(point4[0], t4, point4[2], 1.0f)
			);
		sp_float b = m.determinant() * invertedDeterminant;
		sp_float y = b * -0.5f;

		m = Mat4(
			Vec4(point1[0], point1[1], t1, 1.0f),
			Vec4(point2[0], point2[1], t2, 1.0f),
			Vec4(point3[0], point3[1], t3, 1.0f),
			Vec4(point4[0], point4[1], t4, 1.0f)
			);
		sp_float c = m.determinant() * invertedDeterminant;
		sp_float z = c * -0.5f;

		m = Mat4(
			Vec4(point1[0], point1[1], point1[2], t1),
			Vec4(point2[0], point2[1], point2[2], t2),
			Vec4(point3[0], point3[1], point3[2], t3),
			Vec4(point4[0], point4[1], point4[2], t4)
			);
		sp_float d = m.determinant() * invertedDeterminant;

		center = { x, y, z };
		ray = std::sqrt(a * a + b * b + c * c - 4 * d) / 2.0f;
	}

	void Sphere::translate(const Vec3& translation)
	{
		center += translation;
	}

	void Sphere::scale(const Vec3& factor)
	{
		ray *= factor.x;
	}

	CollisionStatus Sphere::collisionStatus(const Vec3 &point)  const
	{
		sp_float distanceToPoint = distance(center, point);
		
		if (NAMESPACE_FOUNDATION::isCloseEnough(distanceToPoint, ray))
			return CollisionStatus::INLINE;

		if (distanceToPoint > ray)
			return CollisionStatus::OUTSIDE;

		return CollisionStatus::INSIDE;
	}

	CollisionStatus Sphere::collisionStatus(const Sphere& sphere)  const
	{
		Vec3 rayToSphere = center - sphere.center; 
		sp_float squaredDistance = rayToSphere.dot(rayToSphere);

		// Spheres intersect if squared distance is less than squared sum of radius 
		sp_float diameter = ray + sphere.ray;
		sp_float squaredDiameter = diameter * diameter;

		if (NAMESPACE_FOUNDATION::isCloseEnough(squaredDistance, squaredDiameter))
			return CollisionStatus::INLINE;

		if (squaredDistance > squaredDiameter)
			return CollisionStatus::OUTSIDE;

		return CollisionStatus::INSIDE;
	}

	CollisionStatus Sphere::collisionStatus(const Plane& plane)  const
	{
		/*
		Implementation "1"
		T distanceToPlane = plane.distance(center);

		if (isCloseEnough(distanceToPlane, ray))
			return CollisionStatus::INLINE;

		if (distanceToPlane > ray)
			return CollisionStatus::OUTSIDE;

		return CollisionStatus::INSIDE;
		*/

		// optimized implementation

		sp_float d = plane.getDcomponent();
		sp_float distanceToPlane = center.dot(plane.normalVector) + d;

		if (NAMESPACE_FOUNDATION::isCloseEnough(distanceToPlane, ray))
			return CollisionStatus::INLINE;

		if (distanceToPlane > ray)
			return CollisionStatus::OUTSIDE;

		return CollisionStatus::INSIDE;
	}

	Sphere Sphere::buildFrom(const AABB &aabb)
	{
		sp_float maxDistance = aabb.maxPoint[0] - aabb.minPoint[0];

		maxDistance = std::max(maxDistance, aabb.maxPoint[1] - aabb.minPoint[1]);

		maxDistance = std::max(maxDistance, aabb.maxPoint[2] - aabb.minPoint[2]);

		return Sphere(
			aabb.center(),
			maxDistance / 2.0f
			);
	}

	Sphere Sphere::buildFrom(const Vec3List& pointList)
	{
		Vec3* suportPoints = ALLOC_ARRAY(Vec3, pointList.count);

		Sphere result = WelzlSphere(pointList.points, pointList.count, suportPoints, 0);

		ALLOC_RELEASE(suportPoints);
		return result;
	}

	Sphere Sphere::enclose(const Sphere& sphere)
	{
		Sphere result;

		Vec3 d = sphere.center - center;
		sp_float squaredDistance = d.dot(d);  	// Compute the squared distance between the sphere centers 

		if (std::pow(double(sphere.ray - ray), 2) >= squaredDistance)
		{
			// The sphere with the larger radius encloses the other; 
			// just set s to be the larger of the two spheres
			if (sphere.ray >= this->ray)
				result = sphere;
			else
				result = *this;
		}
		else
		{
			// Spheres partially overlapping or disjoint 
			sp_float distance = sp_sqrt(squaredDistance);

			result.ray = (distance + ray + sphere.ray) * 0.5f;
			result.center = center;

			if (distance > DefaultErrorMargin)
				result.center += d * ((result.ray - ray) / distance);
		}

		return result;
	}

	Sphere Sphere::enclose(const AABB& aabb)
	{
		Sphere sphere = Sphere::buildFrom(aabb);	

		return enclose(sphere);
	}

	BoundingVolumeType Sphere::type() const
	{
		return BoundingVolumeType::Sphere;
	}
}