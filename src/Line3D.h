#ifndef LINE_3D_HEADER
#define LINE_3D_HEADER

#include "SpectrumPhysics.h"
#include "Plane3D.h"
#include "Line3D.h"
#include "Sphere.h"
#include "AABB.h"
#include "DetailedCollisionStatus.h"

namespace NAMESPACE_PHYSICS 
{
	class Line3D
	{
	public:
		Vec3 point1;
		Vec3 point2;

		///<summary>
		/// Empty constructor 
		///</summary>
		API_INTERFACE Line3D();

		///<summary>
		/// Build a line from 2 points
		///</summary>
		API_INTERFACE Line3D(const Vec3& point1, const Vec3& point2);

		///<summary>
		/// Build a line from 2 points as vector
		///</summary>
		API_INTERFACE Line3D(Vec3* points);

		///<summary>
		/// Build a line from 2 points (3 numbers for each point)
		///</summary>
		API_INTERFACE Line3D(sp_float* point1, sp_float* point2);

		///<summary>
		/// Get the direction of line
		///</summary>
		API_INTERFACE Vec3 direction() const;

		///<summary>
		/// Get the SQUARED distance from SEGMENT of line and an arbitrary point
		///</summary>
		API_INTERFACE float squaredDistance(const Vec3& target) const;

		///<summary>
		/// Get the SQUARED distance from SEGMENT of line and an arbitrary point
		///</summary>
		API_INTERFACE float distance(const Vec3& target) const;

		///<summary>
		///Returns the center of the segment of line
		///</summary>
		API_INTERFACE Vec3 centerOfSegment() const;

		///<summary>
		///Returns the center of the segment of line
		///</summary>
		API_INTERFACE sp_float lengthOfSegment() const;

		///<summary>
		/// Check the point is on the line
		///</summary>
		API_INTERFACE sp_bool isOnLine(const Vec3& point) const;

		///<summary>
		/// Check the point is on segment
		///</summary>
		API_INTERFACE sp_bool isOnSegment(const Vec3& point) const;
		
		///<summary>
		/// Check the ray has intersection with the sphere
		///</summary>
		API_INTERFACE sp_bool hasIntersectionOnRay(const Sphere& sphere) const;

		///<summary>
		/// Find intersection of line against another one
		///</summary>
		API_INTERFACE void intersection(const Line3D& line2, Vec3* point) const;

		///<summary>
		/// Find intersection of line against a phere
		///</summary>
		API_INTERFACE DetailedCollisionStatus findIntersectionOnRay(const Sphere& sphere) const;

		///<summary>
		/// Find intersection of SEGMENT of line against a phere
		///</summary>
		API_INTERFACE DetailedCollisionStatus findIntersectionOnSegment(const Sphere& sphere) const;

		///<summary>
		/// Find intersection of ray against a AABB
		///</summary>
		API_INTERFACE DetailedCollisionStatus findIntersectionOnRay(const AABB& aabb) const;

		///<summary>
		/// Find intersection of SEGMENT of line against a AABB
		///</summary>
		API_INTERFACE CollisionStatus hasIntersectionOnSegment(const AABB& aabb) const;

		///<summary>
		/// Find intersection SEGMENT of line against a plane
		///</summary>
		API_INTERFACE Vec3* findIntersectionOnSegment(const Plane3D& plane) const;

		/// <summary>
		/// Find intersection RAY of line against a plane
		/// </summary>
		API_INTERFACE void intersectionOnRay(const Plane3D& plane, Vec3* point) const;

		///<summary>
		/// Get the closest point in the SEGMENT of line, given an arbitrary point
		///</summary>
		API_INTERFACE Vec3 closestPointOnTheLine(const Vec3& target) const;

	};

}

#endif // !LINE_3D_HEADER