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
		API_INTERFACE inline void direction(Vec3* output) const
		{
			NAMESPACE_PHYSICS::normalize(point2 - point1, output);
		}

		///<summary>
		/// Get the SQUARED distance from SEGMENT of line and an arbitrary point
		///</summary>
		API_INTERFACE sp_float squaredDistance(const Vec3& target) const;

		///<summary>
		/// Get the SQUARED distance from SEGMENT of line and an arbitrary point
		///</summary>
		API_INTERFACE sp_float distance(const Vec3& target) const;

		///<summary>
		///Returns the center of the segment of line
		///</summary>
		API_INTERFACE void center(Vec3* output) const
		{
			output[0] = (point1 + point2) * HALF_FLOAT;
		}

		///<summary>
		///Returns the center of the segment of line
		///</summary>
		API_INTERFACE sp_float lengthOfSegment() const;

		///<summary>
		/// Check the point is on the line
		///</summary>
		API_INTERFACE sp_bool isOnLine(const Vec3& point, const sp_float _epsilon = DefaultErrorMargin) const;

		///<summary>
		/// Check the point is on segment
		///</summary>
		API_INTERFACE sp_bool isOnSegment(const Vec3& point) const;
		
		/// <summary>
		/// Check if the lines are parallel
		/// </summary>
		/// <param name="line">Line Segment</param>
		/// <param name="_epsilon">Error Margin</param>
		/// <returns>True if they are parallel or else false</returns>
		API_INTERFACE sp_bool isParallel(const Line3D& line, const sp_float _epsilon = DefaultErrorMargin) const
		{
			Vec3 direction1, direction2;
			direction(&direction1);
			line.direction(&direction2);

			Vec3 crossedVector;
			NAMESPACE_PHYSICS::cross(direction2, direction1, &crossedVector);

			return isCloseEnough(crossedVector, ZERO_FLOAT, _epsilon);
		}
		
		/// <summary>
		/// Check if the lines are perpendicular
		/// </summary>
		/// <param name="line">Line Segment</param>
		/// <param name="_epsilon">Error Margin</param>
		/// <returns>True if they are penpendicular or else false</returns>
		API_INTERFACE sp_bool isPerpendicular(const Line3D& line, const sp_float _epsilon = DefaultErrorMargin) const
		{
			Vec3 _direction;
			line.direction(&_direction);

			return isPerpendicular(_direction, _epsilon);
		}

		/// <summary>
		/// Check if this line is perpendicular with a normal vector
		/// </summary>
		/// <param name="direction">Normal vector</param>
		/// <param name="_epsilon">Error Margin</param>
		/// <returns>True if they are penpendicular or else false</returns>
		API_INTERFACE sp_bool isPerpendicular(const Vec3& _direction, const sp_float _epsilon = DefaultErrorMargin) const
		{
			Vec3 _direction2;
			direction(&_direction2);

			return isCloseEnough(_direction.dot(_direction2), ZERO_FLOAT, _epsilon);
		}

		///<summary>
		/// Check the ray has intersection with the sphere
		///</summary>
		API_INTERFACE sp_bool hasIntersectionOnRay(const Sphere& sphere) const;

		///<summary>
		/// Find intersection of line against another one
		///</summary>
		API_INTERFACE sp_bool intersection(const Line3D& line2, Vec3* point, const sp_float _epsilon = DefaultErrorMargin) const;
		
		/// <summary>
		/// Find intersection of line against triangle
		/// </summary>
		/// <param name="triangle">Target</param>
		/// <param name="point">Intersection point</param>
		/// <param name="_epsilon">Error margin</param>
		/// <param name="hasIntersection">True if has intersection or else False</param>
		/// <returns>void</returns>
		API_INTERFACE sp_bool intersection(const Triangle3D& triangle, Vec3* point, const sp_float _epsilon = DefaultErrorMargin) const;

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

		///<summary>
		/// Find the cross-product with two lines
		///</summary>
		API_INTERFACE void cross(const Line3D& otherLine, Vec3* output) const
		{
			NAMESPACE_PHYSICS::cross(point2 - point1, otherLine.point2 - otherLine.point1, output);
		}

		API_INTERFACE void closestPoint(const Line3D& other, Vec3* closestPointOnLine1, Vec3* closestPointOnLine2, sp_float* squaredDistance) const;

	};

}

#endif // !LINE_3D_HEADER