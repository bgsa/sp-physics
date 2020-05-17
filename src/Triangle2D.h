#ifndef TRIANGLE2D_HEADER
#define TRIANGLE2D_HEADER

#include "SpectrumPhysics.h"
#include "Line2D.h"
#include "Orientation.h"
#include "DetailedCollisionStatus.h"

namespace NAMESPACE_PHYSICS
{
	class Triangle2D
	{
	public:
		Vec2 point1;
		Vec2 point2;
		Vec2 point3;

		API_INTERFACE Triangle2D();
		API_INTERFACE Triangle2D(const Vec2& point1, const Vec2& point2, const Vec2& point3);
		API_INTERFACE Triangle2D(sp_float* point1, sp_float* point2, sp_float* point3);

		API_INTERFACE sp_float area() const;

		API_INTERFACE sp_float perimeter() const;

		API_INTERFACE sp_float height() const;
		
		///<summary>
		///Get the lines that makes the rectangle
		///</summary>
		Line2D* getLines() const;

		///<summary>
		///Chech the point is outside, inside or on the line of the triangle
		///</summary>
		API_INTERFACE CollisionStatus getCollisionStatus(const Vec2& point) const;

		///<summary>
		///Chech the line has intersection with the triangle
		///</summary>
		API_INTERFACE sp_bool hasIntersection(const Line2D& line) const;

		///<summary>
		///Chech the circle has intersection with the triangle
		///</summary>
		API_INTERFACE sp_bool hasIntersection(const Circle2D& circle) const;

	};

}

#endif // TRIANGLE2D_HEADER