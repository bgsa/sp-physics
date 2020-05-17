#ifndef RECTANGLE2D_HEADER
#define RECTANGLE2D_HEADER

#include "SpectrumPhysics.h"
#include "Line2D.h"
#include "Triangle2D.h"
#include "DetailedCollisionStatus.h"

namespace NAMESPACE_PHYSICS
{

	class Rectangle2D
	{
	public:
		Vec2 point1;
		Vec2 point2;
		Vec2 point3;
		Vec2 point4;

		API_INTERFACE Rectangle2D();
		API_INTERFACE Rectangle2D(Vec2* points);
		API_INTERFACE Rectangle2D(const Vec2& point1, const Vec2& point2, const Vec2& point3, const Vec2& point4);
		API_INTERFACE Rectangle2D(sp_float* point1, sp_float* point2, sp_float* point3, sp_float* point4);

		///<summary>
		///Get the width of the rectangle
		///</summary>
		API_INTERFACE sp_float width() const;

		///<summary>
		///Get the height of the rectangle
		///</summary>
		API_INTERFACE sp_float height() const;

		///<summary>
		///Get the area of the rectangle
		///</summary>
		API_INTERFACE sp_float area() const;

		///<summary>
		///Get the perimeter of the rectangle
		///</summary>
		API_INTERFACE sp_float perimeter() const;

		///<summary>
		///Get the length of the rectangle diagonal
		///</summary>
		API_INTERFACE sp_float diagonalLength() const;

		///<summary>
		///Get the lines that makes the rectangle
		///</summary>
		API_INTERFACE Line2D* getLines() const;

		///<summary>
		///Get the status of collision of the point into the rectangle
		///</summary>
		API_INTERFACE CollisionStatus getSatusCollision(const Vec2& point) const;
	
		///<summary>
		///Chech the line has intersection with the rectangle
		///</summary>
		API_INTERFACE sp_bool hasIntersection(const Line2D& line) const;

		///<summary>
		///Chech the triangle has intersection with the rectangle
		///</summary>
		API_INTERFACE sp_bool hasIntersection(const Triangle2D& triangle) const;

		///<summary>
		///Chech the circle has intersection with the rectangle
		///</summary>
		API_INTERFACE sp_bool hasIntersection(const Circle2D& circle) const;

		///<summary>
		///Get the bounding box, given a array of 2D points
		///It groups all points in a box
		///</summary>
		API_INTERFACE static Rectangle2D getBoundingBox(Vec2List &points);
	};

}

#endif // RECTANGLE2D_HEADER