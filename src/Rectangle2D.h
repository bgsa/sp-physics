#ifndef RECTANGLE2D_HEADER
#define RECTANGLE2D_HEADER

#include "SpectrumPhysics.h"
#include "Line2D.h"
#include "Triangle2D.h"

namespace NAMESPACE_PHYSICS
{
	template <typename T>
	class Rectangle2D
	{
	public:
		Vec2<T> point1;
		Vec2<T> point2;
		Vec2<T> point3;
		Vec2<T> point4;

		API_INTERFACE Rectangle2D();
		API_INTERFACE Rectangle2D(Vec2<T>* points);
		API_INTERFACE Rectangle2D(const Vec2<T>& point1, const Vec2<T>& point2, const Vec2<T>& point3, const Vec2<T>& point4);
		API_INTERFACE Rectangle2D(T* point1, T* point2, T* point3, T* point4);

		///<summary>
		///Get the width of the rectangle
		///</summary>
		API_INTERFACE T width() const;

		///<summary>
		///Get the height of the rectangle
		///</summary>
		API_INTERFACE T height() const;

		///<summary>
		///Get the area of the rectangle
		///</summary>
		API_INTERFACE T area() const;

		///<summary>
		///Get the perimeter of the rectangle
		///</summary>
		API_INTERFACE T perimeter() const;

		///<summary>
		///Get the length of the rectangle diagonal
		///</summary>
		API_INTERFACE T diagonalLength() const;

		///<summary>
		///Get the lines that makes the rectangle
		///</summary>
		API_INTERFACE Line2D<T>* getLines() const;

		///<summary>
		///Get the status of collision of the point into the rectangle
		///</summary>
		API_INTERFACE CollisionStatus getSatusCollision(const Vec2<T>& point) const;
	
		///<summary>
		///Chech the line has intersection with the rectangle
		///</summary>
		API_INTERFACE bool hasIntersection(const Line2D<T>& line) const;

		///<summary>
		///Chech the triangle has intersection with the rectangle
		///</summary>
		API_INTERFACE bool hasIntersection(const Triangle2D<T>& triangle) const;

		///<summary>
		///Chech the circle has intersection with the rectangle
		///</summary>
		API_INTERFACE bool hasIntersection(const Circle2D<T>& circle) const;

		///<summary>
		///Get the bounding box, given a array of 2D points
		///It groups all points in a box
		///</summary>
		API_INTERFACE static Rectangle2D<T> getBoundingBox(Vec2List<T> &points);
	};

	typedef Rectangle2D<int> Rectangle2Di;
	typedef Rectangle2D<float> Rectangle2Df;
	typedef Rectangle2D<double> Rectangle2Dd;
}

#endif // RECTANGLE2D_HEADER