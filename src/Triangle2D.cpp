#include "Triangle2D.h"

template<typename T>
Triangle2D<T>::Triangle2D() { };

template<typename T>
Triangle2D<T>::Triangle2D(const Vec2<T>& point1, const Vec2<T>& point2, const Vec2<T>& point3)
{
	this->point1 = point1;
	this->point2 = point2;
	this->point3 = point3;
}

template<typename T>
Triangle2D<T>::Triangle2D(T* point1, T* point2, T* point3)
{
	this->point1 = Vec2<T>(point1[0], point1[1]);
	this->point2 = Vec2<T>(point2[0], point2[1]);
	this->point3 = Vec2<T>(point3[0], point3[1]);
}

template<typename T>
T Triangle2D<T>::area() const
{
	double numerator = abs(point1.x * point2.y + point2.x * point3.y + point3.x * point1.y - point1.y * point2.x - point2.y * point3.x - point3.y * point1.x);
	T area = T(numerator / 2);

	return area;
}

template<typename T>
T Triangle2D<T>::perimeter() const
{
	double term1 = sqrt((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y));
	double term2 = sqrt((point2.x - point3.x) * (point2.x - point3.x) + (point2.y - point3.y) * (point2.y - point3.y));
	double term3 = sqrt((point3.x - point1.x) * (point3.x - point1.x) + (point3.y - point1.y) * (point3.y - point1.y));

	T perimeter = T(term1) + T(term2) + T(term3);

	return perimeter;
}

template<typename T>
T Triangle2D<T>::height() const
{
	T lengthVec2 = point2.distance(point3);

	T angle = point1.angle(point2);

	T heigh = T(lengthVec2 * sin(angle));

	return heigh;
}

template<typename T>
Line2D<T>* Triangle2D<T>::getLines() const
{
	Line2D<T>* lines = ALLOC_ARRAY(Line2D<T>, 3);
	lines[0] = Line2D<T>(point1, point2);
	lines[1] = Line2D<T>(point2, point3);
	lines[2] = Line2D<T>(point3, point1);

	return lines;
}

template<typename T>
CollisionStatus Triangle2D<T>::getCollisionStatus(const Vec2<T>& point) const
{
	Line2D<T> line1 = Line2D<T>(point1, point2);
	Line2D<T> line2 = Line2D<T>(point2, point3);
	Line2D<T> line3 = Line2D<T>(point3, point1);

	Orientation orientation = line1.getOrientation(point);

	if (orientation == Orientation::RIGHT)
		return CollisionStatus::OUTSIDE;

	if (orientation == Orientation::NONE)
		return CollisionStatus::INLINE;

	orientation = line2.getOrientation(point);

	if (orientation == Orientation::RIGHT)
		return CollisionStatus::OUTSIDE;

	if (orientation == Orientation::NONE)
		return CollisionStatus::INLINE;

	orientation = line3.getOrientation(point);

	if (orientation == Orientation::RIGHT)
		return CollisionStatus::OUTSIDE;

	if (orientation == Orientation::NONE)
		return CollisionStatus::INLINE;

	return CollisionStatus::INSIDE;
}

template<typename T>
bool Triangle2D<T>::hasIntersection(const Line2D<T>& line) const
{
	Line2D<T> line1 = Line2D<T>(point1, point2);
	Line2D<T> line2 = Line2D<T>(point2, point3);
	Line2D<T> line3 = Line2D<T>(point3, point1);

	Vec2<T>* point = line1.findIntersection(line);

	if (point != nullptr)
		return true;

	point = line2.findIntersection(line);

	if (point != nullptr)
		return true;

	point = line3.findIntersection(line);

	if (point != nullptr)
		return true;

	return false;
}

template<typename T>
bool Triangle2D<T>::hasIntersection(const Circle2D<T>& circle) const
{
	Line2D<T>* linesOfTriangle = getLines();

	for (size_t i = 0; i < 3; i++)
	{
		Line2D<T> line = linesOfTriangle[i];

		CollisionStatus status = line.hasIntersections(circle);

		if (status == CollisionStatus::INLINE || status == CollisionStatus::INSIDE)
		{
			ALLOC_RELEASE(linesOfTriangle);
			return true;
		}
	}

	ALLOC_RELEASE(linesOfTriangle);
	return false;
}

namespace OpenML
{
	template class Triangle2D<int>;
	template class Triangle2D<float>;
	template class Triangle2D<double>;
}