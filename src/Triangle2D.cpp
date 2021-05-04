#include "Triangle2D.h"

namespace NAMESPACE_PHYSICS
{

	Triangle2D::Triangle2D() { };

	Triangle2D::Triangle2D(const Vec2& point1, const Vec2& point2, const Vec2& point3)
	{
		this->point1 = point1;
		this->point2 = point2;
		this->point3 = point3;
	}

	Triangle2D::Triangle2D(sp_float* point1, sp_float* point2, sp_float* point3)
	{
		this->point1 = Vec2(point1[0], point1[1]);
		this->point2 = Vec2(point2[0], point2[1]);
		this->point3 = Vec2(point3[0], point3[1]);
	}

	sp_float Triangle2D::area() const
	{
		// determinant
		const sp_float numerator = std::fabsf(point1.x * point2.y + point2.x * point3.y + point3.x * point1.y - point1.y * point2.x - point2.y * point3.x - point3.y * point1.x);
		
		return numerator * HALF_FLOAT;
	}

	sp_float Triangle2D::perimeter() const
	{
		const sp_float term1 = sp_sqrt((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y));
		const sp_float term2 = sp_sqrt((point2.x - point3.x) * (point2.x - point3.x) + (point2.y - point3.y) * (point2.y - point3.y));
		const sp_float term3 = sp_sqrt((point3.x - point1.x) * (point3.x - point1.x) + (point3.y - point1.y) * (point3.y - point1.y));

		return term1 + term2 + term3;
	}

	sp_float Triangle2D::height() const
	{
		sp_float lengthVec2 = point2.distance(point3);

		sp_float angle = point1.angle(point2);

		sp_float heigh = lengthVec2 * std::sinf(angle);

		return heigh;
	}

	Line2D* Triangle2D::getLines() const
	{
		Line2D* lines = ALLOC_ARRAY(Line2D, 3);
		lines[0] = Line2D(point1, point2);
		lines[1] = Line2D(point2, point3);
		lines[2] = Line2D(point3, point1);

		return lines;
	}

	CollisionStatus Triangle2D::getCollisionStatus(const Vec2& point) const
	{
		Line2D line1(point1, point2);
		Line2D line2(point2, point3);
		Line2D line3(point3, point1);

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

	sp_bool Triangle2D::hasIntersection(const Line2D& line) const
	{
		Line2D line1(point1, point2);
		Line2D line2(point2, point3);
		Line2D line3(point3, point1);

		Vec2* point = line1.findIntersection(line);

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

	sp_bool Triangle2D::hasIntersection(const Circle2D& circle) const
	{
		Line2D* linesOfTriangle = getLines();

		for (sp_uint i = 0; i < 3; i++)
		{
			Line2D line = linesOfTriangle[i];

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

}