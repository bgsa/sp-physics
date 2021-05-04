#include "Rectangle2D.h"
#include "Vec2List.h"

namespace NAMESPACE_PHYSICS
{

	Rectangle2D::Rectangle2D() {
	};

	Rectangle2D::Rectangle2D(Vec2* points)
	{
		this->point1 = points[0];
		this->point2 = points[1];
		this->point3 = points[2];
		this->point4 = points[3];
	}

	Rectangle2D::Rectangle2D(const Vec2& point1, const Vec2& point2, const Vec2& point3, const Vec2& point4)
	{
		this->point1 = point1;
		this->point2 = point2;
		this->point3 = point3;
		this->point4 = point4;
	}

	Rectangle2D::Rectangle2D(sp_float* point1, sp_float* point2, sp_float* point3, sp_float* point4)
	{
		this->point1 = Vec2(point1[0], point1[1]);
		this->point2 = Vec2(point2[0], point2[1]);
		this->point3 = Vec2(point3[0], point3[1]);
		this->point4 = Vec2(point4[0], point4[1]);
	}

	sp_float Rectangle2D::width() const
	{
		sp_float width = point1.x - point2.x;

		if (width == 0.0f)
			width = point1.x - point3.x;

		return std::fabsf(width);
	}

	sp_float Rectangle2D::height() const
	{
		sp_float height = point1.y - point2.y;

		if (height == 0.0f)
			height = point1.y - point3.y;

		return std::fabsf(height);
	}

	sp_float Rectangle2D::area() const
	{
		return width() * height();
	}

	sp_float Rectangle2D::perimeter() const
	{
		return 2.0f * width() + 2.0f * height();
	}

	sp_float Rectangle2D::diagonalLength() const
	{
		sp_float w = width();
		sp_float h = height();

		return sp_sqrt(w * w + h * h);
	}

	Line2D* Rectangle2D::getLines() const
	{
		Line2D* lines = ALLOC_ARRAY(Line2D, 4);
		lines[0] = Line2D(point1, point2);
		lines[1] = Line2D(point2, point3);
		lines[2] = Line2D(point3, point4);
		lines[3] = Line2D(point4, point1);

		return lines;
	}

	CollisionStatus Rectangle2D::getSatusCollision(const Vec2& point) const
	{
		sp_float area1 = Triangle2D(point1, point2, point).area();
		sp_float area2 = Triangle2D(point2, point3, point).area();
		sp_float area3 = Triangle2D(point3, point4, point).area();
		sp_float area4 = Triangle2D(point4, point1, point).area();
		sp_float areaTotal = area1 + area2 + area3 + area4;

		sp_float rectArea = area();

		if (areaTotal > rectArea)
			return CollisionStatus::OUTSIDE;

		return CollisionStatus::INSIDE;
	}

	sp_bool Rectangle2D::hasIntersection(const Line2D& line) const
	{
		Line2D line1(point1, point2);
		Line2D line2(point2, point3);
		Line2D line3(point3, point4);
		Line2D line4(point4, point1);

		Vec2* point = line1.findIntersection(line);
		if (point != NULL) {
			ALLOC_RELEASE(point);
			return true;
		}

		point = line2.findIntersection(line);
		if (point != NULL) {
			ALLOC_RELEASE(point);
			return true;
		}

		point = line3.findIntersection(line);
		if (point != NULL) {
			ALLOC_RELEASE(point);
			return true;
		}

		point = line4.findIntersection(line);
		if (point != NULL) {
			ALLOC_RELEASE(point);
			return true;
		}

		return false;
	}

	sp_bool Rectangle2D::hasIntersection(const Triangle2D& triangle) const
	{
		Line2D* linesOfRectangle = getLines();
		Line2D* linesOfTriangle = triangle.getLines();

		for (sp_uint i = 0; i < 4; i++)
			for (sp_uint j = 0; j < 3; j++)
			{
				Line2D line1 = linesOfRectangle[i];
				Line2D line2 = linesOfTriangle[j];

				Vec2* point = line1.findIntersection(line2);

				if (point != nullptr) 
				{
					ALLOC_RELEASE(linesOfRectangle);
					return true;
				}
			}

		ALLOC_RELEASE(linesOfRectangle);
		return false;
	}

	sp_bool Rectangle2D::hasIntersection(const Circle2D& circle) const
	{
		Line2D* linesOfRectangle = getLines();

		for (sp_uint i = 0; i < 4; i++)
		{
			Line2D line = linesOfRectangle[i];

			CollisionStatus status = line.hasIntersections(circle);

			if (status == CollisionStatus::INLINE || status == CollisionStatus::INSIDE) 
			{
				ALLOC_RELEASE(linesOfRectangle);
				return true;
			}
		}

		ALLOC_RELEASE(linesOfRectangle);
		return false;
	}

	Rectangle2D Rectangle2D::getBoundingBox(Vec2List &points)
	{
		Vec2* minX = points.findMinX();
		Vec2* minY = points.findMinY();
		Vec2* maxX = points.findMaxX();
		Vec2* maxY = points.findMaxY();
		
		return Rectangle2D(
			Vec2(minX->x, minY->y),
			Vec2(maxX->x, minY->y),
			Vec2(maxX->x, maxY->y),
			Vec2(minX->x, maxY->y)
		);
	}

}