#include "Circle2D.h"

namespace NAMESPACE_PHYSICS
{
	Circle2D::Circle2D() 
	{
		ray = ZERO_FLOAT;
		center = Vec2(ZERO_FLOAT);
	}

	Circle2D::Circle2D(const Vec2& center, sp_float ray)
	{
		this->center = center;
		this->ray = ray;
	}

	Circle2D::Circle2D(const Vec2& point1, const Vec2& point2, const Vec2& point3)
	{
		Mat4 matrix = {
			ONE_FLOAT, ONE_FLOAT, ONE_FLOAT, ONE_FLOAT,
			point1.x * point1.x + point1.y * point1.y, point1.x, point1.y, ONE_FLOAT,
			point2.x * point2.x + point2.y * point2.y, point2.x, point2.y, ONE_FLOAT,
			point3.x * point3.x + point3.y * point3.y, point3.x, point3.y, ONE_FLOAT,
		};

		sp_float value = matrix.cofactorIJ(0, 0);

		sp_float a = value / value;
		sp_float b = matrix.cofactorIJ(0, 1) / value;
		sp_float c = matrix.cofactorIJ(0, 2) / value;
		sp_float d = matrix.cofactorIJ(0, 3) / value;

		sp_float numerator = (b*b) + (c*c) - (4 * a * d);
		sp_float denominator = 4 * a * a;

		this->center = Vec2(-(b / 2 * a), -(c / 2 * a));
		this->ray = sp_sqrt(numerator / denominator);
	}

	sp_float Circle2D::area() const
	{
		return (PI * ray * ray);
	}

	sp_float Circle2D::circumference() const
	{
		return 2.0f * PI * ray;
	}

	sp_float* Circle2D::calculatePoints(size_t& pointsCount) const
	{
		const sp_uint vertexCount = 126;
		pointsCount = divideBy2(vertexCount);

		sp_float* points = ALLOC_ARRAY(sp_float, vertexCount);
		size_t index = 0;

		for (sp_float angle = 0.0f; angle < TWO_PI; angle += 0.1f)
		{
			points[index + 0] = ray * std::cosf(angle) + center.x;
			points[index + 1] = ray * std::sinf(angle) + center.y;

			index += 2;
		}

		return points;
	}

	sp_float Circle2D::distance(const Vec2& point) const
	{
		 return center.distance(point);
	}

	CollisionStatus Circle2D::collisionStatus(const Vec2& point) const
	{
		sp_float distance = std::ceilf(point.distance(center));
		sp_float rayDistance = std::ceilf(ray);

		if (distance > rayDistance)
			return CollisionStatus::OUTSIDE;

		if (distance < rayDistance)
			return CollisionStatus::INSIDE;

		return CollisionStatus::INLINE;
	}

	sp_bool Circle2D::hasIntersection(const Circle2D& circle2) const
	{
		sp_float distance = center.distance(circle2.center);

		sp_bool intersectionFound = ray + circle2.ray >= distance;

		return intersectionFound;
	}

	Vec2* Circle2D::findIntersection(const Circle2D& circle2) const
	{
		Vec2 point1AsVector = center;
		Vec2 point2AsVector = circle2.center;

		sp_float distance = point1AsVector.distance(point2AsVector);

		sp_bool intersectionFound = ray + circle2.ray >= distance;

		if (!intersectionFound)
			return nullptr;

		sp_float a = (ray*ray - circle2.ray*circle2.ray + distance * distance) / (2.0f * distance);
		sp_float h = sp_sqrt((ray * ray) - (a * a));

		Vec2 p3 = ((point2AsVector - point1AsVector) * (a / distance)) + center;

		sp_float x3 = p3[0] + h * (circle2.center.y - center.y) / distance;
		sp_float y3 = p3[1] - h * (circle2.center.x - center.x) / distance;

		Vec2* result;

		if (ray + circle2.ray == distance)   //has only one point
		{
			result = ALLOC_ARRAY(Vec2, 1);
			result[0] = Vec2(x3, y3);

			return result;
		}
		else
		{
			sp_float x4 = p3[0] - h * (circle2.center.y - center.y) / distance;
			sp_float y4 = p3[1] + h * (circle2.center.x - center.x) / distance;

			result = ALLOC_ARRAY(Vec2, 2);
			result[0] = Vec2(x3, y3);
			result[1] = Vec2(x4, y4);
		}

		return result;
	}

}