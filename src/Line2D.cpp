#include "Line2D.h"

namespace NAMESPACE_PHYSICS
{

	Line2D::Line2D() {
	};

	Line2D::Line2D(const Vec2& point1, const Vec2& point2)
	{
		sp_assert(point1 != point2, "InvalidArgumentException");

		this->point1 = point1;
		this->point2 = point2;
	}

	Line2D::Line2D(sp_float* point1, sp_float* point2)
	{
		sp_assert(point1 != point2, "InvalidArgumentException");

		this->point1 = Vec2(point1[0], point1[1]);
		this->point2 = Vec2(point2[0], point2[1]);
	}

	sp_float Line2D::angle() const
	{
		sp_float deltaY = point2.y - point1.y;
		sp_float deltaX = point2.x - point1.x;

		sp_float angle = atanf(deltaY / deltaX);

		return angle;
	}

	sp_float Line2D::slope() const
	{
		sp_float deltaY = point2.y - point1.y;
		sp_float deltaX = point2.x - point1.x;

		sp_float slope = (sp_float)(deltaY / deltaX);

		return slope;
	}

	Vec2 Line2D::getParametricEquation() const
	{
		sp_float m = slope();
		sp_float b = -(m * point1.x) + point1.y;

		return Vec2(m, b);
	}

	Vec2 Line2D::toRay()
	{
		Vec2 result = (point2 - point1);

		result = result.normalize();

		return result;
	}

	sp_bool Line2D::isOnTheLine(const Vec2& point) const
	{
		Orientation orientation = getOrientation(point);

		if (orientation != Orientation::NONE)
			return false;

		// check the range of the line x point
		sp_float point1PosX = point1.x > point2.x ? point2.x : point1.x;
		sp_float point2PosX = point1PosX + deltaX();

		sp_float point1PosY = point1.y > point2.y ? point2.y : point1.y;
		sp_float point2PosY = point1PosY + deltaY();

		if (point.x < point1PosX || point.x > point2PosX
			|| point.y < point1PosY || point.y > point2PosY)
			return false;

		return true;
	}

	sp_bool Line2D::isOnTheLeft(const Vec2& point) const {
		Orientation orientation = getOrientation(point);
		return orientation == Orientation::LEFT;
	}

	sp_bool Line2D::isOnTheRight(const Vec2& point) const {
		Orientation orientation = getOrientation(point);
		return orientation == Orientation::RIGHT;
	}

	Orientation Line2D::getOrientation(const Vec2& point) const
	{
		Mat3 lineMatrix = {
			ONE_FLOAT, ONE_FLOAT, ONE_FLOAT,
			point1.x, point2.x, point.x,
			point1.y, point2.y, point.y
		};

		int determinant = (int)lineMatrix.determinant();

		if (determinant == 0)
			return Orientation::NONE;
		if (determinant > 0)
			return Orientation::LEFT;
		else
			return Orientation::RIGHT;
	}

	sp_float Line2D::getDistance(Vec2 point) const
	{
		Vec3 values = equation();

		sp_double numerador = std::fabs(values.x * point.x + values.y * point.y + values.z);
		sp_double denominador = std::sqrt(values.x * values.x + values.y * values.y);

		return (sp_float)(numerador / denominador);
	}

	Vec2* Line2D::findIntersection(const Line2D& otherLine) const
	{
		Vec2 line2Point1 = otherLine.point1;
		Vec2 line2Point2 = otherLine.point2;

		sp_float determinant = (line2Point2.x - line2Point1.x) * (point2.y - point1.y) - (line2Point2.y - line2Point1.y) * (point2.x - point1.x);

		if (determinant == 0.0)
			return nullptr; // intersection not found

		sp_float s = ((line2Point2.x - line2Point1.x) * (line2Point1.y - point1.y) - (line2Point2.y - line2Point1.y) * (line2Point1.x - point1.x)) / determinant;
		//double t = ((point2.x - point1.x) * (line2Point1.y - point1.y) - (point2.y - point1.y) * (line2Point1.x - point1.x)) / determinant;

		Vec2* intersection = ALLOC_NEW(Vec2)(
			point1.x + (point2.x - point1.x)* s,
			point1.y + (point2.y - point1.y)* s
			);

		if (!isOnTheLine(*intersection))
		{
			ALLOC_RELEASE(intersection);
			return nullptr;  // intersection not found on this line
		}

		if (!otherLine.isOnTheLine(*intersection)) 
		{
			ALLOC_RELEASE(intersection);
			return nullptr;  // intersection not found on this line
		}

		return intersection;
	}

	CollisionStatus Line2D::hasIntersections(const Circle2D& circle) const
	{
		sp_float distanceCenterToLine = std::ceilf(getDistance(circle.center));
		sp_float ray = std::ceilf(circle.ray);

		if (distanceCenterToLine > ray)
			return CollisionStatus::OUTSIDE;

		if (distanceCenterToLine < ray)
			return CollisionStatus::INSIDE;

		return CollisionStatus::INLINE;
	}

	sp_float Line2D::deltaX() const
	{
		return std::fabsf(point1.x - point2.x);
	}

	sp_float Line2D::deltaY() const
	{
		return std::fabsf(point1.y - point2.y);
	}

}