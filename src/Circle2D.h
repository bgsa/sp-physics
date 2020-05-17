#ifndef CIRCLE_HEADER
#define CIRCLE_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
	class Circle2D 
	{
	public:
		sp_float ray;
		Vec2 center;

		API_INTERFACE Circle2D();
		API_INTERFACE Circle2D(const Vec2& center, sp_float ray);
		API_INTERFACE Circle2D(const Vec2& point1, const Vec2& point2, const Vec2& point3);

		API_INTERFACE sp_float area() const;

		API_INTERFACE sp_float circumference() const;

		API_INTERFACE sp_float* calculatePoints(size_t& pointsCount) const;

		API_INTERFACE sp_float distance(const Vec2& point) const;

		API_INTERFACE CollisionStatus collisionStatus(const Vec2 &point) const;

		API_INTERFACE bool hasIntersection(const Circle2D& circle2) const;

		API_INTERFACE Vec2* findIntersection(const Circle2D& circle2) const;

	};

}

#endif // CIRCLE_HEADER