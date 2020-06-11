#ifndef LINE_2D_HEADER
#define LINE_2D_HEADER

#include "SpectrumPhysics.h"
#include "Orientation.h"
#include "Circle2D.h"
#include "SystemOfLinearEquations.h"
#include "DetailedCollisionStatus.h"

namespace NAMESPACE_PHYSICS 
{
	class Line2D
	{
	public:
		Vec2 point1;
		Vec2 point2;

		API_INTERFACE Line2D();
		API_INTERFACE Line2D(const Vec2& point1, const Vec2& point2);
		API_INTERFACE Line2D(sp_float* point1, sp_float* point2);

		/// <summary>
		/// Get the angle of the line
		/// </summary>
		API_INTERFACE sp_float angle() const;

		/// <summary>
		/// Get the slope/inclination of the line
		/// </summary>
		API_INTERFACE sp_float slope() const;
				
		/// <summary>
		/// Get the line equation in 2 points for 2D space
		/// The result is a vector with 2 components (m, b)
		/// Parametric line equation: "y = mx + b"
		/// </summary>
		API_INTERFACE Vec2 getParametricEquation() const;

		/// <summary>
		/// Get the line equation in 2 points for 2D space
		/// The result is a vector with 3 components (a, b, c)
		/// Line equation: "ax + bx + c = 0"
		/// </summary>
		API_INTERFACE  Vec3 equation() const
		{
			return SystemOfLinearEquations::getLineEquation(point1, point2);
		}
		
		/// <summary>
		/// Convert line to directional ray
		/// <summary>
		API_INTERFACE Vec2 toRay();

		/// <summary>
		/// Indicate whether the point is ON the vector
		/// It means the point is a point on this line
		/// </summary>
		API_INTERFACE sp_bool isOnTheLine(const Vec2& point) const;

		/// <summary>
		/// Indicate whether the point is on the left of the vector/line
		/// </summary>
		API_INTERFACE sp_bool isOnTheLeft(const Vec2& point) const;

		/// <summary>
		/// Indicate whether the point is on the right of the vector/line
		/// </summary>
		API_INTERFACE sp_bool isOnTheRight(const Vec2& point) const;

		/// <summary>
		/// Indicate whether the point is on the left, right or is a point of the line line/vector
		/// </summary>
		API_INTERFACE Orientation getOrientation(const Vec2& point) const;

		/// <summary>
		/// Get te distance from this line to the point
		/// </summary>
		API_INTERFACE sp_float getDistance(Vec2 point) const;

		/// <summary>
		/// Find the point of intersection of the line and the othr line
		/// It can return NULL if there is no intersection
		/// </summary>
		API_INTERFACE Vec2* findIntersection(const Line2D& otherLine) const;

		/// <summary>
		/// Check the line has intersection with the circle
		/// </summary>
		API_INTERFACE CollisionStatus hasIntersections(const Circle2D& circle) const;

		/// <summary>
		/// Get the delta X - diference max X - min X
		/// </summary>
		API_INTERFACE sp_float deltaX() const;

		/// <summary>
		/// Get the delta Y - diference max Y - min Y
		/// </summary>
		API_INTERFACE sp_float deltaY() const;
	};

}

#endif // LINE_2D_HEADER