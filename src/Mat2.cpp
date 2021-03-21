#include "Mat2.h"

namespace NAMESPACE_PHYSICS
{

	Vec2 Mat2::xAxis() const
	{
		return Vec2(m11, m21);
	}

	Vec2 Mat2::yAxis() const
	{
		return Vec2(m12, m22);
	}

	Vec2 Mat2::primaryDiagonal() const
	{
		return Vec2(m11, m22);
	}

	Vec2 Mat2::secondaryDiagonal() const
	{
		return Vec2(m12, m21);
	}

	void Mat2::multiply(const Vec2& vector, Vec2& output) const
	{
		output.x = m11 * vector.x + m12 * vector.y;
		output.y = m21 * vector.x + m22 * vector.y;
	}

	AutoValueAutoVector2 Mat2::getAutovalueAndAutovector(const sp_ushort maxIteration) const
	{
		Mat2 matrix = *this;
		Vec2 autovector(1.0f, 1.0f);
		sp_float autovalue;

		for (sp_ushort iterationIndex = 0; iterationIndex < maxIteration; iterationIndex++)
		{
			Vec2 ax;
			matrix.multiply(autovector, ax);
			autovalue = ax.maximum();
			autovector = ax / autovalue;
		}

		return AutoValueAutoVector2{ autovalue, {autovector.x, autovector.y} };
	}

}