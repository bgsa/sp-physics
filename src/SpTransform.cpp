#include "SpTransform.h"

namespace NAMESPACE_PHYSICS
{
	SpTransform::SpTransform()
	{
		orientation = Quat::identity();
		position = Vec3(ZERO_FLOAT);
		scaleVector = Vec3(ONE_FLOAT);
	}

	SpTransform* SpTransform::scale(const Vec3& scaleVector)
	{
		this->scaleVector.x *= scaleVector.x;
		this->scaleVector.y *= scaleVector.y;
		this->scaleVector.z *= scaleVector.z;
		return this;
	}

	SpTransform* SpTransform::scale(const sp_float factor)
	{
		scaleVector *= factor;
		return this;
	}

	SpTransform* SpTransform::translate(const Vec3& translationVector)
	{
		position += translationVector;
		return this;
	}

	SpTransform* SpTransform::translate(const sp_float x, const sp_float y, const sp_float z)
	{
		position.x += x;
		position.y += y;
		position.z += z;
		return this;
	}

	Mat4 SpTransform::toMat4() const
	{
		return Mat4::createTranslate(position)
			* orientation.toMat4()
			* Mat4::createScale(scaleVector);
	}
}
