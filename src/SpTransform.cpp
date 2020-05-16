#include "SpTransform.h"

namespace NAMESPACE_PHYSICS
{
	SpTransform::SpTransform()
	{
		orientation = Quat::identity();
		positionVector = Vec3<sp_float>(ZERO_FLOAT);
		scaleVector = Vec3<sp_float>(ZERO_FLOAT);
	}

	SpTransform* SpTransform::scale(const Vec3<sp_float>& scaleVector)
	{
		this->scaleVector += scaleVector;
		return this;
	}

	SpTransform* SpTransform::translate(const Vec3<sp_float>& translationVector)
	{
		positionVector += translationVector;
		return this;
	}

	Mat4f SpTransform::toMat4()
	{
		return Mat4f::createTranslate(positionVector)
			* orientation.toMat4()
			* Mat4f::createScale(scaleVector);
	}
}
