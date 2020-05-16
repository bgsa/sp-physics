#ifndef SP_TRANSFORM_HEADER
#define SP_TRANSFORM_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
	class SpTransform
	{
	public:
		Quat orientation;
		Vec3<sp_float> positionVector;
		Vec3<sp_float> scaleVector;

		/// <summary>
		/// Default construct
		/// </summary>
		API_INTERFACE SpTransform();

		API_INTERFACE SpTransform* scale(const Vec3<sp_float>& scaleVector);

		API_INTERFACE SpTransform* translate(const Vec3<sp_float>& translationVector);

		API_INTERFACE inline Mat4<sp_float> toMat4();
	};

}

#endif // SP_TRANSFORM_HEADER
