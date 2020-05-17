#ifndef SP_TRANSFORM_HEADER
#define SP_TRANSFORM_HEADER

#include "SpectrumPhysics.h"
#include "Vec3.h"
#include "Quat.h"

namespace NAMESPACE_PHYSICS
{
	class SpTransform
	{
	public:
		Quat orientation;
		Vec3 position;
		Vec3 scaleVector;

		/// <summary>
		/// Default construct
		/// </summary>
		API_INTERFACE SpTransform();

		API_INTERFACE SpTransform* scale(const Vec3& scaleVector);

		API_INTERFACE SpTransform* scale(const sp_float factor);

		API_INTERFACE SpTransform* translate(const Vec3& translationVector);

		API_INTERFACE SpTransform* translate(const sp_float x, const sp_float y, const sp_float z);

		API_INTERFACE inline Mat4 toMat4();
	};

}

#endif // SP_TRANSFORM_HEADER
