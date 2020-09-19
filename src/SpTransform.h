#ifndef SP_TRANSFORM_HEADER
#define SP_TRANSFORM_HEADER

#include "SpectrumPhysics.h"
#include "Vec3.h"
#include "Quat.h"

namespace NAMESPACE_PHYSICS
{

#ifdef AVX_ENABLED

#define sp_transform_transform(vec3_simd)

#endif


	class SpTransform
	{
	public:
		Quat orientation;
		Vec3 position;
		Vec3 scaleVector;

		/// <summary>
		/// Default construct
		/// </summary>
		API_INTERFACE inline SpTransform()
		{
			reset();
		}

		API_INTERFACE inline SpTransform* scale(const Vec3& scaleVector)
		{
			this->scaleVector.x *= scaleVector.x;
			this->scaleVector.y *= scaleVector.y;
			this->scaleVector.z *= scaleVector.z;
			return this;
		}

		API_INTERFACE inline SpTransform* scale(const sp_float factor)
		{
			scaleVector *= factor;
			return this;
		}

		API_INTERFACE inline SpTransform* translate(const Vec3& translationVector)
		{
			position += translationVector;
			return this;
		}

		API_INTERFACE inline SpTransform* translate(const sp_float x, const sp_float y, const sp_float z)
		{
			position.x += x;
			position.y += y;
			position.z += z;
			return this;
		}

		API_INTERFACE inline void transform(const Vec3& vertex, Vec3* output) const
		{
			Vec3 scaled;
			multiply(scaleVector, vertex, &scaled);
			rotateAndTranslate(orientation, scaled, position, output);
		}

		API_INTERFACE inline Mat4 toMat4(const Vec3& initialPosition = Vec3(ZERO_FLOAT)) const
		{
			return Mat4::createTranslate(position)
				* orientation.toMat4()
				* Mat4::createScale(scaleVector)
				* Mat4::createTranslate(-initialPosition);
		}

		/// <summary>
		/// Reset all values to default
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline void reset()
		{
			orientation = Quat::identity();
			position = Vec3(ZERO_FLOAT);
			scaleVector = Vec3(ONE_FLOAT);
		}

	};

}

#endif // SP_TRANSFORM_HEADER
