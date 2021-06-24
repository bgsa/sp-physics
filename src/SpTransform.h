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

		API_INTERFACE inline void transform(const Vec3& vertex, Vec3& output) const
		{
			Vec3 scaledVertex;
			multiply(scaleVector, vertex, &scaledVertex); // scale
			rotate(orientation, scaledVertex, output); // rotate
			add(position, output, output); // translate
		}

		API_INTERFACE inline Mat4 toMat4(const Vec3& initialPosition = Vec3Zeros) const
		{
			Mat4 scale, translation1, translation2;
			NAMESPACE_PHYSICS::createScale(scaleVector, scale);
			NAMESPACE_PHYSICS::createTranslate(position, translation1);
			NAMESPACE_PHYSICS::createTranslate(-initialPosition, translation2);

			Mat4 rotation, temp1, temp2;;
			mat4(orientation, rotation);

			scale.multiply(translation2, temp1);

			rotation.multiply(temp1, temp2);

			translation1.multiply(temp2, temp1);

			return temp1;
		}

		/// <summary>
		/// Reset all values to default
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline void reset()
		{
			identity(orientation);

			position = Vec3Zeros;
			scaleVector = Vec3Ones;
		}

	};

}

#endif // SP_TRANSFORM_HEADER
