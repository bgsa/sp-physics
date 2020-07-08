#ifndef SP_INERTIA_TENSOR_HEADER
#define SP_INERTIA_TENSOR_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
	class SpInertiaTensor
	{		
	public:
		
		/// <summary>
		/// Build the inertial tensor from particles with the same mass for all of them
		/// </summary>
		API_INTERFACE static inline Mat3 buildInertialTensor(const Vec3* vertexes, const sp_uint length, const sp_float mass)
		{
			sp_float m00 = ZERO_FLOAT;
			sp_float m11 = ZERO_FLOAT;
			sp_float m22 = ZERO_FLOAT;
			sp_float m10 = ZERO_FLOAT;
			sp_float m20 = ZERO_FLOAT;
			sp_float m21 = ZERO_FLOAT;

			for (sp_uint i = 0; i < length; i++)
			{
				const Vec3 particle = vertexes[i];

				m00 += sp_pow2(particle.y) + sp_pow2(particle.z);
				m11 += sp_pow2(particle.x) + sp_pow2(particle.z);
				m22 += sp_pow2(particle.x) + sp_pow2(particle.y);

				m10 += particle.x * particle.y;
				m20 += particle.x * particle.z;
				m21 += particle.y * particle.z;
			}

			m00 *= mass;
			m11 *= mass;
			m22 *= mass;
			m10 *= mass;
			m20 *= mass;
			m21 *= mass;

			return Mat3(
				m00, -m10, -m20,
				-m10, m11, -m21,
				-m20, -m21, m22
			);
		}

		/// <summary>
		/// Build the inertial tensor for Spherical object
		/// </summary>
		API_INTERFACE static inline Mat3 sphere(const sp_float ray, const sp_float mass)
		{
#define TWO_OVER_FIVE (0.4f)

			const sp_float value = TWO_OVER_FIVE * mass * ray * ray;

			return Mat3(
				value, 0.0f, 0.0f,
				0.0f, value, 0.0f,
				0.0f, 0.0f, value
			);

#undef TWO_OVER_FIVE
		}

	};

}

#endif // SP_INERTIA_TENSOR_HEADER