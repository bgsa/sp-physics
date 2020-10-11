#ifndef TRIANGLE3D_HEADER
#define TRIANGLE3D_HEADER

#include "SpectrumPhysics.h"
#include "Line3D.h"
#include "Ray.h"

namespace NAMESPACE_PHYSICS
{

	/// <summary>
	/// Compute the area of triangle ABC
	/// </summary>
	/// <param name="a">Point A</param>
	/// <param name="b">Point B</param>
	/// <param name="c">Point C</param>
	/// <returns>Total area</returns>
	API_INTERFACE inline sp_float area(const Vec3& a, const Vec3& b, const Vec3& c)
	{
#ifdef AVX_ENABLED
		const __m128 a_simd = sp_vec3_convert_simd(a);
		const __m128 b_simd = sp_vec3_convert_simd(b);
		const __m128 c_simd = sp_vec3_convert_simd(c);
		__m128 output_simd;

		sp_triangle3D_area_simd(a_simd, b_simd, c_simd, output_simd);

		return output_simd.m128_f32[0];
#else
		Vec3 ab, ca;

		diff(b, a, &ab);
		diff(c, a, &ca);

		Vec3 temp;
		cross(ab, ca, &temp);

		return length(temp) * HALF_FLOAT;
#endif
	}

	class Triangle3D
	{
	public:
		Vec3 point1;
		Vec3 point2;
		Vec3 point3;

		API_INTERFACE inline Triangle3D() { }

		API_INTERFACE inline Triangle3D(const Vec3& point1, const Vec3& point2, const Vec3& point3)
		{
			this->point1 = point1;
			this->point2 = point2;
			this->point3 = point3;
		}

		API_INTERFACE inline Triangle3D(sp_float* point1, sp_float* point2, sp_float* point3)
		{
			this->point1 = Vec3(point1[0], point1[1], point1[2]);
			this->point2 = Vec3(point2[0], point2[1], point2[2]);
			this->point3 = Vec3(point3[0], point3[1], point3[2]);
		}

		API_INTERFACE inline void normalFace(Vec3* output) const
		{
			normal(point1, point2, point3, output);
		}

		API_INTERFACE inline sp_float area() const
		{
			return NAMESPACE_PHYSICS::area(point1, point2, point3);
		}

		API_INTERFACE inline void barycentric(const Vec3& point, Vec3* output) const
		{
			const Vec3 v0 = point2 - point1;
			const Vec3 v1 = point3 - point1;
			const Vec3 v2 = point - point1;

			const sp_float d00 = v0.dot(v0);
			const sp_float d01 = v0.dot(v1);
			const sp_float d11 = v1.dot(v1);
			const sp_float d20 = v2.dot(v0);
			const sp_float d21 = v2.dot(v1);

			const sp_float denom = 1.0f / (d00 * d11 - d01 * d01);

			output[0].y = (d11 * d20 - d01 * d21) * denom;
			output[0].z = (d00 * d21 - d01 * d20) * denom;
			output[0].x = 1.0f - output->y - output->z;

			sp_assert(isCloseEnough(output->x + output->y + output->z, ONE_FLOAT), "InvalidOperationException");
		}

		/// <summary>
		/// Get the edges of this plane
		/// </summary>
		/// <param name="line"></param>
		/// <returns>void</returns>
		API_INTERFACE void convert(Line3D* lines) const;

		/// <summary>
		/// Check if the target is Inside the triangle/face
		/// The target can be arbitrary. The point is projected in triangle
		/// </summary>
		/// <param name="target">Arbitrary point</param>
		/// <param name="_epsilon">Error margin</param>
		/// <returns>True if the point relies on triangle</returns>
		API_INTERFACE sp_bool isInside(const Vec3& target, const sp_float _epsilon = DefaultErrorMargin) const;

		/// <summary>
		/// Find the closest point in triangle given a target point
		/// </summary>
		/// <param name="target">Target point</param>
		/// <param name="output">Closest point</param>
		/// <returns>void</returns>
		API_INTERFACE inline void closestPoint(const Vec3& target, Vec3* output) const
		{
			// Check if Target in region outside A  
			const Vec3 ab = point2 - point1;  
			const Vec3 ac = point3 - point1;
			const Vec3 ap = target - point1;
			
			const sp_float d1 = ab.dot(ap);
			const sp_float d2 = ac.dot(ap);
			
			if (d1 <= ZERO_FLOAT && d2 <= ZERO_FLOAT)
			{
				output[0] = point1; // barycentric coordinates (1,0,0)  
				return;
			}

			// Check if Target in vertex region outside B  
			const Vec3 bp = target - point2;
			const sp_float d3 = ab.dot(bp);
			const sp_float d4 = ac.dot(bp);
			if (d3 >= ZERO_FLOAT && d4 <= d3)
			{
				output[0] = point2; // barycentric coordinates (0,1,0)  
				return;
			}

			// Check if Target in edge region of AB, if so return projection of P onto AB  
			const sp_float vc = d1*d4 - d3*d2;
			if (vc <= ZERO_FLOAT && d1 >= ZERO_FLOAT && d3 <= ZERO_FLOAT)
			{  
				const sp_float v = d1 / (d1 - d3);
				output[0] = point1 + ab * v; 	// barycentric coordinates (1-v,v,0)  
				return;
			}  
			
			// Check if Target in vertex region outside C  
			const Vec3 cp = target - point3;
			const sp_float d5 = ab.dot(cp);
			const sp_float d6 = ac.dot(cp);
			if (d6 >= ZERO_FLOAT && d5 <= d6)
			{
				output[0] = point3; // barycentric coordinates (0,0,1)
				return;
			}

			// Check if Target in edge region of AC, if so return projection of Target onto AC
			const sp_float vb = d5 * d2 - d1 * d6;
			if (vb <= ZERO_FLOAT && d2 >= ZERO_FLOAT && d6 <= ZERO_FLOAT) 
			{  
				const sp_float w = d2 / (d2 - d6);
				output[0] = point1 + ac * w; // barycentric coordinates (1-w,0,w) 
				return;
			}  
			
			// Check if Target in edge region of BC, if so return projection of P onto BC  
			const sp_float va = d3 * d6 - d5 * d4;
			if (va <= ZERO_FLOAT && (d4 - d3) >= ZERO_FLOAT && (d5 - d6) >= ZERO_FLOAT) 
			{  
				const sp_float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
				output[0] = point2 + (point3 - point2) * w; // barycentric coordinates (0,1-w,w)  
				return;
			}  
			
			// Target inside face region. Compute Q through its barycentric coordinates (u,v,w)  
			const sp_float denom = ONE_FLOAT / (va + vb + vc);  
			const sp_float v = vb * denom;
			const sp_float w = vc * denom;
			
			output[0] = point1 + ab * v + ac * w; // = u*a + v*b + w*c, u = va * denom = 1.0f - v - w 
		}

		/// <summary>
		/// Find the distance from this triangle to target
		/// </summary>
		/// <param name="target">Target point</param>
		/// <param name="closest">Return closest point</param>
		/// <returns>Distance</returns>
		API_INTERFACE inline sp_float distance(const Vec3& target, Vec3* closest) const
		{
			closestPoint(target, closest);
			return NAMESPACE_PHYSICS::distance(*closest, target);
		}

	};

	/// <summary>
	/// Centroid or Midpoint of the triangle (3 points)
	/// </summary>
	/// <param name="point1">First point</param>
	/// <param name="point2">Second point</param>
	/// <param name="point3">Third point</param>
	/// <param name="output">Midpoint output</param>
	/// <returns>void</returns>
	API_INTERFACE inline void midpoint(const Vec3& point1, const Vec3& point2, const Vec3& point3, Vec3* output)
	{
		output->x = (point1.x + point2.x + point3.x) * 0.3333333f;
		output->y = (point1.y + point2.y + point3.y) * 0.3333333f;
		output->z = (point1.z + point2.z + point3.z) * 0.3333333f;
	}

}

#endif // TRIANGLE3D_HEADER