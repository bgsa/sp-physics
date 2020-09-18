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
		Vec3 ab, ca;

		diff(b, a, &ab);
		diff(c, a, &ca);

		Vec3 temp;
		cross(ab, ca, &temp);

		return length(temp) * HALF_FLOAT;
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
		/// Project the point (target parameter) in the triangle
		/// </summary>
		/// <param name="target">Arbitraty point</param>
		/// <param name="output">Projected point</param>
		/// <returns>Output parameter</returns>
		API_INTERFACE void project(const Vec3& target, Vec3* output) const;

		/* not working ....   taken from book collision detection 
		API_INTERFACE inline Vec3 closestPoint(const Vec3& target) const
		{
			Plane3D plane(point1, point2, point3);
			Ray ray(target, -plane.normalVector);

			Vec3 projectedPoint;
			plane.intersection(ray, &projectedPoint);

			// Check if P in vertex region outside A  
			const Vec3 ab = point2 - point1;  
			const Vec3 ac = point3 - point1;
			const Vec3 ap = projectedPoint - point1;
			
			const sp_float d1 = ab.dot(ap);
			const sp_float d2 = ac.dot(ap);
			
			if (d1 <= ZERO_FLOAT && d2 <= ZERO_FLOAT)
				return point1; // barycentric coordinates (1,0,0)  
						  
			// Check if P in vertex region outside B  
			const Vec3 bp = projectedPoint - point2;
			const sp_float d3 = ab.dot(bp);
			const sp_float d4 = ac.dot(bp);
			
			if (d3 >= ZERO_FLOAT && d4 <= d3)
				return point2; // barycentric coordinates (0,1,0)  
						  
			// Check if P in edge region of AB, if so return projection of P onto AB  
			const sp_float vc = d1*d4 - d3*d2;
			
			if (vc <= ZERO_FLOAT && d1 >= ZERO_FLOAT && d3 <= ZERO_FLOAT)
			{  
				const sp_float v = d1 / (d1 - d3);
				return point1 + ab * v; 	// barycentric coordinates (1-v,v,0)  
			}  
			
			// Check if P in vertex region outside C  
			const Vec3 cp = projectedPoint - point3;
			const sp_float d5 = ab.dot(cp);
			const sp_float d6 = ac.dot(cp);
			
			if (d6 >= ZERO_FLOAT && d5 <= d6)
				return point3; // barycentric coordinates (0,0,1)

			return projectedPoint;
		}
		*/

	};

}

#endif // TRIANGLE3D_HEADER