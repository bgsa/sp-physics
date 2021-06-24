#ifndef SP_FACE_HEADER
#define SP_FACE_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
	class SpFace
	{
	public:
		Vec3 vertex1;
		Vec3 vertex2;
		Vec3 vertex3;
		Vec3 normal;

		///<summary>
		///Default constructur
		///</summary>
		API_INTERFACE inline SpFace()
		{
		}

		///<summary>
		///Constructor face vertexes
		///</summary>
		API_INTERFACE inline SpFace(const Vec3& vertex1, const Vec3& vertex2, const Vec3& vertex3)
		{
			this->vertex1 = vertex1;
			this->vertex2 = vertex2;
			this->vertex3 = vertex3;
			
			Vec3 v12, v13;
			diff(vertex2, vertex1, v12);
			diff(vertex3, vertex1, v13);
			cross(v12, v13, normal);
			normalize(normal);
		}

		/// <summary>
		/// Get a vertex by index
		/// </summary>
		API_INTERFACE inline Vec3& operator[](const sp_int index)
		{
			sp_assert(index >= 0 && index <= 2, "InvalidArgumentException");
			return ((Vec3*)this)[index];
		}

		/// <summary>
		/// Get a vertex by index
		/// </summary>
		API_INTERFACE Vec3& operator[](const sp_int index) const
		{
			sp_assert(index >= 0 && index <= 2, "InvalidArgumentException");
			return ((Vec3*)this)[index];
		}

		/// <summary>
		/// Get a vertex by index
		/// </summary>
		API_INTERFACE inline Vec3& operator[](const sp_uint index)
		{
			sp_assert(index >= 0 && index <= 2, "InvalidArgumentException");
			return ((Vec3*)this)[index];
		}

		/// <summary>
		/// Get a vertex by index
		/// </summary>
		API_INTERFACE Vec3& operator[](const sp_uint index) const
		{
			sp_assert(index >= 0 && index <= 2, "InvalidArgumentException");
			return ((Vec3*)this)[index];
		}

#ifdef ENV_64BITS

		/// <summary>
		/// Get a vertex by index
		/// </summary>
		API_INTERFACE Vec3& operator[](const sp_size index) const
		{
			sp_assert(index >= 0 && index <= 2, "InvalidArgumentException");
			return ((Vec3*)this)[index];
		}

#endif

	};

}

#endif // AABB_HEADER