#ifndef OBB_HEADER
#define OBB_HEADER

#include "SpectrumPhysics.h"
#include "BoundingVolume.h"
#include "DetailedCollisionStatus.h"

namespace NAMESPACE_PHYSICS
{
	class OBB
		: public BoundingVolume
	{
	public:
		Vec3 center;
		Vec3 halfWidth;
		Mat3 orientation;

		/// <summary>
		/// Build a new unit OBB with width, height and depth 1.0 and orientation aligned to axis located on coordinates (0,0,0) 
		/// </summary>
		API_INTERFACE OBB(const Vec3& center = Vec3(0.0f));

		/// <summary>
		/// Not Implemented !!
		/// </summary>
		API_INTERFACE Vec3 centerOfBoundingVolume() const override
		{
			return Vec3(0.0f); // Not Implemented
		}

		/// <summary>
		/// Translate the bounding volume
		/// </summary>
		API_INTERFACE void translate(const Vec3& translation) override;

		/// <summary>
		/// Scale the bounding volume (only in X, Y and Z)
		/// </summary>
		API_INTERFACE void scale(const Vec3& factor) override;

		/// <summary>
		/// Rotate the bounding volume
		/// </summary>
		API_INTERFACE void rotate(const Vec3& angles) override;

		///<summary>
		/// Check whether the OBBs are in contact each other
		///</summary>
		API_INTERFACE CollisionStatus collisionStatus(const OBB& aabb);

		/// <summary>
		/// Returns the type og bounding volume
		/// </summary>
		API_INTERFACE BoundingVolumeType OBB::type() const;

		///<summary>
		/// Releases all allocated resources
		///</summary>
		API_INTERFACE virtual void dispose() override
		{
		}

		API_INTERFACE virtual const sp_char* toString() override
		{
			return "OBB Bounding Volume";
		}

		// TODO: verificar colisao com AABB

		// TODO: verificar colisao com esferas
		
	};
}

#endif // OBB_HEADER
