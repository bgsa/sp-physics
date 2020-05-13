#ifndef OBB_HEADER
#define OBB_HEADER

#include "Vec3.h"
#include "BoundingVolume.h"

namespace NAMESPACE_PHYSICS
{
	class OBB
		: public BoundingVolumeOBB
	{
	public:
		Vec3f center;
		Vec3f halfWidth;
		Mat3f orientation;

		/// <summary>
		/// Build a new unit OBB with width, height and depth 1.0 and orientation aligned to axis located on coordinates (0,0,0) 
		/// </summary>
		API_INTERFACE OBB(const Vec3f& center = Vec3f(0.0f));

		/// <summary>
		/// Not Implemented !!
		/// </summary>
		API_INTERFACE Vec3f centerOfBoundingVolume() const override
		{
			return Vec3f(0.0f); // Not Implemented
		}

		/// <summary>
		/// Translate the OBB
		/// </summary>
		API_INTERFACE OBB* translate(float xAxis, float yAxis, float zAxis) override;

		/// <summary>
		/// Rotate the OBB
		/// </summary>
		API_INTERFACE OBB* rotate(float angleInRadians, float xAxis, float yAxis, float zAxis) override;

		/// <summary>
		/// Scale the OBB
		/// </summary>
		API_INTERFACE OBB* scale(float xAxis, float yAxis, float zAxis) override;

		/// <summary>
		/// Get the model view matrix of OBB
		/// </summary>
		API_INTERFACE Mat3f modelView() override;

		///<summary>
		/// Check whether the OBBs are in contact each other
		///</summary>
		API_INTERFACE CollisionStatus collisionStatus(const OBB& aabb);

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
