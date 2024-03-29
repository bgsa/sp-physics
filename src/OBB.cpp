#include "OBB.h"

namespace NAMESPACE_PHYSICS
{
	OBB::OBB(const Vec3& center)
	{
		this->center = center;
		this->halfWidth = Vec3(0.5f, 0.5f, 0.5f);
		this->orientation = Mat3Identity;
	}

	void OBB::translate(const Vec3& translation)
	{
		center += translation;
	}

	void OBB::scale(const Vec3& factor)
	{
	}

	void OBB::rotate(const Vec3& angles)
	{
	}

	CollisionStatus OBB::collisionStatus(const OBB& obb)
	{
		sp_float ra, rb;
		Mat3 R, AbsR; 
		
		// Compute rotation matrix expressing b in a�s coordinate frame 
		for (sp_int i = 0; i < MAT3_ROW_LENGTH; i++) 
			for (sp_int j = 0; j < MAT3_ROW_LENGTH; j++)
				R[i * MAT3_ROW_LENGTH + j] = orientation.axis(i).dot(obb.orientation.axis(j));

		// Compute translation vector t 
		Vec3 t = obb.center - center; 

		// Bring translation into a�s coordinate frame 
		t = Vec3(t.dot(orientation.xAxis()), t.dot(orientation.yAxis()), t.dot(orientation.zAxis()));

		// Compute common subexpressions. Add in an epsilon term to 
		// counteract arithmetic errors when two edges are parallel and 
		// their cross product is (near) null (see text for details) 
		for (sp_int i = 0; i < MAT3_LENGTH; i++)
				AbsR[i] = sp_abs(R[i]) + DefaultErrorMargin;  // + EPSILON
		
		// Test axes L = A0, L = A1, L = A2 
		for (sp_int i = 0; i < MAT3_ROW_LENGTH; i++)
		{ 
			ra = halfWidth[i]; 
			rb = obb.halfWidth[0] * AbsR[i * MAT3_ROW_LENGTH] 
				+ obb.halfWidth[1] * AbsR[i * MAT3_ROW_LENGTH + 1] 
				+ obb.halfWidth[2] * AbsR[i * MAT3_ROW_LENGTH + 2];
			
			if (std::fabs(t[i]) > ra + rb) 
				return CollisionStatus::OUTSIDE; 
		} 
		
		// Test axes L = B0, L = B1, L = B2 
		for (sp_int i = 0; i < MAT3_ROW_LENGTH; i++)
		{ 
			ra = halfWidth[0] * AbsR[i] 
				+ halfWidth[1] * AbsR[MAT3_ROW_LENGTH + i] 
				+ halfWidth[2] * AbsR[MAT3_TWO_ROW_LENGTH + i];
			rb = obb.halfWidth[i];
			
			if (sp_abs(t[0] * R[i] + t[1] * R[MAT3_ROW_LENGTH + i] + t[2] * R[MAT3_TWO_ROW_LENGTH + i]) > ra + rb)
				return CollisionStatus::OUTSIDE;
		} 
		
		// Test axis L = A0 x B0 
		ra = halfWidth[1] * AbsR[MAT3_TWO_ROW_LENGTH] + halfWidth[2] * AbsR[MAT3_ROW_LENGTH];
		rb = obb.halfWidth[1] * AbsR[2] + obb.halfWidth[2] * AbsR[1];
		
		if (sp_abs(t[2] * R[MAT3_ROW_LENGTH] - t[1] * R[MAT3_TWO_ROW_LENGTH]) > ra + rb)
			return CollisionStatus::OUTSIDE;
		
		// Test axis L = A0 x B1 
		ra = halfWidth[1] * AbsR[7] + halfWidth[2] * AbsR[4];
		rb = obb.halfWidth[0] * AbsR[2] + obb.halfWidth[2] * AbsR[0];
		
		if (sp_abs(t[2] * R[4] - t[1] * R[7]) > ra + rb)
			return CollisionStatus::OUTSIDE;
		
		// Test axis L = A0 x B2 
		ra = halfWidth[1] * AbsR[8] + halfWidth[2] * AbsR[5];
		rb = obb.halfWidth[0] * AbsR[1] + obb.halfWidth[1] * AbsR[0];
		
		if (sp_abs(t[2] * R[MAT3_ROW_LENGTH + 2] - t[1] * R[8]) > ra + rb)
			return CollisionStatus::OUTSIDE;
		
		// Test axis L = A1 x B0 
		ra = halfWidth[0] * AbsR[MAT3_TWO_ROW_LENGTH] + halfWidth[2] * AbsR[0];
		rb = obb.halfWidth[1] * AbsR[5] + obb.halfWidth[2] * AbsR[4];

		if (sp_abs(t[0] * R[MAT3_TWO_ROW_LENGTH] - t[2] * R[0]) > ra + rb)
			return CollisionStatus::OUTSIDE;
		
		// Test axis L = A1 x B1 
		ra = halfWidth[0] * AbsR[7] + halfWidth[2] * AbsR[1];
		rb = obb.halfWidth[0] * AbsR[5] + obb.halfWidth[2] * AbsR[MAT3_ROW_LENGTH];
		
		if (sp_abs(t[0] * R[7] - t[2] * R[1]) > ra + rb)
			return CollisionStatus::OUTSIDE;
		
		// Test axis L = A1 x B2 
		ra = halfWidth[0] * AbsR[8] + halfWidth[2] * AbsR[2];
		rb = obb.halfWidth[0] * AbsR[4] + obb.halfWidth[1] * AbsR[MAT3_ROW_LENGTH];
		
		if (sp_abs(t[0] * R[8] - t[2] * R[2]) > ra + rb)
			return CollisionStatus::OUTSIDE;
		
		// Test axis L = A2 x B0 
		ra = halfWidth[0] * AbsR[MAT3_ROW_LENGTH] + halfWidth[1] * AbsR[0];
		rb = obb.halfWidth[1] * AbsR[8] + obb.halfWidth[2] * AbsR[7];
		
		if (sp_abs(t[1] * R[0] - t[0] * R[MAT3_ROW_LENGTH]) > ra + rb)
			return CollisionStatus::OUTSIDE;
		
		// Test axis L = A2 x B1 
		ra = halfWidth[0] * AbsR[4] + halfWidth[1] * AbsR[1];
		rb = obb.halfWidth[0] * AbsR[8] + obb.halfWidth[2] * AbsR[MAT3_TWO_ROW_LENGTH];
		
		if (sp_abs(t[1] * R[1] - t[0] * R[4]) > ra + rb)
			return CollisionStatus::OUTSIDE;
		
		// Test axis L = A2 x B2 
		ra = halfWidth[0] * AbsR[5] + halfWidth[1] * AbsR[2];
		rb = obb.halfWidth[0] * AbsR[7] + obb.halfWidth[1] * AbsR[MAT3_TWO_ROW_LENGTH];
		
		if (sp_abs(t[1] * R[2] - t[0] * R[5]) > ra + rb)
			return CollisionStatus::OUTSIDE;
			
		return CollisionStatus::INSIDE; // Since no separating axis is found, the OBBs must be intersecting 
	}

	BoundingVolumeType OBB::type() const
	{
		return BoundingVolumeType::OBB;
	}

}