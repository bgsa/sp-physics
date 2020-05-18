#include "DOP18.h"

namespace NAMESPACE_PHYSICS
{

	DOP18::DOP18()
	{
		min[DOP18_AXIS_X] = min[DOP18_AXIS_Y] = min[DOP18_AXIS_Z] = -0.5f;
		max[DOP18_AXIS_X] = max[DOP18_AXIS_Y] = max[DOP18_AXIS_Z] = 0.5f;

		min[DOP18_AXIS_UP_LEFT] = min[DOP18_AXIS_UP_RIGHT] = min[DOP18_AXIS_UP_FRONT] 
			= min[DOP18_AXIS_UP_DEPTH] = min[DOP18_AXIS_LEFT_DEPTH] 
			= min[DOP18_AXIS_RIGHT_DEPTH] = -0.75f;

		max[DOP18_AXIS_UP_LEFT] = max[DOP18_AXIS_UP_RIGHT] = max[DOP18_AXIS_UP_FRONT]
			= max[DOP18_AXIS_UP_DEPTH] = max[DOP18_AXIS_LEFT_DEPTH]
			= max[DOP18_AXIS_RIGHT_DEPTH] = 0.75f;
	}

	Vec3 DOP18::centerOfBoundingVolume() const
	{
		return Vec3(
			(max[0] + min[0]) * 0.5f,
			(max[1] + min[1]) * 0.5f,
			(max[2] + min[2]) * 0.5f
		);
	}

	void DOP18::translate(const Vec3& translation)
	{
		min[DOP18_AXIS_X] += translation.x;
		max[DOP18_AXIS_X] += translation.x;

		min[DOP18_AXIS_Y] += translation.y;
		max[DOP18_AXIS_Y] += translation.y;

		min[DOP18_AXIS_Z] += translation.z;
		max[DOP18_AXIS_Z] += translation.z;

		min[DOP18_AXIS_UP_LEFT] += translation.x - translation.y;
		max[DOP18_AXIS_UP_LEFT] += translation.x - translation.y;

		min[DOP18_AXIS_UP_RIGHT] += translation.x + translation.y;
		max[DOP18_AXIS_UP_RIGHT] += translation.x + translation.y;

		min[DOP18_AXIS_UP_FRONT] += translation.y - translation.z;
		max[DOP18_AXIS_UP_FRONT] += translation.y - translation.z;

		min[DOP18_AXIS_UP_DEPTH] += translation.y + translation.z;
		max[DOP18_AXIS_UP_DEPTH] += translation.y + translation.z;

		min[DOP18_AXIS_LEFT_DEPTH] += translation.x - translation.z;
		max[DOP18_AXIS_LEFT_DEPTH] += translation.x - translation.z;

		min[DOP18_AXIS_RIGHT_DEPTH] += translation.x + translation.z;
		max[DOP18_AXIS_RIGHT_DEPTH] += translation.x + translation.z;
	}

	void DOP18::scale(const Vec3& factor) 
	{
		sp_float diffMinX = (min[DOP18_AXIS_X] * factor.x - min[DOP18_AXIS_X]);
		sp_float diffMinY = (min[DOP18_AXIS_Y] * factor.y - min[DOP18_AXIS_Y]);
		sp_float diffMinZ = (min[DOP18_AXIS_Z] * factor.z - min[DOP18_AXIS_Z]);

		sp_float diffMaxX = (max[DOP18_AXIS_X] * factor.x - max[DOP18_AXIS_X]);
		sp_float diffMaxY = (max[DOP18_AXIS_Y] * factor.y - max[DOP18_AXIS_Y]);
		sp_float diffMaxZ = (max[DOP18_AXIS_Z] * factor.z - max[DOP18_AXIS_Z]);

		min[DOP18_AXIS_X] += diffMinX;
		max[DOP18_AXIS_X] += diffMaxX;

		min[DOP18_AXIS_Y] += diffMinY;
		max[DOP18_AXIS_Y] += diffMaxY;

		min[DOP18_AXIS_Z] += diffMinZ;
		max[DOP18_AXIS_Z] += diffMaxZ;

		min[DOP18_AXIS_UP_LEFT] += diffMinX + diffMinY;
		max[DOP18_AXIS_UP_LEFT] += diffMaxX + diffMaxY;

		min[DOP18_AXIS_UP_RIGHT] += diffMinX + diffMinY;
		max[DOP18_AXIS_UP_RIGHT] += diffMaxX + diffMaxY;

		min[DOP18_AXIS_UP_FRONT] += diffMinY + diffMinZ;
		max[DOP18_AXIS_UP_FRONT] += diffMaxY + diffMaxZ;

		min[DOP18_AXIS_UP_DEPTH] += diffMinY + diffMinZ;
		max[DOP18_AXIS_UP_DEPTH] += diffMaxY + diffMaxZ;

		min[DOP18_AXIS_LEFT_DEPTH] += diffMinX + diffMinZ;
		max[DOP18_AXIS_LEFT_DEPTH] += diffMaxX + diffMaxZ;

		min[DOP18_AXIS_RIGHT_DEPTH] += diffMinX + diffMinZ;
		max[DOP18_AXIS_RIGHT_DEPTH] += diffMaxX + diffMaxZ;
	}

	void DOP18::rotate(const Vec3& angles) { }

	CollisionStatus DOP18::collisionStatus(const DOP18& kDop)
	{
		for (sp_int i = 0; i < DOP18_ORIENTATIONS; i++)
			if (min[i] > kDop.max[i] || max[i] < kDop.min[i])
				return CollisionStatus::OUTSIDE;

		/*
		for (sp_int i = 0; i < 3; i++)
			if (min[i] > kDop.max[i] || max[i] < kDop.min[i])
				return CollisionStatus::OUTSIDE;

		Vec3 center = centerOfBoundingVolume();
		Vec3 kDopCenter = kDop.centerOfBoundingVolume();

		sp_float distanceFromOriginXY = std::sqrtf(center.x * center.x + center.y * center.y);
		sp_float distanceFromOriginKDopXY = std::sqrtf(kDopCenter.x * kDopCenter.x + kDopCenter.y * kDopCenter.y);

		sp_float distanceFromOriginXZ = std::sqrtf(center.x * center.x + center.z * center.z);
		sp_float distanceFromOriginKDopXZ = std::sqrtf(kDopCenter.x * kDopCenter.x + kDopCenter.z * kDopCenter.z);

		sp_float distanceFromOriginYZ = std::sqrtf(center.y * center.y + center.z * center.z);
		sp_float distanceFromOriginKDopYZ = std::sqrtf(kDopCenter.y * kDopCenter.y + kDopCenter.z * kDopCenter.z);

		if (min[3] + distanceFromOriginXY > kDop.max[3] + distanceFromOriginKDopXY
			|| max[3] + distanceFromOriginXY < kDop.min[3] + distanceFromOriginKDopXY)  // up-left
			return CollisionStatus::OUTSIDE;

		if (min[4] + distanceFromOriginXY > kDop.max[4] + distanceFromOriginKDopXY
			|| max[4] + distanceFromOriginXY < kDop.min[4] + distanceFromOriginKDopXY) // down-right
			return CollisionStatus::OUTSIDE;

		if (min[5] + distanceFromOriginXZ > kDop.max[5] + distanceFromOriginKDopXZ
			|| max[5] + distanceFromOriginXZ < kDop.min[5] + distanceFromOriginKDopXZ) // up-front
			return CollisionStatus::OUTSIDE;

		if (min[6] + distanceFromOriginXZ > kDop.max[6] + distanceFromOriginXZ
			|| max[6] + distanceFromOriginKDopXZ < kDop.min[6] + distanceFromOriginKDopXZ) // down-depth
			return CollisionStatus::OUTSIDE;

		if (min[7] + distanceFromOriginYZ > kDop.max[7] + distanceFromOriginKDopYZ
			|| max[7] + distanceFromOriginYZ < kDop.min[7] + distanceFromOriginKDopYZ) // left-depth
			return CollisionStatus::OUTSIDE;

		if (min[8] + distanceFromOriginXZ > kDop.max[8] + distanceFromOriginKDopXZ
			|| max[8] + distanceFromOriginXZ < kDop.min[8] + distanceFromOriginKDopXZ) // right-front
			return CollisionStatus::OUTSIDE;
		*/

		return CollisionStatus::INSIDE;
	}

	Plane3D* DOP18::planes()
	{
		const Vec3* n = normals();

		Plane3D* result = sp_mem_new_array(Plane3D, 18) {
			Plane3D(Vec3(min[DOP18_AXIS_X], ZERO_FLOAT, ZERO_FLOAT), n[0]), // left
			Plane3D(Vec3(max[DOP18_AXIS_X], ZERO_FLOAT, ZERO_FLOAT), n[1]), // right

			Plane3D(Vec3(ZERO_FLOAT, max[DOP18_AXIS_Y], ZERO_FLOAT), n[2]), // up
			Plane3D(Vec3(ZERO_FLOAT, min[DOP18_AXIS_Y], ZERO_FLOAT), n[3]), // down

			Plane3D(Vec3(ZERO_FLOAT, ZERO_FLOAT, min[DOP18_AXIS_Z]), n[4]), // front
			Plane3D(Vec3(ZERO_FLOAT, ZERO_FLOAT, max[DOP18_AXIS_Z]), n[5]), // depth

			Plane3D(Vec3(min[DOP18_AXIS_UP_LEFT], ZERO_FLOAT, ZERO_FLOAT), n[6]), // up-left
			Plane3D(Vec3(max[DOP18_AXIS_UP_LEFT], ZERO_FLOAT, ZERO_FLOAT), n[7]), // down-right

			Plane3D(Vec3(max[DOP18_AXIS_UP_RIGHT], ZERO_FLOAT, ZERO_FLOAT), n[8]), // up-right
			Plane3D(Vec3(min[DOP18_AXIS_UP_RIGHT], ZERO_FLOAT, ZERO_FLOAT), n[9]), // down-left

			Plane3D(Vec3(ZERO_FLOAT, max[DOP18_AXIS_UP_FRONT], ZERO_FLOAT), n[10]), // up-front
			Plane3D(Vec3(ZERO_FLOAT, min[DOP18_AXIS_UP_FRONT], ZERO_FLOAT), n[11]), // down-depth

			Plane3D(Vec3(ZERO_FLOAT, max[DOP18_AXIS_UP_DEPTH], ZERO_FLOAT), n[12]), // up-depth
			Plane3D(Vec3(ZERO_FLOAT, min[DOP18_AXIS_UP_DEPTH], ZERO_FLOAT), n[13]), // down-front

			Plane3D(Vec3(min[DOP18_AXIS_LEFT_DEPTH], ZERO_FLOAT, ZERO_FLOAT), n[14]), // left-depth
			Plane3D(Vec3(max[DOP18_AXIS_LEFT_DEPTH], ZERO_FLOAT, ZERO_FLOAT), n[15]), // right-front

			Plane3D(Vec3(max[DOP18_AXIS_RIGHT_DEPTH], ZERO_FLOAT, ZERO_FLOAT), n[16]), // right-depth
			Plane3D(Vec3(min[DOP18_AXIS_RIGHT_DEPTH], ZERO_FLOAT, ZERO_FLOAT), n[17]), // left-front
		};

		return result;
	}

	void DOP18::fixTopDegeneration(const Plane3D* planes)
	{
		Line3D* line1 = planes[DOP18_PLANES_UP_FRONT_INDEX].findIntersection(planes[DOP18_PLANES_UP_LEFT_INDEX]);
		Line3D* line2 = planes[DOP18_PLANES_UP_DEPTH_INDEX].findIntersection(planes[DOP18_PLANES_UP_LEFT_INDEX]);
		Vec3* point = line1->findIntersection(*line2);

		if (point != NULL) // fix top max
			max[1] = point->y;

		ALLOC_RELEASE(line1);
	}

	void DOP18::fixBottomDegeneration(const Plane3D* planes)
	{
		Line3D* line1 = planes[DOP18_PLANES_DOWN_FRONT_INDEX].findIntersection(planes[DOP18_PLANES_DOWN_LEFT_INDEX]);
		Line3D* line2 = planes[DOP18_PLANES_DOWN_DEPTH_INDEX].findIntersection(planes[DOP18_PLANES_DOWN_LEFT_INDEX]);
		Vec3* point = line1->findIntersection(*line2);

		if (point != NULL) // fix bottom min
			min[1] = point->y;

		ALLOC_RELEASE(line1);
	}

	void DOP18::fixLeftDegeneration(const Plane3D* planes)
	{
		Line3D* line1 = planes[DOP18_PLANES_LEFT_FRONT_INDEX].findIntersection(planes[DOP18_PLANES_UP_LEFT_INDEX]);
		Line3D* line2 = planes[DOP18_PLANES_LEFT_DEPTH_INDEX].findIntersection(planes[DOP18_PLANES_UP_LEFT_INDEX]);
		Vec3* point = line1->findIntersection(*line2);

		if (point != NULL) // fix left min
			min[0] = point->x;

		ALLOC_RELEASE(line1);
	}

	void DOP18::fixRightDegeneration(const Plane3D* planes)
	{
		Line3D* line1 = planes[DOP18_PLANES_RIGHT_FRONT_INDEX].findIntersection(planes[DOP18_PLANES_UP_RIGHT_INDEX]);
		Line3D* line2 = planes[DOP18_PLANES_RIGHT_DEPTH_INDEX].findIntersection(planes[DOP18_PLANES_UP_RIGHT_INDEX]);
		Vec3* point = line1->findIntersection(*line2);

		if (point != NULL) // fix right max
			max[0] = point->x;

		ALLOC_RELEASE(line1);
	}

	void DOP18::fixFrontDegeneration(const Plane3D* planes)
	{
		Line3D* line1 = planes[DOP18_PLANES_DOWN_FRONT_INDEX].findIntersection(planes[DOP18_PLANES_LEFT_FRONT_INDEX]);
		Line3D* line2 = planes[DOP18_PLANES_DOWN_FRONT_INDEX].findIntersection(planes[DOP18_PLANES_RIGHT_FRONT_INDEX]);
		Vec3* point = line1->findIntersection(*line2);

		if (point != NULL) // fix front min
			min[2] = point->z;

		ALLOC_RELEASE(line1);
	}

	void DOP18::fixDepthDegeneration(const Plane3D* planes)
	{
		Line3D* line1 = planes[DOP18_PLANES_DOWN_DEPTH_INDEX].findIntersection(planes[DOP18_PLANES_LEFT_DEPTH_INDEX]);
		Line3D* line2 = planes[DOP18_PLANES_DOWN_DEPTH_INDEX].findIntersection(planes[DOP18_PLANES_RIGHT_DEPTH_INDEX]);
		Vec3* point = line1->findIntersection(*line2);

		if (point != NULL) // fix depth max
			max[2] = point->z;

		ALLOC_RELEASE(line1);
	}

	void DOP18::fixDegenerations()
	{
		Plane3D* p = planes();

		fixTopDegeneration(p);
		fixBottomDegeneration(p);
		fixLeftDegeneration(p);
		fixRightDegeneration(p);
		fixFrontDegeneration(p);
		fixDepthDegeneration(p);

		sp_mem_delete(p, Plane3D);
	}

	BoundingVolumeType DOP18::type() const
	{
		return BoundingVolumeType::DOP18;
	}

}