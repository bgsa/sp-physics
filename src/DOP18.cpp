#include "DOP18.h"

namespace NAMESPACE_PHYSICS
{
	#define DOP18_PLANES_LEFT_INDEX 0
	#define DOP18_PLANES_RIGHT_INDEX 1
	#define DOP18_PLANES_UP_INDEX 2
	#define DOP18_PLANES_DOWN_INDEX 3
	#define DOP18_PLANES_FRONT_INDEX 4
	#define DOP18_PLANES_DEPTH_INDEX 5
	#define DOP18_PLANES_UP_LEFT_INDEX 6
	#define DOP18_PLANES_DOWN_RIGHT_INDEX 7
	#define DOP18_PLANES_UP_RIGHT_INDEX 8
	#define DOP18_PLANES_DOWN_LEFT_INDEX 9
	#define DOP18_PLANES_UP_FRONT_INDEX 10
	#define DOP18_PLANES_DOWN_DEPTH_INDEX 11
	#define DOP18_PLANES_UP_DEPTH_INDEX 12
	#define DOP18_PLANES_DOWN_FRONT_INDEX 13
	#define DOP18_PLANES_LEFT_DEPTH_INDEX 14
	#define DOP18_PLANES_RIGHT_FRONT_INDEX 15
	#define DOP18_PLANES_RIGHT_DEPTH_INDEX 16
	#define DOP18_PLANES_LEFT_FRONT_INDEX 17

	DOP18::DOP18()
	{
		min[0] = min[1] = min[2] = -0.5f;
		max[0] = max[1] = max[2] = 0.5f;
		min[3] = min[4] = min[5] = min[6] = min[7] = min[8] = -0.375f;
		max[3] = max[4] = max[5] = max[6] = max[7] = max[8] = 0.375f;
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
		min[0] += translation.x;
	}

	void DOP18::scale(const Vec3& factor)
	{

	}

	void DOP18::rotate(const Vec3& angles) { }

	CollisionStatus DOP18::collisionStatus(const DOP18& kDop)
	{
		// check aligned-axis orientation
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

		return CollisionStatus::INSIDE;
	}

	Plane3D* DOP18::planes()
	{
		const Vec3* n = normals();
		const Vec3 center = centerOfBoundingVolume();

		Plane3D* result = ALLOC_NEW_ARRAY(Plane3D, 18) {
			Plane3D(Vec3(min[0], center.y, center.z), n[0]), // left
			Plane3D(Vec3(max[0], center.y, center.z), n[1]), // right

			Plane3D(Vec3(center.x, max[1], center.z), n[2]), // up
			Plane3D(Vec3(center.x, min[1], center.z), n[3]), // down

			Plane3D(Vec3(center.x, center.y, min[2]), n[4]), // front
			Plane3D(Vec3(center.x, center.y, max[2]), n[5]), // depth

			Plane3D(Vec3(center.x + min[3], center.y + max[3], center.z), n[6]), // up-left
			Plane3D(Vec3(center.x + max[3], center.y + min[3], center.z), n[7]), // down-right

			Plane3D(Vec3(center.x + max[4], center.y + max[4], center.z), n[8]), // up-right
			Plane3D(Vec3(center.x + min[4], center.y + min[4], center.z), n[9]), // down-left

			Plane3D(Vec3(center.x, center.y + max[5], center.z + min[5]), n[10]), // up-front
			Plane3D(Vec3(center.x, center.y + min[5], center.z + max[5]), n[11]), // down-depth

			Plane3D(Vec3(center.x, center.y + max[6], center.z + max[6]), n[12]), // up-depth
			Plane3D(Vec3(center.x, center.y + min[6], center.z + min[6]), n[13]), // down-front

			Plane3D(Vec3(center.x + min[7], center.y, center.z + max[7]), n[14]), // left-depth
			Plane3D(Vec3(center.x + max[7], center.y, center.z + min[7]), n[15]), // right-front

			Plane3D(Vec3(center.x + max[8], center.y, center.z + max[8]), n[16]), // right-depth
			Plane3D(Vec3(center.x + min[8], center.y, center.z + min[8]), n[17]), // left-front
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

		ALLOC_RELEASE(p);
	}

	BoundingVolumeType DOP18::type() const
	{
		return BoundingVolumeType::DOP18;
	}

	#undef DOP18_PLANES_LEFT_INDEX
	#undef DOP18_PLANES_RIGHT_INDEX
	#undef DOP18_PLANES_UP_INDEX
	#undef DOP18_PLANES_DOWN_INDEX
	#undef DOP18_PLANES_FRONT_INDEX
	#undef DOP18_PLANES_DEPTH_INDEX
	#undef DOP18_PLANES_UP_LEFT_INDEX
	#undef DOP18_PLANES_DOWN_RIGHT_INDEX
	#undef DOP18_PLANES_UP_RIGHT_INDEX
	#undef DOP18_PLANES_DOWN_LEFT_INDEX
	#undef DOP18_PLANES_UP_FRONT_INDEX
	#undef DOP18_PLANES_DOWN_DEPTH_INDEX
	#undef DOP18_PLANES_UP_DEPTH_INDEX
	#undef DOP18_PLANES_DOWN_FRONT_INDEX
	#undef DOP18_PLANES_LEFT_DEPTH_INDEX
	#undef DOP18_PLANES_RIGHT_FRONT_INDEX
	#undef DOP18_PLANES_RIGHT_DEPTH_INDEX
	#undef DOP18_PLANES_LEFT_FRONT_INDEX
}