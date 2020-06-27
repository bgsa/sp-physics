#include "DOP18.h"

namespace NAMESPACE_PHYSICS
{

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

		min[DOP18_AXIS_UP_FRONT] += translation.y + translation.z;
		max[DOP18_AXIS_UP_FRONT] += translation.y + translation.z;

		min[DOP18_AXIS_UP_DEPTH] += translation.y - translation.z;
		max[DOP18_AXIS_UP_DEPTH] += translation.y - translation.z;

		min[DOP18_AXIS_LEFT_DEPTH] += translation.x + translation.z;
		max[DOP18_AXIS_LEFT_DEPTH] += translation.x + translation.z;

		min[DOP18_AXIS_RIGHT_DEPTH] += translation.x - translation.z;
		max[DOP18_AXIS_RIGHT_DEPTH] += translation.x - translation.z;
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

	CollisionStatus DOP18::collisionStatus(const DOP18& kDop) const
	{
		if (min[0] > kDop.max[0] || max[0] < kDop.min[0])
			return CollisionStatus::OUTSIDE;

		if (min[1] > kDop.max[1] || max[1] < kDop.min[1])
			return CollisionStatus::OUTSIDE;

		if (min[2] > kDop.max[2] || max[2] < kDop.min[2])
			return CollisionStatus::OUTSIDE;

		if (min[3] > kDop.max[3] || max[3] < kDop.min[3])
			return CollisionStatus::OUTSIDE;

		if (min[4] > kDop.max[4] || max[4] < kDop.min[4])
			return CollisionStatus::OUTSIDE;

		if (min[5] > kDop.max[5] || max[5] < kDop.min[5])
			return CollisionStatus::OUTSIDE;

		if (min[6] > kDop.max[6] || max[6] < kDop.min[6])
			return CollisionStatus::OUTSIDE;

		if (min[7] > kDop.max[7] || max[7] < kDop.min[7])
			return CollisionStatus::OUTSIDE;

		if (min[8] > kDop.max[8] || max[8] < kDop.min[8])
			return CollisionStatus::OUTSIDE;

		return CollisionStatus::INSIDE;
	}

	CollisionStatus DOP18::collisionStatus(const Plane3D& plane) const
	{	
		Vec3 pointOnThePlane = plane.closestPointOnThePlane(centerOfBoundingVolume());

		return collisionStatus(pointOnThePlane);
	}

	CollisionStatus DOP18::collisionStatus(const Vec3& point) const
	{
		if (point.x < min[DOP18_AXIS_X] || point.x > max[DOP18_AXIS_X] )
			return CollisionStatus::OUTSIDE;

		if (point.y < min[DOP18_AXIS_Y] || point.y > max[DOP18_AXIS_Y])
			return CollisionStatus::OUTSIDE;

		if (point.z < min[DOP18_AXIS_Z] || point.y > max[DOP18_AXIS_Z])
			return CollisionStatus::OUTSIDE;

		Plane3D p1(Vec3(min[DOP18_AXIS_UP_LEFT], ZERO_FLOAT, ZERO_FLOAT), DOP18_NORMALS[6]); // up-left
		if (p1.distance(point) > 0.0f) // if left side of the (left) plane
			return CollisionStatus::OUTSIDE;

		p1 = Plane3D(Vec3(max[DOP18_AXIS_UP_LEFT], ZERO_FLOAT, ZERO_FLOAT), DOP18_NORMALS[7]); // down-right
		if (p1.distance(point) > 0.0f) // if right side of the (right) plane
			return CollisionStatus::OUTSIDE;

		p1 = Plane3D(Vec3(min[DOP18_AXIS_UP_RIGHT], ZERO_FLOAT, ZERO_FLOAT), DOP18_NORMALS[9]); // down-left
		if (p1.distance(point) > 0.0f) // if right side of the (right) plane
			return CollisionStatus::OUTSIDE;

		p1 = Plane3D(Vec3(max[DOP18_AXIS_UP_RIGHT], ZERO_FLOAT, ZERO_FLOAT), DOP18_NORMALS[8]); // up-right
		if (p1.distance(point) > 0.0f) // if left side of the (left) plane
			return CollisionStatus::OUTSIDE;

		p1 = Plane3D(Vec3(ZERO_FLOAT, min[DOP18_AXIS_UP_FRONT], ZERO_FLOAT), DOP18_NORMALS[11]); // down-depth
		if (p1.distance(point) > 0.0f) // if right side of the (right) plane
			return CollisionStatus::OUTSIDE;

		p1 = Plane3D(Vec3(ZERO_FLOAT, max[DOP18_AXIS_UP_FRONT], ZERO_FLOAT), DOP18_NORMALS[10]); // up-front
		if (p1.distance(point) > 0.0f) // if left side of the (left) plane
			return CollisionStatus::OUTSIDE;

		p1 = Plane3D(Vec3(ZERO_FLOAT, min[DOP18_AXIS_UP_DEPTH], ZERO_FLOAT), DOP18_NORMALS[13]); // down-front
		if (p1.distance(point) > 0.0f) // if right side of the (right) plane
			return CollisionStatus::OUTSIDE;
		
		p1 = Plane3D(Vec3(ZERO_FLOAT, max[DOP18_AXIS_UP_DEPTH], ZERO_FLOAT), DOP18_NORMALS[12]); // up-depth
		if (p1.distance(point) > 0.0f) // if left side of the (left) plane
			return CollisionStatus::OUTSIDE;

		p1 = Plane3D(Vec3(min[DOP18_AXIS_LEFT_DEPTH], ZERO_FLOAT, ZERO_FLOAT), DOP18_NORMALS[14]); // left-depth
		if (p1.distance(point) > 0.0f) // if right side of the (right) plane
			return CollisionStatus::OUTSIDE;

		p1 = Plane3D(Vec3(max[DOP18_AXIS_LEFT_DEPTH], ZERO_FLOAT, ZERO_FLOAT), DOP18_NORMALS[15]); // right-front
		if (p1.distance(point) > 0.0f) // if left side of the (left) plane
			return CollisionStatus::OUTSIDE;
		
		p1 = Plane3D(Vec3(min[DOP18_AXIS_RIGHT_DEPTH], ZERO_FLOAT, ZERO_FLOAT), DOP18_NORMALS[17]); // left-front
		if (p1.distance(point) > 0.0f) // if left side of the (left) plane
			return CollisionStatus::OUTSIDE;

		p1 = Plane3D(Vec3(max[DOP18_AXIS_RIGHT_DEPTH], ZERO_FLOAT, ZERO_FLOAT), DOP18_NORMALS[16]); // right-depth
		if (p1.distance(point) > 0.0f) // if right side of the (right) plane
			return CollisionStatus::OUTSIDE;

		return CollisionStatus::INSIDE;
	}

	CollisionStatus DOP18::collisionStatus(const Vec3& point, sp_bool* minPlane, sp_uint* planeIndex, sp_float* distanceFromPlane) const
	{
		if (point.x < min[DOP18_AXIS_X] || point.x > max[DOP18_AXIS_X])
			return CollisionStatus::OUTSIDE;

		if (point.y < min[DOP18_AXIS_Y] || point.y > max[DOP18_AXIS_Y])
			return CollisionStatus::OUTSIDE;

		if (point.z < min[DOP18_AXIS_Z] || point.y > max[DOP18_AXIS_Z])
			return CollisionStatus::OUTSIDE;

		const Plane3D* _planes = planes();
		*minPlane = false;
		*planeIndex = SP_UINT_MAX;
		*distanceFromPlane = SP_FLOAT_MAX;

		for (sp_uint axis = 0; axis < DOP18_ORIENTATIONS; axis++)
		{
			const sp_float distanceMin = _planes[axis * TWO_UINT].distance(point);
			if (distanceMin > 0.0f) // if left side of the (min) plane
				return CollisionStatus::OUTSIDE;

			if (distanceMin < *distanceFromPlane) // it this plane has the smallest distance, keep track
			{
				*distanceFromPlane = distanceMin;
				*planeIndex = axis;
				*minPlane = true;
			}

			const sp_float distanceMax = _planes[axis * TWO_UINT + ONE_UINT].distance(point);
			if (distanceMax > 0.0f) // if left side of the (min) plane
				return CollisionStatus::OUTSIDE;

			if (distanceMax < *distanceFromPlane) // it this plane has the smallest distance, keep track
			{
				*distanceFromPlane = distanceMax;
				*planeIndex = axis;
				*minPlane = false;
			}
		}

		return CollisionStatus::INSIDE;
	}

	Plane3D* DOP18::planes() const
	{
		return sp_mem_new_array(Plane3D, 18) {
			Plane3D(Vec3(min[DOP18_AXIS_X], ZERO_FLOAT, ZERO_FLOAT), DOP18_NORMALS[0]), // left
			Plane3D(Vec3(max[DOP18_AXIS_X], ZERO_FLOAT, ZERO_FLOAT), DOP18_NORMALS[1]), // right

			Plane3D(Vec3(ZERO_FLOAT, max[DOP18_AXIS_Y], ZERO_FLOAT), DOP18_NORMALS[2]), // up
			Plane3D(Vec3(ZERO_FLOAT, min[DOP18_AXIS_Y], ZERO_FLOAT), DOP18_NORMALS[3]), // down

			Plane3D(Vec3(ZERO_FLOAT, ZERO_FLOAT, max[DOP18_AXIS_Z]), DOP18_NORMALS[4]), // front
			Plane3D(Vec3(ZERO_FLOAT, ZERO_FLOAT, min[DOP18_AXIS_Z]), DOP18_NORMALS[5]), // depth

			Plane3D(Vec3(min[DOP18_AXIS_UP_LEFT], ZERO_FLOAT, ZERO_FLOAT), DOP18_NORMALS[6]), // up-left
			Plane3D(Vec3(max[DOP18_AXIS_UP_LEFT], ZERO_FLOAT, ZERO_FLOAT), DOP18_NORMALS[7]), // down-right

			Plane3D(Vec3(max[DOP18_AXIS_UP_RIGHT], ZERO_FLOAT, ZERO_FLOAT), DOP18_NORMALS[8]), // up-right
			Plane3D(Vec3(min[DOP18_AXIS_UP_RIGHT], ZERO_FLOAT, ZERO_FLOAT), DOP18_NORMALS[9]), // down-left

			Plane3D(Vec3(ZERO_FLOAT, max[DOP18_AXIS_UP_FRONT], ZERO_FLOAT), DOP18_NORMALS[10]), // up-front
			Plane3D(Vec3(ZERO_FLOAT, min[DOP18_AXIS_UP_FRONT], ZERO_FLOAT), DOP18_NORMALS[11]), // down-depth

			Plane3D(Vec3(ZERO_FLOAT, max[DOP18_AXIS_UP_DEPTH], ZERO_FLOAT), DOP18_NORMALS[12]), // up-depth
			Plane3D(Vec3(ZERO_FLOAT, min[DOP18_AXIS_UP_DEPTH], ZERO_FLOAT), DOP18_NORMALS[13]), // down-front

			Plane3D(Vec3(min[DOP18_AXIS_LEFT_DEPTH], ZERO_FLOAT, ZERO_FLOAT), DOP18_NORMALS[14]), // left-depth
			Plane3D(Vec3(max[DOP18_AXIS_LEFT_DEPTH], ZERO_FLOAT, ZERO_FLOAT), DOP18_NORMALS[15]), // right-front

			Plane3D(Vec3(max[DOP18_AXIS_RIGHT_DEPTH], ZERO_FLOAT, ZERO_FLOAT), DOP18_NORMALS[16]), // right-depth
			Plane3D(Vec3(min[DOP18_AXIS_RIGHT_DEPTH], ZERO_FLOAT, ZERO_FLOAT), DOP18_NORMALS[17]), // left-front
		};
	}

	void DOP18::fixTopDegeneration(const Plane3D* planes)
	{
		Line3D line1;
		planes[DOP18_PLANES_UP_FRONT_INDEX].intersection(planes[DOP18_PLANES_UP_LEFT_INDEX], &line1);

		Line3D line2;
		planes[DOP18_PLANES_UP_DEPTH_INDEX].intersection(planes[DOP18_PLANES_UP_LEFT_INDEX], &line2);

		Vec3 point;
		line1.intersection(line2, &point);

		if (point != ZERO_FLOAT) // fix top max
			max[1] = point.y;
	}

	void DOP18::fixBottomDegeneration(const Plane3D* planes)
	{
		Line3D line1;
		planes[DOP18_PLANES_DOWN_FRONT_INDEX].intersection(planes[DOP18_PLANES_DOWN_LEFT_INDEX], &line1);
		
		Line3D line2;
		planes[DOP18_PLANES_DOWN_DEPTH_INDEX].intersection(planes[DOP18_PLANES_DOWN_LEFT_INDEX], &line2);
		
		Vec3 point;
		line1.intersection(line2, &point);

		if (point != ZERO_FLOAT) // fix bottom min
			min[1] = point.y;
	}

	void DOP18::fixLeftDegeneration(const Plane3D* planes)
	{
		Line3D line1;
		planes[DOP18_PLANES_LEFT_FRONT_INDEX].intersection(planes[DOP18_PLANES_UP_LEFT_INDEX], &line1);
		
		Line3D line2;
		planes[DOP18_PLANES_LEFT_DEPTH_INDEX].intersection(planes[DOP18_PLANES_UP_LEFT_INDEX], &line2);

		Vec3 point;
		line1.intersection(line2, &point);

		if (point != ZERO_FLOAT) // fix left min
			min[0] = point.x;
	}

	void DOP18::fixRightDegeneration(const Plane3D* planes)
	{
		Line3D line1;
		planes[DOP18_PLANES_RIGHT_FRONT_INDEX].intersection(planes[DOP18_PLANES_UP_RIGHT_INDEX], &line1);
		
		Line3D line2;
		planes[DOP18_PLANES_RIGHT_DEPTH_INDEX].intersection(planes[DOP18_PLANES_UP_RIGHT_INDEX], &line2);
		
		Vec3 point;
		line1.intersection(line2, &point);

		if (point != ZERO_FLOAT) // fix right max
			max[0] = point.x;
	}

	void DOP18::fixFrontDegeneration(const Plane3D* planes)
	{
		Line3D line1;
		planes[DOP18_PLANES_DOWN_FRONT_INDEX].intersection(planes[DOP18_PLANES_LEFT_FRONT_INDEX], &line1);
		
		Line3D line2;
		planes[DOP18_PLANES_DOWN_FRONT_INDEX].intersection(planes[DOP18_PLANES_RIGHT_FRONT_INDEX], &line2);
		
		Vec3 point;
		line1.intersection(line2, &point);

		if (point != ZERO_FLOAT) // fix front min
			min[2] = point.z;
	}

	void DOP18::fixDepthDegeneration(const Plane3D* planes)
	{
		Line3D line1;
		planes[DOP18_PLANES_DOWN_DEPTH_INDEX].intersection(planes[DOP18_PLANES_LEFT_DEPTH_INDEX], &line1);
		
		Line3D line2;
		planes[DOP18_PLANES_DOWN_DEPTH_INDEX].intersection(planes[DOP18_PLANES_RIGHT_DEPTH_INDEX], &line2);
		
		Vec3 point;
		line1.intersection(line2, &point);

		if (point != ZERO_FLOAT) // fix depth max
			max[2] = point.z;
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

	DOP18::operator void*() const
	{
		return (void*)this;
	}

	DOP18::operator sp_float*() const
	{
		return (sp_float*)this;
	}

}