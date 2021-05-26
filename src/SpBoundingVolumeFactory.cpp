#include "SpBoundingVolumeFactory.h"
#include "SpDOP18Factory.h"
#include "SpAABBFactory.h"
#include "SpSphereBoundingVolumeFactory.h"

namespace NAMESPACE_PHYSICS
{

	SpBoundingVolumeFactory* SpBoundingVolumeFactory::create(const BoundingVolumeType type)
	{
		switch (type)
		{
		case BoundingVolumeType::DOP18:
			return sp_mem_new(SpDOP18Factory)();

		case BoundingVolumeType::AABB:
			return sp_mem_new(SpAABBFactory)();

		case BoundingVolumeType::Sphere:
			return sp_mem_new(SpSphereBoundingVolumeFactory)();

		default:
			sp_assert(false, "InvalidBoundingVolumeType");
			break;
		}

		return nullptr;
	}

}