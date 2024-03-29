#include "HashGrid.h"

namespace NAMESPACE_PHYSICS
{
	int findCellIndexByCellId(const Vec3& cell)
	{
		const int h1 = 0x8da6b343; // Large multiplicative constants; 
		const int h2 = 0xd8163841; // here arbitrarily chosen primes 
		const int h3 = 0xcb1ab31f;

		return h1 * int(cell.x) + h2 * int(cell.y) + h3 * int(cell.z);
	}

	HashGrid::HashGrid()
	{
		setCellSize(10);
	}

	HashGrid::HashGrid(size_t cellSize)
	{
		setCellSize(cellSize);
	}

	void HashGrid::setCellSize(size_t cellSize)
	{
		this->cellSize = cellSize;
		this->cellSizeInverted = 1.0f / cellSize;
	}

	size_t HashGrid::getCellSize()
	{
		return cellSize;
	}

	Vec3 HashGrid::findCell(const Vec3& point) 
	{
		int negativeCellX = 0;
		int negativeCellY = 0;
		int negativeCellZ = 0;

		if (point.x < 0.0f)
			negativeCellX = -1;

		if (point.y < 0.0f)
			negativeCellY = -1;

		if (point.z < 0.0f)
			negativeCellZ = -1;

		Vec3 cell = {
			float(int(point.x * cellSizeInverted) + negativeCellX),
			float(int(point.y * cellSizeInverted) + negativeCellY),
			float(int(point.z * cellSizeInverted) + negativeCellZ)
		};
		
		return cell;
	}

	int HashGrid::findCellIndex(const Vec3& point)
	{
		Vec3 cell = findCell(point);

		return findCellIndexByCellId(cell);
	}

	Vec3List* HashGrid::findRangeCell(const AABB& aabb)
	{	
		Vec3List* list = ALLOC(Vec3List);
		Vec3 minCell = findCell(aabb.minPoint);
		Vec3 maxCell = findCell(aabb.maxPoint);

		Vec3 deltaPoints = (maxCell - minCell) + 1.0f;
			
		list->count = sp_int(std::abs(
				(deltaPoints.x == 0.0f ? 1.0f : deltaPoints.x)
			* (deltaPoints.y == 0.0f ? 1.0f : deltaPoints.y)
			* (deltaPoints.z == 0.0f ? 1.0f : deltaPoints.z)));

		list->points = ALLOC_ARRAY(Vec3, list->count);

		sp_size index = 0;
		
		for (sp_float x = minCell.x; x <= maxCell.x; x++)
			for (sp_float y = minCell.y; y <= maxCell.y; y++)
				for (sp_float z = minCell.z; z <= maxCell.z; z++)
				{
					list->points[index] = Vec3(x, y, z);
					index++;
				}

		return list;
	}

	std::vector<sp_int> HashGrid::findRangeCellIndex(const AABB& aabb)
	{
		Vec3List* cells = findRangeCell(aabb);

		std::vector<sp_int> hashes = std::vector<sp_int>(cells->count);

		for (sp_uint i = 0; i < cells->count; i++)
			hashes[i] = findCellIndexByCellId(cells->points[i]);

		ALLOC_RELEASE(cells);
		
		return hashes;
	}

	sp_bool findValue(const std::unordered_multimap<AABB, AABB, AABB, AABB>& map, const AABB& key, const AABB& value)
	{
		std::pair<AABB, AABB> pair = std::make_pair(key, value);
		auto its = map.equal_range(key);
		
		for (auto it = its.first; it != its.second; ++it) 
			if (it->second == value)
				return true;

		return false;
	}

	sp_bool findValue2(const std::unordered_multimap<AABB, AABB, AABB, AABB>& map, const AABB& key, const AABB& value)
	{
		std::pair<AABB, AABB> pair = std::make_pair(key, value);
		auto its = map.equal_range(key);

		for (auto it = its.first; it != its.second; ++it)
			if (it->second == value)
				return true;

		return false;
	}

	std::unordered_multimap<AABB, AABB, AABB, AABB> HashGrid::findCollisions(AABB* aabbs, size_t aabbCount)
	{
		std::unordered_multimap<AABB, AABB, AABB, AABB> pairs;
		std::multimap<int, AABB> spatialVolume;
			
		for (sp_size aabbIndex = 0; aabbIndex < aabbCount; aabbIndex++)
		{
			AABB aabb = aabbs[aabbIndex];
			std::vector<int> hashes = findRangeCellIndex(aabb);
			
			for (sp_size i = 0 ; i < hashes.size() ; i++ )
			{
				auto its = spatialVolume.equal_range(hashes[i]);

				for (auto it = its.first; it != its.second; ++it) 
				{
					bool hasCollisionAABB = aabb.collisionStatus(it->second) == CollisionStatus::INSIDE;

					if (hasCollisionAABB && !findValue(pairs, aabb, it->second))   //check if exists on map
						pairs.emplace(aabb, it->second);
				}
				
				spatialVolume.emplace( hashes[i], aabbs[aabbIndex] );
			}
		}

		return pairs;
	}
}
