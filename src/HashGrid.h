#ifndef HASH_GRID_HEADER
#define HASH_GRID_HEADER

#include "SpectrumPhysics.h"
#include "AABB.h"
#include <unordered_map>
#include <map>

namespace NAMESPACE_PHYSICS
{

	class HashGrid
	{
	private:
		size_t cellSize;
		float cellSizeInverted;

	public:
		API_INTERFACE HashGrid();
		API_INTERFACE HashGrid(size_t cellSize);

		API_INTERFACE void setCellSize(size_t cellSize);
		API_INTERFACE size_t getCellSize();

		API_INTERFACE Vec3f findCell(const Vec3f& point);

		API_INTERFACE int findCellIndex(const Vec3f& point);

		API_INTERFACE Vec3List<float>* findRangeCell(const AABB& aabb);

		API_INTERFACE std::vector<int> findRangeCellIndex(const AABB& aabb);

		API_INTERFACE std::unordered_multimap<AABB, AABB, AABB, AABB> findCollisions(AABB* aabbs, size_t aabbCount);

	};

}

#endif // HASH_GRID_HEADER