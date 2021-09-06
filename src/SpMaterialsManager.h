#ifndef SP_MATERIALS_MANAGER_HEADER
#define SP_MATERIALS_MANAGER_HEADER

#include "SpectrumPhysics.h"
#include "SpObjectManager.h"
#include "SpMaterial.h"
#include "SpContiguousMemoryArray.h"
#include "SpGpuBuffer.h"

namespace NAMESPACE_PHYSICS
{
	class SpMaterialsManager
		: public SpObjectManager
	{
	private:
		SpContiguousMemoryArray<SpMaterial, sp_uint> _materials;
		SpGpuBuffer* _gpuBuffer;
		sp_uint _usageType;

	public:

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns></returns>
		API_INTERFACE SpMaterialsManager();

		/// <summary>
		/// Add new material
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_uint add(const sp_uint length = 1) override
		{
			sp_uint index = _materials.add();
			_length = _materials.length();
			return index;
		}

		/// <summary>
		/// Remove a material from list
		/// </summary>
		/// <param name="index">Index of Material</param>
		/// <returns>void</returns>
		API_INTERFACE inline void remove(const sp_uint index)  override
		{
			_materials.remove(index);
			_length = _materials.length();
		}

		/// <summary>
		/// Get materials object by index
		/// </summary>
		/// <param name="index"></param>
		/// <returns></returns>
		API_INTERFACE inline SpMaterial* get(const sp_uint index) const
		{
			return _materials.get(index);
		}

	};

}

#endif // SP_MATERIALS_MANAGER_HEADER