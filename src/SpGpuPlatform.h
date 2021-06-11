#ifndef SP_GPU_PLATFORM_HEADER
#define SP_GPU_PLATFORM_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{

	class SpGpuPlatform
	{
	public:
		void* id;
		sp_char* name;
		sp_char* version;
		sp_char* vendor;
		sp_char* profile;
		sp_uint extensionsLength;
		sp_char** extensions;

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline SpGpuPlatform()
		{
			id = nullptr;
			name = nullptr;
			version = nullptr;
			vendor = nullptr;
			profile = nullptr;
			extensionsLength = ZERO_UINT;
			extensions = nullptr;
		}

		/// <summary>
		/// Check the platform is NVIDIA
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_bool isNvidia() const
		{
			return strContains(vendor, "NVIDIA");
		}

		/// <summary>
		/// Check the platform is AMD
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_bool isAMD() const
		{
			return strContains(vendor, "AMD");
		}

		/// <summary>
		/// Check the platform is Intel
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_bool isIntel() const
		{
			return strContains(vendor, "Intel");
		}

		/// <summary>
		/// Release all allocated resources
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline void dispose()
		{
			if (name != nullptr)
			{
				sp_mem_release(name);
				name = nullptr;
			}

			if (version != nullptr)
			{
				sp_mem_release(version);
				version = nullptr;
			}

			if (vendor != nullptr)
			{
				sp_mem_release(vendor);
				vendor = nullptr;
			}

			if (profile != nullptr)
			{
				sp_mem_release(profile);
				profile = nullptr;
			}

			if (extensions != nullptr)
			{
				for (sp_uint i = 0; i < extensionsLength; i++)
				{
					sp_mem_release(extensions[i]);
					extensions[i] = nullptr;
				}
				sp_mem_release(extensions);
				extensions = nullptr;
			}
		}

		~SpGpuPlatform()
		{
			dispose();
		}
	};
}

#endif // SP_GPU_PLATFORM_HEADER