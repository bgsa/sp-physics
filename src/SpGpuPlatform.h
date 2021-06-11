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
		sp_uint gpuDevicesLength;
		GpuDevice** gpuDevices;

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
			gpuDevicesLength = ZERO_UINT;
			gpuDevices = nullptr;
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
		API_INTERFACE void dispose();

		~SpGpuPlatform()
		{
			dispose();
		}
	};
}

#endif // SP_GPU_PLATFORM_HEADER