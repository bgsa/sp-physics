#ifndef SP_GPU_PLATFORM_MANAGER_HEADER
#define SP_GPU_PLATFORM_MANAGER_HEADER

#include "SpectrumPhysics.h"
#include "SpGpuPlatform.h"
#include "SpOpenCL.h"
#include "GpuLog.hpp"

namespace NAMESPACE_PHYSICS
{

	class SpGpuPlatformManager
	{
	private:
		SpGpuPlatform* _defaultPlatform;

		static void loadDevices(SpGpuPlatform& platform);

		static void selectDefaultPlatform();

	public:
		sp_uint platformsLength;
		SpGpuPlatform* platforms;

		API_INTERFACE inline SpGpuPlatformManager()
		{
			platformsLength = ZERO_UINT;
			platforms = nullptr;
			_defaultPlatform = nullptr;
		}

		API_INTERFACE inline SpGpuPlatform* defaultPlatform() const
		{
			return _defaultPlatform;
		}

		API_INTERFACE inline void dispose()
		{
			if (platforms != nullptr)
			{
				for (sp_uint i = 0; i < platformsLength; i++)
					platforms[i].dispose();

				sp_mem_release(platforms);
				platforms = nullptr;
			}

			platformsLength = ZERO_UINT;
		}

		~SpGpuPlatformManager()
		{
			dispose();
		}

		API_INTERFACE static void init();
		API_INTERFACE static void release();

	};

	extern SpGpuPlatformManager* SpGpuPlatformManagerInstance;
}

#endif // SP_GPU_PLATFORM_MANAGER_HEADER