#include "SpMaterialsManager.h"
#include "SpGame.h"

namespace NAMESPACE_PHYSICS
{

	SpMaterialsManager::SpMaterialsManager()
	{
		SpRenderingAPI* api = SpGameInstance->renderingAPI();
		_usageType = api->bufferUsageTypeStaticDraw();
		_gpuBuffer = api->createShaderStorageBufferObject();
	}

}