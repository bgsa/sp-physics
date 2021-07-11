#include "SpCameraManager.h"
#include "SpGame.h"

namespace NAMESPACE_PHYSICS
{

	SpCameraManager::SpCameraManager()
		: SpObjectManager()
	{
		_textureBuffer = SpGameInstance->renderingAPI()->createTextureBuffer();
		usageType = SpGameInstance->renderingAPI()->bufferUsageTypeDynamicDraw();
	}

}