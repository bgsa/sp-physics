#include "SpTransformManager.h"
#include "SpGame.h"

namespace NAMESPACE_PHYSICS
{

	SpTransformManager::SpTransformManager()
		: SpObjectManager()
	{
		_textureBuffer = SpGameInstance->renderingAPI()->createTextureBuffer();
		usageType = SpGameInstance->renderingAPI()->bufferUsageTypeDynamicDraw();
	}

}