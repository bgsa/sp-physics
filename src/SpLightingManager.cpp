#include "SpLightingManager.h"
#include "SpGame.h"

namespace NAMESPACE_PHYSICS 
{

	SpLightingManager::SpLightingManager()
		: SpObjectManager()
	{
		_textureBuffer = SpGameInstance->renderingAPI()->createTextureBuffer();
		usageType = SpGameInstance->renderingAPI()->bufferUsageTypeDynamicDraw();

		addDefaultAmbientLight();
		addDefaultDiffuseLight();
	}

}