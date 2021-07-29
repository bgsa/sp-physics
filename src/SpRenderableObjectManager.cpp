#include "SpRenderableObjectManager.h"
#include "SpGame.h"

namespace NAMESPACE_PHYSICS
{

	SpRenderableObjectManager::SpRenderableObjectManager()
		: SpObjectManager()
	{
		_materialsBuffer = SpGameInstance->renderingAPI()->createTextureBuffer();
		usageType = SpGameInstance->renderingAPI()->bufferUsageTypeDynamicDraw();
	}

}