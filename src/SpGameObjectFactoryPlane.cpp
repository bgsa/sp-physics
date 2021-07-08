#include "SpGameObjectFactoryPlane.h"
#include "SpPhysicSettings.h"
#include "SpGameObject.h"
#include "SpRenderableObject.h"
#include "SpMeshData.h"
#include "SpGame.h"

namespace NAMESPACE_PHYSICS
{

	void SpGameObjectFactoryPlane::init(SpScene* scene)
	{
		SpMeshData* meshData = scene->meshManager()->get(SP_MESH_INDEX_PLANE);
		const sp_size arrayBufferSize = sizeof(SpMeshAttribute) * meshData->attributesLength;
		const sp_int staticDraw = SpGameInstance->renderingAPI()->bufferUsageTypeStaticDraw();

		gpuVertexBuffer
			= SpGameInstance->renderingAPI()->createArrayBuffer()
			->use()
			->updateData(arrayBufferSize, (sp_float*)meshData->attributes, staticDraw);

		gpuIndexBuffer
			= SpGameInstance->renderingAPI()->createElementArrayBuffer()
			->use()
			->updateData(meshData->facesLength * 3 * sizeof(sp_size), meshData->faceIndexes, staticDraw);
	}

	sp_uint SpGameObjectFactoryPlane::create(SpScene* scene)
	{
		sp_char planeName[100];
		std::memcpy(planeName, "Plane ", sizeof(sp_char) * 6);
		sp_size len;
		convert(SpPhysicSettings::instance()->frameId(), &planeName[6], len);
		planeName[6 + len] = END_OF_STRING;

		SpGameObject* gameObject = scene->addGameObject(SP_GAME_OBJECT_TYPE_PLANE, planeName);
		SpRenderableObject* renderableObject = scene->renderableObjectManager()->get(gameObject->managerIndex());
		renderableObject->type(SP_RENDERABLE_OBJECT_TYPE_PLANE);
		renderableObject->gameObjectIndex = gameObject->index();
		renderableObject->meshDataIndex = SP_MESH_INDEX_PLANE;
		renderableObject->shaderIndex = 0;

		renderableObject->buffers.add(gpuVertexBuffer);
		renderableObject->buffers.add(gpuIndexBuffer);

		return gameObject->index();
	}

	void SpGameObjectFactoryPlane::dispose()
	{

	}
}