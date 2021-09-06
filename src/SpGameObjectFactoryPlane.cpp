#include "SpGameObjectFactoryPlane.h"
#include "SpPhysicSettings.h"
#include "SpGameObject.h"
#include "SpRenderableObject.h"
#include "SpMeshData.h"
#include "SpGame.h"

namespace NAMESPACE_PHYSICS
{

	void SpGameObjectFactoryPlane::initMesh(SpScene* scene)
	{
		if (planeMeshIndex != SP_UINT_MAX)
			return;

		SpPoolMemoryAllocator::main()->enableMemoryAlignment();

		planeMeshIndex = scene->meshManager()->add();

		SpMeshData* meshData = scene->meshManager()->get(planeMeshIndex);
		meshData->attributesLength = 4;
		meshData->attributes = sp_mem_new_array(SpMeshAttribute, 4);
		meshData->attributes[0].vertex = Vec3(-0.5f, 0.0f, 0.5f);
		meshData->attributes[1].vertex = Vec3(0.5f, 0.0f, 0.5f);
		meshData->attributes[2].vertex = Vec3(0.5f, 0.0f, -0.5f);
		meshData->attributes[3].vertex = Vec3(-0.5f, 0.0f, -0.5f);

		meshData->attributes[0].normal = Vec3Up;
		meshData->attributes[1].normal = Vec3Up;
		meshData->attributes[2].normal = Vec3Up;
		meshData->attributes[3].normal = Vec3Up;

		meshData->facesLength = 2u;
		meshData->faceIndexes = sp_mem_new_array(sp_size, 2 * 3);
		meshData->faceIndexes[0] = 0;
		meshData->faceIndexes[1] = 1;
		meshData->faceIndexes[2] = 2;
		meshData->faceIndexes[3] = 2;
		meshData->faceIndexes[4] = 3;
		meshData->faceIndexes[5] = 0;

		SpPoolMemoryAllocator::main()->disableMemoryAlignment();
	}

	void SpGameObjectFactoryPlane::init(SpScene* scene)
	{
		initMesh(scene);

		SpMeshData* meshData = scene->meshManager()->get(planeMeshIndex);
		const sp_size arrayBufferSize = sizeof(SpMeshAttribute) * meshData->attributesLength;
		const sp_int staticDraw = SpGameInstance->renderingAPI()->bufferUsageTypeStaticDraw();

		buffers[0]
			= SpGameInstance->renderingAPI()->createArrayBuffer()
			->use()
			->updateData(arrayBufferSize, (sp_float*)meshData->attributes, staticDraw);

		buffers[1]
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

		const sp_uint renderableObjectIndex = scene->renderableObjectManager()->add();
		gameObject->renderableObjectIndex(renderableObjectIndex);

		SpRenderableObject* renderableObject = scene->renderableObjectManager()->get(renderableObjectIndex);
		renderableObject->type(SP_RENDERABLE_OBJECT_TYPE_PLANE);
		renderableObject->gameObject(gameObject->index());
		renderableObject->meshData(planeMeshIndex);
		renderableObject->shader(0);

		renderableObject->addBuffer(buffers[0]);
		renderableObject->addBuffer(buffers[1]);

		return gameObject->index();
	}

	void SpGameObjectFactoryPlane::dispose()
	{
		for (sp_uint i = 0; i < _buffersLength; i++)
		{
			SpGpuBuffer* buffer = buffers[i];

			if (buffer != nullptr)
			{
				buffer->dispose();
				sp_mem_release(buffer);
				buffer = nullptr;
			}
		}
	}
}