#include "SpGameObjectFactoryCube.h"
#include "SpPhysicSettings.h"
#include "SpGameObject.h"
#include "SpRenderableObject.h"
#include "SpMeshData.h"
#include "SpGame.h"

namespace NAMESPACE_PHYSICS
{

	void SpGameObjectFactoryCube::initMesh(SpScene* scene)
	{
		if (cubeMeshIndex != SP_UINT_MAX)
			return;

		SpPoolMemoryAllocator::main()->enableMemoryAlignment();

		cubeMeshIndex = scene->meshManager()->add();

		SpMeshData* meshData = scene->meshManager()->get(cubeMeshIndex);
		meshData->attributesLength = 8;
		meshData->attributes = sp_mem_new_array(SpMeshAttribute, 8);
		meshData->attributes[0].vertex = Vec3(0.5f, -0.5f, -0.5f);  // left-bottom-front
		meshData->attributes[1].vertex = Vec3(-0.5f, -0.5f, -0.5f); // right-bottom-front
		meshData->attributes[2].vertex = Vec3(-0.5f, 0.5f, -0.5f);  // right-top-front
		meshData->attributes[3].vertex = Vec3(0.5f, 0.5f, -0.5f);   // left-top-front

		meshData->attributes[4].vertex = Vec3(-0.5f, -0.5f, 0.5f); // left-bottom-back
		meshData->attributes[5].vertex = Vec3(0.5f, -0.5f, 0.5f);  // right-bottom-back
		meshData->attributes[6].vertex = Vec3(0.5f, 0.5f, 0.5f);   // right-top-back
		meshData->attributes[7].vertex = Vec3(-0.5f, 0.5f, 0.5f);  // left-top-back

		meshData->attributes[0].normal = Vec3(0.577350259f, -0.577350259f, -0.577350259f);  //bottom-right-back
		meshData->attributes[1].normal = Vec3(-0.577350259f, -0.577350259f, -0.577350259f); //bottom-left-back
		meshData->attributes[2].normal = Vec3(-0.577350259f, 0.577350259f, -0.577350259f);  //top-left-back
		meshData->attributes[3].normal = Vec3(0.577350259f, 0.577350259f, -0.577350259f);   //top-right-back
		meshData->attributes[4].normal = Vec3(-0.577350259f, -0.577350259f, 0.577350259f);  //bottom-left-front
		meshData->attributes[5].normal = Vec3(0.577350259f, -0.577350259f, 0.577350259f);   //bottom-right-front
		meshData->attributes[6].normal = Vec3(0.577350259f, 0.577350259f, 0.577350259f);    //top-right-front
		meshData->attributes[7].normal = Vec3(-0.577350259f, 0.577350259f, 0.577350259f);    //top-left-front

		const sp_uint cubeIndices[36] = {
			0,1,2, //face frontal
			2,3,0,
			4,5,6, //face-traseira
			6,7,4,
			0,5,4, //fundo
			4,1,0,
			3,2,7, //topo
			7,6,3,
			6,5,0,
			0,3,6,
			1,4,7, //face direita
			7,2,1
		};
		meshData->facesLength = 12u;
		meshData->faceIndexes = sp_mem_new_array(sp_size, meshData->facesLength * 3);
		std::memcpy(meshData->faceIndexes, cubeIndices, 36 * sizeof(sp_float));

		SpPoolMemoryAllocator::main()->disableMemoryAlignment();
	}

	void SpGameObjectFactoryCube::init(SpScene* scene)
	{
		initMesh(scene);

		SpMeshData* meshData = scene->meshManager()->get(cubeMeshIndex);

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

	sp_uint SpGameObjectFactoryCube::create(SpScene* scene)
	{
		sp_char cubeName[100];
		std::memcpy(cubeName, "Cube ", sizeof(sp_char) * 5);
		sp_size len;
		convert(SpPhysicSettings::instance()->frameId(), &cubeName[5], len);
		cubeName[5 + len] = END_OF_STRING;

		SpGameObject* gameObject = scene->addGameObject(SP_GAME_OBJECT_TYPE_CUBE, cubeName);

		const sp_uint renderableObjectIndex = scene->renderableObjectManager()->add();
		gameObject->renderableObjectIndex(renderableObjectIndex);

		SpRenderableObject* renderableObject = scene->renderableObjectManager()->get(renderableObjectIndex);
		renderableObject->type(SP_RENDERABLE_OBJECT_TYPE_CUBE);
		renderableObject->gameObject(gameObject->index());
		renderableObject->meshData(cubeMeshIndex);
		renderableObject->shader(0);

		renderableObject->addBuffer(buffers[0]);
		renderableObject->addBuffer(buffers[1]);

		return gameObject->index();
	}

	void SpGameObjectFactoryCube::dispose()
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