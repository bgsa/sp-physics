#include "SpWorld.h"

namespace NAMESPACE_PHYSICS
{

	void SpWorld::init(const sp_uint objectsLength)
	{
		sp_assert(instanceGpuRendering != nullptr, "NullPointerException");

		_objectsLength = ZERO_UINT;
		_objectsLengthAllocated = objectsLength;
		_rigidBodies3D = sp_mem_new_array(SpRigidBody3D, objectsLength);
		_boundingVolumes = sp_mem_new_array(DOP18, objectsLength);
		_transforms = sp_mem_new_array(SpTransform, objectsLength);
		_objectMapper = sp_mem_new_array(SpCollisionFeatures, objectsLength);
		_meshes = sp_mem_new(SpArray<SpMesh*>)(objectsLength, objectsLength);

		gpu = GpuContext::instance()->defaultDevice();

		_transformsGPUBuffer = instanceGpuRendering->createTextureBuffer();
		_transformsGPUBuffer
			->use()
			->updateData(sizeof(SpTransform) * objectsLength, _transforms);

		_transformsGPU = gpu->createBufferFromOpenGL(_transformsGPUBuffer);

		_inputLengthGPU = sp_mem_new(GpuBufferOpenCL)(gpu);
		_inputLengthGPU->init(sizeof(sp_uint), &_objectsLength, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);

		_objectMapperGPU = sp_mem_new(GpuBufferOpenCL)(gpu);
		_objectMapperGPU->init(sizeof(SpCollisionFeatures) * _objectsLengthAllocated, CL_MEM_READ_ONLY | CL_MEM_ALLOC_HOST_PTR);

		_rigidBodies3DGPU = gpu->createBuffer(_rigidBodies3D, sizeof(SpRigidBody3D) * objectsLength, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, false);

		boundingVolumeFactory = SpBoundingVolumeFactory::create(SpPhysicSettings::instance()->boundingVolumeType());
		boundingVolumeFactory->initOutput(gpu, objectsLength);

		physicSimulator = sp_mem_new(SpPhysicSimulator)();
		physicSimulator->init();
	}

	void SpWorld::update(const sp_float elapsedTime)
	{
		physicSimulator->run(elapsedTime); // update collisions and responses
	}

	void SpWorld::dispose()
	{
		if (physicSimulator != nullptr)
		{
			sp_mem_delete(physicSimulator, SpPhysicSimulator);
			physicSimulator = nullptr;
		}

		if (_boundingVolumes != nullptr)
		{
			sp_mem_release(_boundingVolumes);
			_boundingVolumes = nullptr;
		}

		if (_rigidBodies3D != nullptr)
		{
			sp_mem_release(_rigidBodies3D);
			_rigidBodies3D = nullptr;
		}

		if (_rigidBodies3DGPU != nullptr)
		{
			gpu->releaseBuffer(_rigidBodies3DGPU);
			_rigidBodies3DGPU = nullptr;
		}

		if (boundingVolumeFactory != nullptr)
		{
			boundingVolumeFactory->dispose();
			boundingVolumeFactory = nullptr;
		}

		if (_objectMapperGPU != nullptr)
		{
			sp_mem_delete(_objectMapperGPU, GpuBufferOpenCL);
			_objectMapperGPU = nullptr;
		}

		if (_meshVertexCacheGPU != nullptr)
		{
			sp_mem_delete(_meshVertexCacheGPU, GpuBufferOpenCL);
			_meshVertexCacheGPU = nullptr;
		}

		if (_meshesStridesGPU != nullptr)
		{
			sp_mem_delete(_meshesStridesGPU, GpuBufferOpenCL);
			_meshesStridesGPU = nullptr;
		}

		if (renderer != nullptr)
		{
			renderer->dispose();
			sp_mem_release(renderer);
			renderer = nullptr;
		}
	}

}