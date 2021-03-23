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
		_inputLengthGPU->init(SIZEOF_UINT, &_objectsLength, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);

		_objectMapperGPU = sp_mem_new(GpuBufferOpenCL)(gpu);
		_objectMapperGPU->init(sizeof(SpCollisionFeatures) * _objectsLengthAllocated, CL_MEM_READ_ONLY | CL_MEM_ALLOC_HOST_PTR);

		_boundingVolumesGPU = gpu->createBuffer(_boundingVolumes, sizeof(DOP18) * objectsLength, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, true);
		_rigidBodies3DGPU = gpu->createBuffer(_rigidBodies3D, sizeof(SpRigidBody3D) * objectsLength, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, false);

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

		if (_boundingVolumesGPU != nullptr)
		{
			gpu->releaseBuffer(_boundingVolumesGPU);
			_boundingVolumesGPU = nullptr;
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
		if (_objectMapperGPU != nullptr)
		{
			sp_mem_delete(_objectMapperGPU, GpuBufferOpenCL);
			_objectMapperGPU = nullptr;
		}
	}

}