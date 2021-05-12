#ifndef SP_WORLD_HEADER
#define SP_WORLD_HEADER

#include "SpectrumPhysics.h"
#include "GpuContext.h"
#include "SpRigidBody3D.h"
#include "SpCollisionFeatures.h"
#include "DOP18.h"
#include "SpTransform.h"
#include "SpGpuRenderingFactory.h"
#include "SpMesh.h"
#include "SpDOP18Factory.h"
#include "SpSphereBoundingVolumeFactory.h"
#include "SpAABBFactory.h"
#include "SpMeshCacheUpdaterGPU.h"
#include "SpPhysicSimulator.h"
#include "SpIRendererManager.h"

namespace NAMESPACE_PHYSICS
{
	/// <summary>
	/// Responsable to manager all components in the current world
	/// </summary>
	class SpWorld
	{
	private:
		sp_uint _objectsLengthAllocated;
		sp_uint _objectsLength;

		GpuDevice* gpu;
		
	public:
		sp_char name[256];
		SpPhysicSimulator* physicSimulator;

		DOP18* _boundingVolumes;
		SpRigidBody3D* _rigidBodies3D;
		SpTransform* _transforms;
		SpCollisionFeatures* _objectMapper;
		SpArray<SpMesh*>* _meshes;
		SpArray<SpMeshCache*>* _meshesCache;

		SpSphereBoundingVolumeFactory sphereFactory;
		SpAABBFactory aabbFactory;
		SpDOP18Factory dop18Factory;

		// GPU data
		GpuBufferOpenCL* _inputLengthGPU;
		cl_mem _rigidBodies3DGPU;
		cl_mem _transformsGPU;
		SpGpuTextureBuffer* _transformsGPUBuffer;
		GpuBufferOpenCL* _objectMapperGPU;
		GpuBufferOpenCL* _meshesGPU;
		GpuBufferOpenCL* _meshesIndexesGPU;
		GpuBufferOpenCL* _meshCacheGPU;
		GpuBufferOpenCL* _meshCacheIndexesGPU;
		GpuBufferOpenCL* _meshCacheVertexesLengthGPU;
		SpMeshCacheUpdaterGPU _meshCacheUpdater;

		SpIRendererManager* renderer;

		void initMeshCache()
		{
			sp_uint* meshCacheIndexes = ALLOC_NEW_ARRAY(sp_uint, _objectsLength);
			sp_uint* meshCacheVertexesLength = ALLOC_NEW_ARRAY(sp_uint, _objectsLength);
			sp_uint* meshesIndexes = ALLOC_NEW_ARRAY(sp_uint, _objectsLength * 3u);
			meshCacheIndexes[0] = ZERO_UINT;

			SpMesh* m = mesh(collisionFeatures(0u)->meshIndex);
			meshCacheVertexesLength[0] = m->vertexesMesh->length();

			sp_uint vertexCounter = meshCacheVertexesLength[0];

			const sp_size initialMemoryIndex = (sp_size)_meshes->data()[0];
			sp_size vertexMemoryIndex = (sp_size)_meshes->data()[0]->vertexesMesh->data()[0];
			sp_size faceMemoryIndex = (sp_size)_meshes->data()[0]->faces->data()[0];
			sp_size edgeMemoryIndex = (sp_size)_meshes->data()[0]->edges->data()[0];

			meshesIndexes[0] = divideBy4(vertexMemoryIndex - initialMemoryIndex);
			meshesIndexes[1] = divideBy4(faceMemoryIndex - initialMemoryIndex);
			meshesIndexes[2] = divideBy4(edgeMemoryIndex - initialMemoryIndex);

			SpPoolMemoryAllocator::main()->enableMemoryAlignment();

			_meshesCache = sp_mem_new(SpArray<SpMeshCache*>)(_objectsLength, _objectsLength);
			_meshesCache->data()[0] = sp_mem_new(SpMeshCache)(m->vertexesMesh->length());

			for (sp_uint i = 1; i < _objectsLength; i++)
			{
				SpMesh* m = mesh(collisionFeatures(i)->meshIndex);

				const sp_uint vertexLength = m->vertexesMesh->length();

				_meshesCache->data()[i] = sp_mem_new(SpMeshCache)(vertexLength);

				meshCacheIndexes[i] = meshCacheIndexes[i - 1] + meshCacheVertexesLength[i - 1] * 3u;
				meshCacheVertexesLength[i] = vertexLength;
				vertexCounter += vertexLength;

				vertexMemoryIndex = (sp_size)m->vertexesMesh->data()[0];
				faceMemoryIndex = (sp_size)m->faces->data()[0];
				edgeMemoryIndex = (sp_size)m->edges->data()[0];

				const sp_uint idx = i * 3u;
				meshesIndexes[idx] = divideBy4(vertexMemoryIndex - initialMemoryIndex);
				meshesIndexes[idx + 1u] = divideBy4(faceMemoryIndex - initialMemoryIndex);
				meshesIndexes[idx + 2u] = divideBy4(edgeMemoryIndex - initialMemoryIndex);
			}

			SpPoolMemoryAllocator::main()->disableMemoryAlignment();

#ifdef OPENCL_ENABLED
			cl_event evt;
			_objectMapperGPU->update(_objectMapper, ZERO_UINT, NULL, &evt);
			gpu->releaseEvent(evt);

			SpMesh* lastMesh = _meshes->data()[collisionFeatures(_objectsLength - 1u)->meshIndex];
			SpEdgeMesh* lastEdge = lastMesh->edges->data()[lastMesh->edges->length() - 1u];
			sp_size lastMemoryAddress = (sp_size)&lastEdge->faces.data()[lastEdge->faces.length() - 1u];

			_meshesGPU = sp_mem_new(GpuBufferOpenCL)(gpu);
			_meshesGPU->init(lastMemoryAddress - initialMemoryIndex, _meshes->data()[0], CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR);

			_meshesIndexesGPU = sp_mem_new(GpuBufferOpenCL)(gpu);
			_meshesIndexesGPU->init(_objectsLength * 3u * SIZEOF_UINT, meshesIndexes, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR);

			_meshCacheGPU = sp_mem_new(GpuBufferOpenCL)(gpu);
			_meshCacheGPU->init(vertexCounter * VEC3_SIZE);

			_meshCacheIndexesGPU = sp_mem_new(GpuBufferOpenCL)(gpu);
			_meshCacheIndexesGPU->init(_objectsLength * SIZEOF_UINT, meshCacheIndexes, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR);

			_meshCacheVertexesLengthGPU = sp_mem_new(GpuBufferOpenCL)(gpu);
			_meshCacheVertexesLengthGPU->init(_objectsLength * SIZEOF_UINT, meshCacheVertexesLength, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR);

			_inputLengthGPU->update(&_objectsLength, ZERO_UINT, NULL, &evt);
			gpu->releaseEvent(evt);

			_meshCacheUpdater.init(gpu);
			_meshCacheUpdater.setParameters(_inputLengthGPU, _meshesGPU, _meshesIndexesGPU, _meshCacheVertexesLengthGPU, _transformsGPU, _meshCacheIndexesGPU, _meshCacheGPU, _objectsLength);

			dop18Factory.init(gpu, _inputLengthGPU, _objectsLength, _meshCacheGPU, _meshCacheIndexesGPU, _meshCacheVertexesLengthGPU, _transformsGPU);
			aabbFactory.init(gpu, _inputLengthGPU, _objectsLength, _meshCacheGPU, _meshCacheIndexesGPU, _meshCacheVertexesLengthGPU, _transformsGPU);
			sphereFactory.init(gpu, _inputLengthGPU, _objectsLength, _meshCacheGPU, _meshCacheIndexesGPU, _meshCacheVertexesLengthGPU, _transformsGPU);
#endif
			ALLOC_RELEASE(meshCacheIndexes);
		}

		void updateMeshCache()
		{
			for (sp_uint i = 0u; i < _objectsLength; i++)
				_meshesCache->get(i)->update(mesh(collisionFeatures(i)->meshIndex), transforms(i));
		}

		API_INTERFACE void update(const sp_float elapsedTime);

		API_INTERFACE void init(const sp_uint objectsLength);

		API_INTERFACE inline sp_uint alloc(sp_uint length)
		{
			const sp_uint allocated = _objectsLength;

			for (sp_uint i = _objectsLength; i < _objectsLength + length; i++)
				_objectMapper[i].meshIndex = _objectsLength;

			_objectsLength += length;

			sp_assert(_objectsLength <= _objectsLengthAllocated, "InvalidArgumentException");

			return allocated;
		}

		API_INTERFACE inline sp_uint objectsLength() const
		{
			return _objectsLength;
		}

		API_INTERFACE inline sp_uint objectsLengthAllocated() const
		{
			return _objectsLengthAllocated;
		}

		API_INTERFACE inline SpGpuTextureBuffer* transformsGPU() const
		{
			return _transformsGPUBuffer;
		}

		API_INTERFACE inline SpRigidBody3D* rigidBody3D(const sp_uint index) const
		{
			return &_rigidBodies3D[index];
		}

		API_INTERFACE inline SpTransform* transforms(const sp_uint index) const
		{
			return &_transforms[index];
		}

		API_INTERFACE inline SpCollisionFeatures* collisionFeatures(const sp_uint index) const
		{
			return &_objectMapper[index];
		}

		API_INTERFACE inline void collisionFeatures(const sp_uint index, const sp_uint meshIndex)
		{
			_objectMapper[index].meshIndex = meshIndex;
		}

		API_INTERFACE inline SpMesh* mesh(const sp_uint index) const
		{
			return _meshes->get(index);
		}

		API_INTERFACE inline void mesh(const sp_uint index, SpMesh* mesh)
		{
			_meshes->data()[index] = mesh;
		}

		API_INTERFACE inline SpMeshCache* meshCache(const sp_uint index) const
		{
			return _meshesCache->get(index);
		}

		API_INTERFACE inline void buildDOP18() const
		{
			for (sp_uint i = 0; i < _objectsLength; i++)
			{
				SpMesh* _mesh = mesh(collisionFeatures(i)->meshIndex);
				SpMeshCache* cache = _meshesCache->get(i);

				dop18Factory.build(_mesh, cache, transforms(i)->position, &_boundingVolumes[i]);
			}
		}

		API_INTERFACE inline void buildAABB() const
		{
			for (sp_uint i = 0; i < _objectsLength; i++)
			{
				SpMesh* _mesh = mesh(collisionFeatures(i)->meshIndex);
				SpMeshCache* cache = _meshesCache->get(i);

				aabbFactory.build(_mesh, cache, transforms(i)->position, &_boundingVolumes[i]);
			}
		}

		API_INTERFACE inline DOP18* boundingVolumes(const sp_uint index) const
		{
			return &_boundingVolumes[index];
		}

		/// <summary>
		/// Update transformations and rigid bodies on GPU
		/// </summary>
		API_INTERFACE inline void updateDataOnGPU(cl_event* evt)
		{
			gpu->commandManager->updateBuffer(_transformsGPU, sizeof(SpTransform) * _objectsLength, _transforms, ZERO_UINT, NULL, evt);
		}

		/// <summary>
		/// Update rigid bodies changed by GPU filter response
		/// </summary>
		API_INTERFACE inline cl_event updateDataOnCPU()
		{
			return gpu->commandManager->readBuffer(_rigidBodies3DGPU, sizeof(SpRigidBody3D) * _objectsLength, _rigidBodies3D);
		}

		API_INTERFACE inline void updateTransformsOnGPU()
		{
			sp_size size = sizeof(SpTransform) * _objectsLength;
			sp_double mult = size / 12.0;
			mult = mult - ((int)mult);

			if (mult != ZERO_DOUBLE)  // size must be mulple 12
				if (mult > 0.5)
					size += SIZEOF_WORD;
				else
					size += SIZEOF_TWO_WORDS;

			cl_event evt;
			gpu->commandManager->acquireGLObjects(_transformsGPU, ZERO_UINT, NULL, &evt);
			gpu->releaseEvent(evt);

			_transformsGPUBuffer
				->use()
				->updateData(size, _transforms);
			
			gpu->commandManager->releaseGLObjects(_transformsGPU, ZERO_UINT, NULL, &evt);
			gpu->releaseEvent(evt);
		}

		API_INTERFACE void translate(const sp_uint index, const Vec3& translation)
		{
			sp_assert(_boundingVolumes != nullptr, "InvalidOperationException");
			sp_assert(_transforms != nullptr, "InvalidOperationException");
			sp_assert(index != SP_UINT_MAX, "IndexOutOfRangeException");
			sp_assert(index < _objectsLength, "IndexOutOfRangeException");

			_boundingVolumes[index].translate(translation);
			_transforms[index].translate(translation);
			_rigidBodies3D[index].currentState.translate(translation);
		}

		API_INTERFACE void scale(const sp_uint index, const Vec3& scaleVector)
		{
			sp_assert(_boundingVolumes != nullptr, "InvalidOperationException");
			sp_assert(_transforms != nullptr, "InvalidOperationException");
			sp_assert(index != SP_UINT_MAX, "IndexOutOfRangeException");
			sp_assert(index < _objectsLength, "IndexOutOfRangeException");

			_boundingVolumes[index].scale(scaleVector);
			_transforms[index].scale(scaleVector);
		}

		API_INTERFACE void rotate(const sp_uint index, const Quat& quat)
		{
			sp_assert(_boundingVolumes != nullptr, "InvalidOperationException");
			sp_assert(_transforms != nullptr, "InvalidOperationException");
			sp_assert(index != SP_UINT_MAX, "IndexOutOfRangeException");
			sp_assert(index < _objectsLength, "IndexOutOfRangeException");

			_transforms[index].orientation *= quat;
			_rigidBodies3D[index].currentState.orientation(_transforms[index].orientation);
		}

		API_INTERFACE void position(const sp_uint index, const Vec3& newPosition)
		{
			sp_assert(_boundingVolumes != nullptr, "InvalidOperationException");
			sp_assert(_transforms != nullptr, "InvalidOperationException");
			sp_assert(index != SP_UINT_MAX, "IndexOutOfRangeException");
			sp_assert(index < _objectsLength, "IndexOutOfRangeException");

			Vec3 diff = _transforms[index].position - newPosition;

			_boundingVolumes[index].translate(diff);
			_transforms[index].position = newPosition;
			_rigidBodies3D[index].currentState.position(newPosition);
		}

		API_INTERFACE void orientation(const sp_uint index, const Quat& newOrientation)
		{
			sp_assert(_boundingVolumes != nullptr, "InvalidOperationException");
			sp_assert(_transforms != nullptr, "InvalidOperationException");
			sp_assert(index != SP_UINT_MAX, "IndexOutOfRangeException");
			sp_assert(index < _objectsLength, "IndexOutOfRangeException");

			_transforms[index].orientation = newOrientation;
			_rigidBodies3D[index].currentState.orientation(newOrientation);
		}

		API_INTERFACE void dispose();

		~SpWorld()
		{
			dispose();
		}

	};
}

#endif // SP_OBJECTS_MANAGER_HEADER