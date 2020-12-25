#ifndef SP_PHYSIC_SIMULATOR_HEADER
#define SP_PHYSIC_SIMULATOR_HEADER

#include "SpectrumPhysics.h"
#include "GpuContext.h"
#include "DOP18.h"
#include "SweepAndPrune.h"
#include "SpEventDispatcher.h"
#include "SpPhysicObject.h"
#include "CL/cl.h"
#include "Timer.h"
#include "SpPhysicSettings.h"
#include "SpCollisionDetails.h"
#include "SpThreadPool.h"
#include "SpRigidBodyMapper.h"
#include "SpCollisionDetector.h"
#include "SpGpuRenderingFactory.h"
#include "SpCollisionResponse.h"
#include "SpPhysicIntegrator.h"
#include "SpCollisionGroup.h"
#include "SpDOP18Factory.h"
#include "SpAABBFactory.h"
#include "GpuBufferOpenCL.h"
#include "SpMeshCacheUpdaterGPU.h"
#include "SpSoftBody.h"

namespace NAMESPACE_PHYSICS
{
	class SpBodyMapper 
	{
	public:
		SpBodyType type;
		sp_uint index;

		API_INTERFACE inline SpBodyMapper()
		{
			type = SpBodyType::Unknonw;
			index = SP_UINT_MAX;
		}
	};

	class SpPhysicSimulator
	{
	private:
		GpuDevice* gpu;
		SweepAndPrune* sapDOP18;
		SweepAndPrune* sapAABB;
		
		sp_uint _objectsMaxLength, _objectsLengthAllocated;
		sp_uint _rigidBodiesMaxLength, _rigidBodiesAllocated;
		sp_uint _softBodiesMaxLength, _softBodiesAllocated;
		
		DOP18* _boundingVolumes;
		SpRigidBody* _rigidBodies;
		SpSoftBody* _softBodies;
		SpBodyMapper* _bodyMapper;
		SpTransform* _transforms;
		SpRigidBodyMapper* _rigidBodyMapper;
		SpArray<SpMesh*>* _meshes;
		SpArray<SpMeshCache*>* _meshesCache;
		
		cl_event lastEvent;
		cl_mem _transformsGPU;
		SpGpuTextureBuffer* _transformsGPUBuffer;
		cl_mem _boundingVolumesGPU;
		GpuBufferOpenCL* _rigidBodiesGPU;
		GpuBufferOpenCL* _softBodiesGPU;
		GpuBufferOpenCL* _softBodyIndexesGPU;
		cl_mem _collisionIndexesGPU;
		cl_mem _collisionIndexesLengthGPU;
		cl_mem _sapCollisionIndexesGPU;
		cl_mem _sapCollisionIndexesLengthGPU;
		GpuBufferOpenCL* _inputLengthGPU;
		GpuBufferOpenCL* _meshesGPU;
		GpuBufferOpenCL* _meshesIndexesGPU;
		GpuBufferOpenCL* _meshCacheGPU;
		GpuBufferOpenCL* _meshCacheIndexesGPU;
		GpuBufferOpenCL* _meshCacheVertexesLengthGPU;
		GpuBufferOpenCL* _bodyMapperGPU;
		GpuBufferOpenCL* _rigidBodyMapperGPU;
		SpAABBFactory aabbFactory;
		SpDOP18Factory dop18Factory;
		SpMeshCacheUpdaterGPU _meshCacheUpdater;

		Timer timerToPhysic;

		SpPhysicSimulator(const sp_uint rigidBodiesLength, const sp_uint softbodiesLength);

		inline void dispatchEvent(SpCollisionDetails* details)
		{
			SpCollisionEvent* evt = sp_mem_new(SpCollisionEvent);
			evt->indexBody1 = details->objIndex1;
			evt->indexBody2 = details->objIndex2;

			SpEventDispatcher::instance()->push(evt);
		}

		void addFriction(SpRigidBody* obj1Properties, SpRigidBody* obj2Properties, const Vec3& relativeVel, const Vec3& collisionNormal, const Vec3& rayToContactObj1, const Vec3& rayToContactObj2, const sp_float& j);

		void findCollisionsCpu(SweepAndPruneResult* result);
		void findCollisionsGpuDOP18(SweepAndPruneResult* result);
		void findCollisionsGpuAABB(SweepAndPruneResult* result);
		
		/// <summary>
		/// Update transformations and physicproperties on GPU
		/// </summary>
		void updateDataOnGPU()
		{
			if (_objectsMaxLength != ZERO_UINT)
			{
				gpu->commandManager->updateBuffer(_transformsGPU, sizeof(SpTransform) * _objectsMaxLength, _transforms);
				
				if (_rigidBodiesMaxLength != ZERO_UINT)
					gpu->commandManager->updateBuffer(_rigidBodiesGPU->buffer(), _rigidBodiesGPU->size(), _rigidBodies);

				if (_softBodiesMaxLength != ZERO_UINT)
					gpu->commandManager->updateBuffer(_softBodiesGPU->buffer(), _softBodiesGPU->size(), _softBodies);
			}
		}

		static void handleCollisionCPU(void* collisionParamter);
		static void handleCollisionGPU(void* collisionParamter);

		void buildDOP18() const;
		void buildAABB() const;

		void initMeshCacheIndexes();

	public:
		SpPhysicIntegrator* integrator;

		API_INTERFACE static SpPhysicSimulator* instance();

		API_INTERFACE inline void updateTransformsOnGPU()
		{
			sp_size size = sizeof(SpTransform) * _objectsMaxLength;
			sp_double mult = size / 12.0;
			mult = mult - ((int)mult);

			if (mult != ZERO_DOUBLE)  // size must be mulple 12
				if (mult > 0.5)
					size += SIZEOF_WORD;
				else
					size += SIZEOF_TWO_WORDS;
			
			gpu->commandManager->acquireGLObjects(_transformsGPU);

			_transformsGPUBuffer
				->use()
				->updateData(size, _transforms);

			gpu->commandManager->releaseGLObjects(_transformsGPU);
		}

		API_INTERFACE static SpPhysicSimulator* init(const sp_uint rigidBodiesLength, const sp_uint softbodiesLength);

		API_INTERFACE inline sp_uint allocRigidBody(const sp_uint length, sp_uint* globalBodyIndex)
		{
			sp_assert(_objectsLengthAllocated + length <= _objectsMaxLength, "InvalidArgumentException");
			sp_assert(_rigidBodiesAllocated + length <= _rigidBodiesMaxLength, "InvalidArgumentException");

			globalBodyIndex[0] = _objectsLengthAllocated;
			const sp_uint allocated = _rigidBodiesAllocated;

			for (sp_uint i = _rigidBodiesAllocated; i < _rigidBodiesAllocated + length; i++)
			{
				_bodyMapper[_objectsLengthAllocated + i - _rigidBodiesAllocated].type = SpBodyType::Rigid;
				_bodyMapper[_objectsLengthAllocated + i - _rigidBodiesAllocated].index = i;
				_rigidBodyMapper[i].meshIndex = allocated;
			}

			_objectsLengthAllocated += length;
			_rigidBodiesAllocated += length;

			return allocated;
		}

		API_INTERFACE inline sp_uint allocSoftBody(const sp_uint length, sp_uint* globalBodyIndex)
		{
			sp_assert(_objectsLengthAllocated + length <= _objectsMaxLength, "InvalidArgumentException");
			sp_assert(_softBodiesAllocated + length <= _softBodiesMaxLength, "InvalidArgumentException");

			globalBodyIndex[0] = _objectsLengthAllocated;
			const sp_uint allocated = _softBodiesAllocated;

			for (sp_uint i = 0; i < length; i++)
			{
				_bodyMapper[_objectsLengthAllocated + i].type = SpBodyType::Soft;
				_bodyMapper[_objectsLengthAllocated + i].index = _softBodiesAllocated + i;
			}

			_objectsLengthAllocated += length;
			_softBodiesAllocated += length;

			return allocated;
		}

		API_INTERFACE inline sp_uint objectsMaxLength() const
		{
			return _objectsMaxLength;
		}

		API_INTERFACE inline sp_uint objectsLengthAllocated() const
		{
			return _objectsLengthAllocated;
		}

		API_INTERFACE inline DOP18* boundingVolumes(const sp_uint index) const
		{
			return &_boundingVolumes[index];
		}

		API_INTERFACE inline SpRigidBody* rigidBodies(const sp_uint index) const
		{
			return &_rigidBodies[index];
		}

		API_INTERFACE inline SpSoftBody* softBodies(const sp_uint index) const
		{
			return &_softBodies[index];
		}

		API_INTERFACE inline SpTransform* transforms(const sp_uint index) const
		{
			return &_transforms[index];
		}

		API_INTERFACE inline void initGPUData()
		{
			_bodyMapperGPU->update(_bodyMapper);
		}

		API_INTERFACE inline void initSoftBodyIndexes()
		{
			if (_softBodiesAllocated == ZERO_UINT)
				return;

			sp_uint* indexes = ALLOC_NEW_ARRAY(sp_uint, _softBodiesAllocated);
			indexes[0] = ZERO_UINT;

			for (sp_uint i = 1u; i < _softBodiesAllocated; i++)
				indexes[i] = divideBy4(_softBodies[i].size());

			_softBodyIndexesGPU->update(indexes);
		}
		
		API_INTERFACE inline SpRigidBodyMapper* rigidBodyMapper(const sp_uint index) const
		{
			return &_rigidBodyMapper[index];
		}
		API_INTERFACE inline void rigidBodyMapperMesh(const sp_uint index, const sp_uint meshIndex)
		{
			_rigidBodyMapper[index].meshIndex = meshIndex;
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

		API_INTERFACE inline void initMeshCache();
		API_INTERFACE inline void updateMeshCache();

		API_INTERFACE inline void initBoundingVolumeFactory()
		{
			dop18Factory.init(gpu, _inputLengthGPU, _bodyMapperGPU, _rigidBodiesGPU, _softBodiesGPU, _softBodyIndexesGPU, _objectsMaxLength, _meshCacheGPU, _meshCacheIndexesGPU, _meshCacheVertexesLengthGPU, _transformsGPU, _boundingVolumesGPU);
			aabbFactory.init(gpu, _inputLengthGPU, _bodyMapperGPU, _rigidBodiesGPU, _softBodiesGPU, _softBodyIndexesGPU, _objectsMaxLength, _meshCacheGPU, _meshCacheIndexesGPU, _meshCacheVertexesLengthGPU, _transformsGPU, _boundingVolumesGPU);
		}

		/// <summary>
		/// Update MeshCache structure on GPU, using transformations objects
		/// </summary>
		/// <returns>MeshCache structure updated on GPU </returns>
		API_INTERFACE inline void updateMeshCacheGPU();
		
		API_INTERFACE inline SpGpuTextureBuffer* transformsGPU() const
		{
			return _transformsGPUBuffer;
		}

		/// <summary>
		/// Back the object to the state before timestep
		/// </summary>
		API_INTERFACE inline void backToTime(const sp_uint index)
		{
			SpRigidBody* element = &_rigidBodies[index];

			Vec3 translation;
			diff(element->previousState.position(), element->currentState.position(), &translation);

			element->rollbackState();
			
			_transforms[index].position = element->currentState.position();
			_transforms[index].orientation = element->currentState.orientation();
		
			_boundingVolumes[index].translate(translation);
		}

		API_INTERFACE void translate(const sp_uint index, const Vec3& translation) 
		{
			sp_assert(_boundingVolumes != nullptr, "InvalidOperationException");
			sp_assert(_transforms != nullptr, "InvalidOperationException");
			sp_assert(index != SP_UINT_MAX, "IndexOutOfRangeException");
			sp_assert(index < _objectsMaxLength, "IndexOutOfRangeException");

			_boundingVolumes[index].translate(translation);
			_transforms[index].translate(translation);
			_rigidBodies[index].currentState.translate(translation);
		}

		API_INTERFACE void scale(const sp_uint index, const Vec3& scaleVector)
		{
			sp_assert(_boundingVolumes != nullptr, "InvalidOperationException");
			sp_assert(_transforms != nullptr, "InvalidOperationException");
			sp_assert(index != SP_UINT_MAX, "IndexOutOfRangeException");
			sp_assert(index < _objectsMaxLength, "IndexOutOfRangeException");

			_boundingVolumes[index].scale(scaleVector);
			_transforms[index].scale(scaleVector);
		}

		API_INTERFACE void rotate(const sp_uint index, const Quat& quat)
		{
			sp_assert(_boundingVolumes != nullptr, "InvalidOperationException");
			sp_assert(_transforms != nullptr, "InvalidOperationException");
			sp_assert(index != SP_UINT_MAX, "IndexOutOfRangeException");
			sp_assert(index < _objectsMaxLength, "IndexOutOfRangeException");

			_transforms[index].orientation *= quat;
			_rigidBodies[index].currentState.orientation(_transforms[index].orientation);
		}

		API_INTERFACE void position(const sp_uint index, const Vec3& newPosition)
		{
			sp_assert(_boundingVolumes != nullptr, "InvalidOperationException");
			sp_assert(_transforms != nullptr, "InvalidOperationException");
			sp_assert(index != SP_UINT_MAX, "IndexOutOfRangeException");
			sp_assert(index < _objectsMaxLength, "IndexOutOfRangeException");

			Vec3 diff = _transforms[index].position - newPosition;

			_boundingVolumes[index].translate(diff);
			_transforms[index].position = newPosition;
			_rigidBodies[index].currentState.position(newPosition);
		}

		API_INTERFACE void orientation(const sp_uint index, const Quat& newOrientation)
		{
			sp_assert(_boundingVolumes != nullptr, "InvalidOperationException");
			sp_assert(_transforms != nullptr, "InvalidOperationException");
			sp_assert(index != SP_UINT_MAX, "IndexOutOfRangeException");
			sp_assert(index < _objectsMaxLength, "IndexOutOfRangeException");

			_transforms[index].orientation = newOrientation;
			_rigidBodies[index].currentState.orientation(newOrientation);
		}

		API_INTERFACE void run(const sp_float elapsedTime);

		API_INTERFACE void groupCollisions(const SweepAndPruneResult& sapResult, SpCollisionGroups* collisionGroups);

		API_INTERFACE void moveAwayDynamicObjects()
		{
			for (sp_uint i = 0; i < _objectsMaxLength; i++)
			{
				if (_rigidBodies[i].isStatic())
					continue;

				DOP18 bv1 = _boundingVolumes[i];

				for (sp_uint j = i + 1u; j < _objectsMaxLength; j++)
				{
					if (_rigidBodies[j].isStatic())
						continue;

					if (bv1.collisionStatus(_boundingVolumes[j]) != CollisionStatus::OUTSIDE)
						translate(j, Vec3(0.0f, _boundingVolumes[j].height() + 0.1f, 0.0f));
				}
			}
		}

		API_INTERFACE void dispose();

		~SpPhysicSimulator();

	};
}

#endif // SP_PHYSIC_SIMULATOR_HEADER