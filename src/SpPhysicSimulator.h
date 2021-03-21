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
#include "SpCollisionResponseGPU.h"
#include "SpCollisionFeatures.h"
#include "SpCollisionDetector.h"
#include "SpGpuRenderingFactory.h"
#include "SpCollisionResponse.h"
#include "SpPhysicIntegrator.h"
#include "SpCollisionGroup.h"
#include "SpSphereBoundingVolumeFactory.h"
#include "SpDOP18Factory.h"
#include "SpAABBFactory.h"
#include "GpuBufferOpenCL.h"
#include "SpMeshCacheUpdaterGPU.h"
#include "SpCollisionResponseShapeMatching.h"

namespace NAMESPACE_PHYSICS
{
	class SpPhysicSimulator
	{
	private:
		GpuDevice* gpu;
		SweepAndPrune* sapDOP18;
		SweepAndPrune* sapAABB;
		SweepAndPrune* sapSphere;
		SpCollisionResponseGPU* collisionResponseGPU;

		sp_uint _objectsLengthAllocated;
		sp_uint _objectsLength;
		
		DOP18* _boundingVolumes;
		SpPhysicProperties* _physicProperties;
		SpTransform* _transforms;
		SpCollisionFeatures* _objectMapper;
		SpArray<SpMesh*>* _meshes;
		SpArray<SpMeshCache*>* _meshesCache;
		
		cl_event lastEvent;
		cl_mem _transformsGPU;
		SpGpuTextureBuffer* _transformsGPUBuffer;
		cl_mem _boundingVolumesGPU;
		cl_mem _physicPropertiesGPU;
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
		GpuBufferOpenCL* _objectMapperGPU;
		SpSphereBoundingVolumeFactory sphereFactory;
		SpAABBFactory aabbFactory;
		SpDOP18Factory dop18Factory;
		SpMeshCacheUpdaterGPU _meshCacheUpdater;

		Timer timerToPhysic;

		SpPhysicSimulator(sp_uint objectsLength);

		inline void dispatchEvent(SpCollisionDetails* details)
		{
			SpCollisionEvent* evt = sp_mem_new(SpCollisionEvent);
			evt->indexBody1 = details->objIndex1;
			evt->indexBody2 = details->objIndex2;

			SpEventDispatcher::instance()->push(evt);
		}

		void addFriction(SpPhysicProperties* obj1Properties, SpPhysicProperties* obj2Properties, const Vec3& relativeVel, const Vec3& collisionNormal, const Vec3& rayToContactObj1, const Vec3& rayToContactObj2, const sp_float& j);

		void findCollisionsCpu(SweepAndPruneResult* result);
		void findCollisionsGpuDOP18(SweepAndPruneResult* result);
		void findCollisionsGpuAABB(SweepAndPruneResult* result);
		void findCollisionsGpuSphere(SweepAndPruneResult& result);
		
		/// <summary>
		/// Update transformations and physicproperties on GPU
		/// </summary>
		void updateDataOnGPU()
		{
			gpu->commandManager->updateBuffer(_transformsGPU, sizeof(SpTransform) * _objectsLength, _transforms);
			sapDOP18->updatePhysicProperties(_physicProperties);
			sapAABB->updatePhysicProperties(_physicProperties);
			sapSphere->updatePhysicProperties(_physicProperties);
		}

		/// <summary>
		/// Update physic properties changed by GPU filter response
		/// </summary>
		void updateDataOnCPU()
		{
			cl_event evt1 = gpu->commandManager->readBuffer(_physicPropertiesGPU, sizeof(SpPhysicProperties) * _objectsLength, _physicProperties);
			gpu->waitEvents(ONE_UINT, &evt1);
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
			sp_size size = sizeof(SpTransform) * _objectsLength;
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

		API_INTERFACE static SpPhysicSimulator* init(sp_uint objectsLength);

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

		API_INTERFACE inline DOP18* boundingVolumes(const sp_uint index) const
		{
			return &_boundingVolumes[index];
		}

		API_INTERFACE inline SpPhysicProperties* physicProperties(const sp_uint index) const
		{
			return &_physicProperties[index];
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

		API_INTERFACE inline void initMeshCache();
		API_INTERFACE inline void updateMeshCache();

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
			SpPhysicProperties* element = &_physicProperties[index];

			Vec3 translation;
			diff(element->previousState.position(), element->currentState.position(), translation);

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
			sp_assert(index < _objectsLength, "IndexOutOfRangeException");

			_boundingVolumes[index].translate(translation);
			_transforms[index].translate(translation);
			_physicProperties[index].currentState.translate(translation);
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
			_physicProperties[index].currentState.orientation(_transforms[index].orientation);
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
			_physicProperties[index].currentState.position(newPosition);
		}

		API_INTERFACE void orientation(const sp_uint index, const Quat& newOrientation)
		{
			sp_assert(_boundingVolumes != nullptr, "InvalidOperationException");
			sp_assert(_transforms != nullptr, "InvalidOperationException");
			sp_assert(index != SP_UINT_MAX, "IndexOutOfRangeException");
			sp_assert(index < _objectsLength, "IndexOutOfRangeException");

			_transforms[index].orientation = newOrientation;
			_physicProperties[index].currentState.orientation(newOrientation);
		}

		API_INTERFACE void run(const sp_float elapsedTime);

		API_INTERFACE void groupCollisions(const SweepAndPruneResult& sapResult, SpCollisionGroups* collisionGroups);

		API_INTERFACE void moveAwayDynamicObjects()
		{
			for (sp_uint i = 0; i < _objectsLength; i++)
			{
				if (_physicProperties[i].isStatic())
					continue;

				DOP18 bv1 = _boundingVolumes[i];

				for (sp_uint j = i + 1u; j < _objectsLength; j++)
				{
					if (_physicProperties[j].isStatic())
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