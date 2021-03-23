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
#include "SpWorldManager.h"

namespace NAMESPACE_PHYSICS
{
	class SpPhysicSimulator
	{
	private:
		GpuDevice* gpu;
		
		cl_event lastEvent;
		cl_mem _collisionIndexesGPU;
		cl_mem _collisionIndexesLengthGPU;
		cl_mem _sapCollisionIndexesGPU;
		cl_mem _sapCollisionIndexesLengthGPU;
	
		// collision detection
		SweepAndPrune* sapDOP18;
		SweepAndPrune* sapAABB;
		SweepAndPrune* sapSphere;
		SpCollisionResponseGPU* collisionResponseGPU;

		Timer timerToPhysic;

		SpPhysicSimulator();

		inline void dispatchEvent(SpCollisionDetails* details)
		{
			SpCollisionEvent* evt = sp_mem_new(SpCollisionEvent);
			evt->indexBody1 = details->objIndex1;
			evt->indexBody2 = details->objIndex2;

			SpEventDispatcher::instance()->push(evt);
		}

		void addFriction(SpRigidBody3D* obj1Properties, SpRigidBody3D* obj2Properties, const Vec3& relativeVel, const Vec3& collisionNormal, const Vec3& rayToContactObj1, const Vec3& rayToContactObj2, const sp_float& j);

		void findCollisionsCpu(SweepAndPruneResult* result);
		void findCollisionsGpuDOP18(SweepAndPruneResult* result);
		void findCollisionsGpuAABB(SweepAndPruneResult* result);
		void findCollisionsGpuSphere(SweepAndPruneResult& result);
		
		/// <summary>
		/// Update transformations and physicproperties on GPU
		/// </summary>
		void updateDataOnGPU()
		{
			SpWorld* world = SpWorldManagerInstance->current();

			sapDOP18->updatePhysicProperties(world->_rigidBodies3D);
			sapAABB->updatePhysicProperties(world->_rigidBodies3D);
			sapSphere->updatePhysicProperties(world->_rigidBodies3D);
		}

		static void handleCollisionCPU(void* collisionParamter);
		static void handleCollisionGPU(void* collisionParamter);

	public:
		SpPhysicIntegrator* integrator;

		API_INTERFACE static SpPhysicSimulator* instance();

		API_INTERFACE static SpPhysicSimulator* init();

		/// <summary>
		/// Back the object to the state before timestep
		/// </summary>
		API_INTERFACE inline void backToTime(const sp_uint index)
		{
			SpWorld* world = SpWorldManagerInstance->current();

			SpRigidBody3D* element = &world->_rigidBodies3D[index];

			Vec3 translation;
			diff(element->previousState.position(), element->currentState.position(), translation);

			element->rollbackState();
			
			world->_transforms[index].position = element->currentState.position();
			world->_transforms[index].orientation = element->currentState.orientation();
		
			world->_boundingVolumes[index].translate(translation);
		}

		API_INTERFACE void run(const sp_float elapsedTime);

		API_INTERFACE void groupCollisions(const SweepAndPruneResult& sapResult, SpCollisionGroups* collisionGroups);

		API_INTERFACE void moveAwayDynamicObjects()
		{
			SpWorld* world = SpWorldManagerInstance->current();

			for (sp_uint i = 0; i < world->objectsLength(); i++)
			{
				if (world->_rigidBodies3D[i].isStatic())
					continue;

				DOP18 bv1 = world->_boundingVolumes[i];

				for (sp_uint j = i + 1u; j < world->objectsLength(); j++)
				{
					if (world->_rigidBodies3D[j].isStatic())
						continue;

					if (bv1.collisionStatus(world->_boundingVolumes[j]) != CollisionStatus::OUTSIDE)
						world->translate(j, Vec3(0.0f, world->_boundingVolumes[j].height() + 0.1f, 0.0f));
				}
			}
		}

		API_INTERFACE void dispose();

		~SpPhysicSimulator();

	};
}

#endif // SP_PHYSIC_SIMULATOR_HEADER