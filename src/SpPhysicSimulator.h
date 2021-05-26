#ifndef SP_PHYSIC_SIMULATOR_HEADER
#define SP_PHYSIC_SIMULATOR_HEADER

#include "SpectrumPhysics.h"
#include "GpuContext.h"
#include "SweepAndPrune.h"
#include "SpEventDispatcher.h"
#include "SpPhysicObject.h"
#include "Timer.h"
#include "SpPhysicSettings.h"
#include "SpCollisionDetails.h"
#include "SpThreadPool.h"
#include "SpCollisionResponseGPU.h"
#include "SpCollisionDetector.h"
#include "SpCollisionResponse.h"
#include "SpPhysicIntegrator.h"
#include "SpCollisionGroup.h"
#include "SpSphereBoundingVolumeFactory.h"
#include "SpDOP18Factory.h"
#include "SpAABBFactory.h"
#include "GpuBufferOpenCL.h"
#include "SpCollisionResponseShapeMatching.h"
#include "SpGlobalProperties.h"

namespace NAMESPACE_PHYSICS
{
	class SpPhysicSimulator
	{
	private:
		GpuDevice* gpu;
				
		cl_mem _collisionIndexesGPU;
		cl_mem _collisionIndexesLengthGPU;
		cl_mem _sapCollisionIndexesGPU;
		cl_mem _sapCollisionIndexesLengthGPU;
	
		SweepAndPrune* sap;
		SpCollisionResponseGPU* collisionResponseGPU;

		Timer timerToPhysic;

		inline void dispatchEvent(SpCollisionDetails* details)
		{
			SpCollisionEvent* evt = sp_mem_new(SpCollisionEvent);
			evt->indexBody1 = details->objIndex1;
			evt->indexBody2 = details->objIndex2;

			SpEventDispatcher::instance()->push(evt);
		}

		void addFriction(SpRigidBody3D* obj1Properties, SpRigidBody3D* obj2Properties, const Vec3& relativeVel, const Vec3& collisionNormal, const Vec3& rayToContactObj1, const Vec3& rayToContactObj2, const sp_float& j);

		void findCollisionsCpu(SweepAndPruneResult* result);
		void findCollisionsGpu(SweepAndPruneResult& result, const sp_uint previousEventsLength, cl_event* previousEvents, cl_event* currentEvent);
		
		/// <summary>
		/// Update transformations and physicproperties on GPU
		/// </summary>
		void updateDataOnGPU();

		static void handleCollisionCPU(void* collisionParamter);
		static void handleCollisionGPU(void* collisionParamter);

	public:

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns></returns>
		API_INTERFACE SpPhysicSimulator() 
		{
		}

		SpPhysicIntegrator* integrator;

		API_INTERFACE void init();

		/// <summary>
		/// Back the object to the state before timestep
		/// </summary>
		API_INTERFACE void backToTime(const sp_uint index);

		API_INTERFACE void run(const sp_float elapsedTime);

		API_INTERFACE void groupCollisions(const SweepAndPruneResult& sapResult, SpCollisionGroups* collisionGroups);

		API_INTERFACE void moveAwayDynamicObjects();

		API_INTERFACE void dispose();

		~SpPhysicSimulator();

	};
}

#endif // SP_PHYSIC_SIMULATOR_HEADER