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
#include "SpPhysicSyncronizer.h"
#include "Ray.h"
#include "SpThreadPool.h"

#include <iostream>

namespace NAMESPACE_PHYSICS
{
	class SpHandleCollisionParameter
	{
	public:
		sp_uint objIndex1;
		sp_uint objIndex2;
		sp_float elapsedTime;
	};

	class SpPhysicSimulator
	{
	private:
		GpuDevice* gpu;
		SweepAndPrune* sap;

		sp_uint _objectsLengthAllocated;
		sp_uint _objectsLength;

		DOP18* _boundingVolumes;
		SpPhysicProperties* _physicProperties;

		cl_event lastEvent;
		cl_mem _boundingVolumesGPU;
		cl_mem _physicPropertiesGPU;

		SpPhysicSimulator() 
		{
			lastEvent = nullptr;
			_boundingVolumesGPU = nullptr;
			_physicPropertiesGPU = nullptr;
		}

		inline void dispatchEvent(SpCollisionDetails* details)
		{
			SpCollisionEvent* evt = sp_mem_new(SpCollisionEvent);
			evt->indexBody1 = details->objIndex1;
			evt->indexBody2 = details->objIndex2;

			SpEventDispatcher::instance()->push(evt);
		}

		void timeOfCollision(SpCollisionDetails* details);

		void collisionDetails(SpCollisionDetails* details);
		void collisionDetailsPlanesDownUp(const sp_uint objIndex1, const sp_uint objIndex2, SpCollisionDetails* details);
		void collisionDetailsPlanesRightLeft(const sp_uint objIndex1, const sp_uint objIndex2, SpCollisionDetails* details);
		void collisionDetailsPlanesDepthFront(const sp_uint objIndex1, const sp_uint objIndex2, SpCollisionDetails* details);
		void collisionDetailsPlanesRightDepthAndLeftFront(const sp_uint objIndex1, const sp_uint objIndex2, SpCollisionDetails* details);
		void collisionDetailsPlanesRightFrontAndLeftDepth(const sp_uint objIndex1, const sp_uint objIndex2, SpCollisionDetails* details);
		void collisionDetailsPlanesUpLeftAndDownRight(const sp_uint objIndex1, const sp_uint objIndex2, SpCollisionDetails* details);
		void collisionDetailsPlanesUpRightAndDownLeft(const sp_uint objIndex1, const sp_uint objIndex2, SpCollisionDetails* details);
		void collisionDetailsPlanesUpFrontAndDownDepth(const sp_uint objIndex1, const sp_uint objIndex2, SpCollisionDetails* details);
		void collisionDetailsPlanesUpDepthAndDownFront(const sp_uint objIndex1, const sp_uint objIndex2, SpCollisionDetails* details);

		void handleCollisionResponse(SpCollisionDetails* details);

		void findCollisionsCpu(SweepAndPruneResult* result);
		void findCollisionsGpu(SweepAndPruneResult* result);

		void updateDataOnGPU();

		static void handleCollision(void* collisionParamter);

	public:

		SpPhysicSyncronizer* syncronizer;

		API_INTERFACE static SpPhysicSimulator* instance();

		API_INTERFACE static void init(sp_uint objectsLength);

		API_INTERFACE inline sp_uint alloc(sp_uint length)
		{
			sp_uint allocated = _objectsLength;
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

		API_INTERFACE inline BoundingVolume* boundingVolumes(const sp_uint index) const
		{
			return &_boundingVolumes[index];
		}

		API_INTERFACE inline SpPhysicProperties* physicProperties(const sp_uint index) const
		{
			return &_physicProperties[index];
		}

		API_INTERFACE static void integrate(const sp_uint index, sp_float elapsedTime)
		{
			SpPhysicSimulator* simulator = SpPhysicSimulator::instance();

			sp_assert(elapsedTime > ZERO_FLOAT, "InvalidArgumentException");
			sp_assert(index >= ZERO_UINT, "IndexOutOfRangeException");
			sp_assert(index < simulator->_objectsLength, "IndexOutOfRangeException");
			
			SpPhysicSettings* settings = SpPhysicSettings::instance();
			SpPhysicProperties* element = &simulator->_physicProperties[index];

			const sp_float drag = 0.1f; // rho*C*Area - simplified drag for this example
			const Vec3 dragForce = (element->velocity() * element->velocity().abs()) * HALF_FLOAT * drag;

			elapsedTime = elapsedTime * settings->physicVelocity();

			// Velocity Verlet Integration because regards the velocity
			const Vec3 newPosition = element->position()
				+ element->velocity() * elapsedTime
				+ element->acceleration()  * (elapsedTime * elapsedTime * HALF_FLOAT);

			const Vec3 newAcceleration = (element->force() - dragForce) * element->massInverse();

			Vec3 newVelocity = element->velocity()
				+ ((element->acceleration() + newAcceleration) * elapsedTime * HALF_FLOAT);
			newVelocity *= element->damping();

			const Quat angularVelocity = Quat(0.0f, element->inertialTensorInverse() * element->torque());
			element->_orientation += angularVelocity * element->orientation() * HALF_FLOAT * elapsedTime;
			element->_orientation = element->orientation().normalize();


			const Vec3 translation = newPosition - element->position();
			simulator->_boundingVolumes[index].translate(translation); // Sync with the bounding Volume
			simulator->syncronizer->sync(index, translation, element->orientation());

			element->_previousAcceleration = element->acceleration();
			element->_acceleration = newAcceleration;

			element->_previousVelocity = element->velocity();
			element->_velocity = newVelocity;

			element->_previousPosition = element->position();
			element->_position = newPosition;

			element->_previousForce = element->force();
			element->_force = ZERO_FLOAT;

			element->_previousTorque = element->torque();
			element->_torque *= element->angularDamping();
		}

		/// <summary>
		/// Back the object to the state before timestep
		/// </summary>
		API_INTERFACE inline static void backToTime(const sp_uint index)
		{
			SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
			SpPhysicProperties* element = &simulator->_physicProperties[index];
			DOP18* dop = &simulator->_boundingVolumes[index];

			const Vec3 translation = element->previousPosition() - element->position();

			dop->translate(translation);
			simulator->syncronizer->sync(index, translation, element->previousOrientation());

			element->rollbackState();
		}

		API_INTERFACE sp_bool areMovingAway(sp_uint objIndex1, sp_uint objIndex2)
		{
			const SpPhysicProperties* obj1Properties = &_physicProperties[objIndex1];
			const SpPhysicProperties* obj2Properties = &_physicProperties[objIndex2];

			Vec3 lineOfAction = obj2Properties->position() - obj1Properties->position();
			const Vec3 velocityToObject2 = obj1Properties->velocity() * lineOfAction;

			lineOfAction = obj1Properties->position() - obj2Properties->position();
			const Vec3 velocityToObject1 = obj2Properties->velocity() * lineOfAction;

			return velocityToObject2 <= ZERO_FLOAT && velocityToObject1 <= ZERO_FLOAT;
		}

		API_INTERFACE void run(const Timer& timer);

		API_INTERFACE void dispose();

		~SpPhysicSimulator();

	};
}

#endif // SP_PHYSIC_SIMULATOR_HEADER