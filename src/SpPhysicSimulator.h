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

namespace NAMESPACE_PHYSICS
{
	class SpPhysicSimulator
	{
	private:
		GpuDevice* gpu;
		SweepAndPrune* sap;

		sp_uint objectsLengthAllocated;
		sp_uint objectsLength;

		cl_mem boundingVolumeBuffer = nullptr;
		DOP18* _boundingVolumes;
		SpPhysicProperties* _physicProperties;

		SpPhysicSimulator() { }

		inline void dispatchEvent(const sp_uint objIndex1, const sp_uint objIndex2)
		{
			SpCollisionEvent* evt = sp_mem_new(SpCollisionEvent)();
			evt->indexBody1 = objIndex1;
			evt->indexBody2 = objIndex2;

			SpEventDispatcher::instance()->push(evt);
		}

		sp_float timeOfCollision(const sp_uint objIndex1, const sp_uint objIndex2, sp_float elapsedTime);

		void collisionDetails(const sp_uint objIndex1, const sp_uint objIndex2, sp_float elapsedTime, SpCollisionDetails* details);
		void collisionDetailsPlanesDownUp(const sp_uint objIndex1, const sp_uint objIndex2, SpCollisionDetails* details);
		void collisionDetailsPlanesLeftRight(const sp_uint objIndex1, const sp_uint objIndex2, SpCollisionDetails* details);
		//void collisionDetailsPlanesFrontDepth(const sp_uint objIndex1, const sp_uint objIndex2, SpCollisionDetails* details);

		void handleCollisionResponse(sp_uint objIndex1, sp_uint objIndex2, const SpCollisionDetails& details);

		void handleCollision(const sp_uint objIndex1, const sp_uint objIndex2, sp_float elapsedTime);

		void findCollisions(SweepAndPruneResultCpu* result);

	public:

		SpPhysicSyncronizer* syncronizer;

		API_INTERFACE static SpPhysicSimulator* instance();

		API_INTERFACE static void init(sp_uint objectsLength);

		API_INTERFACE sp_uint alloc(sp_uint length);

		API_INTERFACE inline BoundingVolume* boundingVolumes(const sp_uint index) const
		{
			return &_boundingVolumes[index];
		}

		API_INTERFACE inline SpPhysicProperties* physicProperties(const sp_uint index) const
		{
			return &_physicProperties[index];
		}

		API_INTERFACE inline void integrate(const sp_uint index, sp_float elapsedTime)
		{
			sp_assert(elapsedTime > ZERO_FLOAT, "InvalidArgumentException");
			sp_assert(index >= ZERO_UINT, "IndexOutOfRangeException");
			sp_assert(index < objectsLength, "IndexOutOfRangeException");

			SpPhysicSettings* settings = SpPhysicSettings::instance();

			SpPhysicProperties* element = &_physicProperties[index];

			element->addForce(settings->gravityForce());

			const sp_float drag = 0.1f; // rho*C*Area - simplified drag for this example
			const Vec3 dragForce = (element->velocity() * element->velocity().abs()) * 0.5f * drag;

			// Velocity Verlet Integration
			elapsedTime = elapsedTime * settings->physicVelocity();

			// Velocity Verlet Integration because regards the velocity
			const Vec3 newPosition = element->position()
				+ element->velocity() * elapsedTime
				+ element->acceleration()  * (elapsedTime * elapsedTime * 0.5f);

			const Vec3 newAcceleration = (element->force() - dragForce) * element->massInverse();

			const Vec3 newVelocity = element->velocity()
				+ (element->acceleration() + newAcceleration) * (elapsedTime * 0.5f);

			const Vec3 translation = newPosition - element->position();
			_boundingVolumes[index].translate(translation); // Sync with the bounding Volume
			syncronizer->sync(index, translation);

			element->_previousAcceleration = element->acceleration();
			element->_acceleration = newAcceleration;

			element->_previousVelocity = element->velocity();
			element->_velocity = newVelocity;

			element->_previousPosition = element->position();
			element->_position = newPosition;

			element->_previousForce = element->force();
			element->_force = ZERO_FLOAT;
		}

		/// <summary>
		/// Back the object to the state before timestep
		/// </summary>
		API_INTERFACE inline void backToTime(const sp_uint index)
		{
			SpPhysicProperties* element = &_physicProperties[index];
			DOP18* dop = &_boundingVolumes[index];

			const Vec3 translation = element->previousPosition() - element->position();

			dop->translate(translation);
			syncronizer->sync(index, translation);

			element->rollbackState();
		}

		API_INTERFACE void run(const Timer& timer);

		API_INTERFACE void dispose();

		~SpPhysicSimulator();

	};
}

#endif // SP_PHYSIC_SIMULATOR_HEADER