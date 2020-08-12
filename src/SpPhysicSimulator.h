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
#include "Ray.h"
#include "SpThreadPool.h"
#include "SpCollisionResponseGPU.h"
#include "SpGpuRenderingFactory.h"

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
		SpCollisionResponseGPU* collisionResponseGPU;

		sp_uint _objectsLengthAllocated;
		sp_uint _objectsLength;

		DOP18* _boundingVolumes;
		SpPhysicProperties* _physicProperties;
		SpTransform* _transforms;

		cl_event lastEvent;
		cl_mem _transformsGPU;
		SpGpuTextureBuffer* _transformsGPUBuffer;
		cl_mem _boundingVolumesGPU;
		cl_mem _physicPropertiesGPU;
		cl_mem _collisionIndexesGPU;
		cl_mem _collisionIndexesLengthGPU;
		cl_mem _sapCollisionIndexesGPU;
		cl_mem _sapCollisionIndexesLengthGPU;

		SpPhysicSimulator(sp_uint objectsLength);

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
		void handleCollisionResponseWithStatic(SpCollisionDetails* details, SpPhysicProperties* objProperties, const Vec3& center, const sp_float cor);

		void handleCollisionResponseOLD(SpCollisionDetails* details);

		void findCollisionsCpu(SweepAndPruneResult* result);
		void findCollisionsGpu(SweepAndPruneResult* result);
		void findCollisionsGpuOLD(SweepAndPruneResult* result);

		void updateDataOnGPU()
		{
			//gpu->commandManager->updateBuffer(_boundingVolumesGPU, sizeof(DOP18) * _objectsLengthAllocated, _boundingVolumes, ONE_UINT, &lastEvent);
			//lastEvent = gpu->commandManager->updateBuffer(_physicPropertiesGPU, sizeof(SpPhysicProperties) * _objectsLengthAllocated, _physicProperties, ONE_UINT, &lastEvent);
			sap->updateBoundingVolumes(_boundingVolumes);
			sap->updatePhysicProperties(_physicProperties);
		}

		void updateDataOnCPU()
		{
			gpu->commandManager->readBuffer(_physicPropertiesGPU, sizeof(SpPhysicProperties) * _objectsLength, _physicProperties);
			cl_event evt = 
				gpu->commandManager->readBuffer(_boundingVolumesGPU, DOP18_SIZE * _objectsLength, _boundingVolumes);
			gpu->waitEvents(ONE_UINT, &evt);
		}

		static void handleCollisionCPU(void* collisionParamter);
		static void handleCollisionGPU(void* collisionParamter);

	public:

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
			
			_transformsGPUBuffer
				->use()
				->updateData(size, _transforms);
		}

		API_INTERFACE static SpPhysicSimulator* init(sp_uint objectsLength);

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

		API_INTERFACE inline SpGpuTextureBuffer* transformsGPU() const
		{
			return _transformsGPUBuffer;
		}

		API_INTERFACE static void integrateEuler(const sp_uint index, sp_float elapsedTime)
		{
			SpPhysicSimulator* simulator = SpPhysicSimulator::instance();

			sp_assert(elapsedTime > ZERO_FLOAT, "InvalidArgumentException");
			sp_assert(index >= ZERO_UINT, "IndexOutOfRangeException");
			sp_assert(index < simulator->_objectsLength, "IndexOutOfRangeException");

			SpPhysicSettings* settings = SpPhysicSettings::instance();
			SpPhysicProperties* element = &simulator->_physicProperties[index];

			elapsedTime = elapsedTime * settings->physicVelocity();

			const Vec3 newAcceleration = element->force() * element->massInverse();
			const Vec3 newVelocity = (element->velocity() +  newAcceleration * elapsedTime)
											* element->damping();
			const Vec3 newPosition = element->position() + newVelocity * elapsedTime;

			const Vec3 newAngularAcceleration = element->inertialTensorInverse() * element->torque();
			const Vec3 newAngularVelocity = (element->angularVelocity() + newAngularAcceleration * elapsedTime)
											* element->angularDamping();
			const Quat newOrientation = element->orientation() + Quat(0.0f, newAngularVelocity * elapsedTime);


			const Vec3 translation = newPosition - element->position();
			simulator->translate(index, translation);
			simulator->_transforms[index].orientation = newOrientation;

			element->_previousAcceleration = element->acceleration();
			element->_acceleration = newAcceleration;

			element->_previousVelocity = element->velocity();
			element->_velocity = newVelocity;

			element->_previousPosition = element->position();
			element->_position = newPosition;

			element->_previousForce = element->force();
			element->_force = ZERO_FLOAT;

			element->_previousAngularVelocity = element->angularVelocity();
			element->_angularVelocity = newAngularVelocity;

			element->_previousOrientation = element->orientation();
			element->_orientation = newOrientation;

			element->_previousTorque = element->torque();
			element->_torque = ZERO_FLOAT;
		}

		API_INTERFACE static void integrateVelocityVerlet(const sp_uint index, sp_float elapsedTime)
		{
			SpPhysicSimulator* simulator = SpPhysicSimulator::instance();

			sp_assert(elapsedTime > ZERO_FLOAT, "InvalidArgumentException");
			sp_assert(index >= ZERO_UINT, "IndexOutOfRangeException");
			sp_assert(index < simulator->_objectsLength, "IndexOutOfRangeException");

			SpPhysicSettings* settings = SpPhysicSettings::instance();
			SpPhysicProperties* element = &simulator->_physicProperties[index];

			elapsedTime = elapsedTime * settings->physicVelocity();

			// Velocity Verlet Integration because regards the velocity
			const Vec3 newPosition = element->position()
				+ element->velocity() * elapsedTime
				+ element->acceleration() * (elapsedTime * elapsedTime * HALF_FLOAT);

			const Vec3 newAcceleration = element->force() * element->massInverse();

			Vec3 newVelocity = element->velocity()
				+ ((element->acceleration() + newAcceleration) * elapsedTime * HALF_FLOAT);
			newVelocity *= element->damping();

			const Quat newAngularAcceleration = Quat(0.0f, element->inertialTensorInverse() * element->torque());
			
			Vec3 newAngularVelocity = element->angularVelocity()
				+ ((element->torque() + newAngularAcceleration) * elapsedTime * HALF_FLOAT);
			newAngularVelocity *= element->angularDamping();

			Quat newOrientation = element->orientation() + Quat(0.0f,
				element->angularVelocity() * elapsedTime
				+ element->torque() * (elapsedTime * elapsedTime * HALF_FLOAT)
			);

			element->_orientation = newOrientation.normalize();

			const Vec3 translation = newPosition - element->position();
			simulator->translate(index, translation);
			simulator->_transforms[index].orientation = element->orientation();


			element->_previousAcceleration = element->acceleration();
			element->_acceleration = newAcceleration;

			element->_previousVelocity = element->velocity();
			element->_velocity = newVelocity;

			element->_previousPosition = element->position();
			element->_position = newPosition;

			element->_previousForce = element->force();
			element->_force = ZERO_FLOAT;

			element->_previousTorque = element->torque();
			element->_torque = newAngularAcceleration;
		}


		API_INTERFACE static void integrate(const sp_uint index, sp_float elapsedTime)
		{
			//integrateEuler(index, elapsedTime);
			integrateVelocityVerlet(index, elapsedTime);
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

			simulator->translate(index, translation);
			simulator->transforms(index)->orientation = element->previousOrientation();

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

		API_INTERFACE void translate(const sp_uint index, const Vec3& translation) 
		{
			sp_assert(_boundingVolumes != nullptr, "InvalidOperationException");
			sp_assert(index != SP_UINT_MAX, "InvalidOperationException");
			sp_assert(index < _objectsLength, "IndexOutOfRangeException");

			_boundingVolumes[index].translate(translation);
			_transforms[index].translate(translation);
		}

		API_INTERFACE void run(const Timer& timer);

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