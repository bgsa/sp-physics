#include "SpPhysicIntegrator.h"
#include "SpPhysicSimulator.h"

namespace NAMESPACE_PHYSICS
{

	void SpPhysicIntegratorVelocityVerlet::execute(const sp_uint index, const sp_float elapsedTime)
	{
		SpWorld* world = SpWorldManagerInstance->current();

		sp_assert(elapsedTime > ZERO_FLOAT, "InvalidArgumentException");
		sp_assert(index >= ZERO_UINT, "IndexOutOfRangeException");
		sp_assert(index < world->objectsLength(), "IndexOutOfRangeException");

		SpPhysicSettings* settings = SpPhysicSettings::instance();
		SpRigidBody3D* element = world->rigidBody3D(index);

		if (element->isStatic())
			return;

		//const sp_float newElapsedTime =  (elapsedTime - element->integratedTime()) * settings->physicVelocity();
		const sp_float newElapsedTime = elapsedTime * settings->physicVelocity();
		const sp_float halfElapsedTime = newElapsedTime * HALF_FLOAT;

		// Velocity Verlet Integration because regards the velocity
		const Vec3 newPosition = element->currentState.position()
			+ element->currentState.velocity() * newElapsedTime
			+ element->currentState.acceleration() * newElapsedTime * halfElapsedTime;

		const Vec3 newAcceleration = element->currentState.force() * element->massInverse();

		const Vec3 newVelocity
			= (element->currentState.velocity()
			+ (element->currentState.acceleration() + newAcceleration) * halfElapsedTime
			  ) * element->damping();

		Vec3 angularAcceleration;
		element->inertialTensorInverse().multiply(element->currentState.torque(), angularAcceleration);

		const Quat newAngularAcceleration(ZERO_FLOAT, angularAcceleration);

		const Vec3 newAngularVelocity 
			= (element->currentState.angularVelocity()
			+ ((element->currentState.torque() + newAngularAcceleration) * halfElapsedTime)
			  ) * element->angularDamping();

		Quat temp = Quat(ZERO_FLOAT, 
			element->currentState.angularVelocity() * newElapsedTime
			+ element->currentState.torque() * newElapsedTime * halfElapsedTime);

		Quat newOrientation = element->currentState.orientation() + temp;
		normalize(&newOrientation);

		// update/sync transform
		world->transforms(index)->position = newPosition;
		world->transforms(index)->orientation = newOrientation;

		// update physic properties state
		element->previousState._acceleration = element->currentState.acceleration();
		element->currentState._acceleration = newAcceleration;

		element->previousState._velocity = element->currentState.velocity();
		element->currentState._velocity = newVelocity;

		element->previousState._position = element->currentState.position();
		element->currentState._position = newPosition;

		element->previousState._orientation = element->currentState.orientation();
		element->currentState._orientation = newOrientation;

		element->previousState._angularVelocity = element->currentState.angularVelocity();
		element->currentState._angularVelocity = newAngularVelocity;

		element->previousState._force = element->currentState.force();
		element->currentState._force = Vec3Zeros;

		element->previousState._torque = element->currentState.torque();
		element->currentState._torque = newAngularAcceleration;
	}

	void SpPhysicIntegratorEuler::execute(const sp_uint index, const sp_float elapsedTime)
	{
		SpWorld* world = SpWorldManagerInstance->current();

		sp_assert(elapsedTime > ZERO_FLOAT, "InvalidArgumentException");
		sp_assert(index >= ZERO_UINT, "IndexOutOfRangeException");
		sp_assert(index < world->objectsLength(), "IndexOutOfRangeException");

		SpPhysicSettings* settings = SpPhysicSettings::instance();
		SpRigidBody3D* element = world->rigidBody3D(index);

		const sp_float newElapsedTime = elapsedTime * settings->physicVelocity();

		const Vec3 newAcceleration = element->currentState.force() * element->massInverse();
		const Vec3 newVelocity = (element->currentState.velocity() + newAcceleration * newElapsedTime)
			* element->damping();
		const Vec3 newPosition = element->currentState.position() + newVelocity * newElapsedTime;

		Vec3 newAngularAcceleration;
		element->inertialTensorInverse().multiply(element->currentState.torque(), newAngularAcceleration);
		
		const Vec3 newAngularVelocity = (element->currentState.angularVelocity() + newAngularAcceleration * newElapsedTime)
			* element->angularDamping();
		const Quat newOrientation = element->currentState.orientation() + Quat(0.0f, newAngularVelocity * newElapsedTime);


		const Vec3 translation = newPosition - element->currentState.position();
		world->translate(index, translation);
		world->transforms(index)->orientation = newOrientation;

		element->previousState._acceleration = element->currentState.acceleration();
		element->currentState._acceleration = newAcceleration;

		element->previousState._velocity = element->currentState.velocity();
		element->currentState._velocity = newVelocity;

		element->previousState._position = element->currentState.position();
		element->currentState._position = newPosition;

		element->previousState._force = element->currentState.force();
		element->currentState._force = Vec3Zeros;

		element->previousState._angularVelocity = element->currentState.angularVelocity();
		element->currentState._angularVelocity = newAngularVelocity;

		element->previousState._orientation = element->currentState.orientation();
		element->currentState._orientation = newOrientation;

		element->previousState._torque = element->currentState.torque();
		element->currentState._torque = Vec3Zeros;
	}

}
