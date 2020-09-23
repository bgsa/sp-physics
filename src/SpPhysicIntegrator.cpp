#include "SpPhysicIntegrator.h"
#include "SpPhysicSimulator.h"

namespace NAMESPACE_PHYSICS
{

	void SpPhysicIntegratorVelocityVerlet::execute(const sp_uint index, const sp_float elapsedTime)
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();

		sp_assert(elapsedTime > ZERO_FLOAT, "InvalidArgumentException");
		sp_assert(index >= ZERO_UINT, "IndexOutOfRangeException");
		sp_assert(index < simulator->objectsLength(), "IndexOutOfRangeException");

		SpPhysicSettings* settings = SpPhysicSettings::instance();
		SpPhysicProperties* element = simulator->physicProperties(index);

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

		const Quat newAngularAcceleration(ZERO_FLOAT, element->inertialTensorInverse() * element->currentState.torque());

		const Vec3 newAngularVelocity 
			= (element->currentState.angularVelocity()
			+ ((element->currentState.torque() + newAngularAcceleration) * halfElapsedTime)
			  ) * element->angularDamping();

		Quat newOrientation 
			= element->currentState.orientation() 
			+ Quat(ZERO_FLOAT, element->currentState.angularVelocity() * newElapsedTime
				+ element->currentState.torque() * newElapsedTime * halfElapsedTime);
		normalize(&newOrientation);

		// update/sync bounding volume
		simulator->boundingVolumes(index)->translate(newPosition - element->currentState.position());

		// update/sync transform
		simulator->transforms(index)->position = newPosition;
		simulator->transforms(index)->orientation = newOrientation;

		// TODO: REMOVER
		sp_log_info1s("PREVIOUS POS: ");
		sp_log_info3f(element->currentState._position.x, element->currentState._position.y, element->currentState._position.z);
		sp_log_info1s("    NEW POS: ");
		sp_log_info3f(newPosition.x, newPosition.y, newPosition.z);
		sp_log_info1s("    TIME STEP: ");
		sp_log_info1f(elapsedTime);
		sp_log_newline();


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
		element->currentState._force = ZERO_FLOAT;

		element->previousState._torque = element->currentState.torque();
		element->currentState._torque = newAngularAcceleration;

		element->integratedTime(ZERO_FLOAT);
	}

	void SpPhysicIntegratorEuler::execute(const sp_uint index, const sp_float elapsedTime)
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();

		sp_assert(elapsedTime > ZERO_FLOAT, "InvalidArgumentException");
		sp_assert(index >= ZERO_UINT, "IndexOutOfRangeException");
		sp_assert(index < simulator->objectsLength(), "IndexOutOfRangeException");

		SpPhysicSettings* settings = SpPhysicSettings::instance();
		SpPhysicProperties* element = simulator->physicProperties(index);

		const sp_float newElapsedTime = elapsedTime * settings->physicVelocity();

		const Vec3 newAcceleration = element->currentState.force() * element->massInverse();
		const Vec3 newVelocity = (element->currentState.velocity() + newAcceleration * newElapsedTime)
			* element->damping();
		const Vec3 newPosition = element->currentState.position() + newVelocity * newElapsedTime;

		const Vec3 newAngularAcceleration = element->inertialTensorInverse() * element->currentState.torque();
		const Vec3 newAngularVelocity = (element->currentState.angularVelocity() + newAngularAcceleration * newElapsedTime)
			* element->angularDamping();
		const Quat newOrientation = element->currentState.orientation() + Quat(0.0f, newAngularVelocity * newElapsedTime);


		const Vec3 translation = newPosition - element->currentState.position();
		simulator->translate(index, translation);
		simulator->transforms(index)->orientation = newOrientation;

		element->previousState._acceleration = element->currentState.acceleration();
		element->currentState._acceleration = newAcceleration;

		element->previousState._velocity = element->currentState.velocity();
		element->currentState._velocity = newVelocity;

		element->previousState._position = element->currentState.position();
		element->currentState._position = newPosition;

		element->previousState._force = element->currentState.force();
		element->currentState._force = ZERO_FLOAT;

		element->previousState._angularVelocity = element->currentState.angularVelocity();
		element->currentState._angularVelocity = newAngularVelocity;

		element->previousState._orientation = element->currentState.orientation();
		element->currentState._orientation = newOrientation;

		element->previousState._torque = element->currentState.torque();
		element->currentState._torque = ZERO_FLOAT;
	}

}
