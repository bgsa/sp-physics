#ifndef SP_PHYSIC_PROPERTIES
#define SP_PHYSIC_PROPERTIES

#include "SpectrumPhysics.h"
#include "SpPhysicSettings.h"

namespace NAMESPACE_PHYSICS
{
	class SpPhysicPropertiesState
	{
		friend class SpPhysicObject;
		friend class SpPhysicObjectList;
		friend class SpPhysicProperties;
		friend class SpPhysicSimulator;
		friend class SpCollisionResponse;
		friend class SpPhysicIntegratorEuler;
		friend class SpPhysicIntegratorVelocityVerlet;

	private:
		Vec3 _position;
		Vec3 _velocity;
		Vec3 _acceleration;
		Vec3 _force;
		Quat _orientation;
		Vec3 _angularVelocity;
		Vec3 _torque;

	public:

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE SpPhysicPropertiesState()
		{
			reset();
		}

		/// <summary>
/// Get the position of the object
/// </summary>
		API_INTERFACE inline Vec3 position() const
		{
			return _position;
		}

		/// <summary>
		/// Add to previous position and the current position
		/// </summary>
		API_INTERFACE inline void position(const Vec3& newPosition)
		{
			_position = newPosition;
		}

		/// <summary>
		/// Get the velocity of the object
		/// </summary>
		API_INTERFACE inline Vec3 velocity() const
		{
			return _velocity;
		}

		/// <summary>
		/// Get the velocity of the object
		/// </summary>
		API_INTERFACE inline void velocity(const Vec3& newVelocity)
		{
			_velocity = newVelocity;
		}

		/// <summary>
		/// Get the acceleration of the object
		/// </summary>
		API_INTERFACE inline Vec3 acceleration() const
		{
			return _acceleration;
		}

		/// <summary>
		/// Get the acceleration of the object
		/// </summary>
		API_INTERFACE inline void acceleration(const Vec3& newAcceleration)
		{
			_acceleration = newAcceleration;
		}

		/// <summary>
		/// Add force to this object
		/// </summary>
		API_INTERFACE inline void addForce(const Vec3& force)
		{
			_force.add(force);
		}

		/// <summary>
		/// Get force to this object
		/// </summary>
		API_INTERFACE inline Vec3 force() const
		{
			return _force;
		}

		/// <summary>
		/// Get the orientation
		/// </summary>
		API_INTERFACE inline Quat orientation() const
		{
			return _orientation;
		}

		/// <summary>
		/// Set the orientation
		/// </summary>
		API_INTERFACE inline void orientation(const Quat& newOrientation)
		{
			_orientation = newOrientation;
		}

		/// <summary>
		/// Get the angular velocity of the object
		/// </summary>
		API_INTERFACE inline Vec3 angularVelocity() const
		{
			return _angularVelocity;
		}

		/// <summary>
		/// Set the angular velocity of the object
		/// </summary>
		API_INTERFACE inline void angularVelocity(const Vec3& newAgnularVelocity)
		{
			_angularVelocity = newAgnularVelocity;
		}

		/// <summary>
		/// Get the torque of the object
		/// </summary>
		API_INTERFACE inline Vec3 torque() const
		{
			return _torque;
		}
		
		/// <summary>
		/// Set the torque of the object
		/// </summary>
		API_INTERFACE inline void torque(const Vec3& newTorque)
		{
			_torque = newTorque;
		}

		/// <summary>
		/// Translate the position with the translation parameter
		/// </summary>
		/// <param name="translation">Translation vector to move forward</param>
		/// <returns>void</returns>
		API_INTERFACE inline void translate(const Vec3& translation)
		{
			_position += translation;
		}

		/// <summary>
		/// Reset all values to default
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline void reset()
		{
			_force = Vec3(ZERO_FLOAT);
			_torque = Vec3(ZERO_FLOAT);
			_position = Vec3(ZERO_FLOAT);
			_velocity = Vec3(ZERO_FLOAT);
			_acceleration = Vec3(ZERO_FLOAT);
			_orientation = Quat::identity();
			_angularVelocity = Vec3(ZERO_FLOAT);
		}

	};

	class SpPhysicProperties
	{
		friend class SpPhysicObject;
		friend class SpPhysicObjectList;
		friend class SpPhysicSimulator;
		friend class SpPhysicProperties;
		friend class SpCollisionResponse;
	private:
		
		sp_float _inverseMass;
		sp_float _damping;
		sp_float _angularDamping;
		sp_float _coeficientOfRestitution;
		sp_float _coeficientOfFriction;
		sp_float _integratedTime;

		Mat3 _inertialTensorInverse;

		inline Vec3 restingAcceleration() const
		{
			const Vec3 gravityForce = SpPhysicSettings::instance()->gravityForce();
			const Vec3 restingAcceleration = gravityForce * massInverse();

			return restingAcceleration;
		}

	public:
		SpPhysicPropertiesState currentState;
		SpPhysicPropertiesState previousState;

		/// <summary>
		/// Default constructor
		/// </summary>
		API_INTERFACE SpPhysicProperties()
		{
			_damping = 0.95f;
			_angularDamping = 0.95f;
			_coeficientOfRestitution = 0.60f;
			_coeficientOfFriction = 0.80f;
			_inverseMass = ZERO_FLOAT;
			_integratedTime = ZERO_FLOAT;
			_inertialTensorInverse = Mat3::identity();
		}

		API_INTERFACE inline Mat3 inertialTensorInverse() const
		{
			return _inertialTensorInverse;
		}

		/// <summary>
		/// Set the inertial tensor
		/// </summary>
		API_INTERFACE inline void inertialTensor(const Mat3& tensor)
		{
			const Mat3 orientationAsMatrix = currentState._orientation.toMat3();

			// I^-1 = R * I^-1 * R^T
			_inertialTensorInverse =
				orientationAsMatrix.transpose()
				* tensor
				* orientationAsMatrix;
		}

		/// <summary>
		/// Add Velocity to this object
		/// </summary>
		API_INTERFACE inline void addImpulse(const Vec3& impulse)
		{
			currentState._velocity += impulse;
		}

		/// <summary>
		/// Add torque to this object
		/// </summary>
		API_INTERFACE inline void addImpulseAngular(const Vec3& point, const Vec3& force)
		{
			Vec3 torqueTemp;
			cross((point - currentState._position), force, &torqueTemp);

			currentState._angularVelocity += _inertialTensorInverse * torqueTemp;
		}

		/// <summary>
		/// Define if the object is resting
		/// </summary>
		API_INTERFACE inline sp_bool isResting() const
		{
#define NO_MOVING isCloseEnough(currentState._position, previousState._position, restingEpsilon)
#define NO_ACCELERATION (isCloseEnough(currentState._acceleration, ZERO_FLOAT) || isCloseEnough(currentState._acceleration, restingAcceleration(), restingEpsilon))
#define NO_ANG_VELOCITY isCloseEnough(currentState._angularVelocity, ZERO_FLOAT)

			const sp_float restingEpsilon = SpPhysicSettings::instance()->restingVelocityEpsilon();

			return NO_MOVING && NO_ACCELERATION && NO_ANG_VELOCITY;

#undef NO_ANG_VELOCITY
#undef NO_ACCELERATION
#undef NO_MOVING
		}

		/// <summary>
		/// Get the inverse mass of the object
		/// </summary>
		API_INTERFACE inline sp_float massInverse() const
		{
			return _inverseMass;
		}

		/// <summary>
		/// Set the mass of the object. Ex.: Mass=8.0, so IM=1/8.0f
		/// </summary>
		API_INTERFACE inline void mass(const sp_float mass)
		{
			if (mass == ZERO_FLOAT)
			{
				_inverseMass = ZERO_FLOAT;
				return;
			}

			_inverseMass = 1.0f / mass;
		}

		/// <summary>
		/// Check if this object is not movable (mass == 0)
		/// </summary>
		API_INTERFACE inline sp_bool isStatic() const
		{
			return _inverseMass == ZERO_FLOAT;
		}

		/// <summary>
		/// Check if this object is dynamic (movable) (mass != 0)
		/// </summary>
		API_INTERFACE inline sp_bool isDynamic() const
		{
			return _inverseMass != ZERO_FLOAT;
		}

		/// <summary>
		/// Hold linear velocity over time
		/// </summary>
		API_INTERFACE inline sp_float damping() const
		{
			return _damping;
		}

		/// <summary>
		/// Set linear velocity over time
		/// </summary>
		API_INTERFACE inline void damping(const sp_float newDamping)
		{
			_damping = newDamping;
		}

		/// <summary>
		/// Set angular velocity over time
		/// </summary>
		API_INTERFACE inline void angularDamping(const sp_float newAngularDamping)
		{
			_angularDamping = newAngularDamping;
		}

		/// <summary>
		/// Hold angular velocity over time
		/// </summary>
		API_INTERFACE inline sp_float angularDamping() const
		{
			return _angularDamping;
		}

		/// <summary>
		/// Define the material when it collides
		/// </summary>
		API_INTERFACE inline sp_float coeficientOfRestitution() const
		{
			return _coeficientOfRestitution;
		}

		/// <summary>
		/// Define the material when it collides
		/// </summary>
		API_INTERFACE inline void coeficientOfRestitution(const sp_float newCoR)
		{
			_coeficientOfRestitution = newCoR;
		}

		/// <summary>
		/// Define the material when it collides (tangent)
		/// </summary>
		API_INTERFACE inline sp_float coeficientOfFriction() const
		{
			return _coeficientOfFriction;
		}

		/// <summary>
		/// Define the material when it collides (tangent)
		/// </summary>
		API_INTERFACE inline void coeficientOfFriction(const sp_float newCoF)
		{
			_coeficientOfFriction = newCoF;
		}

		/// <summary>
		/// Redifine the properties with the previous state
		/// </summary>
		API_INTERFACE inline void rollbackState()
		{
			currentState = previousState;
		}

		/// <summary>
		/// Set the integrated time of this element
		/// </summary>
		API_INTERFACE inline void integratedTime(const sp_float value)
		{
			_integratedTime = value;
		}

		/// <summary>
		/// Get the integrated time of this element
		/// </summary>
		API_INTERFACE inline sp_float integratedTime() const
		{
			return _integratedTime;
		}

	};

}

#endif // SP_PHYSIC_PROPERTIES