#ifndef SP_PHYSIC_PROPERTIES
#define SP_PHYSIC_PROPERTIES

#include "SpectrumPhysics.h"
#include "SpPhysicSettings.h"

namespace NAMESPACE_PHYSICS
{
	class SpPhysicProperties
	{
		friend class SpPhysicObject;
		friend class SpPhysicObjectList;
		friend class SpPhysicSimulator;

	private:
		Vec3 _position;
		Vec3 _velocity;
		Vec3 _acceleration;
		Vec3 _force;
		Quat _orientation;
		Vec3 _angularVelocity;
		Vec3 _torque;

		Vec3 _previousPosition;
		Vec3 _previousVelocity;
		Vec3 _previousAcceleration;
		Vec3 _previousForce;
		Quat _previousOrientation;
		Vec3 _previousAngularVelocity;
		Vec3 _previousTorque;

		sp_float _inverseMass;
		sp_float _damping;
		sp_float _angularDamping;
		sp_float _coeficientOfRestitution;
		sp_float _coeficientOfFriction;

		Mat3 _inertialTensorInverse;

		inline Vec3 restingAcceleration() const
		{
			const Vec3 gravityForce = SpPhysicSettings::instance()->gravityForce();
			const Vec3 restingAcceleration = gravityForce * massInverse();

			return restingAcceleration;
		}

	public:

		/// <summary>
		/// Default constructor
		/// </summary>
		API_INTERFACE SpPhysicProperties()
		{
			_force = Vec3(ZERO_FLOAT);
			_previousForce = _force;

			_torque = Vec3(ZERO_FLOAT);
			_previousTorque = _torque;

			_position = Vec3(ZERO_FLOAT);
			_previousPosition = _position;

			_velocity = Vec3(ZERO_FLOAT);
			_previousVelocity = _velocity;

			_acceleration = Vec3(ZERO_FLOAT);
			_previousAcceleration = _acceleration;

			_orientation = Quat::identity();
			_previousOrientation = Quat::identity();

			_angularVelocity = Vec3(ZERO_FLOAT);
			_previousAngularVelocity = _angularVelocity;

			_damping = 0.95f;
			_angularDamping = 0.95f;
			_coeficientOfRestitution = 0.60f;
			_coeficientOfFriction = 0.80f;
			_inverseMass = ZERO_FLOAT;
			_inertialTensorInverse = Mat3::identity();
		}

		API_INTERFACE inline Mat3 inertialTensorInverse() const
		{
			return _inertialTensorInverse;
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
			_previousOrientation = newOrientation;
		}

		/// <summary>
		/// Get the orientation
		/// </summary>
		API_INTERFACE inline Quat previousOrientation()
		{
			return _previousOrientation;
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
		/// Get the previous angular velocity of the object
		/// </summary>
		API_INTERFACE inline Vec3 previousAngularVelocity() const
		{
			return _previousAngularVelocity;
		}

		/// <summary>
		/// Set the inertial tensor
		/// </summary>
		API_INTERFACE inline void inertialTensor(const Mat3& tensor)
		{
			const Mat3 orientationAsMatrix = _orientation.toMat3();

			// I^-1 = R * I^-1 * R^T
			_inertialTensorInverse =
				orientationAsMatrix.transpose()
				* tensor
				* orientationAsMatrix;
		}

		/// <summary>
		/// Add force to this object
		/// </summary>
		API_INTERFACE inline void addForce(const Vec3& force)
		{
			_force.add(force);
		}

		/// <summary>
		/// Add Velocity to this object
		/// </summary>
		API_INTERFACE inline void addImpulse(const Vec3& impulse)
		{
			_velocity += impulse;
		}

		/// <summary>
		/// Add torque to this object
		/// </summary>
		API_INTERFACE inline void addImpulseAngular(const Vec3& point, const Vec3& force)
		{
			//_torque += (point - _position).cross(force);
			const Vec3 torqueTemp = (point - _position).cross(force);
			_angularVelocity += _inertialTensorInverse * torqueTemp;
		}

		/// <summary>
		/// Get force to this object
		/// </summary>
		API_INTERFACE inline Vec3 force() const
		{
			return _force;
		}

		/// <summary>
		/// Get the torque of the object
		/// </summary>
		API_INTERFACE inline Vec3 torque() const
		{
			return _torque;
		}

		/// <summary>
		/// Get the force previous the timestep
		/// </summary>
		API_INTERFACE inline Vec3 previousForce() const
		{
			return _previousForce;
		}

		/// <summary>
		/// Get the position of the object
		/// </summary>
		API_INTERFACE inline Vec3 position() const
		{
			return _position;
		}

		/// <summary>
		/// Get the previous position of the object
		/// </summary>
		API_INTERFACE inline Vec3 previousPosition() const
		{
			return _previousPosition;
		}

		/// <summary>
		/// Add to previous position and the current position
		/// </summary>
		API_INTERFACE inline void position(const Vec3& newPosition)
		{
			_position += newPosition;
			_previousPosition += newPosition;
		}

		/// <summary>
		/// Define if the object is resting
		/// </summary>
		API_INTERFACE inline sp_bool isResting() const
		{
			const sp_float restingEpsilon = SpPhysicSettings::instance()->restingVelocityEpsilon();
			const Vec3 _restingAcceleration = restingAcceleration();

			return
				isCloseEnough(_position.x, _previousPosition.x, restingEpsilon) &&
				isCloseEnough(_position.y, _previousPosition.y, restingEpsilon) &&
				isCloseEnough(_position.z, _previousPosition.z, restingEpsilon) &&
				isCloseEnough(_acceleration.x, _restingAcceleration.x, restingEpsilon) &&
				isCloseEnough(_acceleration.y, _restingAcceleration.y, restingEpsilon) &&
				isCloseEnough(_acceleration.z, _restingAcceleration.z, restingEpsilon);
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
		API_INTERFACE inline void mass(sp_float mass)
		{
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
		/// Uniform scale the mass of the object
		/// </summary>
		API_INTERFACE inline void scale(const sp_float factor)
		{
			_inverseMass *= factor;
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
		/// Get the previous velocity of the object
		/// </summary>
		API_INTERFACE inline Vec3 previousVelocity() const
		{
			return _previousVelocity;
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
			_acceleration = newAcceleration	;
		}

		/// <summary>
		/// Get the acceleration previous the timestep
		/// </summary>
		API_INTERFACE inline Vec3 previousAcceleration() const
		{
			return _previousAcceleration;
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
			_force = _previousForce;
			_acceleration = _previousAcceleration;
			_velocity = _previousVelocity;
			_position = _previousPosition;

			_orientation = _previousOrientation;
			_angularVelocity = _previousAngularVelocity;
			_torque = _previousTorque;
		}

	};

}

#endif // SP_PHYSIC_PROPERTIES