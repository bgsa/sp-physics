#ifndef SP_RIGID_BODY_HEADER
#define SP_RIGID_BODY_HEADER

#include "SpectrumPhysics.h"
#include "SpPhysicSettings.h"
#include "SpBody.h"
#include "SpRigidBodyState.h"

namespace NAMESPACE_PHYSICS
{
	class SpRigidBody
		: public SpBody
	{
		friend class SpPhysicObject;
		friend class SpPhysicObjectList;
		friend class SpPhysicSimulator;
		friend class SpCollisionResponse;
	private:
		
		sp_float _inverseMass;
		sp_float _damping;
		sp_float _angularDamping;
		sp_float _coeficientOfRestitution;
		sp_float _coeficientOfFriction;

		Mat3 _inertialTensorInverse;

		inline Vec3 restingAcceleration() const
		{
			return SpPhysicSettings::instance()->gravityForce() * massInverse();
		}

	public:
		SpRigidBodyState currentState;
		SpRigidBodyState previousState;

		/// <summary>
		/// Default constructor
		/// </summary>
		API_INTERFACE SpRigidBody()
		{
			_damping = 0.95f;
			_angularDamping = 0.70f;
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
			const sp_float restingEpsilon = SpPhysicSettings::instance()->restingVelocityEpsilon();

			const sp_bool isRestingPosition = isCloseEnough(currentState._position, previousState._position, restingEpsilon);
			const sp_bool isRestingOrientation = isCloseEnough(currentState._orientation, previousState._orientation, restingEpsilon);
			//const sp_bool isRestingAcc = (isCloseEnough(currentState._acceleration, ZERO_FLOAT) || isCloseEnough(currentState._acceleration, restingAcceleration(), restingEpsilon));
			//const sp_bool isRestingVelocity = isCloseEnough(currentState._velocity - previousState._velocity, ZERO_FLOAT, restingEpsilon);
			//const sp_bool isRestingAngVelocity = isCloseEnough(currentState._angularVelocity - previousState._angularVelocity, ZERO_FLOAT, restingEpsilon);
		
			return isRestingPosition 
				&& isRestingOrientation
				//&& isRestingAcc 
				//&& isRestingVelocity 
				//&& isRestingAngVelocity
				;
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
		/// Slow down the linear velocity over time
		/// </summary>
		API_INTERFACE inline sp_float damping() const
		{
			return _damping;
		}

		/// <summary>
		/// Slow down the linear velocity over time
		/// </summary>
		API_INTERFACE inline void damping(const sp_float newDamping)
		{
			_damping = newDamping;
		}

		/// <summary>
		/// Slow down the angular velocity over time
		/// </summary>
		API_INTERFACE inline void angularDamping(const sp_float newAngularDamping)
		{
			_angularDamping = newAngularDamping;
		}

		/// <summary>
		/// Slow down the angular velocity over time
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
			std::memcpy(&currentState, &previousState, sizeof(SpRigidBodyState));
		}

	};

}

#endif // SP_RIGID_BODY_HEADER