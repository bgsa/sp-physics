#ifndef SP_PHYSIC_OBJECT_HEADER
#define SP_PHYSIC_OBJECT_HEADER

#include "SpectrumPhysics.h"
#include "SpPhysicSettings.h"
#include "BoundingVolume.h"

namespace NAMESPACE_PHYSICS
{
	class SpPhysicProperties
	{
		friend class SpPhysicObject;
		friend class SpPhysicObjectList;
		friend class SpPhysicSimulator;

	private:
		Vec3 _position;
		Vec3 _previousPosition;

		Vec3 _velocity;
		Vec3 _previousVelocity;

		Vec3 _acceleration;
		Vec3 _previousAcceleration;

		Vec3 _angularVelocity;
		Vec3 _previousAngularVelocity;

		Vec3 _force;
		Vec3 _previousForce;

		sp_float _inverseMass;
		sp_float _damping;
		sp_float _coeficientOfRestitution;
		sp_float _coeficientOfFriction;

		Quat _orientation;

		inline Vec3 restingAcceleration() const
		{
			Vec3 forceX = SpPhysicSettings::instance()->gravityForce();

			const sp_float drag = 0.1f; // rho*C*Area - simplified drag for this example
			const Vec3 dragForce = (velocity() * velocity().abs()) * 0.5f * drag;
			const Vec3 restingAcceleration = (forceX - dragForce) * massInverse();

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
			
			_position = Vec3(ZERO_FLOAT);
			_previousPosition = _position;

			_velocity = Vec3(ZERO_FLOAT);
			_previousVelocity = _velocity;

			_acceleration = Vec3(ZERO_FLOAT);
			_previousAcceleration = _acceleration;

			_damping = 0.95f;
			_coeficientOfRestitution = 0.7f;
			_coeficientOfFriction = 0.8f;
			_inverseMass = ZERO_FLOAT;
			_orientation = Quat();
		}

		/// <summary>
		/// Add force to this object
		/// </summary>
		API_INTERFACE void addForce(const Vec3& force)
		{
			_force.add(force);
		}

		/// <summary>
		/// Add Velocity to this object
		/// </summary>
		API_INTERFACE void addImpulse(const Vec3& impulse)
		{
			_velocity += impulse;
		}

		/// <summary>
		/// Add force to this object
		/// </summary>
		API_INTERFACE inline Vec3 force() const
		{
			return _force;
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
		/// Get the previous velocity of the object
		/// </summary>
		API_INTERFACE inline Vec3 previousVelocity() const
		{
			return _previousVelocity;
		}

		/// <summary>
		/// Get the angular velocity of the object
		/// </summary>
		API_INTERFACE inline Vec3 angularVelocity() const
		{
			return _angularVelocity;
		}

		/// <summary>
		/// Get the previous angular velocity of the object
		/// </summary>
		API_INTERFACE inline Vec3 previousAngularVelocity() const
		{
			return _previousAngularVelocity;
		}

		/// <summary>
		/// Get the acceleration of the object
		/// </summary>
		API_INTERFACE inline Vec3 acceleration() const
		{
			return _acceleration;
		}

		/// <summary>
		/// Get the acceleration previous the timestep
		/// </summary>
		API_INTERFACE inline Vec3 previousAcceleration() const
		{
			return _previousAcceleration;
		}

		/// <summary>
		/// Get the change the velocity over the time
		/// </summary>
		API_INTERFACE inline sp_float damping() const
		{
			return _damping;
		}

		/// <summary>
		/// Define the material when it collides
		/// </summary>
		API_INTERFACE inline sp_float coeficientOfRestitution() const
		{
			return _coeficientOfRestitution;
		}

		/// <summary>
		/// Define the material when it collides (tangent)
		/// </summary>
		API_INTERFACE inline sp_float coeficientOfFriction() const
		{
			return _coeficientOfFriction;
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
			_angularVelocity = _previousAngularVelocity;
		}

	};

	class SpPhysicObject :
		public Object
	{
	private:
		SpPhysicProperties physicData;

	public:

		/// <summary>
		/// Default constructor
		/// </summary>
		API_INTERFACE SpPhysicObject()
		{
		}

		/// <summary>
		/// Update the linear and angular velocity and others parameters
		/// </summary>
		API_INTERFACE virtual void update(sp_float elapsedTime)
		{
			sp_assert(elapsedTime > ZERO_FLOAT, "InvalidArgumentException");

			/*
			SpPhysicProperties* element = &physicData;

			const Vec3 newAcceleration = element->acceleration() + // accumulate new acceleration with the previous one
				element->force() * element->massInverse();

			const Vec3 newVelocity = (element->velocity() * element->damping()) + ((element->acceleration()) / elapsedTime);

			const Vec3 newPosition = element->position() + ((newVelocity + element->velocity) / (elapsedTime * 2.0f));

			//_boundingVolumes->translate(newPosition - element->_position);

			physicData._acceleration     = newAcceleration;
			physicData.velocity   = newVelocity;
			physicData._previousPosition = physicData.position();
			physicData._position         = newPosition;
			physicData._force            = ZERO_FLOAT;
			*/
		}

		/// <summary>
		/// Get the bounding volume for this object
		/// </summary>
		API_INTERFACE virtual BoundingVolume* boundingVolume() const = 0;

		/// <summary>
		/// Get the description for this object
		/// </summary>
		API_INTERFACE inline const sp_char* toString()
		{
			return "SpPhysic Object";
		}

	};
}

#endif // SP_PHYSIC_OBJECT_HEADER