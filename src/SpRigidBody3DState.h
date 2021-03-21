#ifndef SP_RIGID_BODY_3D_STATE_HEADER
#define SP_RIGID_BODY_3D_STATE_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
	class SpRigidBody3DState
	{
		friend class SpPhysicObject;
		friend class SpPhysicObjectList;
		friend class SpRigidBody3D;
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
		API_INTERFACE SpRigidBody3DState()
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
			_force = Vec3Zeros;
			_torque = Vec3Zeros;
			_position = Vec3Zeros;
			_velocity = Vec3Zeros;
			_acceleration = Vec3Zeros;
			_orientation = Quat::identity();
			_angularVelocity = Vec3Zeros;
		}

	};
}

#endif // SP_RIGID_BODY_3D_STATE_HEADER