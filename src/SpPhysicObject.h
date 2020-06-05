#ifndef SP_PHYSIC_OBJECT_HEADER
#define SP_PHYSIC_OBJECT_HEADER

#include "SpectrumPhysics.h"
#include "BoundingVolume.h"

namespace NAMESPACE_PHYSICS
{
	class SpPhysicProperties
	{
		friend class SpPhysicObject;
		friend class SpPhysicObjectList;
		friend class SpPhysicSimulator;

	private:
		Vec3 _force;
		sp_float _inverseMass;

		Vec3 _position;
		sp_float _damping;

		Vec3 _previousPosition;
		sp_float _coeficientOfRestitution;

		Vec3 _linearVelocity;
		Vec3 _acceleration;

		Quat _orientation;

	public:

		/// <summary>
		/// Default constructor
		/// </summary>
		API_INTERFACE SpPhysicProperties()
		{
			_force = Vec3(ZERO_FLOAT);
			_inverseMass = ZERO_FLOAT;
			_previousPosition = Vec3(ZERO_FLOAT);
			_position = Vec3(ZERO_FLOAT);
			_acceleration = Vec3(ZERO_FLOAT);
			_damping = 0.95f;
			_coeficientOfRestitution = 0.99f;
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
		/// Add force to this object
		/// </summary>
		API_INTERFACE inline Vec3 force() const
		{
			return _force;
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
			return isCloseEnough(_linearVelocity.x, 0.0f) 
				&& isCloseEnough(_linearVelocity.y, 0.0f) 
				&& isCloseEnough(_linearVelocity.z, 0.0f);
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
		/// Check if this object is movable (mass != 0)
		/// </summary>
		API_INTERFACE inline sp_bool isMovable() const
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
		API_INTERFACE inline Vec3 linearVelocity() const
		{
			return _linearVelocity;
		}

		/// <summary>
		/// Get the acceleration of the object
		/// </summary>
		API_INTERFACE inline Vec3 acceleration() const
		{
			return _acceleration;
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

			SpPhysicProperties* element = &physicData;

			const Vec3 newAcceleration = element->acceleration() + // accumulate new acceleration with the previous one
				element->force() * element->massInverse();

			const Vec3 newVelocity = (element->linearVelocity() * element->damping()) + ((element->acceleration()) / elapsedTime);

			const Vec3 newPosition = element->position() + ((newVelocity + element->_linearVelocity) / (elapsedTime * 2.0f));

			//_boundingVolumes->translate(newPosition - element->_position);

			physicData._acceleration     = newAcceleration;
			physicData._linearVelocity   = newVelocity;
			physicData._previousPosition = physicData.position();
			physicData._position         = newPosition;
			physicData._force            = ZERO_FLOAT;
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