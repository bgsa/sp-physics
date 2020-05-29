#ifndef SP_PHYSIC_OBJECT_HEADER
#define SP_PHYSIC_OBJECT_HEADER

#include "SpectrumPhysics.h"
#include "BoundingVolume.h"

namespace NAMESPACE_PHYSICS
{
	class SpPhysicProperties
	{
	public:
		Vec3 force;
		sp_float inverseMass;

		Vec3 position;
		sp_float damping;

		Vec3 previousPosition;
		sp_float coeficientOfRestitution;

		Vec3 velocity;
		Vec3 acceleration;

		API_INTERFACE SpPhysicProperties()
		{
			force = Vec3(ZERO_FLOAT);
			inverseMass = ZERO_FLOAT;
			previousPosition = Vec3(ZERO_FLOAT);
			position = Vec3(ZERO_FLOAT);
			acceleration = Vec3(ZERO_FLOAT);
			damping = 0.95f;
			coeficientOfRestitution = 0.99f;
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
		/// Add force to this object
		/// </summary>
		API_INTERFACE void addForce(const Vec3& force)
		{
			physicData.force.add(force);
		}

		/// <summary>
		/// Get the inverse mass of the object
		/// </summary>
		API_INTERFACE inline sp_float massInverse() const
		{
			return physicData.inverseMass;
		}

		/// <summary>
		/// Set the mass of the object. Ex.: Mass=8.0, so IM=1/8.0f
		/// </summary>
		API_INTERFACE inline void mass(sp_float mass)
		{
			physicData.inverseMass = 1.0f / mass;
		}

		/// <summary>
		/// Get the velocity of the object
		/// </summary>
		API_INTERFACE inline Vec3 velocity() const
		{
			return physicData.velocity;
		}

		/// <summary>
		/// Get the acceleration of the object
		/// </summary>
		API_INTERFACE inline Vec3 acceleration() const
		{
			return physicData.acceleration;
		}

		/// <summary>
		/// Get the change the velocity over the time
		/// </summary>
		API_INTERFACE sp_float damping() const
		{
			return physicData.damping;
		}

		/// <summary>
		/// Define the material when it collides
		/// </summary>
		API_INTERFACE sp_float coeficientOfRestitution() const
		{
			return physicData.coeficientOfRestitution;
		}

		/// <summary>
		/// Update the linear and angular velocity and others parameters
		/// </summary>
		API_INTERFACE virtual void update(sp_float elapsedTime)
		{
			sp_assert(elapsedTime > ZERO_FLOAT, "InvalidArgumentException");

			Vec3 newAcceleration = physicData.force * physicData.inverseMass;

			Vec3 newVelocity = (physicData.velocity * physicData.damping) + ((newAcceleration - physicData.acceleration) / elapsedTime);

			Vec3 newPosition = physicData.position + (newVelocity / elapsedTime);

			physicData.acceleration     = newAcceleration;
			physicData.velocity         = newVelocity;
			physicData.previousPosition = physicData.position;
			physicData.position         = newPosition;
			physicData.force            = ZERO_FLOAT;
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