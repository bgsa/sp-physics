#ifndef SP_PHYSIC_OBJECT_HEADER
#define SP_PHYSIC_OBJECT_HEADER

#include "SpectrumPhysics.h"
#include "SpPhysicSettings.h"
#include "BoundingVolume.h"
#include "SpPhysicProperties.h"

namespace NAMESPACE_PHYSICS
{

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