#ifndef SP_PHYSIC_OBJECT_LIST_HEADER
#define SP_PHYSIC_OBJECT_LIST_HEADER

#include "SpectrumPhysics.h"
#include "BoundingVolume.h"
#include "SpPhysicSimulator.h"
#include "SpPhysicSettings.h"

namespace NAMESPACE_PHYSICS
{
	class SpPhysicObjectList :
		public Object
	{
	private:
		sp_uint listLength;
		sp_uint physicIndex;
		DOP18* _boundingVolumes;
		SpPhysicProperties* _physicProperties;

	public:

		/// <summary>
		/// Default constructor
		/// </summary>
		API_INTERFACE SpPhysicObjectList(const sp_uint length)
		{
			listLength = length;
			physicIndex = SpPhysicSimulator::instance()->alloc(length);
			_boundingVolumes = (DOP18*)SpPhysicSimulator::instance()->boundingVolumes(physicIndex);
			_physicProperties = SpPhysicSimulator::instance()->physicProperties(physicIndex);
		}

		/// <summary>
		/// Get physic properties data from list
		/// </summary>
		API_INTERFACE inline SpPhysicProperties* physicProperties(const sp_uint index)
		{
			return &_physicProperties[index];
		}

		/// <summary>
		/// Update the linear and angular velocity and others parameters
		/// </summary>
		API_INTERFACE void update(const sp_uint index, sp_float elapsedTime)
		{
			sp_assert(elapsedTime > ZERO_FLOAT, "InvalidArgumentException");
			sp_assert(index >= ZERO_UINT, "IndexOutOfRangeException");
			sp_assert(index < listLength, "IndexOutOfRangeException");
			
			SpPhysicProperties* element = &_physicProperties[index];

			/*  EULER INTEGRATION
			const Vec3 newAcceleration = element->force() * element->massInverse();
			const Vec3 newVelocity = 
				(   element->linearVelocity()
				  + (element->acceleration() / elapsedTime)
				) * element->damping();

			const Vec3 newPosition = element->position() + newVelocity / elapsedTime;
			*/

			/* Verlet Integration
			const Vec3 deltaPosition = element->position() - element->previousPosition();
			const Vec3 newPosition = element->position() 
				+ deltaPosition + newAcceleration * elapsedTime*elapsedTime;
			*/


			const sp_float drag = 0.1f; // rho*C*Area - simplified drag for this example
			elapsedTime = elapsedTime * SpPhysicSettings::instance()->physicVelocity();

			// Velocity Verlet Integration because regards the velocity
			const Vec3 newPosition = element->position()
				+ element->linearVelocity() * elapsedTime
				+ element->acceleration()  * (elapsedTime * elapsedTime * 0.5f);

			const Vec3 dragForce = (element->linearVelocity() * element->linearVelocity().abs()) * 0.5f * drag;
			const Vec3 newAcceleration = (element->force() - dragForce) * element->massInverse();

			Vec3 newVelocity = element->linearVelocity()
				+ (element->acceleration() + newAcceleration) * (elapsedTime * 0.5f);
			
			_boundingVolumes[index].translate(newPosition - element->_position);

			std::cout << newVelocity.y << END_OF_LINE;

			element->_acceleration = newAcceleration;
			element->_linearVelocity = newVelocity;
			element->_previousPosition = element->position();
			element->_position = newPosition;
			element->_force = 0.0f;
		}

		/// <summary>
		/// Get the bounding volume of these object for collision detection
		/// </summary>
		API_INTERFACE inline DOP18* boundingVolumes(const sp_uint index) { return &_boundingVolumes[index]; }

		/// <summary>
		/// Define how many objects the list contains
		/// </summary>
		API_INTERFACE virtual sp_uint length() const = 0;

		API_INTERFACE void dispose() 
		{
		}

		API_INTERFACE inline const sp_char* toString()
		{
			return "SpPhysic Object List";
		}

		~SpPhysicObjectList()
		{
			dispose();
		}
	};
}

#endif // SP_PHYSIC_OBJECT_LIST_HEADER