#ifndef SP_PHYSIC_OBJECT_LIST_HEADER
#define SP_PHYSIC_OBJECT_LIST_HEADER

#include "SpectrumPhysics.h"
#include "BoundingVolume.h"
#include "SpPhysicSimulator.h"

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
		API_INTERFACE SpPhysicProperties* physicProperties(const sp_uint index)
		{
			return &_physicProperties[index];
		}

		/// <summary>
		/// Add force to this object
		/// </summary>
		API_INTERFACE void addForce(const Vec3& force)
		{
			for (sp_uint i = ZERO_UINT; i < listLength; i++)
				_physicProperties[i].force.add(force);
		}

		/// <summary>
		/// Get the inverse mass of the object
		/// </summary>
		API_INTERFACE inline sp_float massInverse(const sp_uint index) const
		{
			return _physicProperties[index].inverseMass;
		}
		/// <summary>
		/// Set the mass of the object. Ex.: Mass=8.0, so IM=1/8.0f
		/// </summary>
		API_INTERFACE inline void mass(sp_float mass)
		{
			sp_float newMass = 1.0f / mass;

			for (sp_uint i = ZERO_UINT; i < listLength; i++)
				_physicProperties[i].inverseMass = newMass;
		}

		/// <summary>
		/// Get the velocity of the object
		/// </summary>
		API_INTERFACE inline Vec3 velocity(const sp_uint index) const
		{
			return _physicProperties[index].velocity;
		}

		/// <summary>
		/// Get the acceleration of the object
		/// </summary>
		API_INTERFACE inline Vec3 acceleration(const sp_uint index) const
		{
			return _physicProperties[index].acceleration;
		}

		/// <summary>
		/// Get the change the velocity over the time
		/// </summary>
		API_INTERFACE sp_float damping(const sp_uint index) const
		{
			return _physicProperties[index].damping;
		}

		/// <summary>
		/// Define the material when it collides
		/// </summary>
		API_INTERFACE sp_float coeficientOfRestitution(const sp_uint index) const
		{
			return _physicProperties[index].coeficientOfRestitution;
		}

		/// <summary>
		/// Update the linear and angular velocity and others parameters
		/// </summary>
		API_INTERFACE void update(sp_float elapsedTime)
		{
			sp_assert(elapsedTime > ZERO_FLOAT, "InvalidArgumentException");

			for (sp_uint i = ZERO_UINT; i < listLength; i++)
			{
				SpPhysicProperties* element = &_physicProperties[i];

				Vec3 newAcceleration = element->force * element->inverseMass;

				Vec3 newVelocity = (element->velocity * element->damping) + ((newAcceleration - element->acceleration) / elapsedTime);

				Vec3 newPosition = element->position + (newVelocity / elapsedTime);

				element->acceleration = newAcceleration;
				element->velocity = newVelocity;
				element->previousPosition = element->position;
				element->position = newPosition;
				element->force = 0.0f;
			}
		}

		/// <summary>
		/// Get the bounding volume of these object for collision detection
		/// </summary>
		API_INTERFACE inline DOP18* boundingVolumes() { return _boundingVolumes; }

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