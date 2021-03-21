#ifndef SP_PHYSIC_OBJECT_HEADER
#define SP_PHYSIC_OBJECT_HEADER

#include "SpectrumPhysics.h"
#include "SpPhysicSettings.h"
#include "BoundingVolume.h"
#include "SpRigidBody3D.h"

namespace NAMESPACE_PHYSICS
{

	class SpPhysicObject :
		public Object
	{
	private:
		SpRigidBody3D physicData;

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
		}

		/// <summary>
		/// Get the bounding volume for this object
		/// </summary>
		API_INTERFACE virtual DOP18* boundingVolume() const = 0;

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