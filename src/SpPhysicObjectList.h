#ifndef SP_PHYSIC_OBJECT_LIST_HEADER
#define SP_PHYSIC_OBJECT_LIST_HEADER

#include "SpectrumPhysics.h"
#include "BoundingVolume.h"
#include "SpPhysicSimulator.h"
#include "SpPhysicSettings.h"
#include "SpCollisionFeatures.h"

namespace NAMESPACE_PHYSICS
{
	class SpPhysicObjectList :
		public Object
	{
	protected:
		sp_uint listLength;
		sp_uint physicIndex;
		DOP18* _boundingVolumes;
		SpCollisionFeatures* _objectMapper;
		SpRigidBody3D* _rigidBodies3D;

	public:

		/// <summary>
		/// Default constructor
		/// </summary>
		API_INTERFACE SpPhysicObjectList(const sp_uint length)
		{
			listLength = length;
			physicIndex = SpWorldManagerInstance->current()->alloc(length);
			_boundingVolumes = SpWorldManagerInstance->current()->boundingVolumes(physicIndex);
			_rigidBodies3D = SpWorldManagerInstance->current()->rigidBody3D(physicIndex);
			_objectMapper = SpWorldManagerInstance->current()->collisionFeatures(physicIndex);
		}

		/// <summary>
		/// Get physic properties data from list
		/// </summary>
		API_INTERFACE inline SpRigidBody3D* rigidBody3D(const sp_uint index)
		{
			sp_assert(index < listLength, "IndexOutOfRangeException");
			return &_rigidBodies3D[index];
		}

		/// <summary>
		/// Get the bounding volume of these object for collision detection
		/// </summary>
		API_INTERFACE inline DOP18* boundingVolumes(const sp_uint index) 
		{ 
			sp_assert(index < listLength, "IndexOutOfRangeException");
			return &_boundingVolumes[index]; 
		}

		/// <summary>
		/// Get the Transformations of these object
		/// </summary>
		API_INTERFACE inline SpTransform* transforms(const sp_uint index = ZERO_UINT) const
		{
			sp_assert(index < listLength, "IndexOutOfRangeException");
			return SpWorldManagerInstance->current()->transforms(physicIndex + index);
		}

		/// <summary>
		/// Get collision features data from list
		/// </summary>
		API_INTERFACE inline SpCollisionFeatures* collisionFeatures(const sp_uint index)
		{
			sp_assert(index < listLength, "IndexOutOfRangeException");
			return &_objectMapper[index];
		}

		/// <summary>
		/// Define how many objects the list contains
		/// </summary>
		API_INTERFACE inline sp_uint length() const
		{
			return listLength;
		}

		/// <summary>
		/// Update the linear and angular velocity and others parameters
		/// </summary>
		API_INTERFACE void update(const sp_uint index, sp_float elapsedTime)
		{
			SpWorldManagerInstance->current()->physicSimulator->integrator->execute(physicIndex + index, elapsedTime);
		}

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