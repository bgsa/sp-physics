#ifndef SP_PHYSIC_OBJECT_LIST_HEADER
#define SP_PHYSIC_OBJECT_LIST_HEADER

#include "SpectrumPhysics.h"
#include "BoundingVolume.h"
#include "SpPhysicSimulator.h"
#include "SpPhysicSettings.h"
#include "SpRigidBodyMapper.h"
#include "SpBody.h"

namespace NAMESPACE_PHYSICS
{
	class SpPhysicObjectList :
		public Object
	{
	protected:
		sp_uint listLength;
		DOP18* _boundingVolumes;
		SpRigidBodyMapper* _rigidBodyMapper;
		sp_uint _bodyIndex;
		sp_uint _globalBodyIndex;
		SpBodyType _bodyType;

	public:

		/// <summary>
		/// Default constructor
		/// </summary>
		API_INTERFACE SpPhysicObjectList(const SpBodyType bodyType, const sp_uint length)
		{
			_bodyType = bodyType;
			listLength = length;

			switch (bodyType)
			{
			case SpBodyType::Rigid:
				_bodyIndex = SpPhysicSimulator::instance()->allocRigidBody(length, &_globalBodyIndex);
				_rigidBodyMapper = SpPhysicSimulator::instance()->rigidBodyMapper(_bodyIndex);
				break;

			case SpBodyType::Soft:
				_bodyIndex = SpPhysicSimulator::instance()->allocSoftBody(length, &_globalBodyIndex);
				_rigidBodyMapper = nullptr;
				break;

			default:
				sp_assert(false, "InvalidArgumentException");
				break;
			}

			_boundingVolumes = SpPhysicSimulator::instance()->boundingVolumes(_globalBodyIndex);
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
			return SpPhysicSimulator::instance()->transforms(_globalBodyIndex + index);
		}

		/// <summary>
		/// Get collision features data from list
		/// </summary>
		API_INTERFACE inline SpRigidBodyMapper* rigidBodyMapper(const sp_uint rigidBodyIndex)
		{
			sp_assert(rigidBodyIndex < listLength, "IndexOutOfRangeException");
			return &_rigidBodyMapper[rigidBodyIndex];
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
			SpPhysicSimulator::instance()->integrator->execute(_bodyIndex + index, elapsedTime);
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