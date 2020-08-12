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

	protected:
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
			sp_assert(index < listLength, "IndexOutOfRangeException");
			return &_physicProperties[index]; 
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
			return SpPhysicSimulator::instance()->transforms(physicIndex + index);
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
			SpPhysicSimulator::instance()->integrate(physicIndex + index, elapsedTime);
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