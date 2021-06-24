#ifndef SP_OBJECT_MANAGER_HEADER
#define SP_OBJECT_MANAGER_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
	class SpObjectManager
	{
	protected:
		sp_uint _length;

	public:

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline SpObjectManager()
		{
			_length = ZERO_UINT;
		}

		/// <summary>
		/// Add a new object in list
		/// </summary>
		/// <returns></returns>
		API_INTERFACE virtual sp_uint add() = 0;

		/// <summary>
		/// Remove the object in list
		/// </summary>
		/// <param name="index"></param>
		/// <returns></returns>
		API_INTERFACE virtual void remove(const sp_uint index) = 0;

		/// <summary>
		/// Get the length of objects
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_uint length()
		{
			return _length;
		}

		/// <summary>
		/// Dispose all allocated resources
		/// </summary>
		/// <returns></returns>
		API_INTERFACE virtual void dispose()
		{
			_length = ZERO_UINT;
		}

	};
}

#endif // SP_OBJECT_MANAGER_HEADER