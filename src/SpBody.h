#ifndef SP_BODY_HEADER
#define SP_BODY_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
	enum SpBodyType
	{
		Unknonw = 0,
		Rigid = 1,
		Soft = 2
	};

	class SpBody
	{
	private:
		SpBodyType _type;
		sp_bool _enabled;

	public:

		/// <summary>
		/// Default constructor
		/// </summary>
		API_INTERFACE inline SpBody()
		{
			_type = SpBodyType::Unknonw;
			_enabled = true;
		}

		API_INTERFACE inline SpBodyType type() const
		{
			return _type;
		}

		API_INTERFACE inline void type(const SpBodyType newType)
		{
			_type = newType;
		}

		API_INTERFACE inline sp_bool isEnabled() const
		{
			return _enabled;
		}

		API_INTERFACE inline void enable()
		{
			_enabled = true;
		}

		API_INTERFACE inline void disable()
		{
			_enabled = false;
		}

	};
}

#endif // SP_BODY_HEADER