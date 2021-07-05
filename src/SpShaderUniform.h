#ifndef SP_SHADER_UNIFORM_HEADER
#define SP_SHADER_UNIFORM_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
	class SpShaderUniform
	{
	private:
		sp_char* _name;

	public:
		sp_int location;
		sp_int type;
		
		API_INTERFACE inline SpShaderUniform()
		{
			_name = nullptr;
		}

		API_INTERFACE inline SpShaderUniform(sp_int location, sp_int type, const sp_char* name)
		{
			this->location = location;
			this->type = type;

			if (name == nullptr)
				_name = nullptr;
			else 
			{
				this->name(name);
			}
		}

		API_INTERFACE inline sp_char* name() const
		{
			return _name;
		}

		API_INTERFACE inline void name(const sp_char* name)
		{
			if (_name != nullptr)
			{
				sp_mem_release(_name);
			}

			const sp_size len = std::strlen(name);
			_name = sp_mem_new_array(sp_char, len + 1);
			std::memcpy(_name, name, len);
			_name[len] = END_OF_STRING;
		}

		API_INTERFACE inline void dispose()
		{
			if (_name != nullptr)
			{
				sp_mem_release(_name);
				_name = nullptr;
			}
		}

		~SpShaderUniform()
		{
			dispose();
		}
	};
}

#endif // SP_SHADER_UNIFORM_HEADER