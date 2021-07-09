#ifndef SP_SHADER_HEADER
#define SP_SHADER_HEADER

#include "SpectrumPhysics.h"
#include "SpStringId.h"
#include "SpShaderAttribute.h"
#include "SpShaderUniform.h"

namespace NAMESPACE_PHYSICS
{
	class SpShader
	{
	protected:
		SpStringId id;
		sp_uint program;
		SpVector<sp_uint> shadersId;
		SpArray<SpShaderAttribute*>* _attributes;
		SpArray<SpShaderUniform*>* _uniforms;

	public:

		API_INTERFACE inline sp_char* name() const
		{
			return id.name;
		}

		API_INTERFACE inline void name(const sp_char* name)
		{
			id = name;
		}

		/// <summary>
		/// Enable the current shader
		/// </summary>
		/// <returns></returns>
		API_INTERFACE virtual SpShader* enable() = 0;

		/// <summary>
		/// Disable the current shader
		/// </summary>
		/// <returns></returns>
		API_INTERFACE virtual void disable() = 0;

		/// <summary>
		/// Get the uniforms of actived shader
		/// </summary>
		API_INTERFACE inline SpArray<SpShaderUniform*>* uniforms()
		{
			return _uniforms;
		}

		/// <summary>
		/// Get the attributes of actived shader
		/// </summary>
		API_INTERFACE inline SpArray<SpShaderAttribute*>* attributes()
		{
			return _attributes;
		}

		/// <summary>
		/// Set the uniform with Mat3 value
		/// </summary>
		/// <param name="id">Uniform Id</param>
		/// <param name="value">Mat3</param>
		/// <returns>Current shader</returns>
		API_INTERFACE virtual SpShader* setUniform(const sp_int id, const Mat3& value) = 0;

		/// <summary>
		/// Set the uniform with Mat4 value
		/// </summary>
		/// <param name="id">Uniform Id</param>
		/// <param name="value">Mat4</param>
		/// <returns>Current shader</returns>
		API_INTERFACE virtual SpShader* setUniform(const sp_int id, const Mat4& value) = 0;

		/// <summary>
		/// Set the uniform with scalar int value
		/// </summary>
		/// <param name="id">Uniform Id</param>
		/// <param name="value">Iouble</param>
		/// <returns>Current shader</returns>
		API_INTERFACE virtual SpShader* setUniform(const sp_int id, const sp_int value) = 0;

		/// <summary>
		/// Set the uniform with scalar unsigned int value
		/// </summary>
		/// <param name="id">Uniform Id</param>
		/// <param name="value">unsigned int</param>
		/// <returns>Current shader</returns>
		API_INTERFACE virtual SpShader* setUniform(const sp_int id, const sp_uint value) = 0;

#ifdef ENV_64BITS
		/// <summary>
		/// Set the uniform with scalar unsigned size value
		/// </summary>
		/// <param name="id">Uniform Id</param>
		/// <param name="value">unsigned size</param>
		/// <returns>Current shader</returns>
		API_INTERFACE virtual SpShader* setUniform(const sp_int id, const sp_size value) = 0;
#endif

		/// <summary>
		/// Set the uniform with scalar flaot value
		/// </summary>
		/// <param name="id">Uniform Id</param>
		/// <param name="value">Float</param>
		/// <returns>Current shader</returns>
		API_INTERFACE virtual SpShader* setUniform(const sp_int id, const sp_float value) = 0;

		/// <summary>
		/// Set the uniform with scalar double value
		/// </summary>
		/// <param name="id">Uniform Id</param>
		/// <param name="value">Double</param>
		/// <returns>Current shader</returns>
		API_INTERFACE virtual SpShader* setUniform(const sp_int id, const sp_double value) = 0;

		API_INTERFACE virtual SpShader* enableVertexAttribute(const sp_uint index, sp_int size, sp_int type, sp_bool normalized, sp_size stride, const void* pointer) = 0;

		API_INTERFACE virtual SpShader* disableVertexAttribute(const sp_uint index) = 0;

		API_INTERFACE virtual SpShader* drawArray(const sp_uint primitiveTypeId, const sp_int first, const sp_size count) = 0;

		API_INTERFACE virtual SpShader* drawElements(const sp_uint primitiveTypeId, const sp_size indexesLength, const sp_int indexTypeId, const void* indexes = NULL) = 0;

		API_INTERFACE virtual SpShader* drawElementsInstanced(const sp_uint primitiveTypeId, const sp_size indexesLength, const sp_int indexTypeId, const void* indexes = NULL, const sp_size primitiveCount = 0) = 0;

		/// <summary>
		/// Release all allocated resources
		/// </summary>
		/// <returns></returns>
		API_INTERFACE virtual void dispose() = 0;

	};
}

#endif // SP_SHADER_HEADER