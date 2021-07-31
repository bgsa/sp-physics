#ifndef SP_RENDERABLE_OBJECT_HEADER
#define SP_RENDERABLE_OBJECT_HEADER

#include "SpectrumPhysics.h"
#include "SpGpuBuffer.h"

#define SP_RENDERABLE_OBJECT_MAX_BUFFERS (10)

#define SP_RENDERABLE_OBJECT_TYPE_PLANE (1)
#define SP_RENDERABLE_OBJECT_TYPE_CUBE  (2)

namespace NAMESPACE_PHYSICS 
{
	class SpRenderableObject
	{
	private:
		sp_uint _type;
		sp_int _visible;
		sp_uint _gameObjectIndex;
		sp_uint _meshDataIndex;
		sp_uint _shaderIndex;
		sp_int _buffersLength;
		SpGpuBuffer* _buffers[SP_RENDERABLE_OBJECT_MAX_BUFFERS];

	public:
	
		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline SpRenderableObject()
		{
			_shaderIndex = SP_UINT_MAX;
			_visible = (sp_int)true;
			_buffersLength = 0;
		}

		/// <summary>
		/// Constructor with parameter
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline SpRenderableObject(const sp_uint type)
		{
			this->_type = type;
		}

		/// <summary>
		/// Set the type of renderable object
		/// </summary>
		/// <param name="type"></param>
		/// <returns></returns>
		API_INTERFACE inline void type(const sp_uint type)
		{
			_type = type;
		}

		/// <summary>
		/// Get the type of renderable object
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_uint type() const
		{
			return _type;
		}

		/// <summary>
		/// Check this object is visible
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_bool isVisible() const
		{
			return _visible == 1;
		}

		/// <summary>
		/// Change the object visibility
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline void visible(const sp_bool visibility)
		{
			_visible = (sp_bool)visibility;
		}

		/// <summary>
		/// Get the game object index
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_uint gameObject() const
		{
			return _gameObjectIndex;
		}

		/// <summary>
		/// Set the game object index
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline void gameObject(const sp_uint index)
		{
			_gameObjectIndex = index;
		}

		/// <summary>
		/// Get the mesh data index
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_uint meshData() const
		{
			return _meshDataIndex;
		}

		/// <summary>
		/// Set the game object index
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline void meshData(const sp_uint index)
		{
			_meshDataIndex = index;
		}

		/// <summary>
		/// Get the shader index
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_uint shader() const
		{
			return _shaderIndex;
		}

		/// <summary>
		/// Set the shader
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline void shader(const sp_uint index)
		{
			_shaderIndex = index;
		}

		/// <summary>
		/// Get the buffers length 
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_int buffersLength() const
		{
			return _buffersLength;
		}

		/// <summary>
		/// Add new buffer in list
		/// </summary>
		/// <param name="newBuffer"></param>
		/// <returns></returns>
		API_INTERFACE inline void addBuffer(SpGpuBuffer* newBuffer)
		{
			sp_assert(_buffersLength < SP_RENDERABLE_OBJECT_MAX_BUFFERS, "IndexOutOfRangeException");

			_buffers[_buffersLength++] = newBuffer;
		}

		/// <summary>
		/// Get buffer by index
		/// </summary>
		/// <param name="index"></param>
		/// <returns></returns>
		API_INTERFACE inline SpGpuBuffer* buffers(const sp_int index) const
		{
			return _buffers[index];
		}

	};
}

#endif // SP_RENDERABLE_OBJECT_HEADER