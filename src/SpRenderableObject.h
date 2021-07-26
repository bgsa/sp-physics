#ifndef SP_RENDERABLE_OBJECT_HEADER
#define SP_RENDERABLE_OBJECT_HEADER

#include "SpectrumPhysics.h"
#include "SpGpuBuffer.h"

#define SP_GPU_BUFFER_MAX_LENGTH (10)

#define SP_RENDERABLE_OBJECT_TYPE_PLANE (1)
#define SP_RENDERABLE_OBJECT_TYPE_CUBE  (2)

namespace NAMESPACE_PHYSICS 
{
	class SpRenderableObject
	{
	private:
		sp_uint _type;
		
	public:
		sp_uint gameObjectIndex;
		sp_uint meshDataIndex;
		sp_uint shaderIndex;
		SpVector<SpGpuBuffer*> buffers;
	
		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline SpRenderableObject()
		{
			shaderIndex = 0;
		}

		/// <summary>
		/// Constructor with parameter
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline SpRenderableObject(const sp_uint type)
		{
			this->_type = type;
		}

		API_INTERFACE inline void type(const sp_uint type)
		{
			_type = type;
		}

		API_INTERFACE inline sp_uint type() const
		{
			return _type;
		}

	};
}

#endif // SP_RENDERABLE_OBJECT_HEADER