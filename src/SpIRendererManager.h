#ifndef SP_I_RENDERER_MANAGER_HEADER
#define SP_I_RENDERER_MANAGER_HEADER

#include "SpectrumPhysics.h"
#include "SpIGraphicObject2D.h"
#include "SpIGraphicObject3D.h"
#include "SpIGraphicObjectList3D.h"
#include "SpCamera.h"
#include "SpViewportData.h"
#include "SpRenderData.h"

namespace NAMESPACE_PHYSICS
{
	class SpIRendererManager
	{
	protected:
		SpCamera* _camera;
		SpViewportData _viewport;

		SpVector<SpIGraphicObject2D*> graphicObjects2D;
		SpVector<SpIGraphicObject3D*> graphicObjects3D;

		inline void render3D(const SpRenderData& renderData)
		{
			for (SpVectorItem<SpIGraphicObject3D*>* item = graphicObjects3D.begin(); item != nullptr; item = item->next())
			{
				item->value()->render(renderData);
				//LogGL::glErrors(__FILE__, __LINE__);
			}
		}

		inline void render2D(const SpRenderData& renderData)
		{
			for (SpVectorItem<SpIGraphicObject2D*>* item = graphicObjects2D.begin(); item != nullptr; item = item->next())
				item->value()->render(renderData);
		}

	public:

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns></returns>
		API_INTERFACE SpIRendererManager()
		{
			_camera = nullptr;
		}
		
		API_INTERFACE inline SpCamera* camera()
		{
			return _camera;
		}

		API_INTERFACE inline SpViewportData& viewport()
		{
			return _viewport;
		}

		API_INTERFACE virtual void init(SpCamera* camera)
		{
			_camera = camera;
		}

		API_INTERFACE virtual void preRender() = 0;

		API_INTERFACE virtual void render() = 0;

		API_INTERFACE virtual void postRender() = 0;

		API_INTERFACE virtual void resize(sp_float width, sp_float height) = 0;

		API_INTERFACE virtual void addGraphicObject(SpIGraphicObject2D* graphicObject)
		{
			graphicObjects2D.add(graphicObject);
		}

		API_INTERFACE virtual void addGraphicObject(SpIGraphicObject3D* graphicObject)
		{
			graphicObjects3D.add(graphicObject);
		}

		API_INTERFACE virtual void addGraphicObject(SpIGraphicObjectList3D* graphicObjectList)
		{
			graphicObjects3D.add(graphicObjectList);
		}

		API_INTERFACE virtual sp_bool hasGraphicObject(SpIGraphicObject2D* graphicObject)
		{
			SpVectorItem<SpIGraphicObject2D*>* item = graphicObjects2D.find(graphicObject);
			return item != nullptr;
		}

		API_INTERFACE virtual sp_bool hasGraphicObject(SpIGraphicObject3D* graphicObject)
		{
			SpVectorItem<SpIGraphicObject3D*>* item = graphicObjects3D.find(graphicObject);
			return item != nullptr;
		}

		API_INTERFACE virtual void removeGraphicObject(SpIGraphicObject3D* graphicObject)
		{
			SpVectorItem<SpIGraphicObject3D*>* item = graphicObjects3D.find(graphicObject);

			if (item != nullptr)
				graphicObjects3D.remove(item);
		}

		API_INTERFACE virtual void removeGraphicObject(SpIGraphicObject2D* graphicObject)
		{
			SpVectorItem<SpIGraphicObject2D*>* item = graphicObjects2D.find(graphicObject);

			if (item != nullptr)
				graphicObjects2D.remove(item);
		}

		API_INTERFACE virtual void dispose() = 0;

	};
}

#endif // SP_I_RENDERER_MANAGER_HEADER