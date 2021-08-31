#ifndef SP_BASE_MATERIAL_HEADER
#define SP_BASE_MATERIAL_HEADER

#include "SpectrumPhysics.h"
#include "SpColorRGBA.h"
#include "SpColorRGB.h"

namespace NAMESPACE_PHYSICS
{
	class SpBaseMaterial
	{
	public:
		SpColorRGBA color;
		sp_float shininessFactor;
		SpColorRGB ambient;
		SpColorRGB diffuse;
		SpColorRGB specular;

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline SpBaseMaterial()
		{
			color = SpColorRGBAWhite;
			ambient  = SpColorRGBWhite;
			diffuse  = SpColorRGBWhite;
			specular = SpColorRGBWhite;
			shininessFactor = 1.0f;
		}

	};
}

#endif // SP_BASE_MATERIAL_HEADER