#ifndef SP_MATERIAL_HEADER
#define SP_MATERIAL_HEADER

#include "SpectrumPhysics.h"
#include "SpColorRGB.h"
#include "SpColorRGBA.h"

namespace NAMESPACE_PHYSICS
{
	class SpMaterial
	{
	public:
		SpColorRGBA color;
		SpColorRGB ambient;
		sp_uint albedoMapTexture;
		SpColorRGB diffuse;
		sp_uint normalMapTexture;
		SpColorRGB specular;
		sp_float shininessFactor;

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline SpMaterial()
		{
			color = SpColorRGBAWhite;
			ambient = SpColorRGBWhite;
			diffuse = SpColorRGBWhite;
			specular = SpColorRGBWhite;
			shininessFactor = 1.0f;

			albedoMapTexture = SP_UINT_MAX;
			normalMapTexture = SP_UINT_MAX;
		}

	};

}

#endif // SP_MATERIAL_HEADER