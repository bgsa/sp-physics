#ifndef SP_ASSET_MATERIAL_HEADER
#define SP_ASSET_MATERIAL_HEADER

#include "SpectrumPhysics.h"
#include "SpColorRGB.h"
#include "SpColorRGBA.h"
#include "SpDirectory.h"

namespace NAMESPACE_PHYSICS
{
	class SpAssetMaterial
	{
	public:
		SpColorRGBA color;
		SpColorRGB ambient;
		SpColorRGB diffuse;
		SpColorRGB specular;
		sp_float shininessFactor;
		sp_char albedoMapTexture[SP_DIRECTORY_MAX_LENGTH];
		sp_char normalMapTexture[SP_DIRECTORY_MAX_LENGTH];

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline SpAssetMaterial()
		{
			color = SpColorRGBAWhite;
			ambient = SpColorRGBWhite;
			diffuse = SpColorRGBWhite;
			specular = SpColorRGBWhite;
			shininessFactor = 1.0f;

			std::memset(albedoMapTexture, 0, SP_DIRECTORY_MAX_LENGTH);
			std::memset(normalMapTexture, 0, SP_DIRECTORY_MAX_LENGTH);
		}

	};

}

#endif // SP_ASSET_MATERIAL_HEADER