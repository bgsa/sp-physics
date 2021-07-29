#ifndef SP_MATERIAL_HEADER
#define SP_MATERIAL_HEADER

#include "SpectrumPhysics.h"
#include "SpColorRGBA.h"

namespace NAMESPACE_PHYSICS
{
	class SpMaterial
	{
	public:
		SpColorRGBA color;
		sp_float shininessFactor;
		Vec3 ambient;
		Vec3 diffuse;
		Vec3 specular;

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline SpMaterial()
		{
			color = SpColorRGBAWhite;
			ambient  = Vec3Ones;
			diffuse  = Vec3Ones;
			specular = Vec3Ones;
			shininessFactor = 1000.0f;
		}

	};
}

#endif // SP_MATERIAL_HEADER