#ifndef SP_ASSET_MATERIAL_SERIALIZER_JSON_HEADER
#define SP_ASSET_MATERIAL_SERIALIZER_JSON_HEADER

#include "SpectrumPhysics.h"
#include "SpAssetMaterial.h"
#include "nlohmann/json.hpp"

namespace NAMESPACE_PHYSICS
{
	class SpAssetMaterialSerializerJson
	{
	public:
		
		/// <summary>
		/// Serialize a material to json object
		/// </summary>
		/// <param name="materialObject">Material</param>
		/// <param name="materialJson">Json</param>
		/// <returns></returns>
		API_INTERFACE inline void serialize(void* materialObject, nlohmann::ordered_json& materialJson)
		{
			SpAssetMaterial* material = (SpAssetMaterial*)materialObject;

			nlohmann::ordered_json colorJson;
			colorJson["r"] = material->color.red;
			colorJson["g"] = material->color.green;
			colorJson["b"] = material->color.blue;
			colorJson["a"] = material->color.alpha;
			materialJson["color"] = colorJson;

			nlohmann::ordered_json ambientJson;
			ambientJson["r"] = material->ambient.red;
			ambientJson["g"] = material->ambient.green;
			ambientJson["b"] = material->ambient.blue;
			materialJson["ambient"] = ambientJson;

			nlohmann::ordered_json diffuseJson;
			diffuseJson["r"] = material->diffuse.red;
			diffuseJson["g"] = material->diffuse.green;
			diffuseJson["b"] = material->diffuse.blue;
			materialJson["diffuse"] = diffuseJson;

			nlohmann::ordered_json specularJson;
			specularJson["r"] = material->specular.red;
			specularJson["g"] = material->specular.green;
			specularJson["b"] = material->specular.blue;
			materialJson["specular"] = specularJson;

			materialJson["shininess-factor"] = material->shininessFactor;
			materialJson["albedo-map"] = material->albedoMapTexture;
			materialJson["normal-map"] = material->normalMapTexture;
		}

		/// <summary>
		/// Deserialize a Json object to Material
		/// </summary>
		/// <param name="materialObject">Material</param>
		/// <param name="materialJson">Json</param>
		/// <returns></returns>
		API_INTERFACE inline void deserialize(void* materialObject, nlohmann::json& materialJson)
		{
			SpAssetMaterial* material = (SpAssetMaterial*)materialObject;

			nlohmann::json colorJson = materialJson["color"];
			material->color.red = colorJson["r"].get<sp_float>();
			material->color.green = colorJson["g"].get<sp_float>();
			material->color.blue = colorJson["b"].get<sp_float>();
			material->color.alpha = colorJson["a"].get<sp_float>();

			nlohmann::json ambientJson = materialJson["ambient"];
			material->ambient.red = ambientJson["r"].get<sp_float>();
			material->ambient.green = ambientJson["g"].get<sp_float>();
			material->ambient.blue = ambientJson["b"].get<sp_float>();

			nlohmann::json diffuseJson = materialJson["diffuse"];
			material->diffuse.red = diffuseJson["r"].get<sp_float>();
			material->diffuse.green = diffuseJson["g"].get<sp_float>();
			material->diffuse.blue = diffuseJson["b"].get<sp_float>();

			nlohmann::json specularJson = materialJson["specular"];
			material->specular.red = specularJson["r"].get<sp_float>();
			material->specular.green = specularJson["g"].get<sp_float>();
			material->specular.blue = specularJson["b"].get<sp_float>();

			material->shininessFactor = materialJson["shininess-factor"].get<sp_float>();

			std::string texture = materialJson["albedo-map"].get<std::string>();
			std::memcpy(material->albedoMapTexture, texture.c_str(), texture.length());

			texture = materialJson["normal-map"].get<std::string>();
			std::memcpy(material->normalMapTexture, texture.c_str(), texture.length());
		}

	};

}

#endif // SP_ASSET_MATERIAL_SERIALIZER_JSON_HEADER