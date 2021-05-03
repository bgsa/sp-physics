#ifndef MODEL_HEADER
#define MODEL_HEADER

#include "SpectrumPhysics.h"
#include <SpPoint3.h>
#include <SpArray.h>
#include <SpString.h>
#include "Plane.h"

namespace NAMESPACE_PHYSICS
{
	class Model
		: public Object
	{
	public:
		SpString* name;
		SpArray<Vec3>* vertexes;
		SpArray<Vec3>* normals;
		SpArray<SpPoint3<sp_uint>>* faces;
		SpArray<Vec2>* textureCoordinates;

		API_INTERFACE Model()
		{
			name = NULL;
			vertexes = NULL;
			normals = NULL; 
			faces = NULL;
			textureCoordinates = NULL;
		}

		API_INTERFACE inline sp_size sizeOfVertexes() const
		{
			return vertexes->length() * VEC3_SIZE;
		}
		API_INTERFACE inline sp_size sizeOfFaces() const
		{
			return faces->length() * sizeof(SpPoint3<sp_uint>);
		}
		API_INTERFACE inline sp_size sizeOfNormals() const
		{
			return normals->length() * VEC3_SIZE;
		}
		API_INTERFACE inline sp_size sizeOfTextures() const
		{
			return textureCoordinates->length() * sizeof(Vec2);
		}

		API_INTERFACE virtual void load(const SpString& filename) = 0;

		API_INTERFACE virtual sp_size allBuffersLength() const
		{
			return vertexes->length() * VEC3_LENGTH
				+ normals->length() * VEC3_LENGTH
				+ textureCoordinates->length() * VEC2_LENGTH;
		}

		API_INTERFACE virtual void allBuffers(sp_float* allocatedBuffer) const
		{
			const sp_size vertexesLength = vertexes->length() * VEC3_LENGTH;
			const sp_size normalsLength = normals->length() * VEC3_LENGTH;
			const sp_size texturesLength = textureCoordinates->length() * VEC2_LENGTH;

			std::memcpy(allocatedBuffer, (sp_float*) vertexes->data(), vertexesLength * SIZEOF_FLOAT);
			std::memcpy(&allocatedBuffer[vertexesLength], (sp_float*)normals->data(), normalsLength * SIZEOF_FLOAT);
			std::memcpy(&allocatedBuffer[vertexesLength + normalsLength], (sp_float*)textureCoordinates->data(), texturesLength * SIZEOF_FLOAT);
		}

		API_INTERFACE virtual void generateNormals()
		{
			if (normals != NULL)
			{
				sp_mem_delete(normals, SpArray<Vec3>);
			}

			normals = sp_mem_new(SpArray<Vec3>)(vertexes->length(), vertexes->length());

			for (sp_uint i = ZERO_UINT; i < faces->length(); i++)
			{
				const SpPoint3<sp_uint> indexes = faces->data()[i];

				const Plane plane(
					vertexes->data()[indexes[0]],
					vertexes->data()[indexes[1]],
					vertexes->data()[indexes[2]]
				);

				normals->data()[indexes[0]] += plane.normalVector;
				normals->data()[indexes[1]] += plane.normalVector;
				normals->data()[indexes[2]] += plane.normalVector;
			}

			for (sp_uint i = ZERO_UINT; i < vertexes->length(); i++)
				normalize(normals->data()[i]);
		}

		API_INTERFACE virtual void dispose() override
		{
			if (name != NULL)
			{
				sp_mem_delete(name, SpString);
				name = NULL;
			}

			if (vertexes != NULL)
			{
				sp_mem_release(vertexes);
				vertexes = NULL;
			}

			if (normals != NULL)
			{
				sp_mem_release(normals);
				normals = NULL;
			}

			if (textureCoordinates != NULL)
			{
				sp_mem_release(textureCoordinates);
				textureCoordinates = NULL;
			}

			if (faces != NULL)
			{
				sp_mem_release(faces);
				faces = NULL;
			}
		}

		~Model()
		{
			dispose();
		}

	};
}

#endif // MODEL_HEADER