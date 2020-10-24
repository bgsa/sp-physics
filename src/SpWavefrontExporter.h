#ifndef SP_WAVEFRONT_EXPORTER_HEADER
#define SP_WAVEFRONT_EXPORTER_HEADER

#include "SpectrumPhysics.h"
#include "SpString.h"
#include "SpMesh.h"
#include "FileSystem.h"

using namespace NAMESPACE_FOUNDATION;

namespace NAMESPACE_PHYSICS
{
	namespace Wavefront
	{
		class SpWavefrontExporter
		{
		private:
			sp_uint vertexIndex;
			sp_uint meshesLength;
			sp_uint length;
			sp_char* output;
			
			inline void writeHeader()
			{
				std::memcpy(&output[length], "# Blender v2.83.4 OBJ File : ''", 31);
				output[31] = END_OF_LINE;
				length += 32;

				std::memcpy(&output[length], "# www.blender.org", 17);
				output[length + 17] = END_OF_LINE;
				length += 18;
			}

			inline void writeFace(const SpFaceMesh* face)
			{
				output[length++] = 'f';
				output[length++] = SP_STRING_SPACE;

				const sp_uint* indexes = face->vertexesIndexes;

				sp_uint temp;
				NAMESPACE_FOUNDATION::convert(indexes[0] + 1 + vertexIndex, &output[length], &temp);
				output[length + temp] = SP_STRING_SPACE;
				length += temp + 1;

				NAMESPACE_FOUNDATION::convert(indexes[1] + 1 + vertexIndex, &output[length], &temp);
				output[length + temp] = SP_STRING_SPACE;
				length += temp + 1;

				NAMESPACE_FOUNDATION::convert(indexes[2] + 1 + vertexIndex, &output[length], &temp);
				output[length + temp] = END_OF_STRING;
				length += temp;
			}

			inline void writeVertex(const Vec3& vertex)
			{
				output[length++] = 'v';
				output[length++] = SP_STRING_SPACE;

				sp_uint temp;
				NAMESPACE_FOUNDATION::convert(vertex.x, &output[length], &temp);
				output[length + temp] = SP_STRING_SPACE;
				length += temp + 1;

				NAMESPACE_FOUNDATION::convert(vertex.y, &output[length], &temp);
				output[length + temp] = SP_STRING_SPACE;
				length += temp + 1;

				NAMESPACE_FOUNDATION::convert(vertex.z, &output[length], &temp);
				length += temp;
			}

		public:

			API_INTERFACE inline SpWavefrontExporter()
			{
				vertexIndex = ZERO_UINT;
				length = ZERO_UINT;
				meshesLength = ZERO_UINT;

				output = ALLOC_NEW_ARRAY(sp_char, 5000);

				writeHeader();
			}

			API_INTERFACE inline void write(const SpMesh& mesh, const SpTransform& transform, const sp_char* meshName, const sp_char* colorName)
			{
				const sp_uint nameLength = strlen(meshName);

				std::memcpy(&output[length], "o ", 2);
				std::memcpy(&output[length + 2], meshName, nameLength);
				std::memcpy(&output[length + 2 + nameLength], ".001", 4);
				output[length + 2 + nameLength + 4] = END_OF_LINE;
				length += 2 + nameLength + 5;

				for (sp_uint i = 0; i < mesh.vertexesMesh->length(); i++)
				{
					SpVertexMesh* vertexMesh = mesh.vertexesMesh->get(i);
					Vec3 vertex;
					transform.transform(vertexMesh->value(), &vertex);

					writeVertex(vertex);
					output[length++] = END_OF_LINE;
				}

				for (sp_uint i = 0; i < mesh.faces->length(); i++)
				{
					SpFaceMesh* faceMesh = mesh.faces->get(i);

					writeFace(faceMesh);
					output[length++] = END_OF_LINE;
				}

				output[length-1] = END_OF_LINE;
				output[length] = END_OF_STRING;

				meshesLength ++;
				vertexIndex += mesh.vertexesMesh->length();
			}

			API_INTERFACE inline void save(const sp_char* filename)
			{
				SP_FILE file;
				file.open(filename, std::ios_base::out);
				file.write(output);
				file.close();
			}

		};

	}
	
}

#endif // SP_WAVEFRONT_EXPORTER_HEADER