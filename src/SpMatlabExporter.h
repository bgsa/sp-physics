#ifndef SP_MATLAB_EXPORTER_HEADER
#define SP_MATLAB_EXPORTER_HEADER

#include "SpectrumPhysics.h"
#include "SpString.h"
#include "SpMesh.h"

namespace NAMESPACE_PHYSICS
{
	namespace Matlab
	{
		inline void exportGraphic(const sp_char* meshName, const sp_char* colorName, sp_char* output, sp_uint* index)
		{
			const sp_uint nameLength = strlen(meshName);
			const sp_uint colorLength = strlen(colorName);

			std::memcpy(output, "patch('Faces', f", 16);
			std::memcpy(&output[16], meshName, nameLength);
			index[0] += 16 + nameLength;

			std::memcpy(&output[*index], ", 'Vertices', v", 15);
			std::memcpy(&output[*index] + 15, meshName, nameLength);
			index[0] += 15 + nameLength;

			std::memcpy(&output[*index], ", 'FaceColor', '", 16);
			index[0] += 16;

			std::memcpy(&output[*index], colorName, colorLength);
			index[0] += colorLength;

			std::memcpy(&output[*index], "', 'EdgeColor', 'black', 'LineWidth', 1, 'FaceAlpha', 0.5);", 59);
			index[0] += 59;
			
			output[*index] = END_OF_LINE;
			output[*index + 1] = END_OF_STRING;
			index[0] += 2u;
		}

		API_INTERFACE inline void display(const sp_int xmin, const sp_int xmax, const sp_int ymin, const sp_int ymax, const sp_int zmin, const sp_int zmax, sp_char* output, sp_uint* index)
		{
			sp_uint len, temp;

			std::memcpy(output, "surf = gca;", 11);
			output[11] = END_OF_LINE;
			std::memcpy(&output[12], "xlim(surf, [", 12);
			len = 24;
		
			NAMESPACE_FOUNDATION::convert(xmin, &output[len], &temp);
			len += temp;
			output[len] = ' ';
			len++;

			NAMESPACE_FOUNDATION::convert(xmax, &output[len], &temp);
			len += temp;

			std::memcpy(&output[len], "]);", 3);
			len += 3;

			std::memcpy(&output[len], "ylim(surf, [", 12);
			len += 12;

			NAMESPACE_FOUNDATION::convert(ymin, &output[len], &temp);
			len += temp;
			output[len] = ' ';
			len++;

			NAMESPACE_FOUNDATION::convert(ymax, &output[len], &temp);
			len += temp;

			std::memcpy(&output[len], "]);", 3);
			len += 3;

			std::memcpy(&output[len], "zlim(surf, [", 12);
			len += 12;

			NAMESPACE_FOUNDATION::convert(zmin, &output[len], &temp);
			len += temp;
			output[len] = ' ';
			len++;

			NAMESPACE_FOUNDATION::convert(zmax, &output[len], &temp);
			len += temp;

			std::memcpy(&output[len], "]);", 3);
			len += 3;
			output[len] = END_OF_LINE;
			len++;

			std::memcpy(&output[len], "xlabel('x'); ylabel('y'); zlabel('z');", 38);
			output[len + 38] = END_OF_LINE;
			output[len + 39] = END_OF_STRING;
			len += 39;

			index[0] = len;
		}

		API_INTERFACE inline void convert(const Vec3& vector, sp_char* output, sp_uint* length)
		{
			sp_uint temp;
			
			NAMESPACE_FOUNDATION::convert(vector.x, &output[0], &temp);
			length[0] = temp;

			output[*length] = ' ';
			length[0] ++;

			NAMESPACE_FOUNDATION::convert(vector.y, &output[*length], &temp);
			length[0] += temp;

			output[*length] = ' ';
			length[0] ++;

			NAMESPACE_FOUNDATION::convert(vector.z, &output[*length], &temp);
			length[0] += temp;

			output[*length] = END_OF_STRING;
		}

		API_INTERFACE inline void drawSphere(const Vec3& position, const sp_float radius, sp_char* output, sp_uint* outputLength)
		{
			sp_uint len = ZERO_UINT;
			sp_uint temp;

			std::memcpy(output, "[x, y, z] = sphere;", 19);
			output[19] = END_OF_LINE;
			len = 20;
			
			std::memcpy(&output[len], "x = x * ", 8);
			len += 8;

			NAMESPACE_FOUNDATION::convert(radius, &output[len], &temp);
			len += temp;
			output[len] = ';';
			output[len + 1] = END_OF_LINE;
			len += 2u;

			std::memcpy(&output[len], "y = y * ", 8);
			len += 8;

			NAMESPACE_FOUNDATION::convert(radius, &output[len], &temp);
			len += temp;
			output[len] = ';';
			output[len + 1] = END_OF_LINE;
			len += 2u;

			std::memcpy(&output[len], "z = z * ", 8);
			len += 8;

			NAMESPACE_FOUNDATION::convert(radius, &output[len], &temp);
			len += temp;
			output[len] = ';';
			output[len + 1] = END_OF_LINE;
			len += 2u;

			std::memcpy(&output[len], "mesh = surf2patch(x + ", 22);
			len += 22;

			NAMESPACE_FOUNDATION::convert(position.x, &output[len], &temp);
			len += temp;

			std::memcpy(&output[len], ", y + ", 6);
			len += 6;

			NAMESPACE_FOUNDATION::convert(position.y, &output[len], &temp);
			len += temp;

			std::memcpy(&output[len], ", z + ", 6);
			len += 6;

			NAMESPACE_FOUNDATION::convert(position.z, &output[len], &temp);
			len += temp;

			output[len] = ')';
			output[len + 1u] = ';';
			output[len + 2u] = END_OF_LINE;
			len += 3u;

			const sp_char* str = "patch('Faces', mesh.faces, 'Vertices', mesh.vertices, 'FaceColor', [1, 0, 1]);";
			const sp_uint strLength = strlen(str);
			std::memcpy(&output[len], str, strLength);
			len += strLength + 2u;
			output[len - 2u] = END_OF_LINE;
			output[len - 1u] = END_OF_STRING;
			outputLength[0] = len;
		}

		API_INTERFACE inline void convert(const SpMesh& mesh, const Vec3* vertexes, const sp_char* meshName, const sp_char* colorName, sp_char* output, sp_uint* index)
		{
			const sp_uint nameLength = strlen(meshName);
			sp_uint len;

			// vertexes
			output[0] = 'v';
			std::memcpy(&output[1], meshName, nameLength);
			len = nameLength + 1u;

			std::memcpy(&output[len], " = [", 4);
			len += 4u;

			sp_uint tempLength = ZERO_UINT;

			for (sp_uint i = 0; i < mesh.vertexLength() - 1u; i++)
			{
				tempLength = ZERO_UINT;

				convert(vertexes[i], &output[len], &tempLength);
				len += tempLength;
				
				std::memcpy(&output[len], "; ", 2);
				len += 2u;
			}

			tempLength = ZERO_UINT;
			convert(vertexes[mesh.vertexLength() - 1u], &output[len], &tempLength);
			len += tempLength;

			std::memcpy(&output[len], "];", 2);
			output[len + 2u] = END_OF_LINE;
			len += 3u;

			// faces
			output[len] = 'f';
			std::memcpy(&output[len + 1], meshName, nameLength);
			len += nameLength + 1u;

			std::memcpy(&output[len], " = [", 4);
			len += 4u;

			const sp_uint shift = ONE_UINT;
			for (sp_uint i = 0; i < mesh.faces->length(); i++)
			{
				SpFaceMesh* face = mesh.faces->get(i);

				tempLength = ZERO_UINT;
				NAMESPACE_FOUNDATION::convert(face->vertexesIndexes[0] + shift, &output[len], &tempLength);
				output[len + tempLength] = ' ';
				len += tempLength + 1u;

				tempLength = ZERO_UINT;
				NAMESPACE_FOUNDATION::convert(face->vertexesIndexes[1] + shift, &output[len], &tempLength);
				output[len + tempLength] = ' ';
				len += tempLength + 1u;

				tempLength = ZERO_UINT;
				NAMESPACE_FOUNDATION::convert(face->vertexesIndexes[2] + shift, &output[len], &tempLength);
				std::memcpy(&output[len + tempLength], "; ", 2);
				len += tempLength + 2u;
			}
			std::memcpy(&output[len - 2u], "];", 2);
			output[len++] = END_OF_LINE;

			tempLength = ZERO_UINT;
			exportGraphic(meshName, colorName, &output[len], &tempLength);

			index[0] = len + tempLength;
		}

		API_INTERFACE inline void convertFace(const Vec3& point1, const Vec3& point2, const Vec3& point3, const sp_char* faceName, const sp_char* colorName, sp_char* output, sp_uint* index)
		{
			const sp_uint nameLength = strlen(faceName);
			
			// vertexes
			output[0] = 'v';
			std::memcpy(&output[1], faceName, nameLength);
			sp_uint len = nameLength + 1u;

			std::memcpy(&output[len], " = [", 4);
			len += 4u;

			sp_uint tempLength = ZERO_UINT;
			convert(point1, &output[len], &tempLength);
			len += tempLength;
			std::memcpy(&output[len], "; ", 2);
			len += 2u;
	
			tempLength = ZERO_UINT;
			convert(point2, &output[len], &tempLength);
			len += tempLength;
			std::memcpy(&output[len], "; ", 2);
			len += 2u;

			tempLength = ZERO_UINT;
			convert(point3, &output[len], &tempLength);
			len += tempLength;
			std::memcpy(&output[len], "];", 2);
			output[len + 2u] = END_OF_LINE;
			len += 3u;

			// faces
			output[len] = 'f';
			std::memcpy(&output[len + 1], faceName, nameLength);
			len += nameLength + 1u;

			std::memcpy(&output[len], " = [1 2 3];", 11);
			len += 11u;
			output[len++] = END_OF_LINE;

			tempLength = ZERO_UINT;
			exportGraphic(faceName, colorName, &output[len], &tempLength);

			index[0] = len + tempLength;
		}

	}
	
}

#endif // SP_MATLAB_EXPORTER_HEADER