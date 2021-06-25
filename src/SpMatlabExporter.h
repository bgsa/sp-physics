#ifndef SP_MATLAB_EXPORTER_HEADER
#define SP_MATLAB_EXPORTER_HEADER

#include "SpectrumPhysics.h"
#include "SpString.h"
#include "SpMesh.h"

namespace NAMESPACE_PHYSICS
{
	namespace Matlab
	{
		inline void exportGraphic(const sp_char* meshName, const sp_char* colorName, sp_char* output, sp_uint& index)
		{
			const sp_uint nameLength = (sp_uint)std::strlen(meshName);
			const sp_uint colorLength = (sp_uint)std::strlen(colorName);

			std::memcpy(&output[index], "patch('Faces', f", 16);
			std::memcpy(&output[index + 16], meshName, nameLength);
			index += 16 + nameLength;

			std::memcpy(&output[index], ", 'Vertices', v", 15);
			std::memcpy(&output[index] + 15, meshName, nameLength);
			index += 15 + nameLength;

			std::memcpy(&output[index], ", 'FaceColor', '", 16);
			index += 16;

			std::memcpy(&output[index], colorName, colorLength);
			index += colorLength;

			std::memcpy(&output[index], "', 'EdgeColor', 'black', 'LineWidth', 1, 'FaceAlpha', 0.5);", 59);
			index += 59;
			
			output[index++] = END_OF_LINE;
		}

		API_INTERFACE inline void convexHull(const sp_char* vertexesName, sp_char* output, sp_uint& index)
		{
			std::memcpy(&output[index], "[indexes, av1] = convhull(mink, 'Simplify', true);", 50);
			index += 50;
			output[index++] = END_OF_LINE;

			std::memcpy(&output[index], "TR = triangulation(indexes, mink);", 34);
			index += 34;
			output[index++] = END_OF_LINE;

			std::memcpy(&output[index], "trisurf(TR);", 12);
			index += 12;
			output[index++] = END_OF_LINE;
		}

		API_INTERFACE inline void display(const sp_int xmin, const sp_int xmax, const sp_int ymin, const sp_int ymax, const sp_int zmin, const sp_int zmax, sp_char* output, sp_uint& index)
		{
			sp_uint temp;

			std::memcpy(&output[index], "surf = gca;", 11);
			output[index + 11] = END_OF_LINE;
			std::memcpy(&output[index + 12], "xlim(surf, [", 12);
			index += 24;
		
			NAMESPACE_FOUNDATION::convert(xmin, &output[index], temp);
			index += temp;
			output[index++] = ' ';

			NAMESPACE_FOUNDATION::convert(xmax, &output[index], temp);
			index += temp;

			std::memcpy(&output[index], "]);", 3);
			index += 3;

			std::memcpy(&output[index], "ylim(surf, [", 12);
			index += 12;

			NAMESPACE_FOUNDATION::convert(ymin, &output[index], temp);
			index += temp;
			output[index++] = ' ';

			NAMESPACE_FOUNDATION::convert(ymax, &output[index], temp);
			index += temp;

			std::memcpy(&output[index], "]);", 3);
			index += 3;

			std::memcpy(&output[index], "zlim(surf, [", 12);
			index += 12;

			NAMESPACE_FOUNDATION::convert(zmin, &output[index], temp);
			index += temp;
			output[index++] = ' ';

			NAMESPACE_FOUNDATION::convert(zmax, &output[index], temp);
			index += temp;

			std::memcpy(&output[index], "]);", 3);
			index += 3;
			output[index++] = END_OF_LINE;

			std::memcpy(&output[index], "xlabel('x'); ylabel('y'); zlabel('z');", 38);
			index += 38;

			output[index++] = END_OF_LINE;
		}

		API_INTERFACE inline void convert(const Vec3& vector, sp_char* output, sp_uint& index)
		{
			sp_uint temp;
			
			NAMESPACE_FOUNDATION::convert(vector.x, &output[index], temp);
			index += temp;

			output[index++] = ' ';

			NAMESPACE_FOUNDATION::convert(vector.y, &output[index], temp);
			index += temp;

			output[index++] = ' ';

			NAMESPACE_FOUNDATION::convert(vector.z, &output[index], temp);
			index += temp;

			output[index] = END_OF_STRING;
		}

		API_INTERFACE inline void drawSphere(const Vec3& position, const sp_float radius, sp_char* output, sp_uint* outputLength)
		{
			sp_size len = ZERO_UINT;
			sp_uint temp;

			std::memcpy(output, "[x, y, z] = sphere;", 19);
			output[19] = END_OF_LINE;
			len = 20;
			
			std::memcpy(&output[len], "x = x * ", 8);
			len += 8;

			NAMESPACE_FOUNDATION::convert(radius, &output[len], temp);
			len += temp;
			output[len] = ';';
			output[len + 1] = END_OF_LINE;
			len += 2u;

			std::memcpy(&output[len], "y = y * ", 8);
			len += 8;

			NAMESPACE_FOUNDATION::convert(radius, &output[len], temp);
			len += temp;
			output[len] = ';';
			output[len + 1] = END_OF_LINE;
			len += 2u;

			std::memcpy(&output[len], "z = z * ", 8);
			len += 8;

			NAMESPACE_FOUNDATION::convert(radius, &output[len], temp);
			len += temp;
			output[len] = ';';
			output[len + 1] = END_OF_LINE;
			len += 2u;

			std::memcpy(&output[len], "mesh = surf2patch(x + ", 22);
			len += 22;

			NAMESPACE_FOUNDATION::convert(position.x, &output[len], temp);
			len += temp;

			std::memcpy(&output[len], ", y + ", 6);
			len += 6;

			NAMESPACE_FOUNDATION::convert(position.y, &output[len], temp);
			len += temp;

			std::memcpy(&output[len], ", z + ", 6);
			len += 6;

			NAMESPACE_FOUNDATION::convert(position.z, &output[len], temp);
			len += temp;

			output[len] = ')';
			output[len + 1u] = ';';
			output[len + 2u] = END_OF_LINE;
			len += 3u;

			const sp_char* str = "patch('Faces', mesh.faces, 'Vertices', mesh.vertices, 'FaceColor', [1, 0, 1]);";
			const sp_size strLength = std::strlen(str);
			std::memcpy(&output[len], str, strLength);
			len += strLength + 2;
			output[len - 2u] = END_OF_LINE;
			output[len - 1u] = END_OF_STRING;
			outputLength[0] = (sp_uint)len;
		}

		API_INTERFACE inline void convert(const SpMesh& mesh, const Vec3* vertexes, const sp_char* meshName, const sp_char* colorName, sp_char* output, sp_uint& index)
		{
			const sp_uint nameLength = (sp_uint) std::strlen(meshName);

			// vertexes
			output[index] = 'v';
			std::memcpy(&output[index + 1], meshName, nameLength);
			index += nameLength + 1u;

			std::memcpy(&output[index], " = [", 4);
			index += 4u;

			for (sp_uint i = 0; i < mesh.vertexLength() - 1u; i++)
			{
				convert(vertexes[i], output, index);
				
				std::memcpy(&output[index], "; ", 2);
				index += 2u;
			}

			convert(vertexes[mesh.vertexLength() - 1u], output, index);

			std::memcpy(&output[index], "];", 2);
			output[index + 2u] = END_OF_LINE;
			index += 3u;

			// faces
			output[index] = 'f';
			std::memcpy(&output[index + 1], meshName, nameLength);
			index += nameLength + 1u;

			std::memcpy(&output[index], " = [", 4);
			index += 4u;

			const sp_uint shift = ONE_UINT;
			for (sp_uint i = 0; i < mesh.faces->length(); i++)
			{
				SpFaceMesh* face = mesh.faces->get(i);

				sp_uint len;
				NAMESPACE_FOUNDATION::convert(face->vertexesIndexes[0] + shift, &output[index], len);
				index += len;

				output[index++] = ' ';

				NAMESPACE_FOUNDATION::convert(face->vertexesIndexes[1] + shift, &output[index], len);
				index += len;

				output[index++] = ' ';

				NAMESPACE_FOUNDATION::convert(face->vertexesIndexes[2] + shift, &output[index], len);
				index += len;

				std::memcpy(&output[index], "; ", 2);
				index += 2u;
			}
			std::memcpy(&output[index - 2u], "];", 2);
			output[index++] = END_OF_LINE;

			exportGraphic(meshName, colorName, output, index);
		}

		API_INTERFACE inline void convert(const Plane& plane, const sp_char* meshName, const sp_char* colorName, sp_char* output, sp_uint& index)
		{
			const sp_uint nameLength = (sp_uint)std::strlen(meshName);

			// vertexes
			output[index] = 'v';
			std::memcpy(&output[index + 1], meshName, nameLength);
			index += nameLength + 1u;

			std::memcpy(&output[index], " = [", 4);
			index += 4u;

			const sp_uint vertexesLength = 4u;
			Vec3 vertexes[vertexesLength];

			Vec3 tangent;
			plane.tangent(tangent);

			Quat q = Quat::createRotate(radians(-90), plane.normalVector);
			vertexes[0] = plane.point + tangent * 2.0f;
			rotate(q, vertexes[0], vertexes[1]);
			rotate(q, vertexes[1], vertexes[2]);
			vertexes[2] = plane.point + -tangent * 2.0f;
			rotate(q, vertexes[2], vertexes[3]);

			for (sp_uint i = 0; i < vertexesLength -1u; i++)
			{
				convert(vertexes[i], output, index);

				std::memcpy(&output[index], "; ", 2);
				index += 2u;
			}
			convert(vertexes[3u], output, index);
			std::memcpy(&output[index], "];", 2);
			output[index + 2u] = END_OF_LINE;
			index += 3u;

			// faces
			output[index] = 'f';
			std::memcpy(&output[index + 1], meshName, nameLength);
			index += nameLength + 1u;

			std::memcpy(&output[index], " = [1, 2, 3; 3 4 1];", 20);
			index += 20u;
			output[index++] = END_OF_LINE;

			exportGraphic(meshName, colorName, output, index);
		}

		API_INTERFACE inline void convert(const sp_uint vertexesLength, const Vec3* vertexes, const sp_char* listName, sp_char* output, sp_uint& index)
		{
			const sp_uint nameLength = (sp_uint)std::strlen(listName);

			std::memcpy(&output[index], listName, nameLength);
			index += nameLength;

			std::memcpy(&output[index], " = [", 4);
			index += 4u;

			for (sp_uint i = 0; i < vertexesLength; i++)
			{
				convert(vertexes[i], output, index);

				std::memcpy(&output[index], "; ", 2);
				index += 2u;
			}

			convert(vertexes[vertexesLength - 1u], output, index);

			std::memcpy(&output[index], "];", 2);
			output[index + 2u] = END_OF_STRING;
			index += 2u;
		}

		API_INTERFACE inline void convertFace(const Vec3& point1, const Vec3& point2, const Vec3& point3, const sp_char* faceName, const sp_char* colorName, sp_char* output, sp_uint& index)
		{
			const sp_uint nameLength = (sp_uint)std::strlen(faceName);
			
			// vertexes
			output[index] = 'v';
			std::memcpy(&output[index + 1], faceName, nameLength);
			index += nameLength + 1u;

			std::memcpy(&output[index], " = [", 4);
			index += 4u;

			convert(point1, output, index);
			std::memcpy(&output[index], "; ", 2);
			index += 2u;
	
			convert(point2, &output[index], index);
			std::memcpy(&output[index], "; ", 2);
			index += 2u;

			convert(point3, &output[index], index);
			std::memcpy(&output[index], "];", 2);
			output[index + 2u] = END_OF_LINE;
			index += 3u;

			// faces
			output[index] = 'f';
			std::memcpy(&output[index + 1], faceName, nameLength);
			index += nameLength + 1u;

			std::memcpy(&output[index], " = [1 2 3];", 11);
			index += 11u;
			output[index++] = END_OF_LINE;

			exportGraphic(faceName, colorName, output, index);
		}

	}
	
}

#endif // SP_MATLAB_EXPORTER_HEADER