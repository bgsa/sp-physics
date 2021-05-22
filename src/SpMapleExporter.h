#ifndef SP_MAPLE_EXPORTER_HEADER
#define SP_MAPLE_EXPORTER_HEADER

#include "SpectrumPhysics.h"
#include "SpString.h"
#include "SpMesh.h"

using namespace NAMESPACE_FOUNDATION;

namespace NAMESPACE_PHYSICS
{
	namespace Maple
	{

		API_INTERFACE inline void display(const sp_char* meshName1, const sp_char* meshName2, sp_char* output, sp_uint* index)
		{
			std::memcpy(&output[*index], "display( [ ", 11);
			index[0] += 11;

			const sp_uint mesh1Length = (sp_uint)std::strlen(meshName1);
			std::memcpy(&output[*index], meshName1, mesh1Length);
			index[0] += mesh1Length;
			std::memcpy(&output[*index], "_graf, ", 7);
			index[0] += 7;

			const sp_uint mesh2Length = (sp_uint)std::strlen(meshName2);
			std::memcpy(&output[*index], meshName2, mesh2Length);
			index[0] += mesh2Length;
			std::memcpy(&output[*index], "_graf ]);", 9);
			index[0] += 9;

			output[*index] = END_OF_LINE;
			output[*index + 1] = END_OF_STRING;
			index[0] ++;
		}

		inline void exportGraphic(const sp_char* meshName, const sp_char* colorName, sp_char* output, sp_uint* index)
		{
			const sp_uint nameLength = (sp_uint)std::strlen(meshName);

			std::memcpy(&output[*index], "with(plots): with(plottools):", 29);
			output[*index + 29] = END_OF_LINE;
			index[0] += 30;

			std::memcpy(&output[*index], meshName, nameLength);
			index[0] += nameLength;
			std::memcpy(&output[*index], "_graf := display(", 17);
			index[0] += 17;
			std::memcpy(&output[*index], meshName, nameLength);
			index[0] += nameLength;
			std::memcpy(&output[*index], "_list_poly, color = ", 20);
			index[0] += 20;
			const sp_uint colorLength = (sp_uint)std::strlen(colorName);
			std::memcpy(&output[*index], colorName, colorLength);
			index[0] += colorLength;
			sp_char* text = ", thickness = 2, scaling = constrained, axes = boxed, labels = [axisX, axisY, axisZ], orientation = [90, 195, -190]):";
			sp_uint textLength = (sp_uint)std::strlen(text);
			std::memcpy(&output[*index], text, textLength);
			index[0] += textLength;

			output[*index] = END_OF_LINE;
			output[*index+1] = END_OF_STRING;
			index[0] ++;
		}

		API_INTERFACE inline void convert(const Vec3& vector, sp_char* output, sp_uint* length)
		{
			sp_uint temp;
			output[0] = '[';
			output[1] = ' ';
			
			NAMESPACE_FOUNDATION::convert(vector.x, &output[2], &temp);
			length[0] = temp + 2;

			std::memcpy(&output[*length], ", ", 2);
			length[0] += 2;

			NAMESPACE_FOUNDATION::convert(vector.y, &output[*length], &temp);
			length[0] += temp;

			std::memcpy(&output[*length], ", ", 2);
			length[0] += 2;

			NAMESPACE_FOUNDATION::convert(vector.z, &output[*length], &temp);
			length[0] += temp;

			output[*length] = ' ';
			output[*length + 1] = ']';
			output[*length + 2] = END_OF_STRING;
			length[0] += 2;
		}

		API_INTERFACE inline void convert(const SpFaceMesh& face, const SpTransform& transform, sp_char* output, sp_uint* len)
		{
			sp_uint temp;
			Vec3 vertexes[3];
			face.convert(vertexes, transform);

			std::memcpy(&output[0], "[ ", 2);
			len[0] = 2;

			convert(vertexes[0], &output[*len], &temp);
			len[0] += temp;

			std::memcpy(&output[*len], ", ", 2u);
			len[0] += 2;

			convert(vertexes[1], &output[*len], &temp);
			len[0] += temp;

			std::memcpy(&output[*len], ", ", 2u);
			len[0] += 2;

			convert(vertexes[2], &output[*len], &temp);
			len[0] += temp;

			std::memcpy(&output[*len], " ]", 2);
			len[0] += 2;
		}

		API_INTERFACE inline void convert(const Triangle3D& triangle, const sp_char* triangleName, sp_char* output, sp_uint* len)
		{
			const sp_uint nameLength = (sp_uint)std::strlen(triangleName);
			sp_uint temp;

			std::memcpy(&output[*len], triangleName, nameLength);
			len[0] += nameLength;

			std::memcpy(&output[*len], "_poly := polygon([ ", 19);
			len[0] += 19;

			convert(triangle.point1, &output[*len], &temp);
			len[0] += temp;

			std::memcpy(&output[*len], ", ", 2u);
			len[0] += 2;

			convert(triangle.point2, &output[*len], &temp);
			len[0] += temp;

			std::memcpy(&output[*len], ", ", 2u);
			len[0] += 2;

			convert(triangle.point3, &output[*len], &temp);
			len[0] += temp;

			std::memcpy(&output[*len], " ]) :", 5);
			len[0] += 5;

			output[*len] = END_OF_LINE;
			len[0] ++;

			std::memcpy(&output[*len], triangleName, nameLength);
			len[0] += nameLength;
			std::memcpy(&output[*len], "_list_poly := [ ", 16);
			len[0] += 16;
			std::memcpy(&output[*len], triangleName, nameLength);
			len[0] += nameLength;
			std::memcpy(&output[*len], "_poly ] :", 9);
			len[0] += 9;

			output[*len] = END_OF_LINE;
			len[0] ++;
		}

		API_INTERFACE inline void convert(const Line3D& line, const sp_char* lineName, sp_char* output, sp_uint* len)
		{
			const sp_uint nameLength = (sp_uint)std::strlen(lineName);
			sp_uint temp;
			
			std::memcpy(&output[*len], lineName, nameLength);
			len[0] += nameLength;

			std::memcpy(&output[*len], "_poly := polygon( [ ", 20);
			len[0] += 20;

			convert(line.point1, &output[*len], &temp);
			len[0] += temp;

			std::memcpy(&output[*len], ", ", 2u);
			len[0] += 2;

			convert(line.point2, &output[*len], &temp);
			len[0] += temp;

			std::memcpy(&output[*len], " ]) :", 5);
			len[0] += 5;

			output[*len] = END_OF_LINE;
			len[0] ++;

			std::memcpy(&output[*len], lineName, nameLength);
			len[0] += nameLength;
			std::memcpy(&output[*len], "_list_poly := [ ", 16);
			len[0] += 16;
			std::memcpy(&output[*len], lineName, nameLength);
			len[0] += nameLength;
			std::memcpy(&output[*len], "_poly ] :", 9);
			len[0] += 9;

			output[*len] = END_OF_LINE;
			len[0] ++;
		}

		API_INTERFACE inline void convert(const SpMesh& mesh, const SpTransform& transform, const sp_char* meshName, const sp_char* colorName, sp_char* output, sp_uint* index)
		{
			const sp_uint nameLength = (sp_uint)std::strlen(meshName);
			sp_uint len;
			sp_uint temp;

			for (sp_uint i = 0; i < mesh.faces->length(); i++)
			{
				SpFaceMesh* face = mesh.faces->get(i);

				std::memcpy(&output[*index], meshName, nameLength);
				index[0] += nameLength;

				std::memcpy(&output[*index], "_poly", 5);
				index[0] += 5;

				NAMESPACE_FOUNDATION::convert(i, &output[*index], &len);
				index[0] += len;

				std::memcpy(&output[*index], " := polygon(", 12);
				index[0] += 12;

				convert(*face, transform, &output[*index], &temp);
				index[0] += temp;

				std::memcpy(&output[*index], ") :", 3);
				index[0] += 3;
				output[*index] = END_OF_LINE;
				index[0] ++;
			}

			std::memcpy(&output[*index], meshName, nameLength);
			index[0] += nameLength;
			std::memcpy(&output[*index], "_list_poly := [ ", 16);
			index[0] += 16;

			for (sp_uint i = 0; i < mesh.faces->length(); i++)
			{
				std::memcpy(&output[*index], meshName, nameLength);
				index[0] += nameLength;

				std::memcpy(&output[*index], "_poly", 5);
				index[0] += 5;

				NAMESPACE_FOUNDATION::convert(i, &output[*index], &len);
				index[0] += len;
			
				std::memcpy(&output[*index], ", ", 2);
				index[0] += 2;
			}

			index[0] -= 2;
			std::memcpy(&output[*index], "] :", 3);
			output[*index + 3] = END_OF_LINE;
			index[0] += 4;

			exportGraphic(meshName, colorName, output, index);
		}

	}
	
}

#endif // SP_MAPLE_EXPORTER_HEADER