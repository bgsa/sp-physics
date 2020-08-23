#include "SpMesh.h"

namespace NAMESPACE_PHYSICS
{

	void SpVertexEdges::findFacesFromIndexes(Triangle3D* outputFaces, sp_uint* facesLength, const SpTransform* transform) const
	{
		*facesLength = ZERO_UINT;

		Vec3* vertexes = mesh->vertexes->data();
		SpPoint3<sp_uint>* facesIndexes = mesh->facesIndexes->data();
		
		for (sp_uint i = 0; i < mesh->facesIndexes->length(); i++)
		{
			if (facesIndexes[i].x == _vertexIndex || facesIndexes[i].y == _vertexIndex || facesIndexes[i].z == _vertexIndex)
			{
				outputFaces[*facesLength] = Triangle3D(
					vertexes[facesIndexes[i].x],
					vertexes[facesIndexes[i].y],
					vertexes[facesIndexes[i].z]
				);
				*facesLength = (*facesLength) + 1u;
			}
		}
	}

	void SpVertexEdges::findFacesFromEdges(Triangle3D* outputFaces, sp_uint* facesLength, const SpTransform* transform) const
	{
		sp_uint count = ZERO_UINT;

		const Vec3 vertex1 = mesh->vertexes->data()[_vertexIndex];

		for (SpVectorItem<SpVertexEdges*>* item1 = _edges->begin(); item1 != nullptr; item1 = item1->next())
		{
			const Vec3 vertex2 = mesh->vertexes->data()[item1->value()->_vertexIndex];

			for (SpVectorItem<SpVertexEdges*>* item2 = item1->value()->_edges->begin(); item2 != nullptr; item2 = item2->next())
			{
				const Vec3 vertex3 = mesh->vertexes->data()[item2->value()->_vertexIndex];

				if (vertex1 == vertex3)
					continue;

				if (vertex3.orientation(vertex2, vertex1) < ZERO_FLOAT) // check if it is is front-face
				{
					for (SpVectorItem<SpVertexEdges*>* item3 = item2->value()->_edges->begin(); item3 != nullptr; item3 = item3->next())
						if (item3->value()->_vertexIndex == _vertexIndex) // if has edge with vertex 1, it makes a face!
						{
							Vec3 transformedVertex1;
							transform->transform(vertex1, &transformedVertex1);

							Vec3 transformedVertex2;
							transform->transform(vertex2, &transformedVertex2);

							Vec3 transformedVertex3;
							transform->transform(vertex3, &transformedVertex3);

							outputFaces[count].point1 = transformedVertex1;
							outputFaces[count].point2 = transformedVertex2;
							outputFaces[count].point3 = transformedVertex3;
							count++;
						}
				}
			}
		}

		*facesLength = count;
	}


}