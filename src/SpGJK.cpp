#include "SpGJK.h"

namespace NAMESPACE_PHYSICS
{

	sp_bool epa(const Vec3 tetrahedron[4], const SpMesh* mesh1, const Vec3* vertexesMesh1, const SpMesh* mesh2, const Vec3* vertexesMesh2, Vec3& normal, sp_float& depth, const sp_uint maxIterations, sp_uint& iterations, const sp_float _epsilon)
	{
#define EPA_MAX_NUM_FACES 128
#define EPA_MAX_NUM_LOOSE_EDGES 64
#define EPA_MAX_NUM_ITERATIONS 64
#define EPA_BIAS 0.000001
#define EPA_TOLERANCE 0.0001

		SpFace faces[EPA_MAX_NUM_FACES]; //Array of faces, each with 3 verts and a normal

		//Init with final simplex from GJK
		faces[0].vertex1 = faces[1].vertex1 = faces[2].vertex1 = tetrahedron[0];
		faces[0].vertex2 = faces[2].vertex3 = faces[3].vertex1 = tetrahedron[1];
		faces[0].vertex3 = faces[1].vertex2 = faces[3].vertex3 = tetrahedron[2];
		faces[1].vertex3 = faces[2].vertex2 = faces[3].vertex2 = tetrahedron[3];

		cross(tetrahedron[1] - tetrahedron[0], tetrahedron[2] - tetrahedron[0], faces[0].normal);
		normalize(faces[0].normal); //ABC

		cross(tetrahedron[2] - tetrahedron[0], tetrahedron[3] - tetrahedron[0], faces[1].normal);
		normalize(faces[1].normal); //ACD

		cross(tetrahedron[3] - tetrahedron[0], tetrahedron[1] - tetrahedron[0], faces[2].normal);
		normalize(faces[2].normal); //ADB

		cross(tetrahedron[3] - tetrahedron[1], tetrahedron[2] - tetrahedron[1], faces[3].normal);
		normalize(faces[3].normal); //BDC

		sp_uint num_faces = 4u;
		sp_int closest_face;
		Vec3 newSimplexPoint, search_dir;

		iterations = 0;
		for (sp_uint i = 0u; i < maxIterations; i++)
		{
			iterations++;

			//Find face that's closest to origin
			depth = faces[0].vertex1.dot(faces[0].normal);
			closest_face = 0;
			for (sp_uint i = 1u; i < num_faces; i++) 
			{
				const sp_float dist = faces[i].vertex1.dot(faces[i].normal);
			
				if (dist < depth)
				{
					depth = dist;
					closest_face = i;
				}
			}

			//search normal to face that's closest to origin
			search_dir = faces[closest_face].normal;
			diff(
				vertexesMesh2[mesh2->support(search_dir, vertexesMesh2)->index()], 
				vertexesMesh1[mesh1->support(-search_dir, vertexesMesh1)->index()], 
				newSimplexPoint
			);

			const sp_float temp = newSimplexPoint.dot(search_dir);
			if (temp - depth < EPA_TOLERANCE)
			{
				//Convergence (new point is not significantly further from origin)
				if (NAMESPACE_FOUNDATION::isCloseEnough(temp, ZERO_FLOAT, SP_EPSILON_THREE_DIGITS))
					normal = faces[closest_face].normal;
				else 
				{
					// dot vertex with normal to resolve collision along normal!
					multiply(faces[closest_face].normal, temp, normal);
					normalize(normal);
				}
				return true;
			}

			Vec3 loose_edges[EPA_MAX_NUM_LOOSE_EDGES][2]; //keep track of edges we need to fix after removing faces
			sp_int num_loose_edges = 0;

			//Find all triangles that are facing newSimplexPoint
			for (sp_uint i = 0u; i < num_faces; i++)
			{
				if (faces[i].normal.dot(newSimplexPoint - faces[i].vertex1) > ZERO_FLOAT) //triangle i faces newSimplexPoint, remove it
				{
					//Add removed triangle's edges to loose edge list.
					//If it's already there, remove it (both triangles it belonged to are gone)
					for (sp_int j = 0; j < 3; j++) //Three edges per face
					{
						Vec3 current_edge[2] = { faces[i][j], faces[i][(j + 1) % 3] };
						sp_bool found_edge = false;
						for (sp_int k = 0; k < num_loose_edges; k++) //Check if current edge is already in list
						{
							if (loose_edges[k][1] == current_edge[0] && loose_edges[k][0] == current_edge[1]) {
								//Edge is already in the list, remove it
								//THIS ASSUMES EDGE CAN ONLY BE SHARED BY 2 TRIANGLES (which should be true)
								//THIS ALSO ASSUMES SHARED EDGE WILL BE REVERSED IN THE TRIANGLES (which 
								//should be true provided every triangle is wound CCW)
								loose_edges[k][0] = loose_edges[num_loose_edges - 1][0]; //Overwrite current edge
								loose_edges[k][1] = loose_edges[num_loose_edges - 1][1]; //with last edge in list
								num_loose_edges--;
								found_edge = true;
								k = num_loose_edges; //exit loop because edge can only be shared once
							}
						}

						if (!found_edge) //add current edge to list
						{ 
							// sp_assert(num_loose_edges<EPA_MAX_NUM_LOOSE_EDGES, "ApplicationException");
							if (num_loose_edges >= EPA_MAX_NUM_LOOSE_EDGES) 
								break;

							loose_edges[num_loose_edges][0] = current_edge[0];
							loose_edges[num_loose_edges][1] = current_edge[1];
							num_loose_edges++;
						}
					}

					//Remove triangle i from list
					std::memcpy(&faces[i], &faces[num_faces - 1], sizeof(SpFace));
					num_faces--;
					i--;
				}
			}

			//Reconstruct polytope with newSimplexPoint added
			for (sp_int i = 0; i < num_loose_edges; i++)
			{
				if (num_faces >= EPA_MAX_NUM_FACES) // if exceded max faces
					break;

				faces[num_faces].vertex1 = loose_edges[i][0];
				faces[num_faces].vertex2 = loose_edges[i][1];
				faces[num_faces].vertex3 = newSimplexPoint;
				cross(loose_edges[i][0] - loose_edges[i][1], loose_edges[i][0] - newSimplexPoint, faces[num_faces].normal);

				if (faces[num_faces].normal != Vec3Zeros) // check the face is degenerated to a line (two vertexes are equal)
				{
					normalize(faces[num_faces].normal);

					//Check for wrong normal to maintain CCW winding
					if (faces[num_faces].vertex1.dot(faces[num_faces].normal) + EPA_BIAS < ZERO_FLOAT)
					{
						NAMESPACE_PHYSICS::swap(faces[num_faces].vertex1, faces[num_faces].vertex2);
						faces[num_faces].normal = -faces[num_faces].normal;
					}
					num_faces++;
				}
			}
		}
	
		// Return most recent closest point
		const sp_float temp = newSimplexPoint.dot(search_dir);
		// dot vertex with normal to resolve collision along normal!
		if (NAMESPACE_FOUNDATION::isCloseEnough(temp, ZERO_FLOAT, SP_EPSILON_THREE_DIGITS))
			normal = faces[closest_face].normal;
		else
		{
			// multiply(faces[closest_face].normal, faces[closest_face].vertex1.dot(faces[closest_face].normal), normal);
			multiply(faces[closest_face].normal, temp, normal);
			normalize(normal);
		}
			return true;
#undef EPA_TOLERANCE
#undef EPA_BIAS
#undef EPA_MAX_NUM_ITERATIONS
#undef EPA_MAX_NUM_LOOSE_EDGES
#undef EPA_MAX_NUM_FACES
	}

	void update_simplex3(Vec3& a, Vec3& b, Vec3& c, Vec3& d, sp_int& simp_dim, Vec3& search_dir)
	{
		/* Required winding order:
		//  b
		//  | \
		//  |   \
		//  |    a
		//  |   /
		//  | /
		//  c
		*/

		Vec3 ab;
		diff(b, a, ab);

		Vec3 ac;
		diff(c, a, ac);

		Vec3 n;
		NAMESPACE_PHYSICS::cross(ab, ac, n); //triangle's normal

		Vec3 AO = -a; //direction to origin

		//Determine which feature is closest to origin, make that the new simplex
		simp_dim = 2;

		Vec3 temp;
		NAMESPACE_PHYSICS::cross(ab, n, temp);

		if (temp.dot(AO) > ZERO_FLOAT)  //Closest to edge ABs
		{
			c = a;
			//simp_dim = 2;
			ab.tripleProduct(AO, ab, &search_dir);
			return;
		}

		NAMESPACE_PHYSICS::cross(n, ac, temp);

		if (temp.dot(AO) > ZERO_FLOAT) //Closest to edge AC
		{
			b = a;
			//simp_dim = 2;
			ac.tripleProduct(AO, ac, &search_dir);
			return;
		}

		simp_dim = 3;
		if (n.dot(AO) > ZERO_FLOAT) //Above triangle
		{
			d = c;
			c = b;
			b = a;
			//simp_dim = 3;
			search_dir = n;
			return;
		}

		//else //Below triangle
		d = b;
		b = a;
		//simp_dim = 3;
		search_dir = -n;
	}

	sp_bool update_simplex4(Vec3& a, Vec3& b, Vec3& c, Vec3& d, sp_int& simp_dim, Vec3& search_dir)
	{
		// a is peak/tip of pyramid, BCD is the base (counterclockwise winding order)
		//We know a priori that origin is above BCD and below a

		Vec3 ab;
		diff(b, a, ab);

		Vec3 ac;
		diff(c, a, ac);

		//Get normals of three new faces ABC, ACD, ADB
		Vec3 ABC;
		cross(ab, ac, ABC);

		Vec3 AO = -a; //dir to origin
		simp_dim = 3; //hoisting this just cause

		// plane-test origin with 3 faces
		if (ABC.dot(AO) > ZERO_FLOAT)  //In front of ABC
		{
			d = c;
			c = b;
			b = a;
			search_dir = ABC;
			return false;
		}

		Vec3 ad;
		diff(d, a, ad);

		Vec3 ACD;
		cross(ac, ad, ACD);

		if (ACD.dot(AO) > ZERO_FLOAT) //In front of ACD
		{
			b = a;
			search_dir = ACD;
			return false;
		}

		Vec3 ADB;
		cross(ad, ab, ADB);

		if (ADB.dot(AO) > ZERO_FLOAT) //In front of ADB
		{
			c = d;
			d = b;
			b = a;
			search_dir = ADB;
			return false;
		}

		return true; // inside tetrahedron; enclosed!

		//Note: in the case where two of the faces have similar normals,
		//The origin could conceivably be closest to an edge on the tetrahedron
		//Right now I don't think it'll make a difference to limit our new simplices
		//to just one of the faces, maybe test it later.
	}

	sp_bool gjk(const SpMesh* mesh1, const Vec3* vertexesMesh1, const SpMesh* mesh2, const Vec3* vertexesMesh2, Vec3* output, sp_uint maxIterations, sp_uint& iterations)
	{
		Vec3 a, b, c, d; //Simplex: just a set of points (a is always most recently added)
		Vec3 search_dir = Vec3Ones; //initial search direction between colliders

		//Get initial point for simplex
		c = vertexesMesh2[mesh2->support(search_dir, vertexesMesh2)->index()] - vertexesMesh1[mesh1->support(-search_dir, vertexesMesh1)->index()];
		search_dir = -c; //search in direction of origin

		//Get second point for a line segment simplex
		b = vertexesMesh2[mesh2->support(search_dir, vertexesMesh2)->index()] - vertexesMesh1[mesh1->support(-search_dir, vertexesMesh1)->index()];

		if (b.dot(search_dir) < ZERO_FLOAT)
			return false; //we didn't reach the origin, won't enclose it

		Vec3 temp;
		NAMESPACE_PHYSICS::cross(c - b, -b, temp);
		NAMESPACE_PHYSICS::cross(temp, c - b, search_dir); //search perpendicular to line segment towards origin

		if (isCloseEnough(search_dir, Vec3Zeros)) // origin is on this line segment 
		{
			//Apparently any normal search vector will do?
			NAMESPACE_PHYSICS::cross(c - b, Vec3(1.0f, 0.0f, 0.0f), search_dir); //normal with x-axis

			if (isCloseEnough(search_dir, Vec3Zeros))
				NAMESPACE_PHYSICS::cross(c - b, Vec3(0, 0, -1), search_dir); //normal with z-axis
		}
		sp_int simp_dim = 2; //simplex dimension

		iterations = 0;
		for (sp_uint i = 0u; i < maxIterations; i++)
		{
			iterations++;

			a = vertexesMesh2[mesh2->support(search_dir, vertexesMesh2)->index()] - vertexesMesh1[mesh1->support(-search_dir, vertexesMesh1)->index()];
			if (a.dot(search_dir) < 0)
				return false; //we didn't reach the origin, won't enclose it

			simp_dim++;
			if (simp_dim == 3)
			{
				update_simplex3(a, b, c, d, simp_dim, search_dir);
			}
			else if (update_simplex4(a, b, c, d, simp_dim, search_dir))
			{
				if (a == b) // check the origin is on the face
				{
					Vec3 ac;
					diff(c, a, ac);

					Vec3 ad;
					diff(d, a, ad);

					cross(ac, ad, search_dir);

					a = vertexesMesh2[mesh2->support(-search_dir, vertexesMesh2)->index()] - vertexesMesh1[mesh1->support(search_dir, vertexesMesh1)->index()];
				}
				else if (a == c) // check the origin is on the face
				{
					Vec3 ab;
					diff(b, a, ab);

					Vec3 ad;
					diff(d, a, ad);

					cross(ad, ab, search_dir);

					a = vertexesMesh2[mesh2->support(-search_dir, vertexesMesh2)->index()] - vertexesMesh1[mesh1->support(search_dir, vertexesMesh1)->index()];
				}
				else if (a == d) // check the origin is on the face
				{
					Vec3 ab;
					diff(b, a, ab);

					Vec3 ac;
					diff(c, a, ac);

					cross(ab, ac, search_dir);

					a = vertexesMesh2[mesh2->support(-search_dir, vertexesMesh2)->index()] - vertexesMesh1[mesh1->support(search_dir, vertexesMesh1)->index()];
				}
				
				output[0] = a;
				output[1] = b;
				output[2] = c;
				output[3] = d;

				return true;
			}
		}

		return false;
	}

	void minkowsky(const sp_uint vertexesMesh1Length, const Vec3* vertexesMesh1, const sp_uint vertexesMesh2Length, const Vec3* vertexesMesh2, Vec3* output)
	{
		sp_uint outputIndex = ZERO_UINT;

		for (sp_uint i = 0; i < vertexesMesh1Length; i++)
			for (sp_uint j = 0; j < vertexesMesh2Length; j++)
				output[outputIndex++] = vertexesMesh1[i] - vertexesMesh2[j];
	}

}