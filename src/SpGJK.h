#ifndef SP_GJK_HEADER
#define SP_GJK_HEADER

#include "SpectrumPhysics.h"
#include "SpMesh.h"

namespace NAMESPACE_PHYSICS
{
#define SP_GJK_MAX_ITERATIONS 64

	void update_simplex3(Vec3& a, Vec3& b, Vec3& c, Vec3& d, sp_int& simp_dim, Vec3& search_dir);

	sp_bool update_simplex4(Vec3& a, Vec3& b, Vec3& c, Vec3& d, sp_int& simp_dim, Vec3& search_dir);

	/// <summary>
	/// Check 2 meshes intersect
	/// If true, returns 4 vertexes of tetrahedron
	/// </summary>
	/// <param name="mesh1">Mesh 1</param>
	/// <param name="vertexesMesh1">Vertexes of Mesh1</param>
	/// <param name="mesh2">Mesh 2</param>
	/// <param name="vertexesMesh2">Vertexes of Mesh2</param>
	/// <param name="output">4 vertex of tetrahedron</param>
	/// <param name="maxIterations">(optional) Maximum iterations</param>
	/// <returns>True if the method converged orelse False</returns>
	API_INTERFACE sp_bool gjk(const SpMesh* mesh1, const Vec3* vertexesMesh1, const SpMesh* mesh2, const Vec3* vertexesMesh2, Vec3* output, sp_uint maxIterations = SP_GJK_MAX_ITERATIONS);

	/// <summary>
	/// Given a tetrahedron and a Minkowski Difference Simplex, find the closest edge to origin and the depth
	/// </summary>
	/// <param name="a">Tetrahedron's Vertex 1</param>
	/// <param name="b">Tetrahedron's Vertex 2</param>
	/// <param name="c">Tetrahedron's Vertex 3</param>
	/// <param name="d">Tetrahedron's Vertex 4</param>
	/// <param name="mesh1">Mesh 1</param>
	/// <param name="mesh2">Mesh 2</param>
	/// <param name="output"></param>
	/// <returns>output parameter</returns>
	API_INTERFACE void epa(const Vec3 tetrahedron[4], const SpMesh* mesh1, const Vec3* vertexesMesh1, const SpMesh* mesh2, const Vec3* vertexesMesh2, Vec3& normal, sp_float& depth, const sp_float _epsilon = SP_EPSILON_TWO_DIGITS);

#undef SP_GJK_MAX_ITERATIONS
}

#endif // SP_GJK_HEADER