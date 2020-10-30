#ifndef SPECTRUM_PHYSICS_HEADER
#define SPECTRUM_PHYSICS_HEADER

#include <SpectrumFoundation.h>
#include <Object.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <limits.h>

#ifdef SSE4_ENABLED
	#include <nmmintrin.h>
#else	
	#ifdef SSE_ENABLED
		#include <xmmintrin.h>
	#endif
#endif

#ifdef AVX_ENABLED
	#include <immintrin.h>
#endif

#ifndef NAMESPACE_PHYSICS
	#define NAMESPACE_PHYSICS SpPhyiscs
#endif

#define SP_MAX_FACE_PER_VERTEX 10

using namespace NAMESPACE_FOUNDATION;

namespace NAMESPACE_PHYSICS
{
#define ERROR_MARGIN_PHYSIC 0.02f

	enum class CollisionStatus { OUTSIDE, INLINE, INSIDE };

	class Vec2;
	class Vec3;
	class Vec4;

	class Vec2List;
	class Vec3List;

	class Mat;
	class Mat2;
	class Mat3;
	class Mat4;

	class Ray;
	class Quat;
	class SpTransform;

	class BoundingVolume;

	template <typename T>
	class BinaryTree;
	template <typename T>
	class BinaryTreeNode;

	class Line2D;
	class Line3D;
	class Plane3D;
	class Triangle2D;
	class Triangle3D;
	class Circle2D;

	class AABB;
	class OBB;
	class Sphere;
	class DOP18;

	class Rectangle2D;

	class SpCollisionDetails;
	class SpMesh;
	class SpMeshCache;
	class SpVertexMesh;
	class SpEdgeMesh;
	class SpFaceMesh;

#ifdef OPENCL_ENABLED
	class GpuDevice;
	class GpuBufferOpenCL;
	class GpuCommand;
	class GpuCommandManager;
#endif

}

#include "Vec2.h"
#include "Vec3.h"
#include "Vec4.h"

#include "Vec2List.h"
#include "Vec3List.h"

#include "Mat.h"
#include "Mat2.h"
#include "Mat3.h"
#include "Mat4.h"

#include "Quat.h"

#endif // SPECTRUM_PHYSICS_HEADER