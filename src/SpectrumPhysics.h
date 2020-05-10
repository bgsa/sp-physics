#ifndef SPECTRUM_PHYSICS_HEADER
#define SPECTRUM_PHYSICS_HEADER

#include <SpectrumFoundation.h>
#include <Object.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdarg.h>
#include <limits.h>

#ifndef NAMESPACE_PHYSICS
	#define NAMESPACE_PHYSICS SpPhyiscs
#endif

using namespace NAMESPACE_FOUNDATION;

namespace NAMESPACE_PHYSICS
{
	template <typename T>
	class Vec2;
	template <typename T>
	class Vec3;
	template <typename T>
	class Vec4;

	template <typename T>
	class Vec2List;
	template <typename T>
	class Vec3List;

	template <typename T>
	class Mat;
	template <typename T>
	class Mat2;
	template <typename T>
	class Mat3;
	template <typename T>
	class Mat4;

	class Quat;

	template <class T>
	class BoundingVolume;

	template <typename T>
	class BinaryTree;
	template <typename T>
	class BinaryTreeNode;

	template <typename T>
	class Line2D;
	class Line3D;
	class Plane3D;
	template <typename T>
	class Triangle2D;
	template <typename T>
	class Circle2D;

	class AABB;
	class OBB;
	class Sphere;
	class DOP18;

	template <typename T>
	class Rectangle2D;
}

#include "CollisionStatus.h"

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

#include "Line2D.h"
#include "Circle2D.h"

#include "Rectangle2D.h"

#endif // SPECTRUM_PHYSICS_HEADER