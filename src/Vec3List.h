#ifndef VEC3_LIST_HEADER
#define VEC3_LIST_HEADER

#include "SpectrumPhysics.h"
#include <limits>

namespace NAMESPACE_PHYSICS
{

	class Vec3List
	{
	public:
		Vec3* points;
		sp_uint count;

		///<summary>
		///Default Constructor
		///</summary>
		API_INTERFACE Vec3List();

		///<summary>
		///Constructor with points
		///</summary>
		API_INTERFACE Vec3List(Vec3* points, const sp_int count);
		
		///<summary>
		///Find the extreme points along the direction
		///Return the INDEXES of point list
		///</summary>
		API_INTERFACE sp_int* findExtremePointsAlongDirection(const Vec3& direction) const;

		///<summary>
		///Find the extreme points along the axis X
		///Return the INDEXES of point list
		///</summary>
		API_INTERFACE sp_int* findExtremePointsAlongAxisX() const;

		///<summary>
		///Find the extreme points along the axis X
		///Return the INDEXES of point list
		///</summary>
		API_INTERFACE sp_int* findExtremePointsAlongAxisY() const;

		///<summary>
		///Find the extreme points along the axis X
		///Return the INDEXES of point list
		///</summary>
		API_INTERFACE sp_int* findExtremePointsAlongAxisZ() const;

		///<summary>
		///Find the extreme points along the axis X, Y and Z
		///Returns the 6 INDEXES of point list (minX, maxX, minY, maxY, minZ, maxZ)
		///</summary>
		API_INTERFACE sp_int* findExtremePointsAlongAxisXYZ() const;

		///<summary>
		///Find the closest points in the point list using brute force.
		///Complexity: (N^2)
		///Helpful for small quantity of points
		///</summary>
		///<returns>Indexes of Array</returns>
		API_INTERFACE sp_int* closestPoint_UsingBruteForce() const;

		///<summary>
		///Compute the variance, given an axis (axis X = 0, axis Y = 1, axis Z = 2)
		///</summary>
		API_INTERFACE sp_float covarianceOnAxis(const sp_int axisIndex) const;

		///<summary>
		///Compute the covariance matrix
		///</summary>
		API_INTERFACE Mat3 covariance() const;

		///<summary>
		///Destructor
		///</summary>
		API_INTERFACE ~Vec3List();

	};

	//typedef Vec3List<sp_int> Vec3iList;
	//typedef Vec3List<sp_float> Vec3List;
	//typedef Vec3List<sp_double> Vec3List;

}

#endif // !VEC3_LIST_HEADER