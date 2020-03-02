#pragma once

#include "Vec3.h"
#include "Mat3.h"
#include <limits>

namespace OpenML
{

	template <typename T>
	class Vec3List
	{
	public:
		Vec3<T>* points;
		int count;

		///<summary>
		///Default Constructor
		///</summary>
		Vec3List();

		///<summary>
		///Constructor with points
		///</summary>
		Vec3List(Vec3<T>* points, int count);
		
		///<summary>
		///Find the extreme points along the direction
		///Return the INDEXES of point list
		///</summary>
		API_INTERFACE int* findExtremePointsAlongDirection(const Vec3<T>& direction) const;

		///<summary>
		///Find the extreme points along the axis X
		///Return the INDEXES of point list
		///</summary>
		API_INTERFACE int* findExtremePointsAlongAxisX() const;

		///<summary>
		///Find the extreme points along the axis X
		///Return the INDEXES of point list
		///</summary>
		API_INTERFACE int* findExtremePointsAlongAxisY() const;

		///<summary>
		///Find the extreme points along the axis X
		///Return the INDEXES of point list
		///</summary>
		API_INTERFACE int* findExtremePointsAlongAxisZ() const;

		///<summary>
		///Find the extreme points along the axis X, Y and Z
		///Returns the 6 INDEXES of point list (minX, maxX, minY, maxY, minZ, maxZ)
		///</summary>
		API_INTERFACE int* findExtremePointsAlongAxisXYZ() const;

		///<summary>
		///Find the closest points in the point list using brute force.
		///Complexity: (N^2)
		///Helpful for small quantity of points
		///</summary>
		///<returns>Indexes of Array</returns>
		API_INTERFACE int* closestPoint_UsingBruteForce() const;

		///<summary>
		///Compute the variance, given an axis (axis X = 0, axis Y = 1, axis Z = 2)
		///</summary>
		API_INTERFACE T Vec3List<T>::covarianceOnAxis(int axisIndex) const;

		///<summary>
		///Compute the covariance matrix
		///</summary>
		API_INTERFACE Mat3<T> covariance() const;

		///<summary>
		///Destructor
		///</summary>
		~Vec3List();

	};

	//typedef Vec3List<int> Vec3iList;
	//typedef Vec3List<float> Vec3fList;
	//typedef Vec3List<double> Vec3List;

}