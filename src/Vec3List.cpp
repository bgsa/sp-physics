#include "Vec3List.h"

namespace NAMESPACE_PHYSICS
{
	template <typename T>
	Vec3List<T>::Vec3List()
	{
		points = nullptr;
		count = 0;
	}

	template <typename T>
	Vec3List<T>::Vec3List(Vec3<T>* points, const sp_int count)
	{
		this->points = points;
		this->count = count;
	}

	template <typename T>
	sp_int* Vec3List<T>::findExtremePointsAlongDirection(const Vec3<T>& direction) const
	{
		T minProjection = std::numeric_limits<T>().max();
		T maxProjection = -minProjection;

		sp_int* result = ALLOC_ARRAY(int, 2);
			
		for (sp_uint i = 0; i < count; i++) 
		{ 
			// Project vector from origin to point onto direction vector 
			T projection = points[i].dot(direction);
			
			if (projection < minProjection)
			{ 
				minProjection = projection;
				result[0] = i;
			} 
			
			if (projection > maxProjection)
			{ 
				maxProjection = projection;
				result[1] = i;
			} 
		}
		
		return result;
	}

	template <typename T>
	sp_int* Vec3List<T>::findExtremePointsAlongAxisX() const
	{
		return findExtremePointsAlongDirection(Vec3<T>(T(1), T(0), T(0)));
	}

	template <typename T>
	sp_int* Vec3List<T>::findExtremePointsAlongAxisY() const
	{
		return findExtremePointsAlongDirection(Vec3<T>(T(0), T(1), T(0)));
	}

	template <typename T>
	sp_int* Vec3List<T>::findExtremePointsAlongAxisZ() const
	{
		return findExtremePointsAlongDirection(Vec3<T>(T(0), T(0), T(1)));
	}

	template <typename T>
	sp_int* Vec3List<T>::findExtremePointsAlongAxisXYZ() const
	{
		T maxValue = std::numeric_limits<T>().max();

		T minProjectionX = maxValue;
		T maxProjectionX = -maxValue;

		T minProjectionY = maxValue;
		T maxProjectionY = -maxValue;

		T minProjectionZ = maxValue;
		T maxProjectionZ = -maxValue;

		Vec3<T> directionX = Vec3<T>(T(1), T(0), T(0));
		Vec3<T> directionY = Vec3<T>(T(0), T(1), T(0));
		Vec3<T> directionZ = Vec3<T>(T(0), T(0), T(1));

		int* result = ALLOC_ARRAY(int, 6);

		for (int i = 0; i < count; i++)
		{
			T projectionX = points[i].dot(directionX);
			T projectionY = points[i].dot(directionY);
			T projectionZ = points[i].dot(directionZ);

			if (projectionX < minProjectionX)
			{
				minProjectionX = projectionX;
				result[0] = i;
			}
			if (projectionX > maxProjectionX)
			{
				maxProjectionX = projectionX;
				result[1] = i;
			}

			if (projectionY < minProjectionY)
			{
				minProjectionY = projectionY;
				result[2] = i;
			}
			if (projectionY > maxProjectionY)
			{
				maxProjectionY = projectionY;
				result[3] = i;
			}

			if (projectionZ < minProjectionZ)
			{
				minProjectionZ = projectionZ;
				result[4] = i;
			}
			if (projectionZ > maxProjectionZ)
			{
				maxProjectionZ = projectionZ;
				result[5] = i;
			}
		}

		return result;
	}

	template <typename T>
	T Vec3List<T>::covarianceOnAxis(const sp_int axisIndex) const
	{ 
		T u = T(0);
		
		for (sp_uint i = 0; i < count; i++) 
			u += points[i][axisIndex];
		u /= count; 

		T s2 = T(0); 
		
		for (int i = 0; i < count; i++) 
			s2 += (points[i][axisIndex] - u) * (points[i][axisIndex] - u);

		return s2 / count; 
	}

	template <typename T>
	Mat3<T> Vec3List<T>::covariance() const
	{
		T oon = T(1) / count; 
		Vec3<T> centerOfMass = Vec3<T>(T(0)); 
		T e00 = T(0), e11 = T(0), e22 = T(0), e01 = T(0), e02 = T(0), e12 = T(0);
		
		// Compute the center of mass (centroid) of the points 
		for (sp_uint i = 0; i < count; i++)
			centerOfMass += points[i];
		centerOfMass *= oon;
		
		// Compute covariance elements 
		for (sp_uint i = 0; i < count; i++) 
		{ 
			// Translate points so center of mass is at origin 
			Vec3<T> p = points[i] - centerOfMass;
			
			// Compute covariance of translated points 
			e00 += p.x * p.x;
			e11 += p.y * p.y;
			e22 += p.z * p.z;
			e01 += p.x * p.y;
			e02 += p.x * p.z;
			e12 += p.y * p.z;
		} 
		
		// Fill in the covariance matrix elements 
		return Mat3<T>(
			e00 * oon, e01 * oon, e02 * oon,
			e01 * oon, e11 * oon, e12 * oon,
			e02 * oon, e12 * oon, e22 * oon
			);
	}

	template <typename T>
	int* Vec3List<T>::closestPoint_UsingBruteForce() const
	{
		T minimunDistance = std::numeric_limits<T>().max();

		sp_int* result = ALLOC_ARRAY(sp_int, 2);

		for (sp_uint i = 0; i < count; i++)
			for (sp_uint j = i+1; j < count; j++)
			{
				T currentDistance = points[i].squaredDistance(points[j]);

				if (currentDistance < minimunDistance) 
				{
					minimunDistance = currentDistance;
					result[0] = i;
					result[1] = j;
				}
			}

		//T distance = std::sqrt(minimunDistance);
		return result;
	}

	template <typename T>
	Vec3List<T>::~Vec3List()
	{
		if (points != nullptr)
		{
			ALLOC_RELEASE(points);
			points = nullptr;
			count = 0;
		}
	}

	template class Vec3List<sp_int>;
	template class Vec3List<sp_float>;
	template class Vec3List<sp_double>;
}