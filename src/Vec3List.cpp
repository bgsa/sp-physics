#include "Vec3List.h"

namespace NAMESPACE_PHYSICS
{
	
	Vec3List::Vec3List()
	{
		points = NULL;
		count = ZERO_UINT;
	}

	
	Vec3List::Vec3List(Vec3* points, const sp_int count)
	{
		this->points = points;
		this->count = count;
	}

	
	sp_int* Vec3List::findExtremePointsAlongDirection(const Vec3& direction) const
	{
		sp_float minProjection = SP_FLOAT_MAX;
		sp_float maxProjection = -minProjection;

		sp_int* result = ALLOC_ARRAY(sp_int, 2);
			
		for (sp_uint i = ZERO_UINT; i < count; i++)
		{ 
			// Project vector from origin to point onto direction vector 
			sp_float projection = points[i].dot(direction);
			
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

	
	sp_int* Vec3List::findExtremePointsAlongAxisX() const
	{
		return findExtremePointsAlongDirection(Vec3(ONE_FLOAT, ZERO_FLOAT, ZERO_FLOAT));
	}

	
	sp_int* Vec3List::findExtremePointsAlongAxisY() const
	{
		return findExtremePointsAlongDirection(Vec3(ZERO_FLOAT, ONE_FLOAT, ZERO_FLOAT));
	}

	
	sp_int* Vec3List::findExtremePointsAlongAxisZ() const
	{
		return findExtremePointsAlongDirection(Vec3(ZERO_FLOAT, ZERO_FLOAT, ONE_FLOAT));
	}

	
	sp_int* Vec3List::findExtremePointsAlongAxisXYZ() const
	{
		sp_float minProjectionX = SP_FLOAT_MAX;
		sp_float maxProjectionX = -SP_FLOAT_MAX;

		sp_float minProjectionY = SP_FLOAT_MAX;
		sp_float maxProjectionY = -SP_FLOAT_MAX;

		sp_float minProjectionZ = SP_FLOAT_MAX;
		sp_float maxProjectionZ = -SP_FLOAT_MAX;

		Vec3 directionX = Vec3(ONE_FLOAT, ZERO_FLOAT, ZERO_FLOAT);
		Vec3 directionY = Vec3(ZERO_FLOAT, ONE_FLOAT, ZERO_FLOAT);
		Vec3 directionZ = Vec3(ZERO_FLOAT, ZERO_FLOAT, ONE_FLOAT);

		sp_int* result = ALLOC_ARRAY(sp_int, 6);

		for (sp_uint i = ZERO_UINT; i < count; ++i)
		{
			sp_float projectionX = points[i].dot(directionX);
			sp_float projectionY = points[i].dot(directionY);
			sp_float projectionZ = points[i].dot(directionZ);

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

	
	sp_float Vec3List::covarianceOnAxis(const sp_int axisIndex) const
	{ 
		sp_float u = 0.0f;
		
		for (sp_uint i = ZERO_UINT; i < count; ++i)
			u += points[i][axisIndex];
		u /= count; 

		sp_float s2 = 0.0f;
		
		for (sp_uint i = ZERO_UINT; i < count; ++i)
			s2 += (points[i][axisIndex] - u) * (points[i][axisIndex] - u);

		return s2 / count; 
	}

	
	Mat3 Vec3List::covariance() const
	{
		sp_float oon = 1.0f / count; 
		Vec3 centerOfMass = Vec3Zeros; 
		sp_float e00 = 0.0f, e11 = 0.0f, e22 = 0.0f, e01 = 0.0f, e02 = 0.0f, e12 = 0.0f;
		
		// Compute the center of mass (centroid) of the points 
		for (sp_uint i = ZERO_UINT; i < count; ++i)
			centerOfMass += points[i];
		centerOfMass *= oon;
		
		// Compute covariance elements 
		for (sp_uint i = ZERO_UINT; i < count; ++i)
		{ 
			// Translate points so center of mass is at origin 
			Vec3 p = points[i] - centerOfMass;
			
			// Compute covariance of translated points 
			e00 += p.x * p.x;
			e11 += p.y * p.y;
			e22 += p.z * p.z;
			e01 += p.x * p.y;
			e02 += p.x * p.z;
			e12 += p.y * p.z;
		} 
		
		// Fill in the covariance matrix elements 
		return Mat3(
			e00 * oon, e01 * oon, e02 * oon,
			e01 * oon, e11 * oon, e12 * oon,
			e02 * oon, e12 * oon, e22 * oon
			);
	}

	
	sp_int* Vec3List::closestPoint_UsingBruteForce() const
	{
		sp_float minimunDistance = SP_FLOAT_MAX;

		sp_int* result = ALLOC_ARRAY(sp_int, 2);

		for (sp_uint i = ZERO_UINT; i < count; ++i)
			for (sp_uint j = i + ONE_UINT; j < count; j++)
			{
				sp_float currentDistance = points[i].squaredDistance(points[j]);

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

	
	Vec3List::~Vec3List()
	{
		if (points != nullptr)
		{
			ALLOC_RELEASE(points);
			points = NULL;
			count = ZERO_UINT;
		}
	}

}