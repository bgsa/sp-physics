#include "Vec2List.h"

namespace NAMESPACE_PHYSICS
{
	Vec2List::Vec2List()
	{
	}

	Vec2List::Vec2List(const std::vector<Vec2> &list)
	{
		this->list = list;
	}

	void Vec2List::add(const Vec2& value) 
	{
		list.push_back(value);
	}

	size_t Vec2List::size() const
	{
		return list.size();
	}

	Vec2* Vec2List::findMinX()
	{
		if (size() == 0)
			return nullptr;

		Vec2* result = &list[0];

		for (size_t i = 0 ; i != list.size() ; i ++)
		{
			if (list[i].x < result->x)
				result = &list[i];
		}

		return result;
	}

	Vec2* Vec2List::findMinY()
	{
		if (size() == 0)
			return nullptr;

		Vec2* result = &list[0];

		for (size_t i = 0; i != list.size(); i++)
		{
			if (list[i].y < result->y)
				result = &list[i];
		}

		return result;
	}

	
	Vec2* Vec2List::findMaxX()
	{
		if (size() == 0)
			return nullptr;

		Vec2* result = &list[0];

		for (size_t i = 0; i != list.size(); i++)
		{
			if (list[i].x > result->x)
				result = &list[i];
		}

		return result;
	}

	
	Vec2* Vec2List::findMaxY()
	{
		if (size() == 0)
			return nullptr;

		Vec2* result = &list[0];

		for (size_t i = 0; i != list.size(); i++)
		{
			if (list[i].y > result->y)
				result = &list[i];
		}

		return result;
	}

	
	void Vec2List::sortByAxis(Vec2* arr, sp_size left, sp_size right, sp_int axisIndex)
	{
		sp_size i = left, j = right;
		Vec2 tmp;
		Vec2 pivot = arr[ (sp_size) (left + right) / 2 ];
		sp_size nextAxis = axisIndex == 0 ? axisIndex + 1 : axisIndex - 1;

		while (i <= j) 
		{
			while (arr[i][axisIndex] < pivot[axisIndex])
				i++;

			while (arr[j][axisIndex] > pivot[axisIndex])
				j--;

			if (i <= j)
			{
				std::swap(arr[i], arr[j]);
				i++;
				j--;
			}
		};
		
		if (left < j)
			sortByAxis(arr, left, j, axisIndex);
		if (i < right)
			sortByAxis(arr, i, right, axisIndex);
	}

	
	void Vec2List::sortByAxisXY()
	{	
		Vec2List::sortByAxis(&list[0], 0, list.size() - 1, 0);

		for (size_t i = 0; i < list.size(); i++)
		{
			for (size_t j = i + 1; j < list.size(); j++)
			{
				if (list[i][0] != list[j][0])
					break;

				if (list[i][1] > list[j][1]) {
					std::swap(list[i], list[j]);
				}
			}
		}
	}

	
	void Vec2List::sortByX()
	{
		Vec2List::sortByAxis(&list[0], 0, list.size() - 1, 0);
	}

	
	void Vec2List::sortByY()
	{
		Vec2List::sortByAxis(&list[0], 0, list.size() - 1, 1);
	}

	
	Vec2 nextToTop(std::stack<Vec2> values) 
	{
		Vec2 topValue = values.top();

		values.pop();

		Vec2 nextToTopValue = values.top();

		values.push(topValue);

		return nextToTopValue;
	}

	
	std::stack<Vec2> Vec2List::convexUpperHull() const
	{
		Vec2 point1 = list[0];
		Vec2 point2 = list[1];
		Vec2 currentPoint = list[2];
		sp_float determinant = ZERO_FLOAT;
		bool isCounterClockwise = false;

		std::stack<Vec2> upperHull;
		upperHull.push(point1);
		upperHull.push(point2);
		
		for (sp_size i = 2; i < list.size(); i++)
		{
			point1 = nextToTop(upperHull);
			point2 = upperHull.top();
			currentPoint = list[i];

			Mat3 lineMatrix = {
				ONE_FLOAT, ONE_FLOAT, ONE_FLOAT,
				currentPoint[0], point1[0], point2[0],
				currentPoint[1], point1[1], point2[1]
			};

			determinant = lineMatrix.determinant();

			isCounterClockwise = determinant >= ZERO_FLOAT;
			
			while (isCounterClockwise)
			{
				upperHull.pop();

				if (upperHull.size() == 1)
					break;

				point1 = nextToTop(upperHull);
				point2 = upperHull.top();

				Mat3 lineMatrix = {
					ONE_FLOAT, ONE_FLOAT, ONE_FLOAT,
					currentPoint.x, point1.x, point2.x,
					currentPoint.y, point1.y, point2.y
				};

				isCounterClockwise = lineMatrix.determinant() >= ZERO_FLOAT;
			}

			upperHull.push(list[i]);
		}

		return upperHull;
	}

	
	std::stack<Vec2> Vec2List::convexLowerHull() const
	{
		Vec2 point1 = list[list.size() - 1];
		Vec2 point2 = list[list.size() - 2];
		Vec2 currentPoint = list[list.size() - 3];
		sp_float determinant = 0.0f;
		sp_bool isCounterClockwise = false;

		std::stack<Vec2> lowerHull;
		lowerHull.push(point1);
		lowerHull.push(point2);

		for (sp_size i = list.size() - 3; i != std::numeric_limits<size_t>::max() ; i--)
		{
			point1 = nextToTop(lowerHull);
			point2 = lowerHull.top();
			currentPoint = list[i];

			Mat3 lineMatrix = {
				ONE_FLOAT, ONE_FLOAT, ONE_FLOAT,
				currentPoint.x, point1.x, point2.x,
				currentPoint.y, point1.y, point2.y
			};

			determinant = lineMatrix.determinant();

			isCounterClockwise = determinant >= ZERO_FLOAT;

			while (isCounterClockwise)
			{
				lowerHull.pop();

				if (lowerHull.size() == 1)
					break;

				point1 = nextToTop(lowerHull);
				point2 = lowerHull.top();

				Mat3 lineMatrix = {
					ONE_FLOAT, ONE_FLOAT, ONE_FLOAT,
					currentPoint.x, point1.x, point2.x,
					currentPoint.y, point1.y, point2.y
				};

				isCounterClockwise = lineMatrix.determinant() >= ZERO_FLOAT;
			}

			lowerHull.push(list[i]);
		}

		return lowerHull;
	}

	
	Vec2List Vec2List::convexHull() const
	{
		std::stack<Vec2> upperHull = convexUpperHull();
		std::stack<Vec2> lowerHull = convexLowerHull();

		Vec2List result;

		lowerHull.pop();

		while ( ! lowerHull.empty() ) 
		{
			result.add(lowerHull.top());
			lowerHull.pop();
		}

		upperHull.pop();

		while ( ! upperHull.empty() )
		{
			result.add(upperHull.top());
			upperHull.pop();
		}

		return result;
	}

}