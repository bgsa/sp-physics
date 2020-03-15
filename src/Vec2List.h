#ifndef VEC2_LIST_HEADER
#define VEC2_LIST_HEADER

#include "Vec2.h"
#include <stack>
#include <limits>
#include <vector>

namespace NAMESPACE_PHYSICS 
{

	template <typename T>
	class Vec2List
	{
	private:
		std::vector<Vec2<T>> list;

		std::stack<Vec2<T>> convexUpperHull() const;
		std::stack<Vec2<T>> convexLowerHull() const;

	public:

		/// <summary>
		/// Default constructor
		/// </summary>	
		API_INTERFACE Vec2List();

		/// <summary>
		/// Constructor with a initialized list
		/// </summary>	
		API_INTERFACE Vec2List(const std::vector<Vec2<T>> &list);

		/// <summary>
		/// Add the vetor to list
		/// </summary>	
		API_INTERFACE void add(const Vec2<T>& value);

		/// <summary>
		/// Get the size of the list
		/// </summary>	
		API_INTERFACE size_t size() const;

		/// <summary>
		/// Find the vector that contains the minimum X value
		/// </summary>		
		API_INTERFACE Vec2<T>* findMinX();

		/// <summary>
		/// Find the vector that contains the minimum Y value
		/// </summary>		
		API_INTERFACE Vec2<T>* findMinY();

		/// <summary>
		/// Find the vector that contains the maximum X value
		/// </summary>		
		API_INTERFACE Vec2<T>* findMaxX();

		/// <summary>
		/// Find the vector that contains the maximum Y value
		/// </summary>		
		API_INTERFACE Vec2<T>* findMaxY();

		/// <summary>
		/// Sort the list by X axis using Quicksort algoritm
		/// </summary>		
		API_INTERFACE void sortByX();

		/// <summary>
		/// Sort the list by X axis using Quicksort algoritm
		/// </summary>		
		API_INTERFACE void sortByY();

		/// <summary>
		/// Sort the list by a axis using Quicksort algoritm. Axis Index is the index of the exis. e.g: Index "0" is axis X. Index "1" is axis Y.
		/// </summary>		
		API_INTERFACE static void sortByAxis(Vec2<T>* arr, size_t left, size_t right, int axisIndex);

		/// <summary>
		/// Sort the list by a axis X and Y.
		/// </summary>		
		API_INTERFACE void sortByAxisXY();

		/// <summary>
		/// Get te convex Upper hull from point list
		/// </summary>		
		API_INTERFACE Vec2List<T> convexHull() const;

	};

	typedef Vec2List<int> Vec2iList;
	typedef Vec2List<float> Vec2fList;
	typedef Vec2List<double> Vec2dList;

}

#endif // !VEC2_LIST_HEADER