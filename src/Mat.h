#pragma once

#include "OpenML.h"
#include "Mat3.h"
#include <iomanip>

namespace OpenML
{
	template <typename T>
	class Mat
	{
	protected:

		std::string toString(T* values, int size, int precision = 4)
		{
			std::string content;
			size_t total = (int)(size * size);

			for (size_t i = 0; i < total; i++)
			{
				std::stringstream stream;
				stream << std::fixed << std::setprecision(precision) << values[i];
				std::string numberAsString = stream.str();

				bool isPositive = values[i] >= 0;

				if (isPositive)
					numberAsString = " " + numberAsString;

				content += " " + numberAsString + " ";

				if ((i + 1) % size == 0)
					content += '\n';
			}

			return content;
		}
			
	public:

		static void gaussianElimination(T *matrix, int rowSize);

		/// <summary>
		/// Get the Homography Matrix (the transformation perspective matrix from a plane to another one)
		/// <summary>
		static Mat3<T> getPerspectiveTransform2D(Vec2<T> sourcePoints[4], Vec2<T> targetPoints[4]);	

	};
	
}