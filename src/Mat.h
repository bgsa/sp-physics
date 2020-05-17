#ifndef MAT_HEADER
#define MAT_HEADER

#include "SpectrumPhysics.h"
#include <iomanip>

namespace NAMESPACE_PHYSICS
{
	class Mat
	{		
	public:

		static std::string toString(const sp_float* values, const sp_int size, const sp_int precision = 4)
		{
			std::string content;
			sp_size total = (sp_size)(size * size);

			for (sp_size i = 0; i < total; i++)
			{
				std::stringstream stream;
				stream << std::fixed << std::setprecision(precision) << values[i];
				std::string numberAsString = stream.str();

				sp_bool isPositive = values[i] >= 0;

				if (isPositive)
					numberAsString = " " + numberAsString;

				content += " " + numberAsString + " ";

				if ((i + 1) % size == 0)
					content += '\n';
			}

			return content;
		}

		static void gaussianElimination(sp_float *matrix, const sp_int rowSize);

		/// <summary>
		/// Get the Homography Matrix (the transformation perspective matrix from a plane to another one)
		/// <summary>
		static Mat3 getPerspectiveTransform2D(const Vec2 sourcePoints[4], const Vec2 targetPoints[4] );

	};
	
}

#endif // !MAT_HEADER