#ifndef MAT_HEADER
#define MAT_HEADER

#include "SpectrumPhysics.h"
#include <iomanip>

namespace NAMESPACE_PHYSICS
{
	template <typename T>
	class Mat
	{
	protected:

		std::string toString(const T* values, const sp_int size, const sp_int precision = 4) const
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
			
	public:

		static void gaussianElimination(T *matrix, const sp_int rowSize);

		/// <summary>
		/// Get the Homography Matrix (the transformation perspective matrix from a plane to another one)
		/// <summary>
		static Mat3<T> getPerspectiveTransform2D(const Vec2<T> sourcePoints[4], const Vec2<T> targetPoints[4] );

	};
	
}

#endif // !MAT_HEADER