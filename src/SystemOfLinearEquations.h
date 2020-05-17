#ifndef SYSTEM_OF_LINEAR_EQUATIONS_HEADER
#define SYSTEM_OF_LINEAR_EQUATIONS_HEADER

#include "SpectrumPhysics.h"
#include <iomanip>

namespace NAMESPACE_PHYSICS
{
	class SystemOfLinearEquations
	{
	private:

		std::string printMatrix(sp_float* matrix, sp_uint rowSize, sp_uint colSize, sp_int precision = 4)
		{
			sp_uint total = rowSize * colSize;
			std::string content;

			for (sp_uint i = 0; i < total; i++)
			{
				std::stringstream stream;
				stream << std::fixed << std::setprecision(precision) << matrix[i];
				std::string numberAsString = stream.str();

				sp_bool isPositive = matrix[i] >= 0.0f;

				if (isPositive)
					numberAsString = " " + numberAsString;

				content += " " + numberAsString + " ";

				if ((i + 1) % colSize == 0)
					content += '\n';
			}

			return content;
		}

	public:

		/// <summary>
		/// Solve a system of linear equations nxn
		/// </summary>
		API_INTERFACE sp_float* solve(sp_float* matrix, sp_uint rowSize, sp_uint colSize);

		/// <summary>
		/// Get the line equation in 2 points for 2D space
		/// The result is a vector with 3 components (a, b, c)
		/// Line equation: "ax + bx + c = 0"
		/// </summary>
		API_INTERFACE static Vec3 getLineEquation(const Vec2& point1, const Vec2& point2);

		/// <summary>
		/// Get the line equation in 3 points for 2D space
		/// The result is a vector with 1 components (a, b, c, d)
		/// Line equation: "a(x^2 + y) + bx + cy + d = 0"
		/// </summary>
		API_INTERFACE static Vec4 getCircleEquation(const Vec2& point1, const Vec2& point2, const Vec2& point3);

	};
}

#endif // !SYSTEM_OF_LINEAR_EQUATIONS_HEADER