#include "AlgorithmInterpolation.h"

namespace NAMESPACE_PHYSICS
{
	sp_float AlgorithmInterpolation::findInterpolation(sp_float x, Vec2* points, sp_uint pointsCount)
	{
		sp_float* q = ALLOC_ARRAY(sp_float, pointsCount * pointsCount);

		std::memset(q, 0, pointsCount * pointsCount * sizeof(sp_float)); //initialize array with Zeros
		
		for (sp_uint row = 0; row < pointsCount; row++)
			q[row * pointsCount] = points[row][1];


		for (sp_uint row = 1; row < pointsCount; row++)
			for (sp_uint column = 1; column <= row; column++)
			{
				sp_float x0 = points[row - column][0];
				sp_float x1 = points[row][0];
				sp_float q00 = q[(row - 1) * pointsCount + column - 1];
				sp_float q10 = q[row * pointsCount + column - 1];

				q[row * pointsCount + column] = ((x - x0) * q10 - (x - x1) * q00) / (x1 - x0);
			}
		
		sp_float result = q[pointsCount * pointsCount - 1];

		ALLOC_RELEASE(q);

		return result;
	}

	sp_float** AlgorithmInterpolation::naturalSpline(Vec2* points, sp_uint pointsCount)
	{
		sp_float** result = ALLOC_ARRAY(sp_float*, pointsCount - 1);

		for (sp_uint i = 0; i < pointsCount - 1; i++)
			result[i] = ALLOC_ARRAY(sp_float, 4);

		sp_float* h = ALLOC_ARRAY(sp_float, pointsCount);
		h[pointsCount - 1] = 0.0f;

		for (sp_uint i = 0; i < pointsCount - 1; i++)
			h[i] = points[i + 1][0] - points[i][0];

		sp_float* alpha = ALLOC_ARRAY(sp_float, pointsCount);
		alpha[0] = ZERO_FLOAT;
		alpha[pointsCount - 1] = ZERO_FLOAT;

		sp_float* l = ALLOC_ARRAY(sp_float, pointsCount);
		sp_float* u = ALLOC_ARRAY(sp_float, pointsCount);
		sp_float* z = ALLOC_ARRAY(sp_float, pointsCount);
		sp_float* c = ALLOC_ARRAY(sp_float, pointsCount);
		sp_float* b = ALLOC_ARRAY(sp_float, pointsCount);
		sp_float* d = ALLOC_ARRAY(sp_float, pointsCount);

		b[pointsCount - 1] = ZERO_FLOAT;
		d[pointsCount - 1] = ZERO_FLOAT;

		l[0] = ONE_FLOAT;
		l[pointsCount - 1] = ONE_FLOAT;

		u[0] = ZERO_FLOAT;

		z[0] = ZERO_FLOAT;
		z[pointsCount - 1] = ZERO_FLOAT;

		c[pointsCount - 1] = ZERO_FLOAT;
		
		for (sp_uint i = 1; i < pointsCount - 1; i++)
		{
			alpha[i] = ((3.0f / h[i]) * (points[i + 1][1] - points[i][1])) - ((3.0f / h[i - 1]) * (points[i][1] - points[i - 1][1]));

			l[i] = (2 * (points[i + 1][0] - points[i-1][0])) - (h[i - 1] * u[i - 1]);
			u[i] = h[i] / l[i];
			z[i] = (alpha[i] - (h[i - 1] * z[i - 1])) / l[i];
		}

		for (sp_uint j = pointsCount - 2; j != SP_UINT_MAX; j--)
		{
			c[j] = z[j] - u[j] * c[j + 1];
			b[j] = (points[j + 1][1] - points[j][1]) / h[j] - h[j] * (c[j + 1] + 2 * c[j]) / 3.0f;
			d[j] = (c[j + 1] - c[j]) / (3.0f * h[j]);
		}

		for (sp_uint i = 0; i < pointsCount - 1; i++)
		{
			result[i][0] = points[i][1];
			result[i][1] = b[i];
			result[i][2] = c[i];
			result[i][3] = d[i];
		}

		ALLOC_RELEASE(h);
		return result;
	}

	std::string AlgorithmInterpolation::naturalSplineDescription(Vec2* points, sp_uint pointsCount)
	{
		std::ostringstream output;

		sp_float** spline = naturalSpline(points, pointsCount);

		for (sp_uint i = 0; i < pointsCount - 1; i++)
		{
			sp_float diff = points[i][0] - points[0][0];

			output << "(" << spline[i][0] << ") + (" 
				<< spline[i][1] << ")(x - " << diff << ") + (" 
				<< spline[i][2] << ")(x - " << diff << ")^2 + (" 
				<< spline[i][3] << ")(x - " << diff << ")^3 "
				<< std::endl;
		}
		
		ALLOC_RELEASE(spline);
		return output.str();
	}

	void getInterpolationPolynomialRecursive(Vec2* points, size_t pointsCount, sp_float x0, sp_float* result, sp_uint iteration)
	{
		if (pointsCount == 1)
			return;

		Vec2* newPoints = ALLOC_ARRAY(Vec2, pointsCount - 1);

		for (size_t i = 0; i < pointsCount - 1; i++)
		{
			sp_float value = iteration != 0 ? points[1][0] - x0 : points[i + 1][0] - points[i][0];

			newPoints[i][0] = points[i + 1][0];
			newPoints[i][1] = (points[i + 1][1] - points[i][1]) / value;
		}

		iteration++;

		result[iteration] = newPoints[0][1];

		getInterpolationPolynomialRecursive(newPoints, pointsCount - 1, x0, result, iteration);

		ALLOC_RELEASE(newPoints);
	}

	sp_float** AlgorithmInterpolation::fixedSpline(Vec2* points, sp_uint pointsCount, sp_float derivedFx0, sp_float derivedFxn)
	{
		sp_float** result = ALLOC_ARRAY(sp_float*, pointsCount - 1);
		sp_uint n = pointsCount - 1;

		for (sp_uint i = 0; i < n; i++)
			result[i] = ALLOC_ARRAY(sp_float, 4);

		sp_float* h = ALLOC_ARRAY(sp_float, pointsCount);
		h[n] = ZERO_FLOAT;

		for (sp_uint i = 0; i < n; i++)
			h[i] = points[i + 1][0] - points[i][0];

		sp_float* alpha = ALLOC_ARRAY(sp_float, pointsCount);
		alpha[0] = THREE_FLOAT * ((points[1][1] - points[0][1]) / h[0]) - (THREE_FLOAT * derivedFx0);
		alpha[n] = THREE_FLOAT * derivedFxn - (THREE_FLOAT * (points[n][1] - points[n - 1][1])) / h[n - 1];

		sp_float* l = ALLOC_ARRAY(sp_float, pointsCount);
		sp_float* u = ALLOC_ARRAY(sp_float, pointsCount);
		sp_float* z = ALLOC_ARRAY(sp_float, pointsCount);
		sp_float* c = ALLOC_ARRAY(sp_float, pointsCount);
		sp_float* b = ALLOC_ARRAY(sp_float, pointsCount);
		sp_float* d = ALLOC_ARRAY(sp_float, pointsCount);

		b[n] = ZERO_FLOAT;
		d[n] = ZERO_FLOAT;

		l[0] = 2.0f * h[0];
		u[0] = 0.5f;
		z[0] = alpha[0] / l[0];

		for (sp_uint i = 1; i < n; i++)
		{
			alpha[i] = ((THREE_FLOAT / h[i]) * (points[i + 1][1] - points[i][1])) - ((THREE_FLOAT / h[i - 1]) * (points[i][1] - points[i - 1][1]));

			l[i] = (2.0f * (points[i + 1][0] - points[i - 1][0])) - (h[i - 1] * u[i - 1]);
			u[i] = h[i] / l[i];
			z[i] = (alpha[i] - (h[i - 1] * z[i - 1])) / l[i];
		}

		l[n] = h[n - 1] * (2.0f - u[n - 1]);
		z[n] = (alpha[n] - h[n - 1] * z[n - 1] ) / l[n];
		c[n] = z[n];

		for (sp_uint j = n - 1; j != UINT_MAX; j--)
		{
			c[j] = z[j] - u[j] * c[j + 1];
			b[j] = (points[j + 1][1] - points[j][1]) / h[j] - h[j] * (c[j + 1] + 2 * c[j]) / 3.0f;
			d[j] = (c[j + 1] - c[j]) / (3.0f * h[j]);
		}

		for (sp_uint i = 0; i < n; i++)
		{
			result[i][0] = points[i][1];
			result[i][1] = b[i];
			result[i][2] = c[i];
			result[i][3] = d[i];
		}

		ALLOC_RELEASE(h);
		return result;
	}

	sp_float* AlgorithmInterpolation::getInterpolationPolynomial(Vec2* points, sp_uint pointsCount)
	{
		sp_float* result = ALLOC_ARRAY(sp_float, pointsCount);

		result[0] = points[0][1];

		getInterpolationPolynomialRecursive(points, pointsCount, points[0][0], result, 0);

		return result;
	}

	std::string AlgorithmInterpolation::getInterpolationPolynomialDescription(Vec2* points, sp_uint pointsCount)
	{
		sp_float* result = ALLOC_ARRAY(sp_float, pointsCount);
		result[0] = points[0][1];
		getInterpolationPolynomialRecursive(points, pointsCount, points[0][0], result, 0);

		std::stringstream output;

		output << "P" << (pointsCount -1) << "(x) = " << result[0];

		for (size_t i = 1; i < pointsCount; i++)
		{
			if (result[i] < 0.0f)
				output << " - " << (result[i] * -1.0f);
			else
				output << " + " << result[i];
			
			for (size_t j = 0; j < i; j++)
				output << "(x - " << points[j][0] << ")";
		}

		ALLOC_RELEASE(result);

		return output.str();
	}

	sp_float* AlgorithmInterpolation::getInterpolationPolynomialUsingHermite(Vec2* points, sp_uint pointsCount, sp_float* deriveds)
	{
		const sp_uint twoN1 = 2 * pointsCount + 1;

		sp_float* result = ALLOC_ARRAY(sp_float, pointsCount * 2);
		sp_float* Q = ALLOC_ARRAY(sp_float, twoN1 * twoN1);
		sp_float* z = ALLOC_ARRAY(sp_float, twoN1);

		std::memset(Q, 0, sizeof(sp_float) * twoN1 * twoN1);
		
		for (sp_uint i = 0; i < pointsCount; i++)
		{
			size_t row = 2 * i;

			z[row] = points[i][0];
			z[row + 1] = points[i][0];

			Q[row * twoN1] = points[i][1];
			Q[(row + 1) * twoN1 + 0] = points[i][1];
			Q[(row + 1) * twoN1 + 1] = deriveds[i];

			if (i != 0)
				Q[row * twoN1 + 1] = (Q[row * twoN1] - Q[(row - 1) * twoN1]) / (z[row] - z[row - 1]);
		}

		for (sp_uint row = 2; row < twoN1; row++)
			for (sp_uint column = 2; column <= row; column++)
				Q[row * twoN1 + column] = (Q[row* twoN1 + column - 1] - Q[(row -1) * twoN1 + column - 1]) / (z[row] - z[row - column]);

		for (sp_uint i = 0; i < pointsCount * 2; i++)
			result[i] = Q[i * twoN1 + i];

		ALLOC_RELEASE(Q);
		return result;
	}

	std::string AlgorithmInterpolation::getInterpolationPolynomialUsingHermiteDescription(Vec2* points, sp_uint pointsCount, sp_float* deriveds)
	{
		sp_float* polynomial = getInterpolationPolynomialUsingHermite(points, pointsCount, deriveds);
		std::ostringstream output;

		sp_uint index = 0;

		for (sp_uint i = 0; i < 2* pointsCount; i++)
		{
			output << "+ (" << polynomial[i] << ")";

			index = 0;
			for (sp_uint j = 0; j < i; j++)
			{
				if (j +1 < i) {
					output << "(x - " << points[index][0] << ")^2";
					index++;
					j++;
				}
				else
					output << "(x - " << points[index][0] << ")";
			}

			output << " ";
		}

		ALLOC_RELEASE(polynomial);
		return output.str().substr(2);
	}

	/*
	template <typename T>
	Vec4* AlgorithmInterpolation<T>::getInterpolationPolynomialUsingBezier(Vec2* points, size_t pointsCount, Vec2* leftControlPoints, Vec2* rightControlPoints)
	{
		Vec4* result = new Vec4[pointsCount];
		size_t index = 0;

		for (size_t i = 0; i < pointsCount - 1; i++)
		{
			result[index][0] = points[i][0];
			result[index][1] = 3 * (leftControlPoints[i][0] - points[i][0]);
			result[index][2] = 3 * (points[i][0] + rightControlPoints[i][0] - (2* leftControlPoints[i][0]));
			result[index][3] = points[i + 1][0] - points[i][0] + (3* leftControlPoints[i][0]) - (3 * rightControlPoints[i][0]);

			index++;

			result[index][0] = points[i][1];
			result[index][1] = 3 * (leftControlPoints[i][1] - points[i][1]);
			result[index][2] = 3 * (points[i][1] + rightControlPoints[i][1] - 2 * leftControlPoints[i][1]);
			result[index][3] = points[i + 1][1] - points[i][1] + 3 * leftControlPoints[i][1] - 3 * rightControlPoints[i][1];

			index++;
		}

		return result;
	}
	*/

}