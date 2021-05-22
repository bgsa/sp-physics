#ifndef ALGORITHM_INTERPOLATION_HEADER
#define ALGORITHM_INTERPOLATION_HEADER

#include "SpectrumPhysics.h"
#include <iostream>
#include <sstream>

namespace NAMESPACE_PHYSICS
{

	class AlgorithmInterpolation
	{
	public:

		///<summary>
		///Given a function by ordered points ("points" parameter), 
		///find the interpolation on "x" (first parameter)
		///This algorithm uses Neville Method
		///</summary>
		API_INTERFACE sp_float findInterpolation(sp_float x, Vec2* points, sp_uint pointsCount);

		///<summary>
		///Given a function by ordered points ("points" parameter), 
		///find a polynomial that interpolate these points.
		///This method is not continous in the extreme points. For this, uses Fixed Spline Cubic Method
		///This algorithm uses Natural Spline Cubic Method.
		///</summary>
		API_INTERFACE sp_float** naturalSpline(Vec2* points, sp_uint pointsCount);

		///<summary>
		///Given a function by ordered points ("points" parameter), 
		///find a polynomial that interpolate these points.
		///This method is not continous in the extreme points. For this, uses Fixed Spline Cubic Method
		///This algorithm uses Natural Spline Cubic Method.
		///The description suffix mehotds just show how to use the method
		///</summary>
		API_INTERFACE std::string naturalSplineDescription(Vec2* points, sp_uint pointsCount);

		///<summary>
		///Given a function by ordered points ("points" parameter), 
		///find a polynomial that interpolate these points.
		///This method is continous in the extreme points. Due to this factm 2 more parameters are required.
		///1- derivedFx0 = Derived Function in x0
		///2- derivedFx0 = Derived Function in xn, where "n" is the points count
		///This algorithm uses Fixed Spline Cubic Method.
		///</summary>
		API_INTERFACE sp_float** fixedSpline(Vec2* points, sp_uint pointsCount, sp_float derivedFx0, sp_float derivedFxn);

		///<summary>
		///Given a function by ordered points ("points" parameter), 
		///find the interpolation polynomial
		///This algorithm uses Newton Difference Divided Method
		///</summary>
		API_INTERFACE sp_float* getInterpolationPolynomial(Vec2* points, sp_uint pointsCount);

		///<summary>
		///Given a function by ordered points ("points" parameter), 
		///find the description (formula) interpolation polynomial
		///This algorithm uses Newton Difference Divided Method
		///</summary>
		API_INTERFACE std::string getInterpolationPolynomialDescription(Vec2* points, sp_uint pointsCount);

		///<summary>
		///Given a function by ordered points ("points" parameter) 
		///and deriveds of theses points, 
		///find the interpolation polynomial
		///This algorithm uses Hermite Interpolation Method with Divided Difference Newton
		///</summary>
		API_INTERFACE sp_float* getInterpolationPolynomialUsingHermite(Vec2* points, sp_uint pointsCount, sp_float* deriveds);

		///<summary>
		///Given a function by ordered points ("points" parameter) 
		///and deriveds of theses points, 
		///find the interpolation polynomial
		///This algorithm uses Hermite Interpolation Method with Divided Difference Newton
		///</summary>
		API_INTERFACE std::string getInterpolationPolynomialUsingHermiteDescription(Vec2* points, sp_uint pointsCount, sp_float* deriveds);
		/*
		///<summary>
		///Given a function by ordered points ("points" parameter) 
		///and control points (xLeft, yLeft)...; (xRight, yRight)..., 
		///find the interpolation polynomial
		///This algorithm uses Cubic Bezier Interpolation Method
		///Returns a list of Vec4. For each Vec, it contains the polynomial of the interval xi -> xi+1 and the next is the polynomial yi -> yi+1
		///</summary>
		API_INTERFACE Vec4* getInterpolationPolynomialUsingBezier(Vec2* points, size_t pointsCount, Vec2* leftControlPoints, Vec2* rightControlPoints);
		*/
	};

}

#endif // ALGORITHM_INTERPOLATION_HEADER