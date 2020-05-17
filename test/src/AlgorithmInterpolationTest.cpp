#include "SpectrumPhysicsTest.h"
#include <AlgorithmInterpolation.h>

#define CLASS_NAME AlgorithmInterpolationTest

namespace NAMESPACE_PHYSICS_TEST
{

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(AlgorithmInterpolation_findInterpolation_Test);

		SP_TEST_METHOD_DEF(AlgorithmInterpolation_naturalSpline_Test);

		SP_TEST_METHOD_DEF(AlgorithmInterpolation_naturalSplineDescription_Test);

		SP_TEST_METHOD_DEF(AlgorithmInterpolation_fixedSpline_Test1);

		SP_TEST_METHOD_DEF(AlgorithmInterpolation_fixedSpline_Test2);

		SP_TEST_METHOD_DEF(AlgorithmInterpolation_getInterpolationPolynomial_Test);

		SP_TEST_METHOD_DEF(AlgorithmInterpolation_getInterpolationPolynomialDescription_Test);

		SP_TEST_METHOD_DEF(AlgorithmInterpolation_getInterpolationPolynomialUsingHermite_Test1);

		SP_TEST_METHOD_DEF(AlgorithmInterpolation_getInterpolationPolynomialUsingHermite_Test2);

		SP_TEST_METHOD_DEF(AlgorithmInterpolation_getInterpolationPolynomialUsingHermiteDescription_Test);
		
	};

	SP_TEST_METHOD(CLASS_NAME, AlgorithmInterpolation_findInterpolation_Test)
	{
		AlgorithmInterpolation algorithm;
		const float x = 1.5f;
		Vec2 points[5] = {
			{1.0f, 0.7651977f},
			{1.3f, 0.6200860f},
			{1.6f, 0.4554022f},
			{1.9f, 0.2818186f},
			{2.2f, 0.1103623f}
		};

		float result = algorithm.findInterpolation(x, points, 5);

		Assert::IsTrue(isCloseEnough(result, 0.511820018f), L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmInterpolation_naturalSpline_Test)
	{
		AlgorithmInterpolation algorithm;
		const size_t pointsCount = 4;
		Vec2 points[pointsCount] = {
			{0.0f, 1.0f},
			{1.0f, EULER_NUMBER},
			{2.0f, powf(EULER_NUMBER, 2)},
			{3.0f, powf(EULER_NUMBER, 3)}
		};

		float expected[4 * (pointsCount-1)] = {
			1.0f, 1.466f, 0.0f, 0.25228f,
			2.71828f, 2.22285f, 0.75685f, 1.69107f,
			7.38906f, 8.80977f, 5.83007f, -1.94336f
		};

		float** result = algorithm.naturalSpline(points, pointsCount);

		for (size_t i = 0; i < 3; i++)
			for (size_t j = 0; j < pointsCount; j++)
				Assert::IsTrue(isCloseEnough(expected[i * 4 + j], result[i][j]), L"Wrong value.", LINE_INFO());

		ALLOC_RELEASE(result); 
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmInterpolation_naturalSplineDescription_Test)
	{
		AlgorithmInterpolation algorithm;
		const size_t pointsCount = 4;
		Vec2 points[pointsCount] = {
			{0.0f, 1.0f},
			{1.0f, EULER_NUMBER},
			{2.0f, powf(EULER_NUMBER, 2)},
			{3.0f, powf(EULER_NUMBER, 3)}
		};

		std::ostringstream expected;
		expected << "(1) + (1.466)(x - 0) + (0)(x - 0)^2 + (0.252284)(x - 0)^3 " << std::endl;
		expected << "(2.71828) + (2.22285)(x - 1) + (0.756852)(x - 1)^2 + (1.69107)(x - 1)^3 " << std::endl;
		expected << "(7.38905) + (8.80975)(x - 2) + (5.83005)(x - 2)^2 + (-1.94335)(x - 2)^3 " << std::endl;

		std::string result = algorithm.naturalSplineDescription(points, pointsCount);

		Assert::AreEqual(expected.str(), result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmInterpolation_fixedSpline_Test1)
	{
		AlgorithmInterpolation algorithm;
		const size_t pointsCount = 4;
		Vec2 points[pointsCount] = {
			{0.0f, 1.0f},
			{1.0f, EULER_NUMBER},
			{2.0f, powf(EULER_NUMBER, 2)},
			{3.0f, powf(EULER_NUMBER, 3)}
		};
		const float derivedFx0 = 1.0f;
		const float derivedFx3 = powf(EULER_NUMBER, 3);

		float expected[4 * (pointsCount - 1)] = {
			1.0f, 1.0f, 0.44468f, 0.27360f,
			2.71828f, 2.71016f, 1.26548f, 0.69513f,
			7.38906f, 7.32652f, 3.35087f, 2.01909f
		};

		float** result = algorithm.fixedSpline(points, pointsCount, derivedFx0, derivedFx3);

		for (size_t i = 0; i < 3; i++)
			for (size_t j = 0; j < pointsCount; j++)
				Assert::IsTrue(isCloseEnough(expected[i * 4 + j], result[i][j]), L"Wrong value.", LINE_INFO());

		ALLOC_RELEASE(result);
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmInterpolation_fixedSpline_Test2)
	{
		AlgorithmInterpolation algorithm;
		const size_t pointsCount = 3;
		Vec2 points[pointsCount] = {
			{1.0f, 2.0f},
			{2.0f, 3.0f},
			{3.0f, 5.0f}
		};
		const float derivedFx0 = 2.0f;
		const float derivedFx3 = 1.0f;

		float expected[4 * (pointsCount - 1)] = {
			2.0f, 2.0f, -2.5f, 3.0f / 2.0f,
			3.0f, 3.0f / 2.0f, 2.0f, -3.0f / 2.0f
		};

		float** result = algorithm.fixedSpline(points, pointsCount, derivedFx0, derivedFx3);

		for (size_t i = 0; i < pointsCount - 1; i++)
			for (size_t j = 0; j < 4; j++)
				Assert::IsTrue(isCloseEnough(expected[i * 4 + j], result[i][j]), L"Wrong value.", LINE_INFO());

		ALLOC_RELEASE(result);
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmInterpolation_getInterpolationPolynomial_Test)
	{
		AlgorithmInterpolation algorithm;
		const float x = 1.5f;
		Vec2 points[5] = {
			{1.0f, 0.7651977f},
			{1.3f, 0.6200860f},
			{1.6f, 0.4554022f},
			{1.9f, 0.2818186f},
			{2.2f, 0.1103623f}
		};

		float* result = algorithm.getInterpolationPolynomial(points, 5);

		float expected[5] = {
			0.7651977f,
			-0.4837057f,
			-0.1087339f,
			0.0658784f,
			0.0018251f
		};

		for (size_t i = 0; i < 5; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value.", LINE_INFO());			

		ALLOC_RELEASE(result);
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmInterpolation_getInterpolationPolynomialDescription_Test)
	{
		AlgorithmInterpolation algorithm;
		const float x = 1.5f;
		Vec2 points[5] = {
			{1.0f, 0.7651977f},
			{1.3f, 0.6200860f},
			{1.6f, 0.4554022f},
			{1.9f, 0.2818186f},
			{2.2f, 0.1103623f}
		};

		std::string result = algorithm.getInterpolationPolynomialDescription(points, 5);

		std::string expected = "P4(x) = 0.765198 - 0.483706(x - 1) - 0.108734(x - 1)(x - 1.3) + 0.0658779(x - 1)(x - 1.3)(x - 1.6) + 0.00182617(x - 1)(x - 1.3)(x - 1.6)(x - 1.9)";

		Assert::AreEqual(expected, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmInterpolation_getInterpolationPolynomialUsingHermite_Test1)
	{
		AlgorithmInterpolation algorithm;
		const size_t pointsCount = 3;
		Vec2 points[pointsCount] = {
			{-0.5f, -0.02475f},
			{-0.25f, 0.3349375f},
			{0.0f, 1.101f}
		};
		float deriveds[pointsCount] = { 0.751f, 2.189f, 4.002f };

		float expected[6] = {
			-0.02475f, 0.751f, 2.751f, 1.0f, 0.0f, 0.0f
		};

		float* result = algorithm.getInterpolationPolynomialUsingHermite(points, pointsCount, deriveds);
		
		for (size_t i = 0; i < 2 * pointsCount; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value.", LINE_INFO());

		ALLOC_RELEASE(result);
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmInterpolation_getInterpolationPolynomialUsingHermite_Test2)
	{
		AlgorithmInterpolation algorithm;
		const size_t pointsCount = 4;
		Vec2 points[pointsCount] = {
			{0.1f, -0.62049958f},
			{0.2f, -0.28398668f},
			{0.3f, 0.00660095f},
			{0.4f, 0.24842440f}
		};
		float deriveds[pointsCount] = { 3.58502082f, 3.14033271f, 2.66668043f, 2.16529366f };

		float expected[2 * pointsCount] = {
			-0.6205f, 3.58502f, -2.19892f, -0.490355f, 0.0366034f, 0.0446431f, -0.0197349f, 0.0673337f
		};

		/*float expected[8] = {
			-0.62049958f, 3.5850208f, -2.1989182f, -0.490447f, 
			0.037205f, 0.040475f, -0.002527777f, 0.0029629628f
		};*/

		float* result = algorithm.getInterpolationPolynomialUsingHermite(points, pointsCount, deriveds);

		for (size_t i = 0; i < 2 * pointsCount; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value.", LINE_INFO());

		ALLOC_RELEASE(result);
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmInterpolation_getInterpolationPolynomialUsingHermiteDescription_Test)
	{
		AlgorithmInterpolation algorithm;
		const size_t pointsCount = 4;
		Vec2 points[pointsCount] = {
			{0.1f, -0.62049958f},
			{0.2f, -0.28398668f},
			{0.3f, 0.00660095f},
			{0.4f, 0.24842440f}
		};
		float deriveds[pointsCount] = { 3.58502082f, 3.14033271f, 2.66668043f, 2.16529366f };

		std::string expected = "(-0.6205) + (3.58502)(x - 0.1) + (-2.19892)(x - 0.1)^2 + (-0.490355)(x - 0.1)^2(x - 0.2) + (0.0366034)(x - 0.1)^2(x - 0.2)^2 + (0.0446431)(x - 0.1)^2(x - 0.2)^2(x - 0.3) + (-0.0197349)(x - 0.1)^2(x - 0.2)^2(x - 0.3)^2 + (0.0673337)(x - 0.1)^2(x - 0.2)^2(x - 0.3)^2(x - 0.4) ";

		/*float expected[8] = {
			-0.62049958f, 3.5850208f, -2.1989182f, -0.490447f,
			0.037205f, 0.040475f, -0.002527777f, 0.0029629628f
		};*/

		std::string result = algorithm.getInterpolationPolynomialUsingHermiteDescription(points, pointsCount, deriveds);

		Assert::AreEqual(expected, result, L"Wrong value.", LINE_INFO());
	}

	/*
	SP_TEST_METHOD(CLASS_NAME, AlgorithmInterpolation_getInterpolationPolynomialUsingBezier_Test1)
	{
		AlgorithmInterpolation<float> algorithm;
		const size_t pointsCount = 2;
		Vec2 points[pointsCount] = {
			{0.0f, 0.0f},
			{5.0f, 2.0f}
		};
		Vec2 leftControlPoints[1] = { Vec2(1.0f, 1.0f) };
		Vec2 rightControlPoints[1] = { Vec2(6.0f, 1.0f) };

		Vec4 expected[2] = { 
			Vec4(-10.0f, 14.0f, 1.0f, 0.0f),
			Vec4(-2.0f, 3.0f, 1.0f, 0.0f)
		};

		Vec4* result = algorithm.getInterpolationPolynomialUsingBezier(points, pointsCount, leftControlPoints, rightControlPoints);

		for (size_t i = 0; i < 2; i++)
			for (size_t j = 0; j < 4; j++)
				Assert::IsTrue( isCloseEnough(expected[i][j], result[i][j]), L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmInterpolation_getInterpolationPolynomialUsingBezier_Test2)
	{
		AlgorithmInterpolation<float> algorithm;
		const size_t pointsCount = 2;
		Vec2 points[pointsCount] = {
			{0.0f, 0.0f},
			{5.0f, 2.0f}
		};
		Vec2 leftControlPoints[1] = { Vec2(1.0f, 1.0f) };
		Vec2 rightControlPoints[1] = { Vec2(6.0f, 1.0f) };

		Vec4 expected[2] = {
			Vec4(-10.0f, 14.0f, 1.0f, 0.0f),
			Vec4(-4.0f, 5.0f, 1.0f, 0.0f)
		};

		Vec4* result = algorithm.getInterpolationPolynomialUsingBezier(points, pointsCount, leftControlPoints, rightControlPoints);

		for (size_t i = 0; i < 2; i++)
			for (size_t j = 0; j < 4; j++)
				Assert::IsTrue(isCloseEnough(expected[i][j], result[i][j]), L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmInterpolation_getInterpolationPolynomialUsingBezier_Test3)
	{
		AlgorithmInterpolation<float> algorithm;
		const size_t pointsCount = 2;
		Vec2 points[pointsCount] = {
			{0.0f, 0.0f},
			{5.0f, 2.0f}
		};
		Vec2 leftControlPoints[1] = { Vec2(2.0f, 2.0f) };
		Vec2 rightControlPoints[1] = { Vec2(7.0f, 0.0f) };

		Vec4 expected[2] = {
			Vec4(-10.0f, 13.0f, 2.0f, 0.0f),
			Vec4(0.0f, 0.0f, 2.0f, 0.0f)
		};

		Vec4* result = algorithm.getInterpolationPolynomialUsingBezier(points, pointsCount, leftControlPoints, rightControlPoints);

		for (size_t i = 0; i < 2; i++)
			for (size_t j = 0; j < 4; j++)
				Assert::IsTrue(isCloseEnough(expected[i][j], result[i][j]), L"Wrong value.", LINE_INFO());
	}
	*/

}

#undef CLASS_NAME
