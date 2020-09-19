#include "SpectrumPhysicsTest.h"
#include "Quat.h"

#define CLASS_NAME QuatTest

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		SP_TEST_METHOD_DEF(Quat_ConstructorEmpty_Test);
		SP_TEST_METHOD_DEF(Quat_ConstructorWithValues1_Test);
		SP_TEST_METHOD_DEF(Quat_ConstructorWithValues2_Test);
		SP_TEST_METHOD_DEF(Quat_ConstructorWithVector_Test);
		SP_TEST_METHOD_DEF(Quat_values_Test);
		SP_TEST_METHOD_DEF(x);
		SP_TEST_METHOD_DEF(y);
		SP_TEST_METHOD_DEF(z);
		SP_TEST_METHOD_DEF(w);
		SP_TEST_METHOD_DEF(Quat_operator_Index_Test);
		SP_TEST_METHOD_DEF(Quat_sum_Index_Test);
		SP_TEST_METHOD_DEF(Quat_subtract_Index_Test);
		SP_TEST_METHOD_DEF(Quat_scale_Test);
		SP_TEST_METHOD_DEF(multiply_1);
		SP_TEST_METHOD_DEF(Quat_multiply_Test2);
		SP_TEST_METHOD_DEF(Quat_length_Test);
		SP_TEST_METHOD_DEF(Quat_normalize_Test);
		SP_TEST_METHOD_DEF(conjugate);
		SP_TEST_METHOD_DEF(Quat_dot_Test);
		SP_TEST_METHOD_DEF(Quat_inverse_Test);
		SP_TEST_METHOD_DEF(Quat_toVec3_Test);
		SP_TEST_METHOD_DEF(createRotate);
		SP_TEST_METHOD_DEF(rotate);
		SP_TEST_METHOD_DEF(toMat3);
		SP_TEST_METHOD_DEF(toEulerAngles);
		SP_TEST_METHOD_DEF(axis);
		SP_TEST_METHOD_DEF(angle);
		SP_TEST_METHOD_DEF(fromEulerAngles);
		SP_TEST_METHOD_DEF(lerp);
		SP_TEST_METHOD_DEF(slerp);
	};

	SP_TEST_METHOD(CLASS_NAME, Quat_ConstructorEmpty_Test)
	{
		Quat result;

		Assert::AreEqual(ONE_FLOAT, result[0], L"", LINE_INFO());

		for (sp_int i = 1; i < QUAT_LENGTH; i++)
			Assert::AreEqual(ZERO_FLOAT, result[i], L"", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_ConstructorWithValues1_Test)
	{
		Quat result(2.0f, 3.0f, 4.0f, 1.0f);
		sp_float expected[4] = {2.0f, 3.0f, 4.0f, 1.0f};

		for (sp_int i = 0; i < QUAT_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_ConstructorWithValues2_Test)
	{			
		sp_float expected[4] = {2.0f, 3.0f, 4.0f, 1.0f };
		Quat result(expected);

		for (sp_int i = 0; i < QUAT_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_ConstructorWithVector_Test)
	{
		Vec3 vector(1.0f, 2.0f, 3.0f);
		Quat result(vector);
		sp_float expected[QUAT_LENGTH] = { 1.0f, 1.0f, 2.0f, 3.0f };
		
		for (sp_int i = 0; i < QUAT_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_values_Test)
	{
		sp_float expected[4] = {2.0f, 3.0f, 4.0f, 1.0f };
		Quat quat(expected);
		sp_float* result = (sp_float*) &quat;

		for (sp_int i = 0; i < QUAT_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, x)
	{
		Quat quat(2.0f, 3.0f, 4.0f, 1.0f);
		sp_float expected = 3.0f;

		Assert::AreEqual(expected, quat.x, L"", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, y)
	{
		Quat quat(2.0f, 3.0f, 4.0f, 1.0f);
		sp_float expected = 4.0f;

		Assert::AreEqual(expected, quat.y, L"", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, z)
	{
		Quat quat(2.0f, 3.0f, 4.0f, 1.0f);
		sp_float expected = 1.0f;

		Assert::AreEqual(expected, quat.z, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, w)
	{
		Quat quat(2.0f, 3.0f, 4.0f, 1.0f);
		sp_float expected = 2.0f;

		Assert::AreEqual(expected, quat.w, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_operator_Index_Test)
	{
		Quat quat(2.0f, 3.0f, 4.0f, 1.0f);

		Assert::AreEqual(2.0f, quat[0], L"Value shoud be 0", LINE_INFO());
		Assert::AreEqual(3.0f, quat[1], L"Value shoud be 0", LINE_INFO());
		Assert::AreEqual(4.0f, quat[2], L"Value shoud be 0", LINE_INFO());
		Assert::AreEqual(1.0f, quat[3], L"Value shoud be 0", LINE_INFO());

	}

	SP_TEST_METHOD(CLASS_NAME, Quat_sum_Index_Test)
	{
		Quat quatA(2.0f, 3.0f, 4.0f, 1.0f);
		Quat quatB(2.0f, 3.0f, 4.0f, 1.0f);
		Quat result = quatA + quatB;
		Quat expected(4.0f, 6.0f, 8.0f, 2.0f);

		Assert::AreEqual(expected.w, result.w, L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected.x, result.x, L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected.y, result.y, L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected.z, result.z, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_subtract_Index_Test)
	{
		Quat quatA(1.0f, 2.0f, 3.0f, 4.0f);
		Quat quatB(0.5f, 1.0f, 1.0f, 0.5f);
		Quat result = quatA - quatB;
		Quat expected(0.5f, 1.0f, 2.0f, 3.5f);

		Assert::AreEqual(expected[0], result[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[1], result[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[2], result[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[3], result[3], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_scale_Test)
	{
		Quat quat = Quat(1.0f, 2.0f, 3.0f, 4.0f).scale(2.0f);
		Quat expected(2.0f, 4.0f, 6.0f, 8.0f);

		Assert::AreEqual(expected[0], quat[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[1], quat[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[2], quat[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[3], quat[3], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, multiply_1)
	{
		Quat quat1(0.0f, 1.0f, 1.0f, 0.0f);
		Quat quat2(2.0f, 3.0f, 5.0f, 7.0f);
		Quat expected(-8.0f, -5.0f, 9.0f, -2.0f);
		Quat result;

		PerformanceCounter counter;
		counter.start();

		for (sp_uint i = 0; i < 100000; i++)
			NAMESPACE_PHYSICS::multiply(quat1, quat2, &result);

		sp_longlong elapsedTime = counter.diff();

		Assert::AreEqual(expected.w, result.w, L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected.x, result.x, L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected.y, result.y, L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected.z, result.z, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_multiply_Test2)
	{
		Quat quat1(1.0f, 2.0f, 3.0f, 4.0f);
		Quat quat2(-5.0f, 6.0f, -7.0f, 8.0f);
		Quat result = quat1 * quat2;
		Quat expected(-28.0f, -56.0f, -30.0f, 20.0f);

		Assert::AreEqual(expected.w, result.w, L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected.x, result.x, L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected.y, result.y, L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected.z, result.z, L"Wrong value", LINE_INFO());

		result = quat2 * quat1;
		expected = Quat(-28.0f, 48.0f, -14.0f, -44.0f);

		Assert::AreEqual(expected.w, result.w, L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected.x, result.x, L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected.y, result.y, L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected.z, result.z, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_length_Test)
	{
		Quat quat(2.0f, 5.0f, 1.0f, 6.0f);
		sp_float result = quat.length();
		sp_float expected = 8.12403840f;

		Assert::AreEqual(expected, result, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_normalize_Test)
	{
		Quat quat(2.0f, 5.0f, 1.0f, 6.0f);
		normalize(&quat);

		Quat expected(0.246182978f, 0.615457416f, 0.123091489f, 0.738548934f);

		for (sp_int i = 0; i < QUAT_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(quat[i], expected[i]), L"Wrong value", LINE_INFO());

		quat = Quat(-5.0f, 6.0f, -7.0f, 8.0f);
		normalize(&quat);
		expected = Quat(-0.37905f, 0.45486f, -0.53067f, 0.60648f);

		for (sp_int i = 0; i < QUAT_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(quat[i], expected[i]), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, conjugate)
	{
		Quat quat(2.0f, 5.0f, 1.0f, 6.0f);
		Quat result;
		NAMESPACE_PHYSICS::conjugate(quat, &result);
		Quat expected(2.0f, -5.0f, -1.0f, -6.0f);

		Assert::AreEqual(expected[0], result[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[1], result[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[2], result[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[3], result[3], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_dot_Test)
	{
		Quat quatA(2.0f, 5.0f, 1.0f, 6.0f);
		Quat quatB(0.030303f, 0.0757576f, 0.0151515f, 0.0909091f);
		sp_float result = quatA.dot(quatB);
		sp_float expected = 1.0f;

		Assert::IsTrue(isCloseEnough(result, expected), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_inverse_Test)
	{
		Quat quat(2.0f, 5.0f, 1.0f, 6.0f);
		Quat result = quat.inverse();
		Quat expected(0.0303030275f, -0.0757575706f, -0.0151515137f, -0.0909090787f);

		for (sp_int i = 0; i < QUAT_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(result[i], expected[i]), L"Wrong value", LINE_INFO());

		sp_float proof = std::fabsf(result.dot(quat)); // fabsf ???
		Assert::AreEqual(1.0f, std::ceilf(proof), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_toVec3_Test)
	{
		Quat quat(2.0f, 5.0f, 1.0f, 6.0f);
		Vec3 result1 = quat.toVec3();
		Vec3 result2 = quat;

		Vec3 expected(5.0f, 1.0f, 6.0f);

		for (sp_int i = 0; i < VEC3_LENGTH; i++) {
			Assert::AreEqual(expected[i], result1[i], L"Wrong value", LINE_INFO());
			Assert::AreEqual(expected[i], result2[i], L"Wrong value", LINE_INFO());
		}			
	}

	SP_TEST_METHOD(CLASS_NAME, createRotate)
	{
		Vec3 axis(1.0f, 0.0f, 1.0f);
		normalize(&axis);
		SP_CONSTEXPR sp_float angle = degreesToRadians(60.0f);

		Quat result = Quat::createRotate(angle, axis);
		Quat expected(0.866f, 0.3535f, 0.0f, 0.3535f);

		for (sp_int i = 0; i < QUAT_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i], 0.009f), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, rotate)
	{
		Vec3 point(0.7f, 0.5f, 0.0f);
		Quat axis = Quat::createRotationAxisZ(degreesToRadians(30.0f));
		
		Vec3 result = axis.rotate(point);
		Vec3 expected(0.3562f, 0.7830f, 0.0f);

		for (sp_int i = 0; i < VEC3_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, toMat3)
	{
		Quat quat = Quat(1.0f, 3.0f, 6.0f, 2.0f);

		Mat3 result = quat.toMat3();
		Mat3 expected = {
			 -0.6f, 0.64f, 0.48f,
			 0.8f, 0.48f, 0.36f,
			 0.0f, 0.6f, -0.8f
		 };

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());

		quat = Quat(4.0f, 10.0f, 7.0f, 1.0f);

		result = quat.toMat3();
		expected = {
			0.3975904f,  0.7951807f,  0.4578313f,
			0.8915663f, -0.2168675f, -0.3975904f,
			-0.2168675f,  0.5662650f, -0.7951807f
		};

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, toEulerAngles)
	{
		Quat quat = Quat(4.0f, 10.0f, 7.0f, 1.0f);
		normalize(&quat);

		Vec3 result = quat.toEulerAngles();
		Vec3 expected = Vec3(2.677945f, 0.4755543f, -1.1071487f);

		for (sp_int i = 0; i < VEC3_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, angle)
	{
		Quat quat = Quat(4.0f, 10.0f, 7.0f, 1.0f);
		normalize(&quat);

		sp_float result = quat.angle();

		Assert::IsTrue(isCloseEnough(result, 2.51023841f), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, axis)
	{
		Quat quat = Quat(4.0f, 10.0f, 7.0f, 1.0f);
		normalize(&quat);

		Vec3 result = quat.axis();
		Vec3 expected = Vec3(0.816496611f, 0.571547627f, 0.0816496611f);

		for (sp_int i = 0; i < VEC3_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, fromEulerAngles)
	{
		Vec3 angles = Vec3(0.4f, 1.7f, 3.0f);

		Quat result = Quat::fromEulerAngles(angles.x, angles.y, angles.z);
		Quat expected = Quat(-0.1031277f, 0.7437353f, -0.0787058f, 0.6557651f);

		for (sp_int i = 0; i < QUAT_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, lerp)
	{
		Quat quatA = Quat(0.0f, 10.0f, 20.0f, 30.0f);
		Quat quatB = Quat(10.0f, 20.0f, 30.0f, 40.0f);

		Quat result = quatA.lerp(quatB, 0.5f);
		Quat expected = Quat(5.0f, 15.0f, 25.0f, 35.0f);

		for (sp_int i = 0; i < QUAT_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, slerp)
	{
		Quat quatA = Quat(0.1f, 0.1f, 0.4f, 0.3f);
		Quat quatB = Quat(0.1f, 0.4f, 0.3f, 0.2f);

		Quat result = quatA.slerp(quatB, 0.3f);
		Quat expected = Quat(0.1229720f, 0.2434743f, 0.4517205f, 0.3287485f);

		for (sp_int i = 0; i < QUAT_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());
	}
}

#undef CLASS_NAME