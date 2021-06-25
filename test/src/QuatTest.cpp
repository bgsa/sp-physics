#include "SpectrumPhysicsTest.h"
#include "Quat.h"
#include "Asserts.h"

#define CLASS_NAME QuatTest

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
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
		SP_TEST_METHOD_DEF(multiply_2);
		SP_TEST_METHOD_DEF(multiply_3);
		SP_TEST_METHOD_DEF(Quat_length_Test);
		SP_TEST_METHOD_DEF(Quat_normalize_Test);
		SP_TEST_METHOD_DEF(conjugate);
		SP_TEST_METHOD_DEF(Quat_dot_Test);
		SP_TEST_METHOD_DEF(inverse_Test);
		SP_TEST_METHOD_DEF(vec3_Test);
		SP_TEST_METHOD_DEF(createRotate);
		SP_TEST_METHOD_DEF(rotate1);
		SP_TEST_METHOD_DEF(rotate2);
		SP_TEST_METHOD_DEF(rotate3);
		SP_TEST_METHOD_DEF(rotate4);
		SP_TEST_METHOD_DEF(axis);
		SP_TEST_METHOD_DEF(angle_Test);
		SP_TEST_METHOD_DEF(fromEulerAngles_Test);
		SP_TEST_METHOD_DEF(eulerAnglesXYZ_Test);
		SP_TEST_METHOD_DEF(eulerAnglesZYX_Test);
		SP_TEST_METHOD_DEF(lerp_Test);
		SP_TEST_METHOD_DEF(slerp_Test);
		SP_TEST_METHOD_DEF(mat3_Test);
		SP_TEST_METHOD_DEF(mat4_Test);
	};

	SP_TEST_METHOD(CLASS_NAME, Quat_ConstructorWithValues1_Test)
	{
		Quat result(2.0f, 3.0f, 4.0f, 1.0f);
		sp_float expected[4] = {2.0f, 3.0f, 4.0f, 1.0f};

		for (sp_int i = 0; i < 4; i++)
			Assert::AreEqual(expected[i], result[i], L"", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_ConstructorWithValues2_Test)
	{			
		sp_float expected[4] = {2.0f, 3.0f, 4.0f, 1.0f };
		Quat result(expected);

		for (sp_int i = 0; i < 4; i++)
			Assert::AreEqual(expected[i], result[i], L"", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_ConstructorWithVector_Test)
	{
		Vec3 vector(1.0f, 2.0f, 3.0f);
		Quat result(vector);
		sp_float expected[4] = { 1.0f, 1.0f, 2.0f, 3.0f };
		
		for (sp_int i = 0; i < 4; i++)
			Assert::AreEqual(expected[i], result[i], L"", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_values_Test)
	{
		sp_float expected[4] = {2.0f, 3.0f, 4.0f, 1.0f };
		Quat quat(expected);
		sp_float* result = (sp_float*) &quat;

		for (sp_int i = 0; i < 4; i++)
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
		Quat expected(-8.0f, 9.0f, -5.0f, 2.0f);
		Quat result;

		PerformanceCounter counter;
		
		for (sp_uint i = 0; i < 100000; i++)
			NAMESPACE_PHYSICS::multiply(quat1, quat2, result);

		counter.logElapsedTime("Quat::multiply: ");

		Assert::AreEqual(expected.w, result.w, L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected.x, result.x, L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected.y, result.y, L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected.z, result.z, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, multiply_2)
	{
		Quat quat1(1.0f, 2.0f, 3.0f, 4.0f);
		Quat quat2(-5.0f, 6.0f, -7.0f, 8.0f);
		Quat result = quat1 * quat2;
		Quat expected = Quat(-28.0f, 48.0f, -14.0f, -44.0f);

		Assert::AreEqual(expected.w, result.w, L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected.x, result.x, L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected.y, result.y, L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected.z, result.z, L"Wrong value", LINE_INFO());

		result = quat2 * quat1;
		expected = Quat(-28.0f, -56.0f, -30.0f, 20.0f);

		Assert::AreEqual(expected.w, result.w, L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected.x, result.x, L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected.y, result.y, L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected.z, result.z, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, multiply_3)
	{
		Quat quat1(1.0f, 2.0f, 3.0f, 4.0f);
		Quat quat2(9.0f, 8.0f, 7.0f, 6.0f);
		Quat result = quat1 * quat2;
		Quat expected(-52.0f, 16.0f, 54.0f, 32.0f);

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
		normalize(quat);

		Quat expected(0.246182978f, 0.615457416f, 0.123091489f, 0.738548934f);

		for (sp_int i = 0; i < 4; i++)
			Assert::IsTrue(isCloseEnough(quat[i], expected[i]), L"Wrong value", LINE_INFO());

		quat = Quat(-5.0f, 6.0f, -7.0f, 8.0f);
		normalize(quat);
		expected = Quat(-0.37905f, 0.45486f, -0.53067f, 0.60648f);

		for (sp_int i = 0; i < 4; i++)
			Assert::IsTrue(isCloseEnough(quat[i], expected[i]), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, conjugate)
	{
		Quat quat(2.0f, 5.0f, 1.0f, 6.0f);
		Quat result;
		NAMESPACE_PHYSICS::conjugate(quat, result);
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

	SP_TEST_METHOD(CLASS_NAME, inverse_Test)
	{
		Quat quat(2.0f, 5.0f, 1.0f, 6.0f);

		Quat result;
		inverse(quat, result);

		Quat expected(0.0303030275f, -0.0757575706f, -0.0151515137f, -0.0909090787f);

		for (sp_int i = 0; i < 4; i++)
			Assert::IsTrue(isCloseEnough(result[i], expected[i]), L"Wrong value", LINE_INFO());

		sp_float proof = sp_abs(result.dot(quat)); // sp_abs ???
		Assert::AreEqual(1.0f, sp_ceil(proof), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, vec3_Test)
	{
		Quat quat(2.0f, 5.0f, 1.0f, 6.0f);

		Vec3 result1;
		vec3(quat, result1);

		Vec3 expected(5.0f, 1.0f, 6.0f);

		for (sp_int i = 0; i < VEC3_LENGTH; i++)
			Assert::AreEqual(expected[i], result1[i], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, createRotate)
	{
		Vec3 axis(1.0f, 0.0f, 1.0f);
		normalize(axis);
		SP_CONSTEXPR sp_float angle = radians(60.0f);

		Quat result = Quat::createRotate(angle, axis);
		Quat expected(0.866f, 0.3535f, 0.0f, 0.3535f);

		for (sp_int i = 0; i < 4; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i], 0.009f), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, rotate1)
	{
		Vec3 point(0.7f, 0.5f, 0.0f);
		Quat axis = Quat::createRotationAxisZ(radians(30.0f));
		
		Vec3 result;
		rotate(axis, point, result);
		
		Vec3 expected(0.3562f, 0.7830f, 0.0f);

		for (sp_int i = 0; i < VEC3_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, rotate2)
	{
		Vec3 point(-0.358606011f, -6.0f, -0.3586f);
		Quat axis = Quat::createRotationAxisZ(radians(90));

		Vec3 result;
		rotate(axis, point, result);

		Vec3 expected(6.0f, -0.3586f, -0.3586f);

		// Rotating in Z axis, the Z component should not be changed!
		Asserts::isCloseEnough(point.z, result.z, SP_EPSILON_THREE_DIGITS, L"Wrong value", LINE_INFO());

		for (sp_int i = 0; i < VEC3_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, rotate3)
	{
		Vec3 point(-0.3586f, -6.0f, -0.3586f);
		Quat axis = Quat::createRotationAxisX(radians(90));

		Vec3 result;
		rotate(axis, point, result);

		Vec3 expected(-0.3586f, 0.3586f, -6.0f);

		// Rotating in X axis, the X component should not be changed!
		Asserts::isCloseEnough(point.x, result.x, SP_EPSILON_THREE_DIGITS, L"Wrong value", LINE_INFO());

		for (sp_int i = 0; i < VEC3_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, rotate4)
	{
		Vec3 point(-0.3586f, -6.0f, -0.3586f);
		Quat axis = Quat::createRotationAxisY(radians(90));

		Vec3 result;
		rotate(axis, point, result);

		Vec3 expected(-0.3586f, -6.0f, 0.3586f);

		// Rotating in Y axis, the Y component should not be changed!
		Asserts::isCloseEnough(point.y, result.y, SP_EPSILON_THREE_DIGITS, L"Wrong value", LINE_INFO());

		for (sp_int i = 0; i < VEC3_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, mat3_Test)
	{
		Quat quat = Quat(1.0f, 3.0f, 6.0f, 2.0f);

		Mat3 result;
		mat3(quat, result);

		Mat3 expected = {
			 -0.6f, 0.64f, 0.48f,
			 0.8f, 0.48f, 0.36f,
			 0.0f, 0.6f, -0.8f
		 };

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());

		quat = Quat(4.0f, 10.0f, 7.0f, 1.0f);
		mat3(quat, result);

		expected = {
			0.3975904f,  0.7951807f,  0.4578313f,
			0.8915663f, -0.2168675f, -0.3975904f,
			-0.2168675f,  0.5662650f, -0.7951807f
		};

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, mat4_Test)
	{
		Quat quat = Quat(1.0f, 3.0f, 6.0f, 2.0f);

		Mat4 result;
		mat4(quat, result);

		Mat4 expected = {
			 -0.6f, 0.64f, 0.48f, 0.0f,
			 0.8f, 0.48f, 0.36f, 0.0f,
			 0.0f, 0.6f, -0.8f, 0.0f,
			 0.0f, 0.0f, 0.0f, 1.0f
		};

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());

		quat = Quat(4.0f, 10.0f, 7.0f, 1.0f);
		mat4(quat, result);

		expected = {
			0.3975904f,  0.7951807f,  0.4578313f, 0.0f,
			0.8915663f, -0.2168675f, -0.3975904f, 0.0f,
			-0.2168675f,  0.5662650f, -0.7951807f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
		};

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, angle_Test)
	{
		Quat q1 = Quat(0.968f, 0.008f, -0.008f, 0.252f);
		Quat q2 = Quat(0.382f, 0.605f, 0.413f, 0.563f);
		
		const sp_float result = angle(q1, q2);

		Assert::IsTrue(isCloseEnough(result, 2.0639f), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, axis)
	{
		Quat quat = Quat(4.0f, 10.0f, 7.0f, 1.0f);
		normalize(quat);

		Vec3 result = quat.axis();
		Vec3 expected = Vec3(0.816496611f, 0.571547627f, 0.0816496611f);

		for (sp_int i = 0; i < VEC3_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, fromEulerAngles_Test)
	{
		Vec3 angles = Vec3(0.4f, 1.7f, 3.0f);

		Quat result;
		fromEulerAngles(angles.x, angles.y, angles.z, result);

		Quat expected(-0.1031f, 0.7437f, -0.0787f, 0.6557f);

		for (sp_int i = 0; i < 4; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, eulerAnglesXYZ_Test)
	{
		Quat quat(4.0f, 10.0f, 7.0f, 1.0f);
		normalize(quat);

		Vec3 result;
		eulerAnglesXYZ(quat, result);

		Vec3 expected(2.6779f, 0.4756f, -1.1071f);

		for (sp_int i = 0; i < VEC3_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, eulerAnglesZYX_Test)
	{
		Quat quat(4.0f, 10.0f, 7.0f, 1.0f);
		normalize(quat);

		Vec3 result;
		eulerAnglesZYX(quat, result);

		Vec3 expected(1.1513f, 0.2186f, 2.5228f);

		for (sp_int i = 0; i < VEC3_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, lerp_Test)
	{
		Quat quatA = Quat(0.0f, 10.0f, 20.0f, 30.0f);
		normalize(quatA);

		Quat quatB = Quat(10.0f, 20.0f, 30.0f, 40.0f);
		normalize(quatB);

		Quat result;
		lerp(quatA, quatB, 0.5f, result);

		Quat expected(0.0912f, 0.3162f, 0.5411f, 0.7660f);

		for (sp_int i = 0; i < 4; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, slerp_Test)
	{
		Quat quatA = Quat(0.1f, 0.1f, 0.4f, 0.3f);
		normalize(quatA);
		Quat quatB = Quat(0.1f, 0.4f, 0.3f, 0.2f);
		normalize(quatB);

		Quat result;
		slerp(quatA, quatB, 0.3f, result);

		Quat expected(0.1977f, 0.3723f, 0.7324f, 0.5347f);

		for (sp_int i = 0; i < 4; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());
	}

}

#undef CLASS_NAME