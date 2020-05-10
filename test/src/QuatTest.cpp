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

		SP_TEST_METHOD_DEF(Quat_x_Test);

		SP_TEST_METHOD_DEF(Quat_y_Test);

		SP_TEST_METHOD_DEF(Quat_z_Test);

		SP_TEST_METHOD_DEF(Quat_w_Test);

		SP_TEST_METHOD_DEF(Quat_sizeInBytes_Test);

		SP_TEST_METHOD_DEF(Quat_operator_Index_Test);

		SP_TEST_METHOD_DEF(Quat_sum_Index_Test);

		SP_TEST_METHOD_DEF(Quat_subtract_Index_Test);

		SP_TEST_METHOD_DEF(Quat_createScale1_Test);

		SP_TEST_METHOD_DEF(Quat_createScale2_Test);

		SP_TEST_METHOD_DEF(Quat_scale_Test);

		SP_TEST_METHOD_DEF(Quat_multiply_Test);

		SP_TEST_METHOD_DEF(Quat_length_Test);

		SP_TEST_METHOD_DEF(Quat_normalize_Test);

		SP_TEST_METHOD_DEF(Quat_conjugate_Test);

		SP_TEST_METHOD_DEF(Quat_dot_Test);

		SP_TEST_METHOD_DEF(Quat_inverse_Test);

		SP_TEST_METHOD_DEF(Quat_toVec3_Test);

		SP_TEST_METHOD_DEF(Quat_createRotate_Test);

		SP_TEST_METHOD_DEF(Quat_rotate_Test);

	};

	SP_TEST_METHOD(CLASS_NAME, Quat_ConstructorEmpty_Test)
	{
		Quat result;

		for (sp_int i = 0; i < QUAT_LENGTH; i++)
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
		Vec3f vector(1.0f, 2.0f, 3.0f);
		Quat result(vector);
		sp_float expected[QUAT_LENGTH] = { 0.0f, 1.0f, 2.0f, 3.0f };
		
		for (sp_int i = 0; i < QUAT_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_values_Test)
	{
		sp_float expected[4] = {2.0f, 3.0f, 4.0f, 1.0f };
		Quat quat(expected);
		sp_float* result = quat.values();

		for (sp_int i = 0; i < QUAT_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_x_Test)
	{
		Quat quat(2.0f, 3.0f, 4.0f, 1.0f);
		sp_float expected = 3.0f;

		Assert::AreEqual(expected, quat.x, L"", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_y_Test)
	{
		Quat quat(2.0f, 3.0f, 4.0f, 1.0f);
		sp_float expected = 4.0f;

		Assert::AreEqual(expected, quat.y, L"", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_z_Test)
	{
		Quat quat(2.0f, 3.0f, 4.0f, 1.0f);
		sp_float expected = 1.0f;

		Assert::AreEqual(expected, quat.z, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_w_Test)
	{
		Quat quat(2.0f, 3.0f, 4.0f, 1.0f);
		sp_float expected = 2.0f;

		Assert::AreEqual(expected, quat.w, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_sizeInBytes_Test)
	{
		Quat Quat(2.0f, 3.0f, 4.0f, 1.0f);
		sp_size expected = SIZEOF_FLOAT * QUAT_LENGTH;
		Assert::AreEqual(expected, Quat.sizeInBytes(), L"Wrong value", LINE_INFO());
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

	SP_TEST_METHOD(CLASS_NAME, Quat_createScale1_Test)
	{
		Quat quat(1.0f, 2.0f, 3.0f, 4.0f);
		Quat result = quat.createScale(2.0f);
		Quat expected(2.0f, 4.0f, 6.0f, 8.0f);

		Assert::AreEqual(expected[0], result[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[1], result[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[2], result[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[3], result[3], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_createScale2_Test)
	{
		Quat quat(1.0f, 2.0f, 3.0f, 4.0f);
		Quat result = quat * 2.0f;
		Quat expected(2.0f, 4.0f, 6.0f, 8.0f);

		Assert::AreEqual(expected[0], result[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[1], result[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[2], result[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[3], result[3], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_scale_Test)
	{
		Quat quat(1.0f, 2.0f, 3.0f, 4.0f);
		quat.scale(2.0f);
		Quat expected(2.0f, 4.0f, 6.0f, 8.0f);

		Assert::AreEqual(expected[0], quat[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[1], quat[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[2], quat[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[3], quat[3], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_multiply_Test)
	{
		Quat quat1(0.0f, 1.0f, 1.0f, 0.0f);
		Quat quat2(2.0f, 3.0f, 5.0f, 7.0f);
		Quat result = quat1 * quat2;
		Quat expected(-8.0f, 9.0f, -5.0f, 2.0f);

		Assert::AreEqual(expected[0], result[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[1], result[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[2], result[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[3], result[3], L"Wrong value", LINE_INFO());
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
		Quat result = quat.normalize();
		Quat expected(0.246182978f, 0.615457416f, 0.123091489f, 0.738548934f);

		Assert::AreEqual(expected[0], result[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[1], result[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[2], result[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[3], result[3], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_conjugate_Test)
	{
		Quat quat(2.0f, 5.0f, 1.0f, 6.0f);
		Quat result = quat.conjugate();
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

		Assert::AreEqual(expected, result, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_inverse_Test)
	{
		Quat quat(2.0f, 5.0f, 1.0f, 6.0f);
		Quat result = quat.inverse();
		Quat expected(0.0303030275f, 0.0757575706f, 0.0151515137f, 0.0909090787f);

		sp_float proof = result.dot(quat);

		Assert::AreEqual(1.0f, ceil(proof), L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[0], result[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[1], result[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[2], result[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[3], result[3], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_toVec3_Test)
	{
		Quat quat(2.0f, 5.0f, 1.0f, 6.0f);
		Vec3f result1 = quat.toVec3();
		Vec3f result2 = quat;

		Vec3f expected(5.0f, 1.0f, 6.0f);

		for (sp_int i = 0; i < VEC3_SIZE; i++) {
			Assert::AreEqual(expected[i], result1[i], L"Wrong value", LINE_INFO());
			Assert::AreEqual(expected[i], result2[i], L"Wrong value", LINE_INFO());
		}			
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_createRotate_Test)
	{
		Vec3f vector(2.0, 5.0f, 3.0f);
		sp_float angle = degreesToRadians(60);

		Quat result = Quat::createRotate(angle, vector);
		Quat expected(0.866025388f, 0.162221417f, 0.405553550f, 0.243332133f);

		for (sp_int i = 0; i < QUAT_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i], 0.000001f), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_rotate_Test)
	{
		Quat quaternion(0.0f, 10.0f, 0.0f, 0.0f);
		Vec3f vector(0.0f, 0.0f, 1.0f);
		sp_float angle = degreesToRadians(180);

		Quat result = quaternion.rotate(angle, vector);
		Quat expected(0.0f, -10.0f, 0.0f, 0.0f);

		for (sp_int i = 0; i < QUAT_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());
	}
}

#undef CLASS_NAME