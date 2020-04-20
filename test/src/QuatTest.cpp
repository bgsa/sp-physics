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

		SP_TEST_METHOD_DEF(Quat_getValues_Test);

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
		Quatf result;

		for (int i = 0; i < QUAT_SIZE; i++)
			Assert::AreEqual(0.0f, result[i], L"", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_ConstructorWithValues1_Test)
	{
		Quatf result(1.0f, 2.0f, 3.0f, 4.0f);
		float expected[4] = {1.0f, 2.0f, 3.0f, 4.0f};

		for (int i = 0; i < QUAT_SIZE; i++)
			Assert::AreEqual(expected[i], result[i], L"", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_ConstructorWithValues2_Test)
	{			
		float expected[4] = {1.0f, 2.0f, 3.0f, 4.0f};
		Quatf result(expected);

		for (int i = 0; i < QUAT_SIZE; i++)
			Assert::AreEqual(expected[i], result[i], L"", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_ConstructorWithVector_Test)
	{
		Vec3f vector(1.0f, 2.0f, 3.0f);
		Quatf result(vector);
		float expected[QUAT_SIZE] = {1.0f, 2.0f, 3.0f, 0.0f};
		
		for (int i = 0; i < QUAT_SIZE; i++)
			Assert::AreEqual(expected[i], result[i], L"", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_getValues_Test)
	{
		float expected[4] = {1.0f, 2.0f, 3.0f, 4.0f};
		Quatf quat(expected);
		float* result = quat.getValues();

		for (int i = 0; i < QUAT_SIZE; i++)
			Assert::AreEqual(expected[i], result[i], L"", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_x_Test)
	{
		Quatf quat(1.0f, 2.0f, 3.0f, 4.0f);
		float expected = 1.0f;

		Assert::AreEqual(expected, quat.x(), L"", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_y_Test)
	{
		Quatf quat(1.0f, 2.0f, 3.0f, 4.0f);
		float expected = 2.0f;

		Assert::AreEqual(expected, quat.y(), L"", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_z_Test)
	{
		Quatf quat(1.0f, 2.0f, 3.0f, 4.0f);
		float expected = 3.0f;

		Assert::AreEqual(expected, quat.z(), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_w_Test)
	{
		Quatf quat(1.0f, 2.0f, 3.0f, 4.0f);
		float expected = 4.0f;

		Assert::AreEqual(expected, quat.w(), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_sizeInBytes_Test)
	{
		Quatf quatf(1.0f, 2.0f, 3.0f, 4.0f);
		size_t expected = sizeof(float) * QUAT_SIZE;
		Assert::AreEqual(expected, quatf.sizeInBytes(), L"Wrong value", LINE_INFO());

		Quatd quatd(1.0, 2.0, 3.0, 4.0);
		expected = sizeof(double) * QUAT_SIZE;
		Assert::AreEqual(expected, quatd.sizeInBytes(), L"Wrong value", LINE_INFO());

		Quati quati(1, 2, 3, 4);
		expected = sizeof(int) * QUAT_SIZE;
		Assert::AreEqual(expected, quati.sizeInBytes(), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_operator_Index_Test)
	{
		Quatf quat(1.0f, 2.0f, 3.0f, 4.0f);

		Assert::AreEqual(1.0f, quat[0], L"Value shoud be 0", LINE_INFO());
		Assert::AreEqual(2.0f, quat[1], L"Value shoud be 0", LINE_INFO());
		Assert::AreEqual(3.0f, quat[2], L"Value shoud be 0", LINE_INFO());
		Assert::AreEqual(4.0f, quat[3], L"Value shoud be 0", LINE_INFO());

	}

	SP_TEST_METHOD(CLASS_NAME, Quat_sum_Index_Test)
	{
		Quatf quatA(1.0f, 2.0f, 3.0f, 4.0f);
		Quatf quatB(1.0f, 2.0f, 3.0f, 4.0f);
		Quatf result = quatA + quatB;
		Quatf expected(2.0f, 4.0f, 6.0f, 8.0f);

		Assert::AreEqual(expected[0], result[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[1], result[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[2], result[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[3], result[3], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_subtract_Index_Test)
	{
		Quatf quatA(1.0f, 2.0f, 3.0f, 4.0f);
		Quatf quatB(0.5f, 1.0f, 1.0f, 0.5f);
		Quatf result = quatA - quatB;
		Quatf expected(0.5f, 1.0f, 2.0f, 3.5f);

		Assert::AreEqual(expected[0], result[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[1], result[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[2], result[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[3], result[3], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_createScale1_Test)
	{
		Quatf quat(1.0f, 2.0f, 3.0f, 4.0f);
		Quatf result = quat.createScale(2.0f);
		Quatf expected(2.0f, 4.0f, 6.0f, 8.0f);

		Assert::AreEqual(expected[0], result[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[1], result[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[2], result[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[3], result[3], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_createScale2_Test)
	{
		Quatf quat(1.0f, 2.0f, 3.0f, 4.0f);
		Quatf result = quat * 2.0f;
		Quatf expected(2.0f, 4.0f, 6.0f, 8.0f);

		Assert::AreEqual(expected[0], result[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[1], result[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[2], result[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[3], result[3], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_scale_Test)
	{
		Quatf quat(1.0f, 2.0f, 3.0f, 4.0f);
		quat.scale(2.0f);
		Quatf expected(2.0f, 4.0f, 6.0f, 8.0f);

		Assert::AreEqual(expected[0], quat[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[1], quat[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[2], quat[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[3], quat[3], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_multiply_Test)
	{
		Quatf quat1(1.0f, 1.0f, 0.0f, 0.0f);
		Quatf quat2(3.0f, 5.0f, 7.0f, 2.0f);
		Quatf result = quat1 * quat2;
		Quatf expected(9.0f, -5.0f, 2.0f, -8.0f);

		Assert::AreEqual(expected[0], result[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[1], result[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[2], result[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[3], result[3], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_length_Test)
	{
		Quatf quat(2.0f, 5.0f, 1.0f, 6.0f);
		float result = quat.length();
		float expected = 8.12403840f;

		Assert::AreEqual(expected, result, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_normalize_Test)
	{
		Quatf quat(2.0f, 5.0f, 1.0f, 6.0f);
		Quatf result = quat.normalize();
		Quatf expected(0.246182978f, 0.615457416f, 0.123091489f, 0.738548934f);

		Assert::AreEqual(expected[0], result[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[1], result[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[2], result[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[3], result[3], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_conjugate_Test)
	{
		Quatf quat(2.0f, 5.0f, 1.0f, 6.0f);
		Quatf result = quat.conjugate();
		Quatf expected(-2.0f, -5.0f, -1.0f, 6.0f);

		Assert::AreEqual(expected[0], result[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[1], result[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[2], result[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[3], result[3], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_dot_Test)
	{
		Quatf quatA(2.0f, 5.0f, 1.0f, 6.0f);
		Quatf quatB(0.030303f, 0.0757576f, 0.0151515f, 0.0909091f);
		float result = quatA.dot(quatB);
		float expected = 1.0f;

		Assert::AreEqual(expected, result, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_inverse_Test)
	{
		Quatf quat(2.0f, 5.0f, 1.0f, 6.0f);
		Quatf result = quat.inverse();
		Quatf expected(0.0303030275f, 0.0757575706f, 0.0151515137f, 0.0909090787f);

		float proof = result.dot(quat);

		Assert::AreEqual(1.0f, ceil(proof), L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[0], result[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[1], result[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[2], result[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(expected[3], result[3], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_toVec3_Test)
	{
		Quatf quat(2.0f, 5.0f, 1.0f, 6.0f);
		Vec3f result1 = quat.toVec3();
		Vec3f result2 = quat;

		Vec3f expected(2.0f, 5.0f, 1.0f);

		for (int i = 0; i < 3; i++) {
			Assert::AreEqual(expected[i], result1[i], L"Wrong value", LINE_INFO());
			Assert::AreEqual(expected[i], result2[i], L"Wrong value", LINE_INFO());
		}			
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_createRotate_Test)
	{
		Vec3f vector(2.0, 5.0f, 3.0f);
		double angle = degreesToRadians(60);

		Quatf result = Quatf::createRotate(angle, vector);
		Quatf expected(0.162221417f, 0.405553550f, 0.243332133f, 0.866025388f);

		for (int i = 0; i < 4; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i], 0.000001f), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Quat_rotate_Test)
	{
		Quatf quaternion(10.0f, 0.0f, 0.0f, 0.0f);
		Vec3f vector(0.0f, 0.0f, 1.0f);
		double angle = degreesToRadians(180);

		Quatf result = quaternion.rotate(angle, vector);
		Quatf expected(-10.0f, 0.0f, 0.0f, 0.0f);

		for (int i = 0; i < 4; i++)
			Assert::AreEqual(expected[i] + 1, result[i] + 1, L"Wrong value", LINE_INFO());
	}
}

#undef CLASS_NAME