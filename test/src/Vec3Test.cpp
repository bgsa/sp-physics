#include "SpectrumPhysicsTest.h"
#include <Vec3.h>

#define CLASS_NAME Vec3Test

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		
		SP_TEST_METHOD_DEF(x);
		SP_TEST_METHOD_DEF(y);
		SP_TEST_METHOD_DEF(z);
		SP_TEST_METHOD_DEF(length);
		SP_TEST_METHOD_DEF(abs);
		SP_TEST_METHOD_DEF(squaredLength);
		SP_TEST_METHOD_DEF(add);
		SP_TEST_METHOD_DEF(diff);
		SP_TEST_METHOD_DEF(scale);
		SP_TEST_METHOD_DEF(cross);
		SP_TEST_METHOD_DEF(Vec3_dot_Test);
		SP_TEST_METHOD_DEF(scalarTriple);
		SP_TEST_METHOD_DEF(angle);
		SP_TEST_METHOD_DEF(Vec3_normalize_Test);
		SP_TEST_METHOD_DEF(Vec3_distance_Test);
		SP_TEST_METHOD_DEF(copy);
		SP_TEST_METHOD_DEF(Vec3_operatorMultiplyScalar_Test);
		SP_TEST_METHOD_DEF(Vec3_operatorMultiplyVector_Test);
		SP_TEST_METHOD_DEF(Vec3_operatorPlusVector_Test);
		SP_TEST_METHOD_DEF(Vec3_operatorPlusScalar_Test);
		SP_TEST_METHOD_DEF(Vec3_operatorMinus_vector_Test);
		SP_TEST_METHOD_DEF(Vec3_operatorMinus_prefix_Test);
		SP_TEST_METHOD_DEF(Vec3_operatorMinusScalar_Test);
		SP_TEST_METHOD_DEF(Vec3_operatorEqualVector_Test);
		SP_TEST_METHOD_DEF(Vec3_operatorEqualScalar_Test);
		SP_TEST_METHOD_DEF(Vec3_operatorNotEqual_Test);
		SP_TEST_METHOD_DEF(Vec3_operatorIndex_Test);
		SP_TEST_METHOD_DEF(Vec3_operatorDireferent_value_Test);
	};


	SP_TEST_METHOD(CLASS_NAME, x)
	{
		Vec3 vector = { 2.0f, 5.0f, -9.0f };
		sp_float expected = 2.0f;

		sp_float result = vector.x;

		Assert::AreEqual(expected, result, L"Wrong value.", LINE_INFO());
	}
	SP_TEST_METHOD(CLASS_NAME, y)
	{
		Vec3 vector = { 2.0f, 5.0f, -9.0f };
		float expected = 5.0f;

		float result = vector.y;

		Assert::AreEqual(expected, result, L"Wrong value.", LINE_INFO());
	}
	SP_TEST_METHOD(CLASS_NAME, z)
	{
		Vec3 vector = { 2.0f, 5.0f, -9.0f };
		float expected = -9.0f;

		float result = vector.z;

		Assert::AreEqual(expected, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, length)
	{
		Vec3 vector = { 2.0f, 5.0f, -9.0f };

		float expected = sqrt(110.0f);

		float result = vector.length();

		Assert::AreEqual(expected, result, L"Length value wrong.", LINE_INFO());
	}	

	SP_TEST_METHOD(CLASS_NAME, abs)
	{
		Vec3 vector = { -2.0f, -5.0f, 9.0f };
		NAMESPACE_PHYSICS::abs(&vector);
		Vec3 expected = { 2.0f, 5.0f, 9.0f };

		for (int i = 0; i < VEC3_LENGTH; i++)
			Assert::AreEqual(expected[i], vector[i], L"Length value wrong.", LINE_INFO());
	
		vector = { 2.0f, -5.0f, -9.0f };
		NAMESPACE_PHYSICS::abs(vector, &vector);
		for (int i = 0; i < VEC3_LENGTH; i++)
			Assert::AreEqual(expected[i], vector[i], L"Length value wrong.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, squaredLength)
	{
		Vec3 vector = { 2.0f, 5.0f, -9.0f };
		float expected = 4.0f + 25.0f + 81.0f;

		float result = vector.squaredLength();

		Assert::AreEqual(expected, result, L"Length value wrong.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, add)
	{
		Vec3 vector = { 2.0f, 5.0f, -9.0f };
		Vec3 vector2 = { 1.0f, 2.0f, 3.0f };
		float expected[3] = { 3.0f, 7.0f, -6.0f };

		vector.add(vector2);

		for (int i = 0; i < VEC3_LENGTH; i++)
			Assert::AreEqual(expected[i], vector[i], L"Length value wrong.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, diff)
	{
		Vec3 vector = { 2.0f, 5.0f, -9.0f };
		Vec3 vector2 = { 1.0f, 2.0f, 3.0f };
		sp_float expected[3] = { 1.0f, 3.0f, -12.0f };

		Vec3 result;
		NAMESPACE_PHYSICS::diff(vector, vector2, &result);

		for (int i = 0; i < VEC3_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Length value wrong.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, scale)
	{
		Vec3 vector = { 2.0f, 5.0f, -9.0f };
		float expected[3] = { 6.0f, 15.0f, -27.0f };

		vector.scale(3.0f);

		for (int i = 0; i < VEC3_LENGTH; i++)
			Assert::AreEqual(expected[i], vector[i], L"Length value wrong.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, cross)
	{
		Vec3 vector = { 1.0f, 0.0f, 0.0f };
		Vec3 vector2 = { 0.0f, 0.0f, 1.0f };
		Vec3 expected = { 0.0f, 1.0f, 0.0f };
		Vec3 result;
		
		NAMESPACE_PHYSICS::cross(vector, vector2, &result);
		for (sp_int i = 0; i < VEC3_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Vec3_dot_Test)
	{
		Vec3 vector = { 2.0f, 5.0f, -9.0f };
		Vec3 vector2 = { 4.0f, -2.0f, 3.0f };
		sp_float expected = -29.0f;

		sp_float result = vector.dot(vector2);

		Assert::AreEqual(expected, result, L"Length value wrong.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, scalarTriple)
	{
		Vec3 vector1 = { 10.0f, 0.0f, 0.0f };
		Vec3 vector2 = { 0.0f, 0.0f, 10.0f };
		Vec3 vector3 = { 0.0f, 10.0f, 0.0f };
		sp_float expected = 1000.0f;

		sp_float result = NAMESPACE_PHYSICS::scalarTriple(vector1, vector2, vector3);

		Assert::AreEqual(expected, result, L"Length value wrong.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, angle)
	{
		Vec3 vector = { 3.0f, 4.0f, 0.0f };
		Vec3 vector2 = { 4.0f, 4.0f, 2.0f };
		
		float expected = 28.0f / 30.0f;

		float result = vector.angle(vector2);

		Assert::AreEqual(expected, result, L"Value wrong.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Vec3_normalize_Test)
	{
		Vec3 vector = { 3.0f, 1.0f, 2.0f };
		Vec3 expected = { 0.801783681f, 0.267261237f, 0.534522474f };
		
		Vec3 result = vector.normalize();

		for (int i = 0; i < VEC3_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Value wrong.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Vec3_distance_Test)
	{
		Vec3 vector = { 2.0f, 5.0f, -9.0f };
		Vec3 vector2 = { 1.0f, 2.0f, 3.0f };
		sp_float expected = 12.4096737f;

		sp_float result = vector.distance(vector2);

		Assert::AreEqual(expected, result, L"Value wrong.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, copy)
	{
		Vec3 vector = { 2.0f, 5.0f, -9.0f };
		Vec3 result;
		vector.copy(&result);

		for (int i = 0; i < VEC3_LENGTH; i++)
			Assert::AreEqual(vector[i], result[i], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Vec3_operatorMultiplyScalar_Test)
	{
		Vec3 vector = { 2.0f, 5.0f, -9.0f };

		Vec3 expected = { 4.0f, 10.0f, -18.0f };

		Vec3 result = vector * 2;

		for (int i = 0; i < VEC3_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Vec3_operatorMultiplyVector_Test)
	{
		Vec3 vector = { 2.0f, 3.0f, 4.0f };
		Vec3 vector2 = { 5.0f, 6.0f, -7.0f };
		Vec3 expected = { 10.0f, 18.0f, -28.0f };

		Vec3 result = vector * vector2;

		for (int i = 0; i < VEC3_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Vec3_operatorPlusVector_Test)
	{
		Vec3 vector = { 2.0f, 3.0f, 4.0f };
		Vec3 vector2 = { 5.0f, 6.0f, -7.0f };
		Vec3 expected = { 7.0f, 9.0f, -3.0f };

		Vec3 result = vector + vector2;

		for (int i = 0; i < VEC3_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Vec3_operatorPlusScalar_Test)
	{
		Vec3 vector = { 2.0f, -3.0f, 4.0f };
		Vec3 expected = { 4.0f, -1.0f, 6.0f };

		Vec3 result = vector + 2.0f;

		for (int i = 0; i < VEC3_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Vec3_operatorMinus_vector_Test)
	{
		Vec3 vector = { 2.0f, 3.0f, 4.0f };
		Vec3 vector2 = { 5.0f, 6.0f, -7.0f };
		Vec3 expected = { -3.0f, -3.0f, 11.0f };

		Vec3 result = vector - vector2;

		for (int i = 0; i < VEC3_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Vec3_operatorMinus_prefix_Test)
	{
		Vec3 vector = { -2.0f, 3.0f, 4.0f };
		Vec3 expected = { 2.0f, -3.0f, -4.0f };

		Vec3 result = -vector;

		for (int i = 0; i < VEC3_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Vec3_operatorMinusScalar_Test)
	{
		Vec3 vector = { 2.0f, -3.0f, 4.0f };
		Vec3 expected = { 0.0f, -5.0f, 2.0f };

		Vec3 result = vector - 2.0f;

		for (int i = 0; i < VEC3_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Vec3_operatorEqualVector_Test)
	{
		Vec3 vector = { 2.0f, -3.0f, 4.0f };

		Vec3 vectorEqual = { 2.0f, -3.0f, 4.0f };

		Vec3 vectorNotEqual_X = { 0.0f, -3.0f, 4.0f };
		Vec3 vectorNotEqual_Y = { 2.0f, 3.0f, 4.0f };
		Vec3 vectorNotEqual_Z = { 2.0f, -3.0f, 5.0f };
		
		Assert::IsTrue(vector == vectorEqual, L"Vectors should be equal.", LINE_INFO());

		Assert::IsFalse(vector == vectorNotEqual_X, L"Vectors should not be equal.", LINE_INFO());
		
		Assert::IsFalse(vector == vectorNotEqual_Y, L"Vectors should not be equal.", LINE_INFO());

		Assert::IsFalse(vector == vectorNotEqual_Z, L"Vectors should not be equal.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Vec3_operatorEqualScalar_Test)
	{
		Vec3 vector = { 0.0f, 0.0f, 0.0f };
		Assert::IsTrue(vector == 0, L"Vectors should be equal 0.", LINE_INFO());

		vector = { 1.0f, 0.0f, 0.0f };
		Assert::IsFalse(vector == 0, L"Vectors should not be equal 0.", LINE_INFO());

		vector = { 0.0f, 1.0f, 0.0f };
		Assert::IsFalse(vector == 0, L"Vectors should not be equal 0.", LINE_INFO());

		vector = { 0.0f, 0.0f, 1.0f };
		Assert::IsFalse(vector == 0, L"Vectors should not be equal 0.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Vec3_operatorNotEqual_Test)
	{
		Vec3 vector = { 2.0f, -3.0f, 4.0f };

		Vec3 vectorEqual = { 2.0f, -3.0f, 4.0f };

		Vec3 vectorNotEqual_X = { 0.0f, -3.0f, 4.0f };
		Vec3 vectorNotEqual_Y = { 2.0f, 3.0f, 4.0f };
		Vec3 vectorNotEqual_Z = { 2.0f, -3.0f, 5.0f };

		Assert::IsFalse(vector != vectorEqual, L"Vectors should be equal.", LINE_INFO());

		Assert::IsTrue(vector != vectorNotEqual_X, L"Vectors should not be equal.", LINE_INFO());

		Assert::IsTrue(vector != vectorNotEqual_Y, L"Vectors should not be equal.", LINE_INFO());

		Assert::IsTrue(vector != vectorNotEqual_Z, L"Vectors should not be equal.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Vec3_operatorIndex_Test)
	{
		Vec3 vector = { 2.0f, -3.0f, 4.0f };

		Assert::AreEqual(2.0f, vector[0], L"Wrong value.", LINE_INFO());
		Assert::AreEqual(-3.0f, vector[1], L"Wrong value.", LINE_INFO());
		Assert::AreEqual(4.0f, vector[2], L"Wrong value.", LINE_INFO());

		vector[0] = 5.0f;
		vector[1] = 6.0f;
		vector[2] = 7.0f;

		Assert::AreEqual(5.0f, vector[0], L"Wrong value.", LINE_INFO());
		Assert::AreEqual(6.0f, vector[1], L"Wrong value.", LINE_INFO());
		Assert::AreEqual(7.0f, vector[2], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Vec3_operatorDireferent_value_Test)
	{
		Vec3 vector = { 0.0f, 0.0f, 0.0f };
		bool result = vector != 0.0f;
		Assert::IsFalse(result, L"vector should not be diferente of Zero", LINE_INFO());

		vector = { 0.0f, 0.0f, 1.0f };
		result = vector != 0.0f;
		Assert::IsTrue(result, L"vector should be diferente of Zero", LINE_INFO());

		vector = { 0.0f, 1.0f, 0.0f };
		result = vector != 0.0f;
		Assert::IsTrue(result, L"vector should be diferente of Zero", LINE_INFO());

		vector = { 1.0f, 0.0f, 0.0f };
		result = vector != 0.0f;
		Assert::IsTrue(result, L"vector should be diferente of Zero", LINE_INFO());
	}

}

#undef CLASS_NAME