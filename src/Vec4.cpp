#include "Vec4.h"

namespace NAMESPACE_PHYSICS
{
	
	Vec4::Vec4(const Vec2& xyComponents, const Vec2& zwComponents)
	{
		x = xyComponents.x;
		y = xyComponents.y;
		z = zwComponents.x;
		w = zwComponents.y;
	}

	
	Vec4::Vec4(const Vec3& vector, sp_float w)
	{
		x = vector.x;
		y = vector.y;
		z = vector.z;
		this->w = w;
	}
	
	Vec4 Vec4::normalize() const
	{
		//sp_assert(length() != T(0));  // avoid division by zero

		sp_float len = length();

		if (len == 0.0f)
			return Vec4(0.0f);

		sp_float vectorLengthInverted = 1.0f / len;

		return Vec4 {
			x * vectorLengthInverted,
			y * vectorLengthInverted,
			z * vectorLengthInverted,
			w * vectorLengthInverted
		};
	}
	
	Vec3 Vec4::toVec3() const
	{
		return Vec3(x, y, z);
	}
	
	Vec4 Vec4::operator*(const Mat4& matrix4x4) const
	{
		Vec4 result;

	#if MAJOR_COLUMN_ORDER

		result[0]
			= matrix4x4[0 * MAT4_ROW_LENGTH + 0] * x
			+ matrix4x4[0 * MAT4_ROW_LENGTH + 1] * y
			+ matrix4x4[0 * MAT4_ROW_LENGTH + 2] * z
			+ matrix4x4[0 * MAT4_ROW_LENGTH + 3] * w;

		result[1]
			= matrix4x4[1 * MAT4_ROW_LENGTH + 0] * x
			+ matrix4x4[1 * MAT4_ROW_LENGTH + 1] * y
			+ matrix4x4[1 * MAT4_ROW_LENGTH + 2] * z
			+ matrix4x4[1 * MAT4_ROW_LENGTH + 3] * w;

		result[2]
			= matrix4x4[2 * MAT4_ROW_LENGTH + 0] * x
			+ matrix4x4[2 * MAT4_ROW_LENGTH + 1] * y
			+ matrix4x4[2 * MAT4_ROW_LENGTH + 2] * z
			+ matrix4x4[2 * MAT4_ROW_LENGTH + 3] * w;

		result[3]
			= matrix4x4[3 * MAT4_ROW_LENGTH + 0] * x
			+ matrix4x4[3 * MAT4_ROW_LENGTH + 1] * y
			+ matrix4x4[3 * MAT4_ROW_LENGTH + 2] * z
			+ matrix4x4[3 * MAT4_ROW_LENGTH + 3] * w;

	#else

		result[0]
			= x * matrix4x4[0 * MAT4_ROW_LENGTH + 0]
			+ y * matrix4x4[1 * MAT4_ROW_LENGTH + 0]
			+ z * matrix4x4[2 * MAT4_ROW_LENGTH + 0]
			+ w * matrix4x4[3 * MAT4_ROW_LENGTH + 0];

		result[1]
			= x * matrix4x4[0 * MAT4_ROW_LENGTH + 0]
			+ y * matrix4x4[1 * MAT4_ROW_LENGTH + 0]
			+ z * matrix4x4[2 * MAT4_ROW_LENGTH + 0]
			+ w * matrix4x4[3 * MAT4_ROW_LENGTH + 0];

		result[2]
			= x * matrix4x4[0 * MAT4_ROW_LENGTH + 0]
			+ y * matrix4x4[1 * MAT4_ROW_LENGTH + 0]
			+ z * matrix4x4[2 * MAT4_ROW_LENGTH + 0]
			+ w * matrix4x4[3 * MAT4_ROW_LENGTH + 0];

		result[3]
			= x * matrix4x4[0 * MAT4_ROW_LENGTH + 0]
			+ y * matrix4x4[1 * MAT4_ROW_LENGTH + 0]
			+ z * matrix4x4[2 * MAT4_ROW_LENGTH + 0]
			+ w * matrix4x4[3 * MAT4_ROW_LENGTH + 0];
	#endif

		return result;
	}

	sp_bool Vec4::operator!=(const Vec4& vector) const
	{
		return x != vector.x
			|| y != vector.y
			|| z != vector.z
			|| w != vector.w;
	}

	void multiply(const Vec4& vectorA, const Vec4& vectorB, Mat4* output)
	{
		output[0][0] = vectorA.x * vectorB.x;
		output[0][1] = vectorA.x * vectorB.y;
		output[0][2] = vectorA.x * vectorB.z;
		output[0][3] = vectorA.x * vectorB.w;

		output[0][4] = vectorA.y * vectorB.x;
		output[0][5] = vectorA.y * vectorB.y;
		output[0][6] = vectorA.y * vectorB.z;
		output[0][7] = vectorA.y * vectorB.w;

		output[0][8] = vectorA.z * vectorB.x;
		output[0][9] = vectorA.z * vectorB.y;
		output[0][10] = vectorA.z * vectorB.z;
		output[0][11] = vectorA.z * vectorB.w;

		output[0][12] = vectorA.w * vectorB.x;
		output[0][13] = vectorA.w * vectorB.y;
		output[0][14] = vectorA.w * vectorB.z;
		output[0][15] = vectorA.w * vectorB.w;
	}

}