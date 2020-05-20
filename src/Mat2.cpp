#include "Mat2.h"

namespace NAMESPACE_PHYSICS
{
	Mat2::Mat2(const sp_float defaultValue)
	{
		static sp_float emptyMatrix[MAT2_LENGTH] = {
			defaultValue, defaultValue,
			defaultValue, defaultValue
		};

		std::memcpy(&values, emptyMatrix, sizeof(values));
	}

	Mat2::Mat2(sp_float* values)
	{
		std::memcpy(&this->values, values, sizeof(this->values));
	}

	Mat2::Mat2(const sp_float value11, const sp_float value12,const sp_float value21, const sp_float value22)
	{
		values[0] = value11;
		values[1] = value12;
		values[2] = value21;
		values[3] = value22;
	}

	Mat2 Mat2::identity()
	{
		static sp_float identityMatrix[MAT2_LENGTH] = {
			ONE_FLOAT, ZERO_FLOAT,
			ZERO_FLOAT, ONE_FLOAT
		};

		Mat2 result;
		std::memcpy(&result, identityMatrix, sizeof(values));

		return result;
	}

	sp_float* Mat2::getValues()
	{
		return values;
	}

	sp_float Mat2::getValue(const sp_int x, const sp_int y)
	{
		return values[ (y-1) * MAT2_ROW_LENGTH + (x-1)];
	}

	Vec2 Mat2::xAxis() const
	{
		Vec2 result;

	#ifdef MAJOR_COLUMN_ORDER
		result = Vec2{
			values[0],
			values[2]
		};
	#else
		result = Vec2{
			values[0],
			values[1]
		};
	#endif

		return result;
	}

	Vec2 Mat2::yAxis() const
	{
		Vec2 result;

	#ifdef MAJOR_COLUMN_ORDER
		result = Vec2{
			values[1],
			values[3]
		};
	#else
		result = Vec2{
			values[2],
			values[3]
		};
	#endif

		return result;
	}

	Vec2 Mat2::primaryDiagonal() const
	{
		return Vec2 {
			values[0],
				values[3]
		};
	}

	Vec2 Mat2::secondaryDiagonal() const
	{
		return Vec2 {
			values[1],
				values[2]
		};
	}

	Mat2 Mat2::transpose() const
	{
		Mat2 result;

		//copy principal diagonal
		result[0] = values[0];
		result[3] = values[3];

		//swap others numbers
		sp_float temp = values[1];
		result[1] = values[2];
		result[2] = temp;

		return result;
	}

	Mat2 Mat2::multiply(const Mat2& matrixB) const
	{
		Mat2 result;

	#ifdef MAJOR_COLUMN_ORDER
		for (sp_int line = 0; line < MAT2_ROW_LENGTH; line++)
		{
			sp_float ai0 = values[(0 * MAT2_ROW_LENGTH) + line];
			sp_float ai1 = values[(1 * MAT2_ROW_LENGTH) + line];

			result[(0 * MAT2_ROW_LENGTH) + line] = ai0 * matrixB[(0 * MAT2_ROW_LENGTH) + 0] + ai1 * matrixB[(0 * MAT2_ROW_LENGTH) + 1];
			result[(1 * MAT2_ROW_LENGTH) + line] = ai0 * matrixB[(1 * MAT2_ROW_LENGTH) + 0] + ai1 * matrixB[(1 * MAT2_ROW_LENGTH) + 1];
		}
	#else
		for (sp_int column = 0; column < MAT2_ROW_LENGTH; column++)
		{
			sp_float ai0 = values[(column * MAT2_ROW_LENGTH) + 0];
			sp_float ai1 = values[(column * MAT2_ROW_LENGTH) + 1];

			result[(column * MAT2_ROW_LENGTH) + 0] = ai0 * matrixB[(0 * MAT2_ROW_LENGTH) + 0] + ai1 * matrixB[(1 * MAT2_ROW_LENGTH) + 0];
			result[(column * MAT2_ROW_LENGTH) + 1] = ai0 * matrixB[(0 * MAT2_ROW_LENGTH) + 1] + ai1 * matrixB[(1 * MAT2_ROW_LENGTH) + 1];
		}
	#endif

		return result;
	}

	Vec2 Mat2::multiply(const Vec2& vector) const
	{
		Vec2 result;

		result[0] = values[0 * MAT2_ROW_LENGTH + 0] * vector[0] + values[0 * MAT2_ROW_LENGTH + 1] * vector[1];
		result[1] = values[1 * MAT2_ROW_LENGTH + 0] * vector[0] + values[1 * MAT2_ROW_LENGTH + 1] * vector[1];

		return result;
	}

	Mat2 Mat2::createScale(const sp_float xScale, const sp_float yScale)
	{
		Mat2 result = Mat2::identity();

		result.scale(xScale, yScale);

		return result;
	}

	void Mat2::scale(const sp_float xScale, const sp_float yScale)
	{
		values[0] *= xScale;
		values[3] *= yScale;
	}

	Mat2 Mat2::createTranslate(const sp_float x, const sp_float y)
	{
		Mat2 result = Mat2::identity();

	#ifdef MAJOR_COLUMN_ORDER
		result[2] = x;
		result[3] = y;
	#else
		result[1] = x;
		result[3] = y;
	#endif

		return result;
	}

	sp_float Mat2::determinant() const
	{
		return values[0] * values[3] - values[1] * values[2];
	}

	AutoValueAutoVector2 Mat2::getAutovalueAndAutovector(const sp_ushort maxIteration) const
	{
		Mat2 matrix = *this;
		Vec2 autovector(1.0f, 1.0f);
		sp_float autovalue;

		for (unsigned short iterationIndex = 0; iterationIndex < maxIteration; iterationIndex++)
		{
			Vec2 ax = matrix * autovector;
			autovalue = ax.maximum();
			autovector = ax / autovalue;
		}

		return AutoValueAutoVector2{ autovalue, {autovector.x, autovector.y} };
	}

	sp_size Mat2::sizeInBytes() const
	{
		return MAT2_LENGTH * SIZEOF_FLOAT;
	}

	Mat2 Mat2::clone() const
	{
		Mat2 result;

		std::memcpy(&result, this, sizeof(Mat2));

		return result;
	}

	sp_float& Mat2::operator[](sp_int index)
	{
		sp_assert(index >= 0 && index < MAT2_LENGTH, "IndexOutOfrangeException");

		return values[index];
	}

	sp_float Mat2::operator[](sp_int index) const
	{
		sp_assert(index >= 0 && index < MAT2_LENGTH, "IndexOutOfrangeException");

		return values[index];
	}

	Mat2::operator void*() const
	{
		return (void*) values;
	}

	Mat2::operator sp_float*() const
	{
		return (sp_float*) values;
	}

	Mat2 Mat2::operator-() const
	{
		return Mat2 {
			-values[0],
			-values[1],
			-values[2],
			-values[3]
		};
	}

	Mat2 Mat2::operator-(const Mat2& matrix) const
	{
		return Mat2 {
			values[0] - matrix[0],
			values[1] - matrix[1],
			values[2] - matrix[2],
			values[3] - matrix[3]
		};
	}

	Mat2 Mat2::operator+(const Mat2& matrix) const
	{
		return Mat2 {
			values[0] + matrix[0],
			values[1] + matrix[1],
			values[2] + matrix[2],
			values[3] + matrix[3]
		};
	}

	Mat2 Mat2::operator*(const Mat2& matrix) const
	{
		return multiply(matrix);
	}

	Vec2 Mat2::operator*(const Vec2& vector) const
	{
		return multiply(vector);
	}

	void Mat2::operator*=(const Mat2& matrix)
	{
		std::memcpy(&this->values, multiply(matrix).values, sizeof(this->values));
	}

	Mat2 Mat2::operator/(const sp_float value) const
	{
		sp_float inverted = 1.0f / value;

		return Mat2 {
			values[0] * inverted,
			values[1] * inverted,
			values[2] * inverted,
			values[3] * inverted
		};
	}

	void Mat2::operator/=(const sp_float value)
	{
		sp_float inverted = 1.0f / value;

		values[0] *= inverted;
		values[1] *= inverted;
		values[2] *= inverted;
		values[3] *= inverted;
	}

	std::string Mat2::toString() const
	{
		return Mat::toString(values, MAT2_LENGTH);
	}

}