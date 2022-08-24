#pragma once

#include "Math.h"

class Matrix4
{
public:
	union
	{
		struct
		{
			real m00, m01, m02, m03;
			real m10, m11, m12, m13;
			real m20, m21, m22, m23;
		};

		real data[12];
	};

	Matrix4()
	{
		data[1] = data[2] = data[3] = data[4] = data[6] = data[7] = data[8] = data[9] = data[11] = 0.0f;
		data[0] = data[5] = data[10] = 1.0f;
	}

	void setDiagonal(const real a, const real b, const real c)
	{
		data[0] = a;
		data[5] = b;
		data[10] = c;
	}

	Vector3 operator*(const Vector3& v) const
	{
		return Vector3(
			data[0] * v.x + data[1] * v.y + data[2] * v.z + data[3],
			data[4] * v.x + data[5] * v.y + data[6] * v.z + data[7],
			data[8] * v.x + data[9] * v.y + data[10] * v.z + data[11]);
	}

	Vector3 transform(const Vector3& v) const
	{
		return Vector3(
			data[0] * v.x + data[1] * v.y + data[2] * v.z + data[3],
			data[4] * v.x + data[5] * v.y + data[6] * v.z + data[7],
			data[8] * v.x + data[9] * v.y + data[10] * v.z + data[11]);
	}

	Matrix4 operator*(const Matrix4& o) const
	{
		Matrix4 result;

		result.data[0] = (o.data[0] * data[0]) + (o.data[4] * data[1]) + (o.data[8] * data[2]);
		result.data[4] = (o.data[0] * data[4]) + (o.data[4] * data[5]) + (o.data[8] * data[6]);
		result.data[8] = (o.data[0] * data[8]) + (o.data[4] * data[9]) + (o.data[8] * data[10]);

		result.data[1] = (o.data[1] * data[0]) + (o.data[5] * data[1]) + (o.data[9] * data[2]);
		result.data[5] = (o.data[1] * data[4]) + (o.data[5] * data[5]) + (o.data[9] * data[6]);
		result.data[9] = (o.data[1] * data[8]) + (o.data[5] * data[9]) + (o.data[9] * data[10]);

		result.data[2] = (o.data[2] * data[0]) + (o.data[6] * data[1]) + (o.data[10] * data[2]);
		result.data[6] = (o.data[2] * data[4]) + (o.data[6] * data[5]) + (o.data[10] * data[6]);
		result.data[10] = (o.data[2] * data[8]) + (o.data[6] * data[9]) + (o.data[10] * data[10]);

		result.data[3] = (o.data[3] * data[0]) + (o.data[7] * data[1]) + (o.data[11] * data[2]) + data[3];
		result.data[7] = (o.data[3] * data[4]) + (o.data[7] * data[5]) + (o.data[11] * data[6]) + data[7];
		result.data[11] = (o.data[3] * data[8]) + (o.data[7] * data[9]) + (o.data[11] * data[10]) + data[11];

		return result;
	}

	real getDeterminant() const;

	void setInverse(const Matrix4& m);

	Matrix4 inverse() const;

	void invert();

	Vector3 getAxisVector(unsigned index) const;

	void fillArray(float array[16]) const
	{
		array[0] = (float)data[0];
		array[1] = (float)data[4];
		array[2] = (float)data[8];
		array[3] = (float)0;

		array[4] = (float)data[1];
		array[5] = (float)data[5];
		array[6] = (float)data[9];
		array[7] = (float)0;

		array[8] = (float)data[2];
		array[9] = (float)data[6];
		array[10] = (float)data[10];
		array[11] = (float)0;

		array[12] = (float)data[3];
		array[13] = (float)data[7];
		array[14] = (float)data[11];
		array[15] = (float)1;
	}
};

