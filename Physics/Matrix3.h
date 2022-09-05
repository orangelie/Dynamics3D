#pragma once

#include "Math.h"

class Matrix3
{
public:
	union
	{
		struct
		{
			real m00, m01, m02;
			real m10, m11, m12;
			real m20, m21, m22;
		};

		real data[9];
	};

	Matrix3()
	{
		data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = data[6] = data[7] = data[8] = 0.0f;
	}

	Matrix3(
		real x1, real y1, real z1,
		real x2, real y2, real z2,
		real x3, real y3, real z3)
	{
		data[0] = x1; data[1] = y1; data[2] = z1;
		data[3] = x2; data[4] = y2; data[5] = z2;
		data[6] = x3; data[7] = y3; data[8] = z3;
	}
	
	void setComponents(const Vector3& compOne, const Vector3& compTwo, const Vector3& compThree)
	{
		data[0] = compOne.x; data[1] = compTwo.x; data[2] = compThree.x;
		data[3] = compOne.y; data[4] = compTwo.y; data[5] = compThree.y;
		data[6] = compOne.z; data[7] = compTwo.z; data[8] = compThree.z;
	}

	Matrix3(const Vector3& compOne, const Vector3& compTwo, const Vector3& compThree)
	{
		setComponents(compOne, compTwo, compThree);
	}

	void setInertiaTensorCoeffs(real ix, real iy, real iz, real ixy = 0.0f, real ixz = 0.0f, real iyz = 0.0f);

	void setDiagonal(real a, real b, real c);

	void setBlockInertiaTensor(const Vector3& v, const real mass);

	void setSkewSymmetric(real x, real y, real z)
	{
		data[0] = 0;	data[1] = -z;	data[2] = y;
		data[3] = z;	data[4] = 0;	data[5] = -x;
		data[6] = -y;	data[7] = x;	data[8] = 0;
	}

	void setSkewSymmetric(const Vector3& v)
	{
		data[0] = 0;	data[1] = -v.z;	data[2] = v.y;
		data[3] = v.z;	data[4] = 0;	data[5] = -v.x;
		data[6] = -v.y;	data[7] = v.x;	data[8] = 0;
	}

	Vector3 operator*(const Vector3& v) const
	{
		return Vector3(
			data[0] * v.x + data[1] * v.y + data[2] * v.z,
			data[3] * v.x + data[4] * v.y + data[5] * v.z,
			data[6] * v.x + data[7] * v.y + data[8] * v.z
		);
	}

	Vector3 transform(const Vector3& v) const
	{
		return (*this) * v;
	}

	Vector3 transformTranspose(const Vector3& v) const
	{
		return Vector3(
			data[0] * v.x + data[3] * v.y + data[6] * v.z,
			data[1] * v.x + data[4] * v.y + data[7] * v.z,
			data[2] * v.x + data[5] * v.y + data[8] * v.z
		);
	}

	Vector3 getAxisVector(unsigned index) const;

	void setTranspose(const Matrix3& m)
	{
		data[0] = m.data[0]; data[1] = m.data[3]; data[2] = m.data[6];
		data[3] = m.data[1]; data[4] = m.data[4]; data[5] = m.data[7];
		data[6] = m.data[2]; data[7] = m.data[5]; data[8] = m.data[8];
	}

	Matrix3 transpose() const
	{
		Matrix3 m;
		m.setTranspose(*this);

		return m;
	}

	Matrix3 operator*(const Matrix3& o) const
	{
		return Matrix3(
			data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6],
			data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7],
			data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8],

			data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6],
			data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7],
			data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8],

			data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6],
			data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7],
			data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8]
		);
	}

	void operator*=(const Matrix3& o)
	{
		real t1;
		real t2;
		real t3;

		t1 = data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6];
		t2 = data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7];
		t3 = data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8];
		data[0] = t1;
		data[1] = t2;
		data[2] = t3;

		t1 = data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6];
		t2 = data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7];
		t3 = data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8];
		data[3] = t1;
		data[4] = t2;
		data[5] = t3;

		t1 = data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6];
		t2 = data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7];
		t3 = data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8];
		data[6] = t1;
		data[7] = t2;
		data[8] = t3;
	}

	void operator*=(const real scalar)
	{
		data[0] *= scalar; data[1] *= scalar; data[2] *= scalar;
		data[3] *= scalar; data[4] *= scalar; data[5] *= scalar;
		data[6] *= scalar; data[7] *= scalar; data[8] *= scalar;
	}

	void operator+=(const Matrix3& o)
	{
		data[0] += o.data[0]; data[1] += o.data[1]; data[2] += o.data[2];
		data[3] += o.data[3]; data[4] += o.data[4]; data[5] += o.data[5];
		data[6] += o.data[6]; data[7] += o.data[7]; data[8] += o.data[8];
	}

	void setInverse(const Matrix3& m)
	{
		real t4 = m.data[0] * m.data[4];
		real t6 = m.data[0] * m.data[5];
		real t8 = m.data[1] * m.data[3];
		real t10 = m.data[2] * m.data[3];
		real t12 = m.data[1] * m.data[6];
		real t14 = m.data[2] * m.data[6];

		real t16 = (t4 * m.data[8] - t6 * m.data[7] - t8 * m.data[8] +
			t10 * m.data[7] + t12 * m.data[5] - t14 * m.data[4]);

		if (t16 == (real)0.0f) return;
		real t17 = 1.0f / t16;

		data[0] = (m.data[4] * m.data[8] - m.data[5] * m.data[7]) * t17;
		data[1] = -(m.data[1] * m.data[8] - m.data[2] * m.data[7]) * t17;
		data[2] = (m.data[1] * m.data[5] - m.data[2] * m.data[4]) * t17;
		data[3] = -(m.data[3] * m.data[8] - m.data[5] * m.data[6]) * t17;
		data[4] = (m.data[0] * m.data[8] - t14) * t17;
		data[5] = -(t6 - t10) * t17;
		data[6] = (m.data[3] * m.data[7] - m.data[4] * m.data[6]) * t17;
		data[7] = -(m.data[0] * m.data[7] - t12) * t17;
		data[8] = (t4 - t8) * t17;
	}

	Matrix3 inverse()
	{
		Matrix3 m;
		m.setInverse(*this);

		return m;
	}

	void setOrientation(const Quaternion& q)
	{
		data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
		data[1] = 2 * q.i * q.j + 2 * q.k * q.r;
		data[2] = 2 * q.i * q.k - 2 * q.j * q.r;
		data[3] = 2 * q.i * q.j - 2 * q.k * q.r;
		data[4] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
		data[5] = 2 * q.j * q.k + 2 * q.i * q.r;
		data[6] = 2 * q.i * q.k + 2 * q.j * q.r;
		data[7] = 2 * q.j * q.k - 2 * q.i * q.r;
		data[8] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
	}

};

