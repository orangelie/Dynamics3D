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

	Matrix4 operator*(const Matrix4& muliplier) const
	{
		Matrix4 m;



		return m;
	}
};

