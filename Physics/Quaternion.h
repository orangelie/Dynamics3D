#pragma once

#include "Math.h"

class Quaternion
{
public:
	union
	{
		struct
		{
			real r;
			real i;
			real j;
			real k;
		};

		real data[4];
	};

	Quaternion() : r(0.0f), i(0.0f), j(0.0f), k(0.0f) {}
	Quaternion(const real r, const real i, const real j, const real k) : r(r), i(i), j(j), k(k) {}

	void operator*=(const Quaternion& multiplier)
	{
		Quaternion q = *this;

		r = (q.r * multiplier.r) - (q.i * multiplier.i) - (q.j * multiplier.j) - (q.k * multiplier.k);
		i = (q.r * multiplier.i) + (q.i * multiplier.r) + (q.j * multiplier.k) - (q.k * multiplier.j);
		j = (q.r * multiplier.j) + (q.j * multiplier.r) + (q.k * multiplier.i) - (q.i * multiplier.k);
		k = (q.r * multiplier.k) + (q.k * multiplier.r) + (q.i * multiplier.j) - (q.j * multiplier.i);
	}

	void normalize();
	void addScaledVector(const Vector3& vec, real scale);
	void rotateByVector(const Vector3& vec);

};
