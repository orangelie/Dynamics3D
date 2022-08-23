#include "pch.h"
#include "Quaternion.h"

void Quaternion::normalize()
{
	real d = (r * r) + (i * i) + (j * j) + (k * k);

	if (d < FLT_EPSILON)
	{
		r = 1.0f;
		return;
	}

	d = 1.0f / sqrtf(d);
	r *= d;
	i *= d;
	j *= d;
	k *= d;
}

void Quaternion::addScaledVector(const Vector3& vec, real scale)
{
	Quaternion q(0.0f, vec.x * scale, vec.y * scale, vec.z * scale);

	q *= *this;
	r += q.r * 0.5f;
	i += q.i * 0.5f;
	j += q.j * 0.5f;
	k += q.k * 0.5f;
}

void Quaternion::rotateByVector(const Vector3& vec)
{
	Quaternion q(0.0f, vec.x, vec.y, vec.z);

	*this *= q;
}
