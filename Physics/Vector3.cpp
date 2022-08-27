#include "pch.h"
#include "Vector3.h"


Vector3 Vector3::componentProduct(const Vector3& v) const
{
	return Vector3(x * v.x, y * v.y, z * v.z);
}

void Vector3::componentProductUpdate(const Vector3& v)
{
	x *= v.x;
	y *= v.y;
	z *= v.z;
}

Vector3 Vector3::vectorProduct(const Vector3& v) const
{
	return Vector3(
		y * v.z - z * v.y,
		z * v.x - x * v.z,
		x * v.y - y * v.x);
}

void Vector3::vectorProductUpdate(const Vector3& v)
{
	*this = vectorProduct(v);
}

real Vector3::scalarProduct(const Vector3& v) const
{
	return (x * v.x) + (y * v.y) + (z * v.z);
}

void Vector3::addScaledVector(const Vector3 v, float scale)
{
	x += v.x * scale;
	y += v.y * scale;
	z += v.z * scale;
}


real Vector3::magnitude() const
{
	return sqrtf((x * x) + (y * y) + (z * z));
}

void Vector3::normalize()
{
	float l = magnitude();
	if (l > 0.0f)
	{
		float length = 1.0f / l;
		*this *= length;
	}
}
