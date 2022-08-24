#include "pch.h"
#include "Matrix3.h"

void Matrix3::setInertiaTensorCoeffs(real ix, real iy, real iz, real ixy, real ixz, real iyz)
{
	data[0] = ix;	data[1] = -ixy; data[2] = -ixz;
	data[3] = -ixy;	data[4] = iy;	data[5] = -iyz;
	data[6] = -ixz;	data[7] = -iyz;	data[8] = iz;
}

void Matrix3::setDiagonal(real a, real b, real c)
{
	setInertiaTensorCoeffs(a, b, c);
}

void Matrix3::setBlockInertiaTensor(const Vector3& v, const real mass)
{
	Vector3 squared = v.componentProduct(v);

	setInertiaTensorCoeffs(
		0.3f * mass * (squared.y + squared.z),
		0.3f * mass * (squared.x + squared.z),
		0.3f * mass * (squared.x + squared.y));
}

Vector3 Matrix3::getAxisVector(unsigned index) const
{
	return Vector3(data[0 + index], data[3 + index], data[6 + index]);
}
