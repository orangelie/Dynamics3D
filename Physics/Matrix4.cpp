#include "pch.h"
#include "Matrix4.h"


real Matrix4::getDeterminant() const
{
	return -data[8] * data[5] * data[2] +
		data[4] * data[9] * data[2] +
		data[8] * data[1] * data[6] -
		data[0] * data[9] * data[6] -
		data[4] * data[1] * data[10] +
		data[0] * data[5] * data[10];
}

void Matrix4::setInverse(const Matrix4& m)
{
	real det = getDeterminant();
	if (det == 0) return;
	det = ((real)1.0) / det;

	data[0] = (-m.data[9] * m.data[6] + m.data[5] * m.data[10]) * det;
	data[4] = (m.data[8] * m.data[6] - m.data[4] * m.data[10]) * det;
	data[8] = (-m.data[8] * m.data[5] + m.data[4] * m.data[9]) * det;

	data[1] = (m.data[9] * m.data[2] - m.data[1] * m.data[10]) * det;
	data[5] = (-m.data[8] * m.data[2] + m.data[0] * m.data[10]) * det;
	data[9] = (m.data[8] * m.data[1] - m.data[0] * m.data[9]) * det;

	data[2] = (-m.data[5] * m.data[2] + m.data[1] * m.data[6]) * det;
	data[6] = (+m.data[4] * m.data[2] - m.data[0] * m.data[6]) * det;
	data[10] = (-m.data[4] * m.data[1] + m.data[0] * m.data[5]) * det;

	data[3] = (m.data[9] * m.data[6] * m.data[3]
		- m.data[5] * m.data[10] * m.data[3]
		- m.data[9] * m.data[2] * m.data[7]
		+ m.data[1] * m.data[10] * m.data[7]
		+ m.data[5] * m.data[2] * m.data[11]
		- m.data[1] * m.data[6] * m.data[11]) * det;
	data[7] = (-m.data[8] * m.data[6] * m.data[3]
		+ m.data[4] * m.data[10] * m.data[3]
		+ m.data[8] * m.data[2] * m.data[7]
		- m.data[0] * m.data[10] * m.data[7]
		- m.data[4] * m.data[2] * m.data[11]
		+ m.data[0] * m.data[6] * m.data[11]) * det;
	data[11] = (m.data[8] * m.data[5] * m.data[3]
		- m.data[4] * m.data[9] * m.data[3]
		- m.data[8] * m.data[1] * m.data[7]
		+ m.data[0] * m.data[9] * m.data[7]
		+ m.data[4] * m.data[1] * m.data[11]
		- m.data[0] * m.data[5] * m.data[11]) * det;
}

Matrix4 Matrix4::inverse() const
{
	Matrix4 m;
	m.setInverse(*this);

	return m;
}

void Matrix4::invert()
{
	setInverse(*this);
}

Vector3 Matrix4::getAxisVector(unsigned index) const
{
	return Vector3(data[0 + index], data[4 + index], data[8 + index]);
}
