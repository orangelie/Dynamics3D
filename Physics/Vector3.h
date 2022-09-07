#pragma once

#include "Math.h"

class Vector3
{
public:
	union
	{
		struct
		{
			real x;
			real y;
			real z;
		};

		real data[3];
	};
private:
	real _pad;

public:
	Vector3() : x(0.0f), y(0.0f), z(0.0f) {}
	Vector3(real x, real y, real z) : x(x), y(y), z(z) {}

	real operator[](unsigned index) const
	{
		if (index == 0)
			return x;
		if (index == 1)
			return y;

		return z;
	}
	real& operator[](unsigned index)
	{
		if (index == 0)
			return x;
		if (index == 1)
			return y;

		return z;
	}

	void operator+=(const Vector3& v)
	{
		x += v.x;
		y += v.y;
		z += v.z;
	}

	Vector3 operator+(const Vector3& adder) const
	{
		return Vector3(x + adder.x, y + adder.y, z + adder.z);
	}

	void operator-=(const Vector3& v)
	{
		x -= v.x;
		y -= v.y;
		z -= v.z;
	}

	Vector3 operator-(const Vector3& adder) const
	{
		return Vector3(x - adder.x, y - adder.y, z - adder.z);
	}

	void operator*=(const float scalar)
	{
		x *= scalar;
		y *= scalar;
		z *= scalar;
	}

	Vector3 operator*(const float scalar) const
	{
		return Vector3(x * scalar, y * scalar, z * scalar);
	}

	Vector3 componentProduct(const Vector3& v) const;
	void componentProductUpdate(const Vector3& v);
	Vector3 vectorProduct(const Vector3& v) const;
	void vectorProductUpdate(const Vector3& v);
	real scalarProduct(const Vector3& v) const;

	Vector3 operator%(const Vector3& v) const
	{
		return Vector3(
			y * v.z - z * v.y,
			z * v.x - x * v.z,
			x * v.y - y * v.x);
	}

	void operator%=(const Vector3& v)
	{
		*this = vectorProduct(v);
	}

	real operator*(const Vector3& v) const
	{
		return (x * v.x) + (y * v.y) + (z * v.z);
	}

	void addScaledVector(const Vector3 v, float scale);

	real magnitude() const;

	real squaredMagnitude() const { return (x * x) + (y * y) + (z * z); }

	void normalize();

	Vector3 unit() const
	{
		Vector3 v = *this;
		v.normalize();

		return v;
	}

	void clear() { x = y = z = 0.0f; }

	void invert()
	{
		x = -x;
		y = -y;
		z = -z;
	}

};

