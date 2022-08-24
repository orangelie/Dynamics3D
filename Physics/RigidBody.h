#pragma once

static inline void calculateTransformMatrix(Matrix4& transformMatrix, const Vector3& position, const Quaternion& orientation);
static inline void transformInertiaTensor(Matrix3& iitWorld, const Matrix3& iitBody, const Matrix4& rotmat);

class RigidBody
{
protected:
	Vector3 _acceleration;
	Vector3 _lastFrameAcceleration;

	Vector3 _velocity;
	Vector3 _rotation;

	Vector3 _positon;
	Quaternion _orientation;

	real _inverseMass;
	
	real _linearDamping;
	real _angularDamping;

	Matrix3 _inverseInertiaTensor;
	Matrix3 _inverseInertiaTensorWorld;

	Matrix4 _transformMatrix;

	bool _isAwake;
	bool _isCanSleep;

	Vector3 _forceAccum;
	Vector3 _torqueAccum;

public:
	void calculateDerivedData();

	void integrate(real dt);
	void clearAccumulators();

};

