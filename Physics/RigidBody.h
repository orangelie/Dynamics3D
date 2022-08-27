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

	Vector3 _position;
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

	void setAcceleration(const Vector3& acceleration) { _acceleration = acceleration; }
	Vector3 getAcceleration() const { return _acceleration; }
	Vector3 getLastFrameAcceleration() const { return _lastFrameAcceleration; }

	void setRotation(const Vector3& rotation) { _rotation = rotation; }
	Vector3 getRotation() const { return _rotation; }

	void setVelocity(const Vector3& velocity) { _velocity = velocity; }
	Vector3 getVelocity() const { return _velocity; }

	void setOrientation(const Quaternion& orientation) { _orientation = orientation; }
	Quaternion getOrientation() const { return _orientation; }

	void setPosition(const Vector3& position) { _position = position; }
	Vector3 getPosition() const { return _position; }

	Matrix3 getInverseInertiaTensorWorld() const { return _inverseInertiaTensorWorld; }

	void getOTransform(float matrix[16]) const;

	void addVelocity(const Vector3& velocity);
	void addRotation(const Vector3& rotation);

	void setMass(const real mass) { _inverseMass = 1.0f / mass; }
	real getInverseMass() const { return _inverseMass; }

	void setInertiaTensor(const Matrix3& inertiaTensor) { _inverseInertiaTensor.setInverse(inertiaTensor); }

	void setDamping(const real linear, const real angular) { _linearDamping = linear; _angularDamping = angular; }

	void setAwake(const bool awake = true);
	bool getAwake() const { return _isAwake; }

	void setCanSleep(const bool canSleep);

};

