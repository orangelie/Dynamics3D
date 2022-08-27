#pragma once

void calculateOrthonormalBasis(Vector3* x, Vector3* y, Vector3* z);

class ContactResolver;

class Contacts
{
private:
	friend class ContactResolver;

public:
	RigidBody* _body[2];

	Vector3 _contactNormal;
	Vector3 _contactPoint;

	real _friction;
	real _penetration;
	real _restitution;

	void setBodyData(RigidBody* one, RigidBody* two, real friction, real restitution);
	void calculateInternals(real dt);
	void swapBodies();
	void calculateContactBasis();
	Vector3 calculateLocalVelocity(unsigned bodyIndex, real dt);
	void calculateDesiredDeltaVelocity(real dt);

	inline Vector3 calculateFrictionLessImpulse(Matrix3* inverseInertiaTensor);
	inline Vector3 calculateFrictionImpulse(Matrix3* inverseInertiaTensor);
	void applyVelocityChange(Vector3 velocityChange[2], Vector3 rotationChange[2]);
	void applyPositionChange(Vector3 linearChange[2], Vector3 angularChange[2], real penetration);

protected:
	Matrix3 _contactToWorld;

	Vector3 _contactVelocity;
	Vector3 _relativeContactPosition[2];

	real _desiredDeltaVelocity;

protected:


};


class ContactResolver
{
protected:
	unsigned _velocityIteration;
	unsigned _positionIteration;

	real _velocityEpsilon;
	real _positionEpsilon;

public:
	unsigned _velocityIterationUsed;
	unsigned _positionIterationUsed;

private:
	bool _validSettings;

public:


};
