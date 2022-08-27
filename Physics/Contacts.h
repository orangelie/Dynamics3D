#pragma once

void calculateOrthonormalBasis(Vector3* x, Vector3* y, Vector3* z);

class Contacts
{
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

protected:
	Matrix3 _contactToWorld;

	Vector3 _contactVelocity;
	Vector3 _relativeContactPosition[2];

	real _desiredDeltaVelocity;

protected:


};

