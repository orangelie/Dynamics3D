#include "pch.h"
#include "Contacts.h"

void Contacts::setBodyData(RigidBody* one, RigidBody* two, real friction, real restitution)
{
	_body[0] = one;
	_body[1] = two;

	_friction = friction;
	_restitution = restitution;
}

void Contacts::calculateInternals(real dt)
{
	if(!_body[0])
		swapBodies();
	assert(_body[0]);

	calculateContactBasis();


}

void Contacts::swapBodies()
{
	_contactNormal *= -1.0f;

	RigidBody* tmp = _body[0];
	_body[0] = _body[1];
	_body[1] = tmp;
}

void Contacts::calculateContactBasis()
{
	Vector3 contactTangent[2];

	if (fabsf(_contactNormal.x) > fabsf(_contactNormal.y))
	{
		const real s = 1.0f / sqrtf(_contactNormal.z * _contactNormal.z + _contactNormal.x * _contactNormal.x);

		contactTangent[0].x = _contactNormal.z * s;
		contactTangent[0].y = 0.0f;
		contactTangent[0].z = -_contactNormal.x * s;

		contactTangent[1].x = _contactNormal.y * contactTangent[0].z;
		contactTangent[1].y = _contactNormal.z * contactTangent[0].x - _contactNormal.x * contactTangent[0].z;
		contactTangent[1].z = -_contactNormal.y * contactTangent[0].x;
	}
	else
	{
		const real s = 1.0f / sqrtf(_contactNormal.z * _contactNormal.z + _contactNormal.y * _contactNormal.y);

		contactTangent[0].x = 0.0f;
		contactTangent[0].y = -_contactNormal.z * s;
		contactTangent[0].z = _contactNormal.y * s;

		contactTangent[1].x = _contactNormal.y * contactTangent[0].z - _contactNormal.z * contactTangent[0].y;
		contactTangent[1].y = -_contactNormal.x * contactTangent[0].z;
		contactTangent[1].z = _contactNormal.x * contactTangent[0].y;
	}

	/*
	if (fabsf(_contactNormal.x) > fabsf(_contactNormal.y))
		contactTangent[1].y = 1.0f;
	else
		contactTangent[1].x = 1.0f;

	calculateOrthonormalBasis(&_contactNormal, &contactTangent[1], &contactTangent[0]);
	contactTangent[0] *= -1.0f;
	*/

	_contactToWorld.setComponents(_contactNormal, contactTangent[0], contactTangent[1]);
}

void calculateOrthonormalBasis(Vector3* x, Vector3* y, Vector3* z)
{
	// x->normalize();
	(*z) = (*x) % (*y);
	if (z->squaredMagnitude() == 0.0f)
		return;
	z->normalize();
	(*y) = (*z) % (*x);
}
