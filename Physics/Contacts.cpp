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

	_relativeContactPosition[0] = _contactPoint - _body[0]->getPosition();
	if (_body[1])
		_relativeContactPosition[1] = _contactPoint - _body[1]->getPosition();

	_contactVelocity = calculateLocalVelocity(0, dt);
	if(_body[1])
		_contactVelocity -= calculateLocalVelocity(1, dt);


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

Vector3 Contacts::calculateLocalVelocity(unsigned bodyIndex, real dt)
{
	RigidBody* body = _body[bodyIndex];

	Vector3 deltaVelWorld = body->getRotation() % _relativeContactPosition[bodyIndex];
	deltaVelWorld += body->getVelocity();

	deltaVelWorld = _contactToWorld.transformTranspose(deltaVelWorld);

	Vector3 AccVelocity = body->getLastFrameAcceleration() * dt;
	AccVelocity = _contactToWorld.transformTranspose(AccVelocity);

	AccVelocity.x = 0.0f;
	deltaVelWorld += AccVelocity;

	return deltaVelWorld;
}

void Contacts::calculateDesiredDeltaVelocity(real dt)
{
	// 후에 보류

	const real velLimit = 0.25f;
	real deltaVelocity = 0.0f;

	if (_body[0]->getAwake())
		deltaVelocity += _body[0]->getLastFrameAcceleration() * dt * _contactNormal;
	if (_body[1] && _body[1]->getAwake())
		deltaVelocity -= _body[1]->getLastFrameAcceleration() * dt * _contactNormal;

	real thisRestitution = _restitution;
	if (fabsf(_contactVelocity.x) < velLimit)
		thisRestitution = 0.0f;

	_desiredDeltaVelocity = -_contactVelocity.x -thisRestitution * (_contactVelocity.x - deltaVelocity);
}

inline
Vector3 Contacts::calculateFrictionLessImpulse(Matrix3* inverseInertiaTensor)
{
	Vector3 impulseContact;

	Vector3 deltaVelWorld = _relativeContactPosition[0] % _contactNormal;	// 충격토크의 총합
	deltaVelWorld = inverseInertiaTensor[0].transform(deltaVelWorld);		// 월드좌표계에서의 각속도 변화량
	deltaVelWorld = deltaVelWorld % _relativeContactPosition[0];			// 회전유도 속도

	real deltaVelocity = deltaVelWorld * _contactNormal;					// 각운동에 관한 운동 변화량
	deltaVelocity += _body[0]->getInverseMass();							// 선운동에 관한 운동 변화량

	if (_body[1])
	{
		Vector3 deltaVelWorld2 = _relativeContactPosition[1] % _contactNormal;
		deltaVelWorld2 = inverseInertiaTensor[1].transform(deltaVelWorld2);
		deltaVelWorld2 = deltaVelWorld2 % _relativeContactPosition[1];

		deltaVelocity += deltaVelWorld2 * _contactNormal;
		deltaVelocity += _body[1]->getInverseMass();
	}

	impulseContact.x = _desiredDeltaVelocity / deltaVelocity;
	impulseContact.y = 0.0f;
	impulseContact.z = 0.0f;

	return impulseContact;
}

inline
Vector3 Contacts::calculateFrictionImpulse(Matrix3* inverseInertiaTensor)
{
	return Vector3();
}

void Contacts::applyVelocityChange(Vector3 velocityChange[2], Vector3 rotationChange[2])
{
	Matrix3 inverseInertiaTensor[2];

	inverseInertiaTensor[0] = _body[0]->getInverseInertiaTensorWorld();
	if(_body[1])
		inverseInertiaTensor[1] = _body[1]->getInverseInertiaTensorWorld();

	Vector3 impulseContact;
	if (_friction == 0.0f)
		impulseContact = calculateFrictionLessImpulse(inverseInertiaTensor);
	else
		impulseContact = calculateFrictionImpulse(inverseInertiaTensor);


	Vector3	impulse = _contactToWorld.transform(impulseContact);
	Vector3 impulsiveTorque = _relativeContactPosition[0] % impulse;

	rotationChange[0] = inverseInertiaTensor[0].transform(impulsiveTorque);
	velocityChange[0].clear();
	velocityChange[0].addScaledVector(impulse, _body[0]->getInverseMass());

	_body[0]->addVelocity(velocityChange[0]);
	_body[0]->addRotation(rotationChange[0]);

	if (_body[1])
	{
		Vector3 impulsiveTorque = impulse % _relativeContactPosition[1];

		rotationChange[1] = inverseInertiaTensor[1].transform(impulsiveTorque);
		velocityChange[1].clear();
		velocityChange[1].addScaledVector(impulse, -_body[1]->getInverseMass());

		_body[1]->addVelocity(velocityChange[1]);
		_body[1]->addRotation(rotationChange[1]);
	}
}

void Contacts::applyPositionChange(Vector3 linearChange[2], Vector3 angularChange[2], real penetration)
{
	const real angularLimit = 0.2f;

	real totalInertia = 0.0f;
	real linearInertia[2], angularInertia[2];
	real linearMove[2], angularMove[2];

	Matrix3 inverseInertiaTensor[2];

	inverseInertiaTensor[0] = _body[0]->getInverseInertiaTensorWorld();
	if(_body[1])
		inverseInertiaTensor[1] = _body[1]->getInverseInertiaTensorWorld();

	for (unsigned i = 0; i < 2; ++i) if (_body[i])
	{
		Vector3 deltaVelWorld = _relativeContactPosition[i] % _contactNormal;
		deltaVelWorld = inverseInertiaTensor[i].transform(deltaVelWorld);
		deltaVelWorld = deltaVelWorld % _relativeContactPosition[i];

		angularInertia[i] = deltaVelWorld * _contactNormal;
		linearInertia[i] = _body[i]->getInverseMass();

		totalInertia += angularInertia[i] + linearInertia[i];
	}

	for (unsigned i = 0; i < 2; ++i) if (_body[i])
	{
		real sign = (i == 0) ? 1.0f : -1.0f;
		angularMove[i] = sign * penetration * (angularInertia[i] / totalInertia);
		linearMove[i] = sign * penetration * (linearInertia[i] / totalInertia);

		Vector3 projection = _relativeContactPosition[i];
		projection.addScaledVector(_contactNormal, -_relativeContactPosition[i].scalarProduct(_contactNormal));

		real maxMagnitude = angularLimit * projection.magnitude();
		if (angularMove[i] < -maxMagnitude)
		{
			real totalMove = angularMove[i] + linearMove[i];
			angularMove[i] = -maxMagnitude;
			linearMove[i] = totalMove - angularMove[i];
		}
		else if (angularMove[i] > maxMagnitude)
		{
			real totalMove = angularMove[i] + linearMove[i];
			angularMove[i] = maxMagnitude;
			linearMove[i] = totalMove - angularMove[i];
		}

		if (angularMove[i] == 0.0f)
			angularChange[i].clear();
		else
		{
			Vector3 deltaVelWorld = _relativeContactPosition[i] % _contactNormal;
			angularChange[i] = inverseInertiaTensor[i].transform(deltaVelWorld) * (angularMove[i] / angularInertia[i]);
		}

		linearChange[i] = _contactNormal * linearMove[i];


		Vector3 position = _body[i]->getPosition();
		position.addScaledVector(_contactNormal, linearMove[i]);
		_body[i]->setPosition(position);

		Quaternion orientation = _body[i]->getOrientation();
		orientation.addScaledVector(angularChange[i], 1.0f);
		_body[i]->setOrientation(orientation);

		if (!_body[i]->getAwake())
			_body[i]->calculateDerivedData();
	}
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
