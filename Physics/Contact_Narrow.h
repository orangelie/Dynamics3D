#pragma once


struct CollisionData
{
	Contact* contactArray;
	Contact* contacts;

	int contactsLeft;
	unsigned contactCount;

	real friction;
	real restitution;
	real tolerance;

	bool hasMoreContacts() const { return (contactsLeft > 0); }

	void addContacts(unsigned count)
	{
		contactsLeft -= count;
		contactCount += count;

		contacts += count;
	}

	void reset(unsigned maxContacts)
	{
		contactsLeft = maxContacts;
		contactCount = 0;

		contacts = contactArray;
	}
};


struct CollisionPrimitive
{
	friend class IntersectionTests;
	friend class CollisionDetector;

	RigidBody* body;
	Matrix4 offset;

	Vector3 getAxis(unsigned index) { return _transform.getAxisVector(index); }

	const Matrix4& getTransform() const { return _transform; }

	void calculateInternals();

protected:
	Matrix4 _transform;

};

struct CollisionBox : public CollisionPrimitive
{
	Vector3 halfSize;
};
