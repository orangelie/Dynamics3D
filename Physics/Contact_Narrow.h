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

public:
	RigidBody* body;
	Matrix4 offset;

	Vector3 getAxis(unsigned index) const { return _transform.getAxisVector(index); }

	const Matrix4& getTransform() const { return _transform; }

	void calculateInternals();

protected:
	Matrix4 _transform;

};

struct CollisionBox : public CollisionPrimitive
{
	Vector3 halfSize;
};

struct CollisionSphere : public CollisionPrimitive
{
	real radius;
};

struct CollisionPlane
{
	Vector3 direction;
	real offset;
};


inline static bool tryAxis(
	const CollisionBox& one,
	const CollisionBox& two,
	Vector3 axis,
	const Vector3& toCentre,
	unsigned index,
	real& smallestPenetration,
	unsigned& smallestCase);

inline static real penetrationOnAxis(
	const CollisionBox& one,
	const CollisionBox& two,
	const Vector3& axis,
	const Vector3& toCentre);

inline static real transformToAxis(
	const CollisionBox& box,
	const Vector3& axis);

static void fillPointFaceBoxBox(
	const CollisionBox& one,
	const CollisionBox& two,
	const Vector3& toCentre,
	CollisionData* data,
	unsigned best,
	real pen);

static inline Vector3 contactPoint(
	const Vector3& pOne,
	const Vector3& dOne,
	real oneSize,
	const Vector3& pTwo,
	const Vector3& dTwo,
	real twoSize,

	// If this is true, and the contact point is outside
	// the edge (in the case of an edge-face contact) then
	// we use one's midpoint, otherwise we use two's.
	bool useOne);

class CollisionDetector
{
public:

	static unsigned boxAndBox(const CollisionBox& one, const CollisionBox& two, CollisionData* data);

};
