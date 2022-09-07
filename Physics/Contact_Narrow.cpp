#include "pch.h"
#include "Contact_Narrow.h"

void CollisionPrimitive::calculateInternals()
{
	_transform = body->getTransform() * offset;
}


#define CHECK_OVERLAP(axis, index) if(!tryAxis(one, two, (axis), toCentre, (index), pen, best)) return 0;

unsigned CollisionDetector::boxAndBox(const CollisionBox& one, const CollisionBox& two, CollisionData* data)
{
	Vector3 toCentre = two.getAxis(3) - one.getAxis(3);

	real pen = FLT_MAX;
	unsigned best = 0xffffffff;

	CHECK_OVERLAP(one.getAxis(0), 0);
	CHECK_OVERLAP(one.getAxis(1), 1);
	CHECK_OVERLAP(one.getAxis(2), 2);

	CHECK_OVERLAP(two.getAxis(0), 3);
	CHECK_OVERLAP(two.getAxis(1), 4);
	CHECK_OVERLAP(two.getAxis(2), 5);

	unsigned bestSingleAxis = best;

	CHECK_OVERLAP(one.getAxis(0) % two.getAxis(0), 6);
	CHECK_OVERLAP(one.getAxis(0) % two.getAxis(1), 7);
	CHECK_OVERLAP(one.getAxis(0) % two.getAxis(2), 8);
	CHECK_OVERLAP(one.getAxis(1) % two.getAxis(0), 9);
	CHECK_OVERLAP(one.getAxis(1) % two.getAxis(1), 10);
	CHECK_OVERLAP(one.getAxis(1) % two.getAxis(2), 11);
	CHECK_OVERLAP(one.getAxis(2) % two.getAxis(0), 12);
	CHECK_OVERLAP(one.getAxis(2) % two.getAxis(1), 13);
	CHECK_OVERLAP(one.getAxis(2) % two.getAxis(2), 14);

	assert(best != 0xffffffff);


	/* one상자 평면에 two상자 꼭짓점이 있음 */
	if (best < 3)
	{
		fillPointFaceBoxBox(one, two, toCentre, data, best, pen);
		data->addContacts(1);

		return 1;
	}
	if (best < 6)
	{
		fillPointFaceBoxBox(two, one, toCentre * -1.0f, data, best - 3, pen);
		data->addContacts(1);

		return 1;
	}
	else
	{
		//
		best -= 6;
		unsigned oneAxisIndex = best / 3;
		unsigned twoAxisIndex = best % 3;
		Vector3 oneAxis = one.getAxis(oneAxisIndex);
		Vector3 twoAxis = two.getAxis(twoAxisIndex);
		Vector3 axis = oneAxis % twoAxis;
		axis.normalize();

		// The axis should point from box one to box two.
		if (axis * toCentre > 0) axis = axis * -1.0f;

		// We have the axes, but not the edges: each axis has 4 edges parallel
		// to it, we need to find which of the 4 for each object. We do
		// that by finding the point in the centre of the edge. We know
		// its component in the direction of the box's collision axis is zero
		// (its a mid-point) and we determine which of the extremes in each
		// of the other axes is closest.
		Vector3 ptOnOneEdge = one.halfSize;
		Vector3 ptOnTwoEdge = two.halfSize;
		for (unsigned i = 0; i < 3; i++)
		{
			if (i == oneAxisIndex) ptOnOneEdge[i] = 0;
			else if (one.getAxis(i) * axis > 0) ptOnOneEdge[i] = -ptOnOneEdge[i];

			if (i == twoAxisIndex) ptOnTwoEdge[i] = 0;
			else if (two.getAxis(i) * axis < 0) ptOnTwoEdge[i] = -ptOnTwoEdge[i];
		}

		// Move them into world coordinates (they are already oriented
		// correctly, since they have been derived from the axes).
		ptOnOneEdge = one.getTransform() * ptOnOneEdge;
		ptOnTwoEdge = two.getTransform() * ptOnTwoEdge;

		// So we have a point and a direction for the colliding edges.
		// We need to find out point of closest approach of the two
		// line-segments.
		Vector3 vertex = contactPoint(
			ptOnOneEdge, oneAxis, one.halfSize[oneAxisIndex],
			ptOnTwoEdge, twoAxis, two.halfSize[twoAxisIndex],
			bestSingleAxis > 2
		);

		// We can fill the contact.
		Contact* contact = data->contacts;

		contact->_penetration = pen;
		contact->_contactNormal = axis;
		contact->_contactPoint = vertex;
		contact->setBodyData(one.body, two.body,
			data->friction, data->restitution);
		data->addContacts(1);
		//

		return 1;
	}

	return 0;
}

#undef CHECK_OVERLAP

inline bool tryAxis(
	const CollisionBox& one,
	const CollisionBox& two,
	Vector3 axis,
	const Vector3& toCentre,
	unsigned index,
	real& smallestPenetration,
	unsigned& smallestCase)
{
	if (axis.squaredMagnitude() < 0.0001f)
		return true;
	axis.normalize();

	real penetration = penetrationOnAxis(one, two, axis, toCentre);

	if (penetration < 0.0f)
		return false;

	if (penetration < smallestPenetration)
	{
		smallestPenetration = penetration;
		smallestCase = index;
	}

	return true;
}

inline real penetrationOnAxis(
	const CollisionBox& one,
	const CollisionBox& two,
	const Vector3& axis,
	const Vector3& toCentre)
{
	real oneProject = transformToAxis(one, axis);
	real twoProject = transformToAxis(two, axis);

	real distance = fabsf(toCentre * axis);

	return oneProject + twoProject - distance;
}

inline real transformToAxis(const CollisionBox& box, const Vector3& axis)
{
	return
		box.halfSize.x * fabsf(axis * box.getAxis(0)) + 
		box.halfSize.y * fabsf(axis * box.getAxis(1)) +
		box.halfSize.z * fabsf(axis * box.getAxis(2));
}

void fillPointFaceBoxBox(const CollisionBox& one, const CollisionBox& two, const Vector3& toCentre, CollisionData* data, unsigned best, real pen)
{
	Contact* contact = data->contacts;

	Vector3 normal = one.getAxis(best);
	if (one.getAxis(best) * toCentre > 0.0f)
		normal *= -1.0f;

	Vector3 vertex = two.halfSize;
	if (two.getAxis(0) * normal < 0.0f)
		vertex.x = -vertex.x;
	if (two.getAxis(1) * normal < 0.0f)
		vertex.y = -vertex.y;
	if (two.getAxis(2) * normal < 0.0f)
		vertex.z = -vertex.z;


	contact->_contactNormal = normal;
	contact->_contactPoint = two.getTransform() * vertex;
	contact->_penetration = pen;
	contact->setBodyData(one.body, two.body, data->friction, data->restitution);
}

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
	bool useOne)
{
	Vector3 toSt, cOne, cTwo;
	real dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo;
	real denom, mua, mub;

	smOne = dOne.squaredMagnitude();
	smTwo = dTwo.squaredMagnitude();
	dpOneTwo = dTwo * dOne;

	toSt = pOne - pTwo;
	dpStaOne = dOne * toSt;
	dpStaTwo = dTwo * toSt;

	denom = smOne * smTwo - dpOneTwo * dpOneTwo;

	// Zero denominator indicates parrallel lines
	if (fabsf(denom) < 0.0001f) {
		return useOne ? pOne : pTwo;
	}

	mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denom;
	mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denom;

	// If either of the edges has the nearest point out
	// of bounds, then the edges aren't crossed, we have
	// an edge-face contact. Our point is on the edge, which
	// we know from the useOne parameter.
	if (mua > oneSize ||
		mua < -oneSize ||
		mub > twoSize ||
		mub < -twoSize)
	{
		return useOne ? pOne : pTwo;
	}
	else
	{
		cOne = pOne + dOne * mua;
		cTwo = pTwo + dTwo * mub;

		return cOne * 0.5 + cTwo * 0.5;
	}
}

