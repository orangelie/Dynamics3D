#include "pch.h"
#include "Contact_Narrow.h"

void CollisionPrimitive::calculateInternals()
{
	_transform = body->getTransform() * offset;
}
