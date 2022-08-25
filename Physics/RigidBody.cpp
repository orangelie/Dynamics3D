#include "pch.h"
#include "RigidBody.h"

void RigidBody::calculateDerivedData()
{
    _orientation.normalize();

    calculateTransformMatrix(_transformMatrix, _position, _orientation);
    transformInertiaTensor(_inverseInertiaTensorWorld, _inverseInertiaTensor, _transformMatrix);
}

void RigidBody::integrate(real dt)
{
    if (!_isAwake)
        return;

    _lastFrameAcceleration = _acceleration;
    _lastFrameAcceleration.addScaledVector(_forceAccum, _inverseMass);

    Vector3 angularAcceleration = _inverseInertiaTensorWorld.transform(_torqueAccum);

    _velocity.addScaledVector(_lastFrameAcceleration, dt);
    _rotation.addScaledVector(angularAcceleration, dt);

    _velocity *= powf(_linearDamping, dt);
    _rotation *= powf(_angularDamping, dt);

    _position.addScaledVector(_velocity, dt);
    _orientation.addScaledVector(_rotation, dt);

    calculateDerivedData();
    clearAccumulators();
}

void RigidBody::clearAccumulators()
{
    _forceAccum.clear();
    _torqueAccum.clear();
}

void RigidBody::getOTransform(float matrix[16]) const
{
    matrix[0] = (float)_transformMatrix.data[0];
    matrix[1] = (float)_transformMatrix.data[4];
    matrix[2] = (float)_transformMatrix.data[8];
    matrix[3] = 0.0f;

    matrix[4] = (float)_transformMatrix.data[1];
    matrix[5] = (float)_transformMatrix.data[5];
    matrix[6] = (float)_transformMatrix.data[9];
    matrix[7] = 0.0f;

    matrix[8] = (float)_transformMatrix.data[2];
    matrix[9] = (float)_transformMatrix.data[6];
    matrix[10] = (float)_transformMatrix.data[10];
    matrix[11] = 0.0f;

    matrix[12] = (float)_transformMatrix.data[3];
    matrix[13] = (float)_transformMatrix.data[7];
    matrix[14] = (float)_transformMatrix.data[11];
    matrix[15] = 1.0f;
}

void RigidBody::setAwake(const bool awake)
{
    _isAwake = awake;

    if (!awake)
    {
        _velocity.clear();
        _rotation.clear();
    }
}

void RigidBody::setCanSleep(const bool canSleep)
{
    _isCanSleep = canSleep;

    if (!_isCanSleep && !_isAwake)
        setAwake();
}

inline
void calculateTransformMatrix(Matrix4& transformMatrix, const Vector3& position, const Quaternion& orientation)
{
    transformMatrix.data[0] = 1 - 2 * orientation.j * orientation.j -
        2 * orientation.k * orientation.k;
    transformMatrix.data[1] = 2 * orientation.i * orientation.j -
        2 * orientation.r * orientation.k;
    transformMatrix.data[2] = 2 * orientation.i * orientation.k +
        2 * orientation.r * orientation.j;
    transformMatrix.data[3] = position.x;

    transformMatrix.data[4] = 2 * orientation.i * orientation.j +
        2 * orientation.r * orientation.k;
    transformMatrix.data[5] = 1 - 2 * orientation.i * orientation.i -
        2 * orientation.k * orientation.k;
    transformMatrix.data[6] = 2 * orientation.j * orientation.k -
        2 * orientation.r * orientation.i;
    transformMatrix.data[7] = position.y;

    transformMatrix.data[8] = 2 * orientation.i * orientation.k -
        2 * orientation.r * orientation.j;
    transformMatrix.data[9] = 2 * orientation.j * orientation.k +
        2 * orientation.r * orientation.i;
    transformMatrix.data[10] = 1 - 2 * orientation.i * orientation.i -
        2 * orientation.j * orientation.j;
    transformMatrix.data[11] = position.z;
}

inline void transformInertiaTensor(Matrix3& iitWorld, const Matrix3& iitBody, const Matrix4& rotmat)
{
    real t4 = rotmat.data[0] * iitBody.data[0] +
        rotmat.data[1] * iitBody.data[3] +
        rotmat.data[2] * iitBody.data[6];
    real t9 = rotmat.data[0] * iitBody.data[1] +
        rotmat.data[1] * iitBody.data[4] +
        rotmat.data[2] * iitBody.data[7];
    real t14 = rotmat.data[0] * iitBody.data[2] +
        rotmat.data[1] * iitBody.data[5] +
        rotmat.data[2] * iitBody.data[8];
    real t28 = rotmat.data[4] * iitBody.data[0] +
        rotmat.data[5] * iitBody.data[3] +
        rotmat.data[6] * iitBody.data[6];
    real t33 = rotmat.data[4] * iitBody.data[1] +
        rotmat.data[5] * iitBody.data[4] +
        rotmat.data[6] * iitBody.data[7];
    real t38 = rotmat.data[4] * iitBody.data[2] +
        rotmat.data[5] * iitBody.data[5] +
        rotmat.data[6] * iitBody.data[8];
    real t52 = rotmat.data[8] * iitBody.data[0] +
        rotmat.data[9] * iitBody.data[3] +
        rotmat.data[10] * iitBody.data[6];
    real t57 = rotmat.data[8] * iitBody.data[1] +
        rotmat.data[9] * iitBody.data[4] +
        rotmat.data[10] * iitBody.data[7];
    real t62 = rotmat.data[8] * iitBody.data[2] +
        rotmat.data[9] * iitBody.data[5] +
        rotmat.data[10] * iitBody.data[8];

    iitWorld.data[0] = t4 * rotmat.data[0] +
        t9 * rotmat.data[1] +
        t14 * rotmat.data[2];
    iitWorld.data[1] = t4 * rotmat.data[4] +
        t9 * rotmat.data[5] +
        t14 * rotmat.data[6];
    iitWorld.data[2] = t4 * rotmat.data[8] +
        t9 * rotmat.data[9] +
        t14 * rotmat.data[10];
    iitWorld.data[3] = t28 * rotmat.data[0] +
        t33 * rotmat.data[1] +
        t38 * rotmat.data[2];
    iitWorld.data[4] = t28 * rotmat.data[4] +
        t33 * rotmat.data[5] +
        t38 * rotmat.data[6];
    iitWorld.data[5] = t28 * rotmat.data[8] +
        t33 * rotmat.data[9] +
        t38 * rotmat.data[10];
    iitWorld.data[6] = t52 * rotmat.data[0] +
        t57 * rotmat.data[1] +
        t62 * rotmat.data[2];
    iitWorld.data[7] = t52 * rotmat.data[4] +
        t57 * rotmat.data[5] +
        t62 * rotmat.data[6];
    iitWorld.data[8] = t52 * rotmat.data[8] +
        t57 * rotmat.data[9] +
        t62 * rotmat.data[10];
}
