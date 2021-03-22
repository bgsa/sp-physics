#include "OpenCLBase.cl"
#include "DOP18.cl"
#include "SpRigidBody3D.cl"

__kernel void handleCollision(
    __constant sp_uint * indexes,
	__constant sp_uint * indexesLength,
    __global   sp_float* rigidBodies3D,
    __global   sp_uint * outputIndexesLength,
    __global   sp_uint * outputIndexes
)
{
    if (THREAD_ID + 1u > divideBy2(*indexesLength))
        return;

    const sp_uint index1 = indexes[multiplyBy2(THREAD_ID)    ];
    const sp_uint index2 = indexes[multiplyBy2(THREAD_ID) + 1];
    
    const sp_uint physicIndex1 = index1 * SP_RIGID_BODY_3D_SIZE;
    const sp_uint physicIndex2 = index2 * SP_RIGID_BODY_3D_SIZE;

    const sp_bool isStaticObj1 = SpRigidBody3D_isStatic(rigidBodies3D, physicIndex1);
    const sp_bool isStaticObj2 = SpRigidBody3D_isStatic(rigidBodies3D, physicIndex2);

    if (isStaticObj1 && isStaticObj2) // if the objects are static, ignre them
        return;

    const sp_bool isRestingObj1 = SpRigidBody3D_isResting(rigidBodies3D, physicIndex1);
    const sp_bool isRestingObj2 = SpRigidBody3D_isResting(rigidBodies3D, physicIndex2);

    const Vec3 VEC3_ZERO = { 0.0f, 0.0f, 0.0f };

    if (isRestingObj1 && isRestingObj2)
    {
        if (!isStaticObj1 && !isStaticObj2)
            return;

        const Vec3 previousPositionObj1 = SpRigidBody3D_getPreviousPosition(rigidBodies3D, physicIndex1); 
        const Quat previousOrientationObj1 = SpRigidBody3D_getPreviousOrientation(rigidBodies3D, physicIndex1);

        SpRigidBody3D_setPosition(rigidBodies3D, physicIndex1, previousPositionObj1);
        SpRigidBody3D_setVelocity(rigidBodies3D, physicIndex1, VEC3_ZERO);
        SpRigidBody3D_setAcceleration(rigidBodies3D, physicIndex1, VEC3_ZERO);
        SpRigidBody3D_setOrientation(rigidBodies3D, physicIndex1, previousOrientationObj1);
        SpRigidBody3D_setAngVelocity(rigidBodies3D, physicIndex1, VEC3_ZERO);
        SpRigidBody3D_setTorque(rigidBodies3D, physicIndex1, VEC3_ZERO);

        const Vec3 previousPositionObj2 = SpRigidBody3D_getPreviousPosition(rigidBodies3D, physicIndex2);
        const Quat previousOrientationObj2 = SpRigidBody3D_getPreviousOrientation(rigidBodies3D, physicIndex2);

        SpRigidBody3D_setPosition(rigidBodies3D, physicIndex2, previousPositionObj2);
        SpRigidBody3D_setVelocity(rigidBodies3D, physicIndex2, VEC3_ZERO);
        SpRigidBody3D_setAcceleration(rigidBodies3D, physicIndex2, VEC3_ZERO);
        SpRigidBody3D_setOrientation(rigidBodies3D, physicIndex2, previousOrientationObj2);
        SpRigidBody3D_setAngVelocity(rigidBodies3D, physicIndex2, VEC3_ZERO);
        SpRigidBody3D_setTorque(rigidBodies3D, physicIndex2, VEC3_ZERO);

        return;
    }

    if (isStaticObj1 && isRestingObj2)
    {
        const Vec3 previousPositionObj2 = SpRigidBody3D_getPreviousPosition(rigidBodies3D, physicIndex2);
        const Quat previousOrientationObj2 = SpRigidBody3D_getPreviousOrientation(rigidBodies3D, physicIndex2);

        SpRigidBody3D_setPosition(rigidBodies3D, physicIndex2, previousPositionObj2);
        SpRigidBody3D_setVelocity(rigidBodies3D, physicIndex2, VEC3_ZERO);
        SpRigidBody3D_setAcceleration(rigidBodies3D, physicIndex2, VEC3_ZERO);
        SpRigidBody3D_setOrientation(rigidBodies3D, physicIndex2, previousOrientationObj2);
        SpRigidBody3D_setAngVelocity(rigidBodies3D, physicIndex2, VEC3_ZERO);
        SpRigidBody3D_setTorque(rigidBodies3D, physicIndex2, VEC3_ZERO);
        return;
    }

    if (isStaticObj2 && isRestingObj1)
    {
        const Vec3 previousPositionObj1 = SpRigidBody3D_getPreviousPosition(rigidBodies3D, physicIndex1); 
        const Quat previousOrientationObj1 = SpRigidBody3D_getPreviousOrientation(rigidBodies3D, physicIndex1);

        SpRigidBody3D_setPosition(rigidBodies3D, physicIndex1, previousPositionObj1);
        SpRigidBody3D_setVelocity(rigidBodies3D, physicIndex1, VEC3_ZERO);
        SpRigidBody3D_setAcceleration(rigidBodies3D, physicIndex1, VEC3_ZERO);
        SpRigidBody3D_setOrientation(rigidBodies3D, physicIndex1, previousOrientationObj1);
        SpRigidBody3D_setAngVelocity(rigidBodies3D, physicIndex1, VEC3_ZERO);
        SpRigidBody3D_setTorque(rigidBodies3D, physicIndex1, VEC3_ZERO);
        return;
    }

    const sp_uint temp = atomic_add(outputIndexesLength, 2);
    outputIndexes[temp    ] = index1;
    outputIndexes[temp + 1] = index2;
}
