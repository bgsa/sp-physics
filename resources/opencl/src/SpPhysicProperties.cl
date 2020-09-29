#include "OpenCLBase.cl"
#include "Vec3.cl"
#include "Quat.cl"

#define SP_PHYSIC_GRAVITY_FORCE { 0.0f, -9.8f, 0.0f }
#define SP_PHYSIC_DRAG_FORCE (0.1f)
#define SP_PHYSIC_RESTING_VELOCITY (0.09f)

#define SP_PHYSIC_PROPERTY_SIZE                 (59)
#define SP_PHYSIC_PROPERTY_INV_MASS_INDEX       (0)
#define SP_PHYSIC_PROPERTY_DAMPING_INDEX        (1)
#define SP_PHYSIC_PROPERTY_ANG_DAMPING_INDEX    (2)
#define SP_PHYSIC_PROPERTY_COR_INDEX            (3)
#define SP_PHYSIC_PROPERTY_COF_INDEX            (4)
#define SP_PHYSIC_PROPERTY_INTEGRATE_TIME_INDEX (5)
#define SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX   (6)
#define SP_PHYSIC_PROPERTY_POSITION_INDEX       (14)
#define SP_PHYSIC_PROPERTY_VELOCITY_INDEX       (17)
#define SP_PHYSIC_PROPERTY_ACCELERATION_INDEX   (20)
#define SP_PHYSIC_PROPERTY_FORCE_INDEX          (23)
#define SP_PHYSIC_PROPERTY_ORIENTATION_INDEX    (26)
#define SP_PHYSIC_PROPERTY_ANG_VELOCITY_INDEX   (30)
#define SP_PHYSIC_PROPERTY_TORQUE_INDEX         (33)
#define SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX  (36)
#define SP_PHYSIC_PROPERTY_PREV_VEL_INDEX       (39)
#define SP_PHYSIC_PROPERTY_PREV_ACC_INDEX       (42)
#define SP_PHYSIC_PROPERTY_PREV_FORCE_INDEX     (45)
#define SP_PHYSIC_PROPERTY_PREV_ORIENTAT_INDEX  (48)
#define SP_PHYSIC_PROPERTY_PREV_ANG_VEL_INDEX   (52)
#define SP_PHYSIC_PROPERTY_PREV_TORQUE_INDEX    (56)


#define SpPhysicProperties_isStatic(properties, stride) \
    properties[stride + SP_PHYSIC_PROPERTY_INV_MASS_INDEX] == ZERO_FLOAT


#define SpPhysicProperties_getPosition(properties, stride) {    \
    properties[stride + SP_PHYSIC_PROPERTY_POSITION_INDEX    ], \
    properties[stride + SP_PHYSIC_PROPERTY_POSITION_INDEX + 1], \
    properties[stride + SP_PHYSIC_PROPERTY_POSITION_INDEX + 2]  \
}
#define SpPhysicProperties_setPosition(properties, stride, newPosition) \
    properties[stride + SP_PHYSIC_PROPERTY_POSITION_INDEX    ] = newPosition.x; \
    properties[stride + SP_PHYSIC_PROPERTY_POSITION_INDEX + 1] = newPosition.y; \
    properties[stride + SP_PHYSIC_PROPERTY_POSITION_INDEX + 2] = newPosition.z;

#define SpPhysicProperties_getPreviousPosition(properties, stride) {    \
    properties[stride + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX    ], \
    properties[stride + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX + 1], \
    properties[stride + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX + 2]  \
}

#define SpPhysicProperties_getVelocity(properties, stride) {    \
    properties[stride + SP_PHYSIC_PROPERTY_VELOCITY_INDEX    ], \
    properties[stride + SP_PHYSIC_PROPERTY_VELOCITY_INDEX + 1], \
    properties[stride + SP_PHYSIC_PROPERTY_VELOCITY_INDEX + 2]  \
}
#define SpPhysicProperties_setVelocity(properties, stride, newVelocity) \
    properties[stride + SP_PHYSIC_PROPERTY_VELOCITY_INDEX    ] = newVelocity.x; \
    properties[stride + SP_PHYSIC_PROPERTY_VELOCITY_INDEX + 1] = newVelocity.y; \
    properties[stride + SP_PHYSIC_PROPERTY_VELOCITY_INDEX + 2] = newVelocity.z;

#define SpPhysicProperties_getPreviousVelocity(properties, stride) {    \
    properties[stride + SP_PHYSIC_PROPERTY_PREV_VEL_INDEX    ], \
    properties[stride + SP_PHYSIC_PROPERTY_PREV_VEL_INDEX + 1], \
    properties[stride + SP_PHYSIC_PROPERTY_PREV_VEL_INDEX + 2]  \
}

#define SpPhysicProperties_getAcceleration(properties, stride) {    \
    properties[stride + SP_PHYSIC_PROPERTY_ACCELERATION_INDEX    ], \
    properties[stride + SP_PHYSIC_PROPERTY_ACCELERATION_INDEX + 1], \
    properties[stride + SP_PHYSIC_PROPERTY_ACCELERATION_INDEX + 2]  \
}
#define SpPhysicProperties_setAcceleration(properties, stride, newAcceleration) \
    properties[stride + SP_PHYSIC_PROPERTY_ACCELERATION_INDEX    ] = newAcceleration.x; \
    properties[stride + SP_PHYSIC_PROPERTY_ACCELERATION_INDEX + 1] = newAcceleration.y; \
    properties[stride + SP_PHYSIC_PROPERTY_ACCELERATION_INDEX + 2] = newAcceleration.z;

#define SpPhysicProperties_getOrientation(properties, stride) {    \
    properties[stride + SP_PHYSIC_PROPERTY_ORIENTATION_INDEX    ], \
    properties[stride + SP_PHYSIC_PROPERTY_ORIENTATION_INDEX + 1], \
    properties[stride + SP_PHYSIC_PROPERTY_ORIENTATION_INDEX + 2]  \
    properties[stride + SP_PHYSIC_PROPERTY_ORIENTATION_INDEX + 3]  \
}
#define SpPhysicProperties_setOrientation(properties, stride, newOrientation) \
    properties[stride + SP_PHYSIC_PROPERTY_ORIENTATION_INDEX    ] = newOrientation.w; \
    properties[stride + SP_PHYSIC_PROPERTY_ORIENTATION_INDEX + 1] = newOrientation.x; \
    properties[stride + SP_PHYSIC_PROPERTY_ORIENTATION_INDEX + 2] = newOrientation.y; \
    properties[stride + SP_PHYSIC_PROPERTY_ORIENTATION_INDEX + 3] = newOrientation.z;

#define SpPhysicProperties_getPreviousOrientation(properties, stride) {    \
    properties[stride + SP_PHYSIC_PROPERTY_PREV_ORIENTAT_INDEX    ], \
    properties[stride + SP_PHYSIC_PROPERTY_PREV_ORIENTAT_INDEX + 1], \
    properties[stride + SP_PHYSIC_PROPERTY_PREV_ORIENTAT_INDEX + 2], \
    properties[stride + SP_PHYSIC_PROPERTY_PREV_ORIENTAT_INDEX + 3]  \
}

#define SpPhysicProperties_getAngVelocity(properties, stride) {    \
    properties[stride + SP_PHYSIC_PROPERTY_ANG_VELOCITY_INDEX    ], \
    properties[stride + SP_PHYSIC_PROPERTY_ANG_VELOCITY_INDEX + 1], \
    properties[stride + SP_PHYSIC_PROPERTY_ANG_VELOCITY_INDEX + 2]  \
}
#define SpPhysicProperties_setAngVelocity(properties, stride, newAngVelocity) \
    properties[stride + SP_PHYSIC_PROPERTY_ANG_VELOCITY_INDEX   ] = newAngVelocity.x; \
    properties[stride + SP_PHYSIC_PROPERTY_ANG_VELOCITY_INDEX + 1] = newAngVelocity.y; \
    properties[stride + SP_PHYSIC_PROPERTY_ANG_VELOCITY_INDEX + 2] = newAngVelocity.z;

#define SpPhysicProperties_getPreviousAngVelocity(properties, stride) {    \
    properties[stride + SP_PHYSIC_PROPERTY_PREV_ANG_VEL_INDEX    ], \
    properties[stride + SP_PHYSIC_PROPERTY_PREV_ANG_VEL_INDEX + 1], \
    properties[stride + SP_PHYSIC_PROPERTY_PREV_ANG_VEL_INDEX + 2]  \
}

#define SpPhysicProperties_getTorque(properties, stride) {    \
    properties[stride + SP_PHYSIC_PROPERTY_TORQUE_INDEX    ], \
    properties[stride + SP_PHYSIC_PROPERTY_TORQUE_INDEX + 1], \
    properties[stride + SP_PHYSIC_PROPERTY_TORQUE_INDEX + 2]  \
}
#define SpPhysicProperties_setTorque(properties, stride, newTorque) \
    properties[stride + SP_PHYSIC_PROPERTY_TORQUE_INDEX    ] = newTorque.x; \
    properties[stride + SP_PHYSIC_PROPERTY_TORQUE_INDEX + 1] = newTorque.y; \
    properties[stride + SP_PHYSIC_PROPERTY_TORQUE_INDEX + 2] = newTorque.z;


inline void restingAcceleration(__global sp_float* properties, const sp_uint stride, Vec3* result)
{
    const sp_float mass = properties[stride + SP_PHYSIC_PROPERTY_INV_MASS_INDEX];
    Vec3 gravityForce = SP_PHYSIC_GRAVITY_FORCE;

    const Vec3 velocity = SpPhysicProperties_getVelocity(properties, stride);

    Vec3 absoluteVelocity;
    vec3_abs(velocity, &absoluteVelocity);

    Vec3 dragForce;
    vec3_multiply_vec3(velocity, absoluteVelocity, &dragForce);
    vec3_multiply_float(dragForce, 0.5f * SP_PHYSIC_DRAG_FORCE, &dragForce);

    Vec3 restingAcc;
    vec3_minus_vec3(gravityForce, dragForce, restingAcc);
    vec3_multiply_float(restingAcc, mass, result);
}

inline sp_bool SpPhysicProperties_isResting(__global sp_float* properties, const sp_uint stride)
{
    Vec3 restingAcc;
    restingAcceleration(properties, stride, &restingAcc);

    const Vec3 acc = SpPhysicProperties_getAcceleration(properties, stride);
    const Vec3 velocity = SpPhysicProperties_getVelocity(properties, stride);
    const Vec3 prevVelocity = SpPhysicProperties_getPreviousVelocity(properties, stride);
    const Vec3 angVelocity = SpPhysicProperties_getAngVelocity(properties, stride);
    const Vec3 prevAngVelocity = SpPhysicProperties_getPreviousAngVelocity(properties, stride);

    return vec3_isCloseEnough_vec3(velocity, prevVelocity, SP_PHYSIC_RESTING_VELOCITY)
        && vec3_isCloseEnough_vec3(angVelocity, prevAngVelocity, SP_PHYSIC_RESTING_VELOCITY)
        && vec3_isCloseEnough_vec3(acc, restingAcc, SP_PHYSIC_RESTING_VELOCITY);
 }

__kernel void isResting(
    __global sp_float* properties,
    __global sp_bool * output
)
{
    const sp_uint stride = SP_PHYSIC_PROPERTY_SIZE * THREAD_ID;
    const sp_uint outputIndex = SP_PHYSIC_PROPERTY_SIZE * (THREAD_ID - THREAD_OFFSET);
 
    output[outputIndex] = SpPhysicProperties_isResting(properties, stride);
}

__kernel void isStatic(
    __global sp_float* properties,
    __global sp_bool* output
)
{
    const sp_uint stride = SP_PHYSIC_PROPERTY_SIZE * THREAD_ID;
    output[0] = SpPhysicProperties_isStatic(properties, stride);
}


__kernel void fetch(
    __global sp_float* physicProperties,
    __global   sp_float* output
)
{
    const sp_uint stride = SP_PHYSIC_PROPERTY_SIZE * THREAD_ID;
    const sp_uint outputIndex   = SP_PHYSIC_PROPERTY_SIZE * (THREAD_ID - THREAD_OFFSET);

    output[outputIndex     ] = physicProperties[stride     ];
    output[outputIndex +  1] = physicProperties[stride +  1];
    output[outputIndex +  2] = physicProperties[stride +  2];
    output[outputIndex +  3] = physicProperties[stride +  3];
    output[outputIndex +  4] = physicProperties[stride +  4];
    output[outputIndex +  5] = physicProperties[stride +  5];
    output[outputIndex +  6] = physicProperties[stride +  6];
    output[outputIndex +  7] = physicProperties[stride +  7];
    output[outputIndex +  8] = physicProperties[stride +  8];
    output[outputIndex +  9] = physicProperties[stride +  9];
    output[outputIndex + 10] = physicProperties[stride + 10];
    output[outputIndex + 11] = physicProperties[stride + 11];
    output[outputIndex + 12] = physicProperties[stride + 12];
    output[outputIndex + 13] = physicProperties[stride + 13];
    output[outputIndex + 14] = physicProperties[stride + 14];
    output[outputIndex + 15] = physicProperties[stride + 15];
    output[outputIndex + 16] = physicProperties[stride + 16];
    output[outputIndex + 17] = physicProperties[stride + 17];
    output[outputIndex + 18] = physicProperties[stride + 18];
    output[outputIndex + 19] = physicProperties[stride + 19];
    output[outputIndex + 20] = physicProperties[stride + 20];
    output[outputIndex + 21] = physicProperties[stride + 21];
    output[outputIndex + 22] = physicProperties[stride + 22];
    output[outputIndex + 23] = physicProperties[stride + 23];
    output[outputIndex + 24] = physicProperties[stride + 24];
    output[outputIndex + 25] = physicProperties[stride + 25];
    output[outputIndex + 26] = physicProperties[stride + 26];
    output[outputIndex + 27] = physicProperties[stride + 27];
    output[outputIndex + 28] = physicProperties[stride + 28];
    output[outputIndex + 29] = physicProperties[stride + 29];
    output[outputIndex + 30] = physicProperties[stride + 30];
    output[outputIndex + 31] = physicProperties[stride + 31];
    output[outputIndex + 32] = physicProperties[stride + 32];
    output[outputIndex + 33] = physicProperties[stride + 33];
    output[outputIndex + 34] = physicProperties[stride + 34];
    output[outputIndex + 35] = physicProperties[stride + 35];
    output[outputIndex + 36] = physicProperties[stride + 36];
    output[outputIndex + 37] = physicProperties[stride + 37];
    output[outputIndex + 38] = physicProperties[stride + 38];
    output[outputIndex + 39] = physicProperties[stride + 39];
    output[outputIndex + 40] = physicProperties[stride + 40];
    output[outputIndex + 41] = physicProperties[stride + 41];
    output[outputIndex + 42] = physicProperties[stride + 42];
    output[outputIndex + 43] = physicProperties[stride + 43];
    output[outputIndex + 44] = physicProperties[stride + 44];
    output[outputIndex + 45] = physicProperties[stride + 45];
    output[outputIndex + 46] = physicProperties[stride + 46];
    output[outputIndex + 47] = physicProperties[stride + 47];
    output[outputIndex + 48] = physicProperties[stride + 48];
    output[outputIndex + 49] = physicProperties[stride + 49];
    output[outputIndex + 50] = physicProperties[stride + 50];
    output[outputIndex + 51] = physicProperties[stride + 51];
    output[outputIndex + 52] = physicProperties[stride + 52];
    output[outputIndex + 53] = physicProperties[stride + 53];
    output[outputIndex + 54] = physicProperties[stride + 54];
    output[outputIndex + 55] = physicProperties[stride + 55];
    output[outputIndex + 56] = physicProperties[stride + 56];
    output[outputIndex + 57] = physicProperties[stride + 57];
}