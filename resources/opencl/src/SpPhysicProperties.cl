#include "OpenCLBase.cl"
#include "Vec3.cl"

#define SP_PHYSIC_GRAVITY_FORCE { 0.0f, -9.8f, 0.0f }
#define SP_PHYSIC_DRAG_FORCE (0.1f)
#define SP_PHYSIC_RESTING_VELOCITY (0.09f)

#define SP_PHYSIC_PROPERTY_SIZE                (52)
#define SP_PHYSIC_PROPERTY_POSITION_INDEX      (0)
#define SP_PHYSIC_PROPERTY_VELOCITY_INDEX      (3)
#define SP_PHYSIC_PROPERTY_ACCELERATION_INDEX  (6)
#define SP_PHYSIC_PROPERTY_FORCE_INDEX         (9)
#define SP_PHYSIC_PROPERTY_ORIENTATION_INDEX   (12)
#define SP_PHYSIC_PROPERTY_TORQUE_INDEX        (16)
#define SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX (19)
#define SP_PHYSIC_PROPERTY_PREV_VEL_INDEX      (22)
#define SP_PHYSIC_PROPERTY_PREV_ACC_INDEX      (25)
#define SP_PHYSIC_PROPERTY_PREV_FORCE_INDEX    (28)
#define SP_PHYSIC_PROPERTY_PREV_ORIENTAT_INDEX (31)
#define SP_PHYSIC_PROPERTY_PREV_TORQUE_INDEX   (35)
#define SP_PHYSIC_PROPERTY_INV_MASS_INDEX      (38) // OK!!
#define SP_PHYSIC_PROPERTY_DAMPING_INDEX       (39)
#define SP_PHYSIC_PROPERTY_ANG_DAMPING_INDEX   (40)
#define SP_PHYSIC_PROPERTY_COR_INDEX           (41)
#define SP_PHYSIC_PROPERTY_COF_INDEX           (42)
#define SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX  (43)

inline void restingAcceleration(__global sp_float* properties, const sp_uint stride, Vec3* result)
{
    const sp_float mass = properties[stride + SP_PHYSIC_PROPERTY_INV_MASS_INDEX];
    Vec3 gravityForce = SP_PHYSIC_GRAVITY_FORCE;

    const Vec3 velocity = {
        properties[stride + SP_PHYSIC_PROPERTY_VELOCITY_INDEX],
        properties[stride + SP_PHYSIC_PROPERTY_VELOCITY_INDEX + 1],
        properties[stride + SP_PHYSIC_PROPERTY_VELOCITY_INDEX + 2]
    };

    Vec3 absoluteVelocity;
    vec3_abs(velocity, &absoluteVelocity);

    Vec3 dragForce;
    vec3_multiply_vec3(velocity, absoluteVelocity, &dragForce);
    vec3_multiply_float(dragForce, 0.5f * SP_PHYSIC_DRAG_FORCE, &dragForce);

    Vec3 restingAcc;
    vec3_minus_vec3(gravityForce, dragForce, &restingAcc);
    vec3_multiply_float(restingAcc, mass, result);
}

inline sp_bool SpPhysicProperties_isStatic(__global sp_float* properties, const sp_uint stride)
{
    return properties[stride + SP_PHYSIC_PROPERTY_INV_MASS_INDEX] == ZERO_FLOAT;
}

inline sp_bool SpPhysicProperties_isResting(__global sp_float* properties, const sp_uint stride)
{
    const sp_uint positionIndex = stride + SP_PHYSIC_PROPERTY_POSITION_INDEX;
    const sp_uint previousPpositionIndex = stride + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX;
    const sp_uint accIndex = stride + SP_PHYSIC_PROPERTY_ACCELERATION_INDEX;
    const sp_uint prevAccIndex = stride + SP_PHYSIC_PROPERTY_PREV_ACC_INDEX;

    const Vec3 acc = {
        properties[accIndex    ],
        properties[accIndex + 1],
        properties[accIndex + 2],
    };

    Vec3 restingAcc;
    restingAcceleration(properties, stride, &restingAcc);

    return
        isCloseEnough(properties[positionIndex  ], properties[previousPpositionIndex   ], SP_PHYSIC_RESTING_VELOCITY) &&
        isCloseEnough(properties[positionIndex+1], properties[previousPpositionIndex +1], SP_PHYSIC_RESTING_VELOCITY) &&
        isCloseEnough(properties[positionIndex+2], properties[previousPpositionIndex +2], SP_PHYSIC_RESTING_VELOCITY) &&
        vec3_isCloseEnough_vec3(acc, restingAcc, SP_PHYSIC_RESTING_VELOCITY);
 }

inline sp_bool SpPhysicProperties_areMovingAway(__global sp_float* properties, const sp_uint strideObj1, const sp_uint strideObj2)
{
    const Vec3 positionObj1 = {
        properties[strideObj1 + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX],
        properties[strideObj1 + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX + 1],
        properties[strideObj1 + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX + 2]
    };

    const Vec3 velocityObj1 = {
        properties[strideObj1 + SP_PHYSIC_PROPERTY_VELOCITY_INDEX],
        properties[strideObj1 + SP_PHYSIC_PROPERTY_VELOCITY_INDEX + 1],
        properties[strideObj1 + SP_PHYSIC_PROPERTY_VELOCITY_INDEX + 2]
    };

    const Vec3 positionObj2 = {
        properties[strideObj2 + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX],
        properties[strideObj2 + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX + 1],
        properties[strideObj2 + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX + 2]
    };

    const Vec3 velocityObj2 = {
        properties[strideObj2 + SP_PHYSIC_PROPERTY_VELOCITY_INDEX],
        properties[strideObj2 + SP_PHYSIC_PROPERTY_VELOCITY_INDEX + 1],
        properties[strideObj2 + SP_PHYSIC_PROPERTY_VELOCITY_INDEX + 2]
    };

    Vec3 lineOfAction;
    vec3_minus_vec3(positionObj2, positionObj1, &lineOfAction);
    
    Vec3 velocityToObject2;
    vec3_multiply_vec3(velocityObj1, lineOfAction, &velocityToObject2);

    vec3_minus_vec3(positionObj1, positionObj2, &lineOfAction);
    
    Vec3 velocityToObject1;
    vec3_multiply_vec3(velocityObj2, lineOfAction, &velocityToObject1);

    return
           vec3_lesserThanOrEqual_float(velocityToObject2, ZERO_FLOAT)
        && vec3_lesserThanOrEqual_float(velocityToObject1, ZERO_FLOAT);
}

#define SpPhysicProperties_setPosition(properties, stride, newPosition) \
    properties[stride + SP_PHYSIC_PROPERTY_POSITION_INDEX    ] = newPosition.x; \
    properties[stride + SP_PHYSIC_PROPERTY_POSITION_INDEX + 1] = newPosition.y; \
    properties[stride + SP_PHYSIC_PROPERTY_POSITION_INDEX + 2] = newPosition.z;

#define SpPhysicProperties_setVelocity(properties, stride, newVelocity) \
    properties[stride + SP_PHYSIC_PROPERTY_VELOCITY_INDEX    ] = newVelocity.x; \
    properties[stride + SP_PHYSIC_PROPERTY_VELOCITY_INDEX + 1] = newVelocity.y; \
    properties[stride + SP_PHYSIC_PROPERTY_VELOCITY_INDEX + 2] = newVelocity.z;

#define SpPhysicProperties_setAcceleration(properties, stride, newAcceleration) \
    properties[stride + SP_PHYSIC_PROPERTY_ACCELERATION_INDEX    ] = newAcceleration.x; \
    properties[stride + SP_PHYSIC_PROPERTY_ACCELERATION_INDEX + 1] = newAcceleration.y; \
    properties[stride + SP_PHYSIC_PROPERTY_ACCELERATION_INDEX + 2] = newAcceleration.z;




__kernel void isResting(
    __global sp_float* properties,
    __global sp_bool * output
)
{
    const sp_uint stride = SP_PHYSIC_PROPERTY_SIZE * THREAD_ID;
    const sp_uint outputIndex = SP_PHYSIC_PROPERTY_SIZE * (THREAD_ID - THREAD_OFFSET);
 
    output[outputIndex] = SpPhysicProperties_isResting(properties, stride);

    /* DEBUG 
    output[0] = properties[stride + SP_PHYSIC_PROPERTY_POSITION_INDEX];
    output[1] = properties[stride + SP_PHYSIC_PROPERTY_POSITION_INDEX+1];
    output[2] = properties[stride + SP_PHYSIC_PROPERTY_POSITION_INDEX+2];

    output[3] = properties[stride + SP_PHYSIC_PROPERTY_VELOCITY_INDEX];
    output[4] = properties[stride + SP_PHYSIC_PROPERTY_VELOCITY_INDEX+1];
    output[5] = properties[stride + SP_PHYSIC_PROPERTY_VELOCITY_INDEX+2];
    
    output[6] = properties[stride + SP_PHYSIC_PROPERTY_ACCELERATION_INDEX];
    output[7] = properties[stride + SP_PHYSIC_PROPERTY_ACCELERATION_INDEX+1];
    output[8] = properties[stride + SP_PHYSIC_PROPERTY_ACCELERATION_INDEX+2];

    output[9] = properties[stride + SP_PHYSIC_PROPERTY_FORCE_INDEX];
    output[10] = properties[stride + SP_PHYSIC_PROPERTY_FORCE_INDEX+1];
    output[11] = properties[stride + SP_PHYSIC_PROPERTY_FORCE_INDEX+2];

    output[12] = properties[stride + SP_PHYSIC_PROPERTY_ORIENTATION_INDEX];
    output[13] = properties[stride + SP_PHYSIC_PROPERTY_ORIENTATION_INDEX+1];
    output[14] = properties[stride + SP_PHYSIC_PROPERTY_ORIENTATION_INDEX+2];
    output[15] = properties[stride + SP_PHYSIC_PROPERTY_ORIENTATION_INDEX+3];

    output[16] = properties[stride + SP_PHYSIC_PROPERTY_TORQUE_INDEX];
    output[17] = properties[stride + SP_PHYSIC_PROPERTY_TORQUE_INDEX+1];
    output[18] = properties[stride + SP_PHYSIC_PROPERTY_TORQUE_INDEX+2];

    output[19] = properties[stride + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX];
    output[20] = properties[stride + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX + 1];
    output[21] = properties[stride + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX + 2];

    output[22] = properties[stride + SP_PHYSIC_PROPERTY_PREV_VEL_INDEX];
    output[23] = properties[stride + SP_PHYSIC_PROPERTY_PREV_VEL_INDEX + 1];
    output[24] = properties[stride + SP_PHYSIC_PROPERTY_PREV_VEL_INDEX + 2];

    output[25] = properties[stride + SP_PHYSIC_PROPERTY_PREV_ACC_INDEX];
    output[26] = properties[stride + SP_PHYSIC_PROPERTY_PREV_ACC_INDEX + 1];
    output[27] = properties[stride + SP_PHYSIC_PROPERTY_PREV_ACC_INDEX + 2];

    output[28] = properties[stride + SP_PHYSIC_PROPERTY_PREV_FORCE_INDEX];
    output[29] = properties[stride + SP_PHYSIC_PROPERTY_PREV_FORCE_INDEX + 1];
    output[30] = properties[stride + SP_PHYSIC_PROPERTY_PREV_FORCE_INDEX + 2];

    output[31] = properties[stride + SP_PHYSIC_PROPERTY_PREV_ORIENTAT_INDEX];
    output[32] = properties[stride + SP_PHYSIC_PROPERTY_PREV_ORIENTAT_INDEX + 1];
    output[33] = properties[stride + SP_PHYSIC_PROPERTY_PREV_ORIENTAT_INDEX + 2];
    output[34] = properties[stride + SP_PHYSIC_PROPERTY_PREV_ORIENTAT_INDEX + 3];

    output[35] = properties[stride + SP_PHYSIC_PROPERTY_PREV_TORQUE_INDEX];
    output[36] = properties[stride + SP_PHYSIC_PROPERTY_PREV_TORQUE_INDEX + 1];
    output[37] = properties[stride + SP_PHYSIC_PROPERTY_PREV_TORQUE_INDEX + 2];

    output[38] = properties[stride + SP_PHYSIC_PROPERTY_INV_MASS_INDEX];
    output[39] = properties[stride + SP_PHYSIC_PROPERTY_DAMPING_INDEX];
    output[40] = properties[stride + SP_PHYSIC_PROPERTY_ANG_DAMPING_INDEX];
    output[41] = properties[stride + SP_PHYSIC_PROPERTY_COR_INDEX];
    output[42] = properties[stride + SP_PHYSIC_PROPERTY_COF_INDEX];
    
    output[43] = properties[stride + SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX    ];
    output[44] = properties[stride + SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 1];
    output[45] = properties[stride + SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 2];
    output[46] = properties[stride + SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 3];
    output[47] = properties[stride + SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 4];
    output[48] = properties[stride + SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 5];
    output[49] = properties[stride + SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 6];
    output[50] = properties[stride + SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 7];
    output[51] = properties[stride + SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 8];
    */
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
}