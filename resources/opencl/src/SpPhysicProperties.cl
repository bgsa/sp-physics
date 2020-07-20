#include "OpenCLBase.cl"

#define SP_PHYSIC_RESTING_VELOCITY (0.09f)

#define SP_PHYSIC_PROPERTY_SIZE               (52)
#define SP_PHYSIC_PROPERTY_POSITION_INDEX     (0)
#define SP_PHYSIC_PROPERTY_VELOCITY_INDEX     (3)
#define SP_PHYSIC_PROPERTY_ACCELERATION_INDEX (6)
#define SP_PHYSIC_PROPERTY_FORCE_INDEX        (9)
#define SP_PHYSIC_PROPERTY_ORIENTATION_INDEX  (12)
#define SP_PHYSIC_PROPERTY_TORQUE_INDEX       (15)

#define SP_PHYSIC_PROPERTY_INV_MASS_INDEX     (36)
#define SP_PHYSIC_PROPERTY_DAMPING_INDEX      (37)
#define SP_PHYSIC_PROPERTY_ANG_DAMPING_INDEX  (38)
#define SP_PHYSIC_PROPERTY_COR_INDEX          (39)
#define SP_PHYSIC_PROPERTY_COF_INDEX          (40)

#define SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX (41)


inline sp_bool SpPhysicProperties_isStatic(__global sp_float* properties, const sp_uint index)
{
    return properties[(SP_PHYSIC_PROPERTY_SIZE * index) + SP_PHYSIC_PROPERTY_INV_MASS_INDEX];
}

inline sp_bool SpPhysicProperties_isResting(__global sp_float* properties, const sp_uint index)
{
    return false;
    /*
    return
        isCloseEnough(_position.x, _previousPosition.x, restingEpsilon) &&
        isCloseEnough(_position.y, _previousPosition.y, restingEpsilon) &&
        isCloseEnough(_position.z, _previousPosition.z, restingEpsilon) &&
        isCloseEnough(_acceleration.x, _restingAcceleration.x, restingEpsilon) &&
        isCloseEnough(_acceleration.y, _restingAcceleration.y, restingEpsilon) &&
        isCloseEnough(_acceleration.z, _restingAcceleration.z, restingEpsilon);
        */
}


__kernel void fetch(
    __constant sp_float* physicProperties,
    __global   sp_float* output
)
{
    const sp_uint propertyIndex = SP_PHYSIC_PROPERTY_SIZE * THREAD_ID;
    const sp_uint outputIndex   = SP_PHYSIC_PROPERTY_SIZE * (THREAD_ID - THREAD_OFFSET);

    output[outputIndex     ] = physicProperties[propertyIndex     ];
    output[outputIndex +  1] = physicProperties[propertyIndex +  1];
    output[outputIndex +  2] = physicProperties[propertyIndex +  2];
    output[outputIndex +  3] = physicProperties[propertyIndex +  3];
    output[outputIndex +  4] = physicProperties[propertyIndex +  4];
    output[outputIndex +  5] = physicProperties[propertyIndex +  5];
    output[outputIndex +  6] = physicProperties[propertyIndex +  6];
    output[outputIndex +  7] = physicProperties[propertyIndex +  7];
    output[outputIndex +  8] = physicProperties[propertyIndex +  8];
    output[outputIndex +  9] = physicProperties[propertyIndex +  9];
    output[outputIndex + 10] = physicProperties[propertyIndex + 10];
    output[outputIndex + 11] = physicProperties[propertyIndex + 11];
    output[outputIndex + 12] = physicProperties[propertyIndex + 12];
    output[outputIndex + 13] = physicProperties[propertyIndex + 13];
    output[outputIndex + 14] = physicProperties[propertyIndex + 14];
    output[outputIndex + 15] = physicProperties[propertyIndex + 15];
    output[outputIndex + 16] = physicProperties[propertyIndex + 16];
    output[outputIndex + 17] = physicProperties[propertyIndex + 17];
    output[outputIndex + 18] = physicProperties[propertyIndex + 18];
    output[outputIndex + 19] = physicProperties[propertyIndex + 19];
    output[outputIndex + 20] = physicProperties[propertyIndex + 20];
    output[outputIndex + 21] = physicProperties[propertyIndex + 21];
    output[outputIndex + 22] = physicProperties[propertyIndex + 22];
    output[outputIndex + 23] = physicProperties[propertyIndex + 23];
    output[outputIndex + 24] = physicProperties[propertyIndex + 24];
    output[outputIndex + 25] = physicProperties[propertyIndex + 25];
    output[outputIndex + 26] = physicProperties[propertyIndex + 26];
    output[outputIndex + 27] = physicProperties[propertyIndex + 27];
    output[outputIndex + 28] = physicProperties[propertyIndex + 28];
    output[outputIndex + 29] = physicProperties[propertyIndex + 29];
    output[outputIndex + 30] = physicProperties[propertyIndex + 30];
    output[outputIndex + 31] = physicProperties[propertyIndex + 31];
    output[outputIndex + 32] = physicProperties[propertyIndex + 32];
    output[outputIndex + 33] = physicProperties[propertyIndex + 33];
    output[outputIndex + 34] = physicProperties[propertyIndex + 34];
    output[outputIndex + 35] = physicProperties[propertyIndex + 35];
    output[outputIndex + 36] = physicProperties[propertyIndex + 36];
    output[outputIndex + 37] = physicProperties[propertyIndex + 37];
    output[outputIndex + 38] = physicProperties[propertyIndex + 38];
    output[outputIndex + 39] = physicProperties[propertyIndex + 39];
    output[outputIndex + 40] = physicProperties[propertyIndex + 40];
    output[outputIndex + 41] = physicProperties[propertyIndex + 41];
    output[outputIndex + 42] = physicProperties[propertyIndex + 42];
    output[outputIndex + 43] = physicProperties[propertyIndex + 43];
    output[outputIndex + 44] = physicProperties[propertyIndex + 44];
    output[outputIndex + 45] = physicProperties[propertyIndex + 45];
    output[outputIndex + 46] = physicProperties[propertyIndex + 46];
    output[outputIndex + 47] = physicProperties[propertyIndex + 47];
    output[outputIndex + 48] = physicProperties[propertyIndex + 48];
    output[outputIndex + 49] = physicProperties[propertyIndex + 49];
    output[outputIndex + 50] = physicProperties[propertyIndex + 50];
    output[outputIndex + 51] = physicProperties[propertyIndex + 51];
    output[outputIndex + 52] = physicProperties[propertyIndex + 52];
}