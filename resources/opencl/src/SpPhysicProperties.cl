#include "OpenCLBase.cl"
#include "Vec3.cl"
#include "Mat3.cl"
#include "Quat.cl"

#define SP_PHYSIC_GRAVITY_FORCE { 0.0f, -9.8f, 0.0f }
#define SP_PHYSIC_DRAG_FORCE (0.1f)
#define SP_PHYSIC_RESTING_VELOCITY (0.09f)

#define SP_PHYSIC_PROPERTY_SIZE                 (58)
#define SP_PHYSIC_PROPERTY_INV_MASS_INDEX       (0)
#define SP_PHYSIC_PROPERTY_DAMPING_INDEX        (1)
#define SP_PHYSIC_PROPERTY_ANG_DAMPING_INDEX    (2)
#define SP_PHYSIC_PROPERTY_COR_INDEX            (3)
#define SP_PHYSIC_PROPERTY_COF_INDEX            (4)
#define SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX   (5)
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
#define SP_PHYSIC_PROPERTY_PREV_TORQUE_INDEX    (55)


#define SpPhysicProperties_isStatic(properties, stride) \
    properties[stride + SP_PHYSIC_PROPERTY_INV_MASS_INDEX] == ZERO_FLOAT


#define SpPhysicProperties_getInvMass(properties, stride) \
    properties[stride + SP_PHYSIC_PROPERTY_INV_MASS_INDEX]

#define SpPhysicProperties_getDamping(properties, stride) \
    properties[stride + SP_PHYSIC_PROPERTY_DAMPING_INDEX]

#define SpPhysicProperties_getAngDamping(properties, stride) \
    properties[stride + SP_PHYSIC_PROPERTY_ANG_DAMPING_INDEX]

#define SpPhysicProperties_getCoeficientOfRestitution(properties, stride) \
    properties[stride + SP_PHYSIC_PROPERTY_COR_INDEX]

#define SpPhysicProperties_getCoeficientOfFriction(properties, stride) \
    properties[stride + SP_PHYSIC_PROPERTY_COF_INDEX]

#define SpPhysicProperties_getInertialTensor(properties, stride) {  \
    properties[stride + SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX    ], \
    properties[stride + SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 1], \
    properties[stride + SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 2], \
    properties[stride + SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 3], \
    properties[stride + SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 4], \
    properties[stride + SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 5], \
    properties[stride + SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 6], \
    properties[stride + SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 7], \
    properties[stride + SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 8] \
}

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

#define SpPhysicProperties_getPreviousAcceleration(properties, stride) {    \
    properties[stride + SP_PHYSIC_PROPERTY_PREV_ACC_INDEX    ], \
    properties[stride + SP_PHYSIC_PROPERTY_PREV_ACC_INDEX + 1], \
    properties[stride + SP_PHYSIC_PROPERTY_PREV_ACC_INDEX + 2]  \
}

#define SpPhysicProperties_getOrientation(properties, stride) {    \
    properties[stride + SP_PHYSIC_PROPERTY_ORIENTATION_INDEX    ], \
    properties[stride + SP_PHYSIC_PROPERTY_ORIENTATION_INDEX + 1], \
    properties[stride + SP_PHYSIC_PROPERTY_ORIENTATION_INDEX + 2], \
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

#define SpPhysicProperties_setTorque(properties, stride, newTorque)         \
    properties[stride + SP_PHYSIC_PROPERTY_TORQUE_INDEX    ] = newTorque.x; \
    properties[stride + SP_PHYSIC_PROPERTY_TORQUE_INDEX + 1] = newTorque.y; \
    properties[stride + SP_PHYSIC_PROPERTY_TORQUE_INDEX + 2] = newTorque.z;

#define SpPhysicProperties_getPreviousTorque(properties, stride) { \
    properties[stride + SP_PHYSIC_PROPERTY_PREV_TORQUE_INDEX    ], \
    properties[stride + SP_PHYSIC_PROPERTY_PREV_TORQUE_INDEX + 1], \
    properties[stride + SP_PHYSIC_PROPERTY_PREV_TORQUE_INDEX + 2]  \
}

#define SpPhysicProperties_getForce(properties, stride) {    \
    properties[stride + SP_PHYSIC_PROPERTY_FORCE_INDEX    ], \
    properties[stride + SP_PHYSIC_PROPERTY_FORCE_INDEX + 1], \
    properties[stride + SP_PHYSIC_PROPERTY_FORCE_INDEX + 2]  \
}

#define SpPhysicProperties_getPreviousForce(properties, stride) { \
    properties[stride + SP_PHYSIC_PROPERTY_PREV_FORCE_INDEX    ], \
    properties[stride + SP_PHYSIC_PROPERTY_PREV_FORCE_INDEX + 1], \
    properties[stride + SP_PHYSIC_PROPERTY_PREV_FORCE_INDEX + 2]  \
}

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
    /*
    Vec3 restingAcc;
    restingAcceleration(properties, stride, &restingAcc);

    const Vec3 acc = SpPhysicProperties_getAcceleration(properties, stride);
    const Vec3 velocity = SpPhysicProperties_getVelocity(properties, stride);
    const Vec3 prevVelocity = SpPhysicProperties_getPreviousVelocity(properties, stride);
    const Vec3 angVelocity = SpPhysicProperties_getAngVelocity(properties, stride);
    const Vec3 prevAngVelocity = SpPhysicProperties_getPreviousAngVelocity(properties, stride);
    */
    const Vec3 position = SpPhysicProperties_getPosition(properties, stride);
    const Vec3 prevPosition = SpPhysicProperties_getPreviousPosition(properties, stride);
    const Vec3 orientation = SpPhysicProperties_getOrientation(properties, stride);
    const Vec3 prevOrientation = SpPhysicProperties_getPreviousOrientation(properties, stride);

    return 
        vec3_isCloseEnough_vec3(position, prevPosition, SP_PHYSIC_RESTING_VELOCITY)
        && vec3_isCloseEnough_vec3(orientation, prevOrientation, SP_PHYSIC_RESTING_VELOCITY)
        //&& vec3_isCloseEnough_vec3(velocity, prevVelocity, SP_PHYSIC_RESTING_VELOCITY)
        //&& vec3_isCloseEnough_vec3(angVelocity, prevAngVelocity, SP_PHYSIC_RESTING_VELOCITY)
        //&& vec3_isCloseEnough_vec3(acc, restingAcc, SP_PHYSIC_RESTING_VELOCITY)
        ;
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
    __global sp_float* output
)
{
    const sp_uint stride = THREAD_OFFSET * SP_PHYSIC_PROPERTY_SIZE;
    
    output[SP_PHYSIC_PROPERTY_INV_MASS_INDEX] = SpPhysicProperties_getInvMass(physicProperties, stride);
    output[SP_PHYSIC_PROPERTY_DAMPING_INDEX] = SpPhysicProperties_getDamping(physicProperties, stride);
    output[SP_PHYSIC_PROPERTY_ANG_DAMPING_INDEX] = SpPhysicProperties_getAngDamping(physicProperties, stride);
    output[SP_PHYSIC_PROPERTY_COR_INDEX] = SpPhysicProperties_getCoeficientOfRestitution(physicProperties, stride);
    output[SP_PHYSIC_PROPERTY_COF_INDEX] = SpPhysicProperties_getCoeficientOfFriction(physicProperties, stride);

    Mat3 inertialTensor = SpPhysicProperties_getInertialTensor(physicProperties, stride);
    output[SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX    ] = inertialTensor.m00;
    output[SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 1] = inertialTensor.m01;
    output[SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 2] = inertialTensor.m02;
    output[SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 3] = inertialTensor.m10;
    output[SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 4] = inertialTensor.m11;
    output[SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 5] = inertialTensor.m12;
    output[SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 6] = inertialTensor.m20;
    output[SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 7] = inertialTensor.m21;
    output[SP_PHYSIC_PROPERTY_INRTIAL_TENS_INDEX + 8] = inertialTensor.m22;

    Vec3 position = SpPhysicProperties_getPosition(physicProperties, stride);
    output[SP_PHYSIC_PROPERTY_POSITION_INDEX    ] = position.x;
    output[SP_PHYSIC_PROPERTY_POSITION_INDEX + 1] = position.y;
    output[SP_PHYSIC_PROPERTY_POSITION_INDEX + 2] = position.z;

    Vec3 velocity = SpPhysicProperties_getVelocity(physicProperties, stride);
    output[SP_PHYSIC_PROPERTY_VELOCITY_INDEX    ] = velocity.x;
    output[SP_PHYSIC_PROPERTY_VELOCITY_INDEX + 1] = velocity.y;
    output[SP_PHYSIC_PROPERTY_VELOCITY_INDEX + 2] = velocity.z;

    Vec3 acceleration = SpPhysicProperties_getAcceleration(physicProperties, stride);
    output[SP_PHYSIC_PROPERTY_ACCELERATION_INDEX    ] = acceleration.x;
    output[SP_PHYSIC_PROPERTY_ACCELERATION_INDEX + 1] = acceleration.y;
    output[SP_PHYSIC_PROPERTY_ACCELERATION_INDEX + 2] = acceleration.z;

    Vec3 force = SpPhysicProperties_getForce(physicProperties, stride);
    output[SP_PHYSIC_PROPERTY_FORCE_INDEX    ] = force.x;
    output[SP_PHYSIC_PROPERTY_FORCE_INDEX + 1] = force.y;
    output[SP_PHYSIC_PROPERTY_FORCE_INDEX + 2] = force.z;

    Quat orientation = SpPhysicProperties_getOrientation(physicProperties, stride);
    output[SP_PHYSIC_PROPERTY_ORIENTATION_INDEX    ] = orientation.w;
    output[SP_PHYSIC_PROPERTY_ORIENTATION_INDEX + 1] = orientation.x;
    output[SP_PHYSIC_PROPERTY_ORIENTATION_INDEX + 2] = orientation.y;
    output[SP_PHYSIC_PROPERTY_ORIENTATION_INDEX + 3] = orientation.z;

    Vec3 angVelocity = SpPhysicProperties_getAngVelocity(physicProperties, stride);
    output[SP_PHYSIC_PROPERTY_ANG_VELOCITY_INDEX    ] = angVelocity.x;
    output[SP_PHYSIC_PROPERTY_ANG_VELOCITY_INDEX + 1] = angVelocity.y;
    output[SP_PHYSIC_PROPERTY_ANG_VELOCITY_INDEX + 2] = angVelocity.z;

    Vec3 torque = SpPhysicProperties_getTorque(physicProperties, stride);
    output[SP_PHYSIC_PROPERTY_TORQUE_INDEX    ] = torque.x;
    output[SP_PHYSIC_PROPERTY_TORQUE_INDEX + 1] = torque.y;
    output[SP_PHYSIC_PROPERTY_TORQUE_INDEX + 2] = torque.z;

    Vec3 prevPosition = SpPhysicProperties_getPreviousPosition(physicProperties, stride);
    output[SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX    ] = prevPosition.x;
    output[SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX + 1] = prevPosition.y;
    output[SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX + 2] = prevPosition.z;

    Vec3 prevVelocity = SpPhysicProperties_getPreviousVelocity(physicProperties, stride);
    output[SP_PHYSIC_PROPERTY_PREV_VEL_INDEX    ] = prevVelocity.x;
    output[SP_PHYSIC_PROPERTY_PREV_VEL_INDEX + 1] = prevVelocity.y;
    output[SP_PHYSIC_PROPERTY_PREV_VEL_INDEX + 2] = prevVelocity.z;

    Vec3 prevAcceleration = SpPhysicProperties_getPreviousAcceleration(physicProperties, stride);
    output[SP_PHYSIC_PROPERTY_PREV_ACC_INDEX    ] = prevAcceleration.x;
    output[SP_PHYSIC_PROPERTY_PREV_ACC_INDEX + 1] = prevAcceleration.y;
    output[SP_PHYSIC_PROPERTY_PREV_ACC_INDEX + 2] = prevAcceleration.z;

    Vec3 prevForce = SpPhysicProperties_getPreviousForce(physicProperties, stride);
    output[SP_PHYSIC_PROPERTY_PREV_FORCE_INDEX    ] = prevForce.x;
    output[SP_PHYSIC_PROPERTY_PREV_FORCE_INDEX + 1] = prevForce.y;
    output[SP_PHYSIC_PROPERTY_PREV_FORCE_INDEX + 2] = prevForce.z;

    Quat prevOrientation = SpPhysicProperties_getPreviousOrientation(physicProperties, stride);
    output[SP_PHYSIC_PROPERTY_PREV_ORIENTAT_INDEX    ] = prevOrientation.w;
    output[SP_PHYSIC_PROPERTY_PREV_ORIENTAT_INDEX + 1] = prevOrientation.x;
    output[SP_PHYSIC_PROPERTY_PREV_ORIENTAT_INDEX + 2] = prevOrientation.y;
    output[SP_PHYSIC_PROPERTY_PREV_ORIENTAT_INDEX + 3] = prevOrientation.z;

    Vec3 prevAngVelocity = SpPhysicProperties_getPreviousAngVelocity(physicProperties, stride);
    output[SP_PHYSIC_PROPERTY_PREV_ANG_VEL_INDEX] = prevAngVelocity.x;
    output[SP_PHYSIC_PROPERTY_PREV_ANG_VEL_INDEX + 1] = prevAngVelocity.y;
    output[SP_PHYSIC_PROPERTY_PREV_ANG_VEL_INDEX + 2] = prevAngVelocity.z;

    Vec3 prevTorque = SpPhysicProperties_getPreviousTorque(physicProperties, stride);
    output[SP_PHYSIC_PROPERTY_PREV_TORQUE_INDEX    ] = prevTorque.x;
    output[SP_PHYSIC_PROPERTY_PREV_TORQUE_INDEX + 1] = prevTorque.y;
    output[SP_PHYSIC_PROPERTY_PREV_TORQUE_INDEX + 2] = prevTorque.z;
}