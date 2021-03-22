#include "OpenCLBase.cl"
#include "Vec3.cl"
#include "Mat3.cl"
#include "Quat.cl"

#define SP_RIGID_BODY_3D_GRAVITY_FORCE { 0.0f, -9.8f, 0.0f }
#define SP_RIGID_BODY_3D_DRAG_FORCE (0.1f)
#define SP_RIGID_BODY_3D_RESTING_VELOCITY (0.09f)

#define SP_RIGID_BODY_3D_SIZE                   (58)
#define SP_RIGID_BODY_3D_INV_MASS_INDEX       (0)
#define SP_RIGID_BODY_3D_DAMPING_INDEX        (1)
#define SP_RIGID_BODY_3D_ANG_DAMPING_INDEX    (2)
#define SP_RIGID_BODY_3D_COR_INDEX            (3)
#define SP_RIGID_BODY_3D_COF_INDEX            (4)
#define SP_RIGID_BODY_3D_INRTIAL_TENS_INDEX   (5)
#define SP_RIGID_BODY_3D_POSITION_INDEX       (14)
#define SP_RIGID_BODY_3D_VELOCITY_INDEX       (17)
#define SP_RIGID_BODY_3D_ACCELERATION_INDEX   (20)
#define SP_RIGID_BODY_3D_FORCE_INDEX          (23)
#define SP_RIGID_BODY_3D_ORIENTATION_INDEX    (26)
#define SP_RIGID_BODY_3D_ANG_VELOCITY_INDEX   (30)
#define SP_RIGID_BODY_3D_TORQUE_INDEX         (33)
#define SP_RIGID_BODY_3D_PREV_POSITION_INDEX  (36)
#define SP_RIGID_BODY_3D_PREV_VEL_INDEX       (39)
#define SP_RIGID_BODY_3D_PREV_ACC_INDEX       (42)
#define SP_RIGID_BODY_3D_PREV_FORCE_INDEX     (45)
#define SP_RIGID_BODY_3D_PREV_ORIENTAT_INDEX  (48)
#define SP_RIGID_BODY_3D_PREV_ANG_VEL_INDEX   (52)
#define SP_RIGID_BODY_3D_PREV_TORQUE_INDEX    (55)


#define SpRigidBody3D_isStatic(properties, stride) \
    properties[stride + SP_RIGID_BODY_3D_INV_MASS_INDEX] == ZERO_FLOAT


#define SpRigidBody3D_getInvMass(properties, stride) \
    properties[stride + SP_RIGID_BODY_3D_INV_MASS_INDEX]

#define SpRigidBody3D_getDamping(properties, stride) \
    properties[stride + SP_RIGID_BODY_3D_DAMPING_INDEX]

#define SpRigidBody3D_getAngDamping(properties, stride) \
    properties[stride + SP_RIGID_BODY_3D_ANG_DAMPING_INDEX]

#define SpRigidBody3D_getCoeficientOfRestitution(properties, stride) \
    properties[stride + SP_RIGID_BODY_3D_COR_INDEX]

#define SpRigidBody3D_getCoeficientOfFriction(properties, stride) \
    properties[stride + SP_RIGID_BODY_3D_COF_INDEX]

#define SpRigidBody3D_getInertialTensor(properties, stride) {  \
    properties[stride + SP_RIGID_BODY_3D_INRTIAL_TENS_INDEX    ], \
    properties[stride + SP_RIGID_BODY_3D_INRTIAL_TENS_INDEX + 1], \
    properties[stride + SP_RIGID_BODY_3D_INRTIAL_TENS_INDEX + 2], \
    properties[stride + SP_RIGID_BODY_3D_INRTIAL_TENS_INDEX + 3], \
    properties[stride + SP_RIGID_BODY_3D_INRTIAL_TENS_INDEX + 4], \
    properties[stride + SP_RIGID_BODY_3D_INRTIAL_TENS_INDEX + 5], \
    properties[stride + SP_RIGID_BODY_3D_INRTIAL_TENS_INDEX + 6], \
    properties[stride + SP_RIGID_BODY_3D_INRTIAL_TENS_INDEX + 7], \
    properties[stride + SP_RIGID_BODY_3D_INRTIAL_TENS_INDEX + 8] \
}

#define SpRigidBody3D_getPosition(properties, stride) {    \
    properties[stride + SP_RIGID_BODY_3D_POSITION_INDEX    ], \
    properties[stride + SP_RIGID_BODY_3D_POSITION_INDEX + 1], \
    properties[stride + SP_RIGID_BODY_3D_POSITION_INDEX + 2]  \
}

#define SpRigidBody3D_setPosition(properties, stride, newPosition) \
    properties[stride + SP_RIGID_BODY_3D_POSITION_INDEX    ] = newPosition.x; \
    properties[stride + SP_RIGID_BODY_3D_POSITION_INDEX + 1] = newPosition.y; \
    properties[stride + SP_RIGID_BODY_3D_POSITION_INDEX + 2] = newPosition.z;

#define SpRigidBody3D_getPreviousPosition(properties, stride) {    \
    properties[stride + SP_RIGID_BODY_3D_PREV_POSITION_INDEX    ], \
    properties[stride + SP_RIGID_BODY_3D_PREV_POSITION_INDEX + 1], \
    properties[stride + SP_RIGID_BODY_3D_PREV_POSITION_INDEX + 2]  \
}

#define SpRigidBody3D_getVelocity(properties, stride) {    \
    properties[stride + SP_RIGID_BODY_3D_VELOCITY_INDEX    ], \
    properties[stride + SP_RIGID_BODY_3D_VELOCITY_INDEX + 1], \
    properties[stride + SP_RIGID_BODY_3D_VELOCITY_INDEX + 2]  \
}

#define SpRigidBody3D_setVelocity(properties, stride, newVelocity) \
    properties[stride + SP_RIGID_BODY_3D_VELOCITY_INDEX    ] = newVelocity.x; \
    properties[stride + SP_RIGID_BODY_3D_VELOCITY_INDEX + 1] = newVelocity.y; \
    properties[stride + SP_RIGID_BODY_3D_VELOCITY_INDEX + 2] = newVelocity.z;

#define SpRigidBody3D_getPreviousVelocity(properties, stride) {    \
    properties[stride + SP_RIGID_BODY_3D_PREV_VEL_INDEX    ], \
    properties[stride + SP_RIGID_BODY_3D_PREV_VEL_INDEX + 1], \
    properties[stride + SP_RIGID_BODY_3D_PREV_VEL_INDEX + 2]  \
}

#define SpRigidBody3D_getAcceleration(properties, stride) {    \
    properties[stride + SP_RIGID_BODY_3D_ACCELERATION_INDEX    ], \
    properties[stride + SP_RIGID_BODY_3D_ACCELERATION_INDEX + 1], \
    properties[stride + SP_RIGID_BODY_3D_ACCELERATION_INDEX + 2]  \
}

#define SpRigidBody3D_setAcceleration(properties, stride, newAcceleration) \
    properties[stride + SP_RIGID_BODY_3D_ACCELERATION_INDEX    ] = newAcceleration.x; \
    properties[stride + SP_RIGID_BODY_3D_ACCELERATION_INDEX + 1] = newAcceleration.y; \
    properties[stride + SP_RIGID_BODY_3D_ACCELERATION_INDEX + 2] = newAcceleration.z;

#define SpRigidBody3D_getPreviousAcceleration(properties, stride) {    \
    properties[stride + SP_RIGID_BODY_3D_PREV_ACC_INDEX    ], \
    properties[stride + SP_RIGID_BODY_3D_PREV_ACC_INDEX + 1], \
    properties[stride + SP_RIGID_BODY_3D_PREV_ACC_INDEX + 2]  \
}

#define SpRigidBody3D_getOrientation(properties, stride) {    \
    properties[stride + SP_RIGID_BODY_3D_ORIENTATION_INDEX    ], \
    properties[stride + SP_RIGID_BODY_3D_ORIENTATION_INDEX + 1], \
    properties[stride + SP_RIGID_BODY_3D_ORIENTATION_INDEX + 2], \
    properties[stride + SP_RIGID_BODY_3D_ORIENTATION_INDEX + 3]  \
}

#define SpRigidBody3D_setOrientation(properties, stride, newOrientation) \
    properties[stride + SP_RIGID_BODY_3D_ORIENTATION_INDEX    ] = newOrientation.w; \
    properties[stride + SP_RIGID_BODY_3D_ORIENTATION_INDEX + 1] = newOrientation.x; \
    properties[stride + SP_RIGID_BODY_3D_ORIENTATION_INDEX + 2] = newOrientation.y; \
    properties[stride + SP_RIGID_BODY_3D_ORIENTATION_INDEX + 3] = newOrientation.z;

#define SpRigidBody3D_getPreviousOrientation(properties, stride) {    \
    properties[stride + SP_RIGID_BODY_3D_PREV_ORIENTAT_INDEX    ], \
    properties[stride + SP_RIGID_BODY_3D_PREV_ORIENTAT_INDEX + 1], \
    properties[stride + SP_RIGID_BODY_3D_PREV_ORIENTAT_INDEX + 2], \
    properties[stride + SP_RIGID_BODY_3D_PREV_ORIENTAT_INDEX + 3]  \
}

#define SpRigidBody3D_getAngVelocity(properties, stride) {    \
    properties[stride + SP_RIGID_BODY_3D_ANG_VELOCITY_INDEX    ], \
    properties[stride + SP_RIGID_BODY_3D_ANG_VELOCITY_INDEX + 1], \
    properties[stride + SP_RIGID_BODY_3D_ANG_VELOCITY_INDEX + 2]  \
}

#define SpRigidBody3D_setAngVelocity(properties, stride, newAngVelocity) \
    properties[stride + SP_RIGID_BODY_3D_ANG_VELOCITY_INDEX   ] = newAngVelocity.x; \
    properties[stride + SP_RIGID_BODY_3D_ANG_VELOCITY_INDEX + 1] = newAngVelocity.y; \
    properties[stride + SP_RIGID_BODY_3D_ANG_VELOCITY_INDEX + 2] = newAngVelocity.z;

#define SpRigidBody3D_getPreviousAngVelocity(properties, stride) {    \
    properties[stride + SP_RIGID_BODY_3D_PREV_ANG_VEL_INDEX    ], \
    properties[stride + SP_RIGID_BODY_3D_PREV_ANG_VEL_INDEX + 1], \
    properties[stride + SP_RIGID_BODY_3D_PREV_ANG_VEL_INDEX + 2]  \
}

#define SpRigidBody3D_getTorque(properties, stride) {    \
    properties[stride + SP_RIGID_BODY_3D_TORQUE_INDEX    ], \
    properties[stride + SP_RIGID_BODY_3D_TORQUE_INDEX + 1], \
    properties[stride + SP_RIGID_BODY_3D_TORQUE_INDEX + 2]  \
}

#define SpRigidBody3D_setTorque(properties, stride, newTorque)         \
    properties[stride + SP_RIGID_BODY_3D_TORQUE_INDEX    ] = newTorque.x; \
    properties[stride + SP_RIGID_BODY_3D_TORQUE_INDEX + 1] = newTorque.y; \
    properties[stride + SP_RIGID_BODY_3D_TORQUE_INDEX + 2] = newTorque.z;

#define SpRigidBody3D_getPreviousTorque(properties, stride) { \
    properties[stride + SP_RIGID_BODY_3D_PREV_TORQUE_INDEX    ], \
    properties[stride + SP_RIGID_BODY_3D_PREV_TORQUE_INDEX + 1], \
    properties[stride + SP_RIGID_BODY_3D_PREV_TORQUE_INDEX + 2]  \
}

#define SpRigidBody3D_getForce(properties, stride) {    \
    properties[stride + SP_RIGID_BODY_3D_FORCE_INDEX    ], \
    properties[stride + SP_RIGID_BODY_3D_FORCE_INDEX + 1], \
    properties[stride + SP_RIGID_BODY_3D_FORCE_INDEX + 2]  \
}

#define SpRigidBody3D_getPreviousForce(properties, stride) { \
    properties[stride + SP_RIGID_BODY_3D_PREV_FORCE_INDEX    ], \
    properties[stride + SP_RIGID_BODY_3D_PREV_FORCE_INDEX + 1], \
    properties[stride + SP_RIGID_BODY_3D_PREV_FORCE_INDEX + 2]  \
}

inline void restingAcceleration(__global sp_float* properties, const sp_uint stride, Vec3* result)
{
    const sp_float mass = properties[stride + SP_RIGID_BODY_3D_INV_MASS_INDEX];
    Vec3 gravityForce = SP_RIGID_BODY_3D_GRAVITY_FORCE;

    const Vec3 velocity = SpRigidBody3D_getVelocity(properties, stride);

    Vec3 absoluteVelocity;
    vec3_abs(velocity, &absoluteVelocity);

    Vec3 dragForce;
    vec3_multiply_vec3(velocity, absoluteVelocity, &dragForce);
    vec3_multiply_float(dragForce, 0.5f * SP_RIGID_BODY_3D_DRAG_FORCE, &dragForce);

    Vec3 restingAcc;
    vec3_minus_vec3(gravityForce, dragForce, restingAcc);
    vec3_multiply_float(restingAcc, mass, result);
}

inline sp_bool SpRigidBody3D_isResting(__global sp_float* properties, const sp_uint stride)
{
    /*
    Vec3 restingAcc;
    restingAcceleration(properties, stride, &restingAcc);

    const Vec3 acc = SpRigidBody3D_getAcceleration(properties, stride);
    const Vec3 velocity = SpRigidBody3D_getVelocity(properties, stride);
    const Vec3 prevVelocity = SpRigidBody3D_getPreviousVelocity(properties, stride);
    const Vec3 angVelocity = SpRigidBody3D_getAngVelocity(properties, stride);
    const Vec3 prevAngVelocity = SpRigidBody3D_getPreviousAngVelocity(properties, stride);
    */

    const Vec3 position = SpRigidBody3D_getPosition(properties, stride);
    const Vec3 prevPosition = SpRigidBody3D_getPreviousPosition(properties, stride);
    const Quat orientation = SpRigidBody3D_getOrientation(properties, stride);
    const Quat prevOrientation = SpRigidBody3D_getPreviousOrientation(properties, stride);

    return 
        vec3_isCloseEnough_vec3(position, prevPosition, SP_RIGID_BODY_3D_RESTING_VELOCITY)
        && quat_isCloseEnough_quat(orientation, prevOrientation, SP_RIGID_BODY_3D_RESTING_VELOCITY)
        //&& vec3_isCloseEnough_vec3(velocity, prevVelocity, SP_RIGID_BODY_3D_RESTING_VELOCITY)
        //&& vec3_isCloseEnough_vec3(angVelocity, prevAngVelocity, SP_RIGID_BODY_3D_RESTING_VELOCITY)
        //&& vec3_isCloseEnough_vec3(acc, restingAcc, SP_RIGID_BODY_3D_RESTING_VELOCITY)
        ;
}

__kernel void isResting(
    __global sp_float* properties,
    __global sp_bool * output
)
{
    const sp_uint stride = SP_RIGID_BODY_3D_SIZE * THREAD_ID;
    const sp_uint outputIndex = SP_RIGID_BODY_3D_SIZE * (THREAD_ID - THREAD_OFFSET);
 
    output[outputIndex] = SpRigidBody3D_isResting(properties, stride);
}

__kernel void isStatic(
    __global sp_float* properties,
    __global sp_bool* output
)
{
    const sp_uint stride = SP_RIGID_BODY_3D_SIZE * THREAD_ID;
    output[0] = SpRigidBody3D_isStatic(properties, stride);
}


__kernel void fetch(
    __global sp_float* rigidBodies3D,
    __global sp_float* output
)
{
    const sp_uint stride = THREAD_OFFSET * SP_RIGID_BODY_3D_SIZE;
    
    output[SP_RIGID_BODY_3D_INV_MASS_INDEX] = SpRigidBody3D_getInvMass(rigidBodies3D, stride);
    output[SP_RIGID_BODY_3D_DAMPING_INDEX] = SpRigidBody3D_getDamping(rigidBodies3D, stride);
    output[SP_RIGID_BODY_3D_ANG_DAMPING_INDEX] = SpRigidBody3D_getAngDamping(rigidBodies3D, stride);
    output[SP_RIGID_BODY_3D_COR_INDEX] = SpRigidBody3D_getCoeficientOfRestitution(rigidBodies3D, stride);
    output[SP_RIGID_BODY_3D_COF_INDEX] = SpRigidBody3D_getCoeficientOfFriction(rigidBodies3D, stride);

    Mat3 inertialTensor = SpRigidBody3D_getInertialTensor(rigidBodies3D, stride);
    output[SP_RIGID_BODY_3D_INRTIAL_TENS_INDEX    ] = inertialTensor.m00;
    output[SP_RIGID_BODY_3D_INRTIAL_TENS_INDEX + 1] = inertialTensor.m01;
    output[SP_RIGID_BODY_3D_INRTIAL_TENS_INDEX + 2] = inertialTensor.m02;
    output[SP_RIGID_BODY_3D_INRTIAL_TENS_INDEX + 3] = inertialTensor.m10;
    output[SP_RIGID_BODY_3D_INRTIAL_TENS_INDEX + 4] = inertialTensor.m11;
    output[SP_RIGID_BODY_3D_INRTIAL_TENS_INDEX + 5] = inertialTensor.m12;
    output[SP_RIGID_BODY_3D_INRTIAL_TENS_INDEX + 6] = inertialTensor.m20;
    output[SP_RIGID_BODY_3D_INRTIAL_TENS_INDEX + 7] = inertialTensor.m21;
    output[SP_RIGID_BODY_3D_INRTIAL_TENS_INDEX + 8] = inertialTensor.m22;

    Vec3 position = SpRigidBody3D_getPosition(rigidBodies3D, stride);
    output[SP_RIGID_BODY_3D_POSITION_INDEX    ] = position.x;
    output[SP_RIGID_BODY_3D_POSITION_INDEX + 1] = position.y;
    output[SP_RIGID_BODY_3D_POSITION_INDEX + 2] = position.z;

    Vec3 velocity = SpRigidBody3D_getVelocity(rigidBodies3D, stride);
    output[SP_RIGID_BODY_3D_VELOCITY_INDEX    ] = velocity.x;
    output[SP_RIGID_BODY_3D_VELOCITY_INDEX + 1] = velocity.y;
    output[SP_RIGID_BODY_3D_VELOCITY_INDEX + 2] = velocity.z;

    Vec3 acceleration = SpRigidBody3D_getAcceleration(rigidBodies3D, stride);
    output[SP_RIGID_BODY_3D_ACCELERATION_INDEX    ] = acceleration.x;
    output[SP_RIGID_BODY_3D_ACCELERATION_INDEX + 1] = acceleration.y;
    output[SP_RIGID_BODY_3D_ACCELERATION_INDEX + 2] = acceleration.z;

    Vec3 force = SpRigidBody3D_getForce(rigidBodies3D, stride);
    output[SP_RIGID_BODY_3D_FORCE_INDEX    ] = force.x;
    output[SP_RIGID_BODY_3D_FORCE_INDEX + 1] = force.y;
    output[SP_RIGID_BODY_3D_FORCE_INDEX + 2] = force.z;

    Quat orientation = SpRigidBody3D_getOrientation(rigidBodies3D, stride);
    output[SP_RIGID_BODY_3D_ORIENTATION_INDEX    ] = orientation.w;
    output[SP_RIGID_BODY_3D_ORIENTATION_INDEX + 1] = orientation.x;
    output[SP_RIGID_BODY_3D_ORIENTATION_INDEX + 2] = orientation.y;
    output[SP_RIGID_BODY_3D_ORIENTATION_INDEX + 3] = orientation.z;

    Vec3 angVelocity = SpRigidBody3D_getAngVelocity(rigidBodies3D, stride);
    output[SP_RIGID_BODY_3D_ANG_VELOCITY_INDEX    ] = angVelocity.x;
    output[SP_RIGID_BODY_3D_ANG_VELOCITY_INDEX + 1] = angVelocity.y;
    output[SP_RIGID_BODY_3D_ANG_VELOCITY_INDEX + 2] = angVelocity.z;

    Vec3 torque = SpRigidBody3D_getTorque(rigidBodies3D, stride);
    output[SP_RIGID_BODY_3D_TORQUE_INDEX    ] = torque.x;
    output[SP_RIGID_BODY_3D_TORQUE_INDEX + 1] = torque.y;
    output[SP_RIGID_BODY_3D_TORQUE_INDEX + 2] = torque.z;

    Vec3 prevPosition = SpRigidBody3D_getPreviousPosition(rigidBodies3D, stride);
    output[SP_RIGID_BODY_3D_PREV_POSITION_INDEX    ] = prevPosition.x;
    output[SP_RIGID_BODY_3D_PREV_POSITION_INDEX + 1] = prevPosition.y;
    output[SP_RIGID_BODY_3D_PREV_POSITION_INDEX + 2] = prevPosition.z;

    Vec3 prevVelocity = SpRigidBody3D_getPreviousVelocity(rigidBodies3D, stride);
    output[SP_RIGID_BODY_3D_PREV_VEL_INDEX    ] = prevVelocity.x;
    output[SP_RIGID_BODY_3D_PREV_VEL_INDEX + 1] = prevVelocity.y;
    output[SP_RIGID_BODY_3D_PREV_VEL_INDEX + 2] = prevVelocity.z;

    Vec3 prevAcceleration = SpRigidBody3D_getPreviousAcceleration(rigidBodies3D, stride);
    output[SP_RIGID_BODY_3D_PREV_ACC_INDEX    ] = prevAcceleration.x;
    output[SP_RIGID_BODY_3D_PREV_ACC_INDEX + 1] = prevAcceleration.y;
    output[SP_RIGID_BODY_3D_PREV_ACC_INDEX + 2] = prevAcceleration.z;

    Vec3 prevForce = SpRigidBody3D_getPreviousForce(rigidBodies3D, stride);
    output[SP_RIGID_BODY_3D_PREV_FORCE_INDEX    ] = prevForce.x;
    output[SP_RIGID_BODY_3D_PREV_FORCE_INDEX + 1] = prevForce.y;
    output[SP_RIGID_BODY_3D_PREV_FORCE_INDEX + 2] = prevForce.z;

    Quat prevOrientation = SpRigidBody3D_getPreviousOrientation(rigidBodies3D, stride);
    output[SP_RIGID_BODY_3D_PREV_ORIENTAT_INDEX    ] = prevOrientation.w;
    output[SP_RIGID_BODY_3D_PREV_ORIENTAT_INDEX + 1] = prevOrientation.x;
    output[SP_RIGID_BODY_3D_PREV_ORIENTAT_INDEX + 2] = prevOrientation.y;
    output[SP_RIGID_BODY_3D_PREV_ORIENTAT_INDEX + 3] = prevOrientation.z;

    Vec3 prevAngVelocity = SpRigidBody3D_getPreviousAngVelocity(rigidBodies3D, stride);
    output[SP_RIGID_BODY_3D_PREV_ANG_VEL_INDEX] = prevAngVelocity.x;
    output[SP_RIGID_BODY_3D_PREV_ANG_VEL_INDEX + 1] = prevAngVelocity.y;
    output[SP_RIGID_BODY_3D_PREV_ANG_VEL_INDEX + 2] = prevAngVelocity.z;

    Vec3 prevTorque = SpRigidBody3D_getPreviousTorque(rigidBodies3D, stride);
    output[SP_RIGID_BODY_3D_PREV_TORQUE_INDEX    ] = prevTorque.x;
    output[SP_RIGID_BODY_3D_PREV_TORQUE_INDEX + 1] = prevTorque.y;
    output[SP_RIGID_BODY_3D_PREV_TORQUE_INDEX + 2] = prevTorque.z;
}