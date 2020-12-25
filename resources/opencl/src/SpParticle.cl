#ifndef SP_PARTICLE_OPENCL_HEADER
#define SP_PARTICLE_OPENCL_HEADER

#include "OpenCLBase.cl"
#include "Vec3.cl"

#define SP_PARTICLE_SIZE           (60)
#define SP_PARTICLE_POSITION_INDEX (0)
#define SP_PARTICLE_VELOCITY_INDEX (3)
#define SP_PARTICLE_ACC_INDEX      (6)
#define SP_PARTICLE_FORCE_INDEX    (9)
#define SP_PHYSIC_INV_MASS_INDEX   (12)
#define SP_PHYSIC_DAMPING_INDEX    (13)
#define SP_PHYSIC_LIFETIME_INDEX   (14)

#define SpPhysicProperties_getPosition(particles, stride, output)   \
    output[0] = particles[stride + SP_PARTICLE_POSITION_INDEX    ]; \
    output[1] = particles[stride + SP_PARTICLE_POSITION_INDEX + 1]; \
    output[2] = particles[stride + SP_PARTICLE_POSITION_INDEX + 2];

#define SpPhysicProperties_getVelocity(particles, stride, output)   \
    output[0] = particles[stride + SP_PARTICLE_VELOCITY_INDEX    ]; \
    output[1] = particles[stride + SP_PARTICLE_VELOCITY_INDEX + 1]; \
    output[2] = particles[stride + SP_PARTICLE_VELOCITY_INDEX + 2];

#define SpPhysicProperties_getAcceleration(particles, stride, output)   \
    output[0] = particles[stride + SP_PARTICLE_ACC_INDEX    ]; \
    output[1] = particles[stride + SP_PARTICLE_ACC_INDEX + 1]; \
    output[2] = particles[stride + SP_PARTICLE_ACC_INDEX + 2];

#define SpPhysicProperties_getForce(particles, stride, output)   \
    output[0] = particles[stride + SP_PARTICLE_FORCE_INDEX    ]; \
    output[1] = particles[stride + SP_PARTICLE_FORCE_INDEX + 1]; \
    output[2] = particles[stride + SP_PARTICLE_FORCE_INDEX + 2];

#define SpPhysicProperties_getInverseMass(particles, stride) \
    particles[stride + SP_PHYSIC_INV_MASS_INDEX];

#define SpPhysicProperties_getDamping(particles, stride) \
    particles[stride + SP_PHYSIC_DAMPING_INDEX];

#define SpPhysicProperties_getLifetime(particles, stride) \
    particles[stride + SP_PHYSIC_LIFETIME_INDEX];

#endif // SP_PARTICLE_OPENCL_HEADER