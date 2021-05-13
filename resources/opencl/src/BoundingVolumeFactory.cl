#ifndef BOUNDING_VOLUME_FACTORY_OPENCL_HEADER
#define BOUNDING_VOLUME_FACTORY_OPENCL_HEADER

#include "OpenCLBase.cl"
#include "DOP18.cl"
#include "AABB.cl"
#include "Sphere.cl"
#include "Plane3D.cl"
#include "SpTransformation.cl"

__kernel void buildDOP18 (
    __global   sp_uint * meshCacheLength,
    __constant sp_uint * meshCacheIndexes,
    __constant sp_uint * meshCacheVertexesLength,
    __global   sp_float* meshCache,
    __global   sp_float* transformations,
    __global   sp_float* output
)
{
    if (THREAD_ID + 1u > *meshCacheLength)
        return;

    Vec3 position;
    sp_transformation_get_position(transformations, THREAD_ID * SP_TRANSFORMATION_STRIDER, &position);

    Vec3 orientationUpLeft;
    vec3_up_left(orientationUpLeft);

    Vec3 orientationUpRight;
    vec3_up_right(orientationUpRight);

    Vec3 orientationUpFront;
    vec3_up_front(orientationUpFront);

    Vec3 orientationUpDepth;
    vec3_up_depth(orientationUpDepth);

    Vec3 orientationLeftDepth;
    vec3_left_depth(orientationLeftDepth);

    Vec3 orientationRightDepth;
    vec3_right_depth(orientationRightDepth);

    sp_float 
        right = SP_FLOAT_MIN,
        up = SP_FLOAT_MIN,
        front = SP_FLOAT_MIN,
        left = SP_FLOAT_MAX,
        down = SP_FLOAT_MAX,
        depth = SP_FLOAT_MAX;

    sp_float
        distanceUpLeft = SP_FLOAT_MIN,
        distanceRightDown = SP_FLOAT_MAX,
        distanceUpRight = SP_FLOAT_MIN,
        distanceDownLeft = SP_FLOAT_MAX,
        distanceUpFront = SP_FLOAT_MIN,
        distanceDownDepth = SP_FLOAT_MAX,
        distanceUpDepth = SP_FLOAT_MIN,
        distanceDownFront = SP_FLOAT_MAX,
        distanceLeftDepth = SP_FLOAT_MIN,
        distanceRightFront = SP_FLOAT_MAX,
        distanceRightDepth = SP_FLOAT_MAX,
        distanceLeftFront = SP_FLOAT_MIN;

    const sp_uint vertexLength = meshCacheVertexesLength[THREAD_ID];
    sp_uint vertexIndex = meshCacheIndexes[THREAD_ID];

    for (sp_uint i = 0u; i < vertexLength; i++)
    {
        Vec3 vertex;
        vertex.x = meshCache[vertexIndex     ];
        vertex.y = meshCache[vertexIndex + 1u];
        vertex.z = meshCache[vertexIndex + 2u];

        if (vertex.x > right) right = vertex.x;
        if (vertex.y > up) up = vertex.y;
        if (vertex.z > front) front = vertex.z;

        if (vertex.x < left) left = vertex.x;
        if (vertex.y < down) down = vertex.y;
        if (vertex.z < depth) depth = vertex.z;
    
        Vec3 arrowToVertex;
        vec3_diff_vec3(vertex, position, arrowToVertex);

        sp_float newDistance = vec3_dot_vec3(orientationUpLeft, arrowToVertex);
        if (newDistance > distanceUpLeft)
            distanceUpLeft = newDistance;
        if (newDistance < distanceRightDown)
            distanceRightDown = newDistance;

        newDistance = vec3_dot_vec3(orientationUpRight, arrowToVertex);
        if (newDistance > distanceUpRight)
            distanceUpRight = newDistance;
        if (newDistance < distanceDownLeft)
            distanceDownLeft = newDistance;

        newDistance = vec3_dot_vec3(orientationUpFront, arrowToVertex);
        if (newDistance > distanceUpFront)
            distanceUpFront = newDistance;
        if (newDistance < distanceDownDepth)
            distanceDownDepth = newDistance;

        newDistance = vec3_dot_vec3(orientationUpDepth, arrowToVertex);
        if (newDistance > distanceUpDepth)
            distanceUpDepth = newDistance;
        if (newDistance < distanceDownFront)
            distanceDownFront = newDistance;

        newDistance = vec3_dot_vec3(orientationLeftDepth, arrowToVertex);
        if (newDistance > distanceLeftDepth)
            distanceLeftDepth = newDistance;
        if (newDistance < distanceRightFront)
            distanceRightFront = newDistance;

        newDistance = vec3_dot_vec3(orientationRightDepth, arrowToVertex);
        if (newDistance < distanceRightDepth)
            distanceRightDepth = newDistance;
        if (newDistance > distanceLeftFront)
            distanceLeftFront = newDistance;

        vertexIndex += 3;
    }

    if (isCloseEnough(up, down, SP_EPSILON_NUMBER))
    {
        down -= 0.1f;
        up += 0.1f;
    }
    if (isCloseEnough(front, depth, SP_EPSILON_NUMBER))
    {
        depth -= 0.1f;
        front += 0.1f;
    }
    if (isCloseEnough(left, right, SP_EPSILON_NUMBER))
    {
        left -= 0.1f;
        right += 0.1f;
    }

    // compute position
    const sp_float upLeft = sp_sqrt((distanceUpLeft * distanceUpLeft) * 2.0f);
    const sp_float downLeft = sp_sqrt((distanceDownLeft * distanceDownLeft) * 2.0f);
    const sp_float downDepth = sp_sqrt((distanceDownDepth * distanceDownDepth) * 2.0f);
    const sp_float upDepth = sp_sqrt((distanceUpDepth * distanceUpDepth) * 2.0f);
    const sp_float leftDepth = sp_sqrt((distanceLeftDepth * distanceLeftDepth) * 2.0f);
    const sp_float rightDepth = sp_sqrt((distanceRightDepth * distanceRightDepth) * 2.0f);

    const sp_float rightDown = sp_sqrt((distanceRightDown * distanceRightDown) * 2.0f);
    const sp_float upRight = sp_sqrt((distanceUpRight * distanceUpRight) * 2.0f);
    const sp_float upFront = sp_sqrt((distanceUpFront * distanceUpFront) * 2.0f);
    const sp_float downFront = sp_sqrt((distanceDownFront * distanceDownFront) * 2.0f);
    const sp_float rightFront = sp_sqrt((distanceRightFront * distanceRightFront) * 2.0f);
    const sp_float leftFront = sp_sqrt((distanceLeftFront * distanceLeftFront) * 2.0f);

    sp_uint outputIndex = THREAD_ID * DOP18_STRIDE;

    output[outputIndex + DOP18_AXIS_X] = left;
    output[outputIndex + DOP18_AXIS_Y] = down;
    output[outputIndex + DOP18_AXIS_Z] = depth;
    output[outputIndex + DOP18_AXIS_UP_LEFT]     = position.x - upLeft;
    output[outputIndex + DOP18_AXIS_UP_RIGHT]    = position.x + downLeft;
    output[outputIndex + DOP18_AXIS_UP_FRONT]    = position.z + downDepth;
    output[outputIndex + DOP18_AXIS_UP_DEPTH]    = position.z - upDepth;
    output[outputIndex + DOP18_AXIS_LEFT_DEPTH]  = position.x - leftDepth;
    output[outputIndex + DOP18_AXIS_RIGHT_DEPTH] = position.x + rightDepth;

    outputIndex += DOP18_ORIENTATIONS;
    output[outputIndex + DOP18_AXIS_X] = right;
    output[outputIndex + DOP18_AXIS_Y] = up;
    output[outputIndex + DOP18_AXIS_Z] = front;
    output[outputIndex + DOP18_AXIS_UP_LEFT]     = position.x - rightDown;
    output[outputIndex + DOP18_AXIS_UP_RIGHT]    = position.x + upRight;
    output[outputIndex + DOP18_AXIS_UP_FRONT]    = position.z + upFront;
    output[outputIndex + DOP18_AXIS_UP_DEPTH]    = position.z - downFront;
    output[outputIndex + DOP18_AXIS_LEFT_DEPTH]  = position.x + rightFront;
    output[outputIndex + DOP18_AXIS_RIGHT_DEPTH] = position.x - leftFront;
}
    

__kernel void buildAABB(
    __global   sp_uint* meshCacheLength,
    __constant sp_uint* meshCacheIndexes,
    __constant sp_uint* meshCacheVertexesLength,
    __global   sp_float* meshCache,
    __global   sp_float* transformations,
    __global   sp_float* output
)
{
    if (THREAD_ID + 1u > *meshCacheLength)
        return;

    sp_float
        right = SP_FLOAT_MIN,
        up = SP_FLOAT_MIN,
        front = SP_FLOAT_MIN,
        left = SP_FLOAT_MAX,
        down = SP_FLOAT_MAX,
        depth = SP_FLOAT_MAX;

    const sp_uint vertexLength = meshCacheVertexesLength[THREAD_ID];
    sp_uint vertexIndex = meshCacheIndexes[THREAD_ID];

    for (sp_uint i = 0u; i < vertexLength; i++)
    {
        Vec3 vertex;
        vertex.x = meshCache[vertexIndex     ];
        vertex.y = meshCache[vertexIndex + 1u];
        vertex.z = meshCache[vertexIndex + 2u];

        if (vertex.x > right) right = vertex.x;
        if (vertex.y > up) up = vertex.y;
        if (vertex.z > front) front = vertex.z;

        if (vertex.x < left) left = vertex.x;
        if (vertex.y < down) down = vertex.y;
        if (vertex.z < depth) depth = vertex.z;

        vertexIndex += 3;
    }

    if (isCloseEnough(up, down, SP_EPSILON_NUMBER))
    {
        down -= 0.1f;
        up += 0.1f;
    }
    if (isCloseEnough(front, depth, SP_EPSILON_NUMBER))
    {
        depth -= 0.1f;
        front += 0.1f;
    }
    if (isCloseEnough(left, right, SP_EPSILON_NUMBER))
    {
        left -= 0.1f;
        right += 0.1f;
    }

    sp_uint outputIndex = THREAD_ID * AABB_STRIDE;

    output[outputIndex + AABB_AXIS_X] = left;
    output[outputIndex + AABB_AXIS_Y] = down;
    output[outputIndex + AABB_AXIS_Z] = depth;
    
    outputIndex += AABB_ORIENTATIONS;
    output[outputIndex + AABB_AXIS_X] = right;
    output[outputIndex + AABB_AXIS_Y] = up;
    output[outputIndex + AABB_AXIS_Z] = front;
}


__kernel void buildSphere(
    __global   sp_uint* meshCacheLength,
    __constant sp_uint* meshCacheIndexes,
    __constant sp_uint* meshCacheVertexesLength,
    __global   sp_float* meshCache,
    __global   sp_float* transformations,
    __global   sp_float* output
)
{
    if (THREAD_ID + 1u > *meshCacheLength)
        return;

    Vec3 position;
    sp_transformation_get_position(transformations, THREAD_ID * SP_TRANSFORMATION_STRIDER, &position);

    const sp_uint vertexIndex = meshCacheIndexes[THREAD_ID];
    sp_float distance = SP_FLOAT_MIN;

    for (sp_uint i = 0u; i < *meshCacheVertexesLength; i++)
    {
        Vec3 vertex;
        vertex.x = meshCache[vertexIndex     ];
        vertex.y = meshCache[vertexIndex + 1u];
        vertex.z = meshCache[vertexIndex + 2u];

        const sp_float currentDistance = squared_distance(position, vertex);
        
        if (currentDistance > distance)
            distance = currentDistance;
    }

    const sp_uint outputIndex = THREAD_ID * SPHERE_STRIDE;

    output[outputIndex     ] = position.x;
    output[outputIndex + 1u] = position.y;
    output[outputIndex + 2u] = position.z;    
    output[outputIndex + 3u] = (sp_float) sqrt(distance);
}

#endif // BOUNDING_VOLUME_FACTORY_OPENCL_HEADER