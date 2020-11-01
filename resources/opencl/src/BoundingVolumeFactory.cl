#include "OpenCLBase.cl"
#include "DOP18.cl"
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
    sp_float dotOrientationUpLeft = vec3_dot_vec3(orientationUpLeft, orientationUpLeft);
    sp_float dfoUpLeft = vec3_dot_vec3(position, orientationUpLeft);

    Vec3 orientationUpRight;
    vec3_up_right(orientationUpRight);
    sp_float dotOrientationUpRight = vec3_dot_vec3(orientationUpRight, orientationUpRight);
    sp_float dfoUpRight = vec3_dot_vec3(position, orientationUpRight);

    Vec3 orientationUpFront;
    vec3_up_front(orientationUpFront);
    sp_float dotOrientationUpFront = vec3_dot_vec3(orientationUpFront, orientationUpFront);
    sp_float dfoUpFront = vec3_dot_vec3(position, orientationUpFront);

    Vec3 orientationUpDepth;
    vec3_up_depth(orientationUpDepth);
    sp_float dotOrientationUpDepth = vec3_dot_vec3(orientationUpDepth, orientationUpDepth);
    sp_float dfoUpDepth = vec3_dot_vec3(position, orientationUpDepth);

    Vec3 orientationLeftDepth;
    vec3_left_depth(orientationLeftDepth);
    sp_float dotOrientationLeftDepth = vec3_dot_vec3(orientationLeftDepth, orientationLeftDepth);
    sp_float dfoLeftDepth = vec3_dot_vec3(position, orientationLeftDepth);

    Vec3 orientationRightDepth;
    vec3_right_depth(orientationRightDepth);
    sp_float dotOrientationRightDepth = vec3_dot_vec3(orientationRightDepth, orientationRightDepth);
    sp_float dfoRightDepth = vec3_dot_vec3(position, orientationRightDepth);

    sp_float 
        right = SP_FLOAT_MIN,
        up = SP_FLOAT_MIN,
        front = SP_FLOAT_MIN,
        left = SP_FLOAT_MAX,
        down = SP_FLOAT_MAX,
        depth = SP_FLOAT_MAX,
        upLeft, rightDown, upRight, downLeft, upFront, downDepth, upDepth, downFront,
        leftDepth, rightFront, rightDepth, leftFront;

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
    
        sp_float newDistance = (vec3_dot_vec3(orientationUpLeft, vertex) - dfoUpLeft) / dotOrientationUpLeft;
        
        if (newDistance > distanceUpLeft)
        {
            upLeft = vertex.x - (vertex.y - position.y);
            distanceUpLeft = newDistance;
        }
        if (newDistance < distanceRightDown)
        {
            rightDown = vertex.x + (position.y - vertex.y);
            distanceRightDown = newDistance;
        }

        newDistance = (vec3_dot_vec3(orientationUpRight, vertex) - dfoUpRight) / dotOrientationUpRight;

        if (newDistance > distanceUpRight)
        {
            upRight = vertex.x + (vertex.y - position.y);
            distanceUpRight = newDistance;
        }
        if (newDistance < distanceDownLeft)
        {
            downLeft = vertex.x + (vertex.y - position.y);
            distanceDownLeft = newDistance;
        }

        newDistance = (vec3_dot_vec3(orientationUpFront, vertex) - dfoUpFront) / dotOrientationUpFront;

        if (newDistance > distanceUpFront)
        {
            upFront = vertex.z + (vertex.y - position.y);
            distanceUpFront = newDistance;
        }
        if (newDistance < distanceDownDepth)
        {
            downDepth = vertex.z + (vertex.y - position.y);
            distanceDownDepth = newDistance;
        }

        newDistance = (vec3_dot_vec3(orientationUpDepth, vertex) - dfoUpDepth) / dotOrientationUpDepth;

        if (newDistance > distanceUpDepth)
        {
            upDepth = vertex.z + (position.y - vertex.y);
            distanceUpDepth = newDistance;
        }
        if (newDistance < distanceDownFront)
        {
            downFront = vertex.z + (position.y - vertex.y);
            distanceDownFront = newDistance;
        }

        newDistance = (vec3_dot_vec3(orientationLeftDepth, vertex) - dfoLeftDepth) / dotOrientationLeftDepth;

        if (newDistance > distanceLeftDepth)
        {
            leftDepth = vertex.x + (vertex.z - position.z);
            distanceLeftDepth = newDistance;
        }
        if (newDistance < distanceRightFront)
        {
            rightFront = vertex.x + (vertex.z - position.z);
            distanceRightFront = newDistance;
        }

        newDistance = (vec3_dot_vec3(orientationRightDepth, vertex) - dfoRightDepth) / dotOrientationRightDepth;

        if (newDistance < distanceRightDepth)
        {
            rightDepth = vertex.x + (position.z - vertex.z);
            distanceRightDepth = newDistance;
        }
        if (newDistance > distanceLeftFront)
        {
            leftFront = vertex.x + (position.z - vertex.z);
            distanceLeftFront = newDistance;
        }

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

    sp_uint outputIndex = THREAD_ID * DOP18_STRIDE;

    output[outputIndex + DOP18_AXIS_X] = left;
    output[outputIndex + DOP18_AXIS_Y] = down;
    output[outputIndex + DOP18_AXIS_Z] = depth;
    output[outputIndex + DOP18_AXIS_UP_LEFT] = upLeft;
    output[outputIndex + DOP18_AXIS_UP_RIGHT] = downLeft;
    output[outputIndex + DOP18_AXIS_UP_FRONT] = downDepth;
    output[outputIndex + DOP18_AXIS_UP_DEPTH] = upDepth;
    output[outputIndex + DOP18_AXIS_LEFT_DEPTH] = leftDepth;
    output[outputIndex + DOP18_AXIS_RIGHT_DEPTH] = rightDepth;

    outputIndex += DOP18_ORIENTATIONS;
    output[outputIndex + DOP18_AXIS_X] = right;
    output[outputIndex + DOP18_AXIS_Y] = up;
    output[outputIndex + DOP18_AXIS_Z] = front;
    output[outputIndex + DOP18_AXIS_UP_LEFT] = rightDown;
    output[outputIndex + DOP18_AXIS_UP_RIGHT] = upRight;
    output[outputIndex + DOP18_AXIS_UP_FRONT] = upFront;
    output[outputIndex + DOP18_AXIS_UP_DEPTH] = downFront;
    output[outputIndex + DOP18_AXIS_LEFT_DEPTH] = rightFront;
    output[outputIndex + DOP18_AXIS_RIGHT_DEPTH] = leftFront;
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

    sp_uint outputIndex = THREAD_ID * DOP18_STRIDE;

    output[outputIndex + DOP18_AXIS_X] = left;
    output[outputIndex + DOP18_AXIS_Y] = down;
    output[outputIndex + DOP18_AXIS_Z] = depth;
    
    outputIndex += DOP18_ORIENTATIONS;
    output[outputIndex + DOP18_AXIS_X] = right;
    output[outputIndex + DOP18_AXIS_Y] = up;
    output[outputIndex + DOP18_AXIS_Z] = front;
}
