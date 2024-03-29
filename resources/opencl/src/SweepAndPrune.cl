#include "OpenCLBase.cl"
#include "DOP18.cl"
#include "Sphere.cl"
#include "AABB.cl"
#include "SpRigidBody3D.cl"

#ifndef INPUT_STRIDE
    #define INPUT_STRIDE (1)
#endif

#define DOP18_MIN_POINT_NEXT_ELEMENT    boundingVolumes[boundingVolumeIndex2 + axis]
#define DOP18_MIN_POINT_NEXT_ELEMENT_X  boundingVolumes[boundingVolumeIndex2 + DOP18_AXIS_X]
#define DOP18_MIN_POINT_NEXT_ELEMENT_Y  boundingVolumes[boundingVolumeIndex2 + DOP18_AXIS_Y]
#define DOP18_MIN_POINT_NEXT_ELEMENT_Z  boundingVolumes[boundingVolumeIndex2 + DOP18_AXIS_Z]
#define DOP18_MIN_POINT_NEXT_ELEMENT_XY boundingVolumes[boundingVolumeIndex2 + DOP18_AXIS_UP_LEFT]
#define DOP18_MIN_POINT_NEXT_ELEMENT_YX boundingVolumes[boundingVolumeIndex2 + DOP18_AXIS_UP_RIGHT]
#define DOP18_MIN_POINT_NEXT_ELEMENT_YZ boundingVolumes[boundingVolumeIndex2 + DOP18_AXIS_UP_FRONT]
#define DOP18_MIN_POINT_NEXT_ELEMENT_ZY boundingVolumes[boundingVolumeIndex2 + DOP18_AXIS_UP_DEPTH]
#define DOP18_MIN_POINT_NEXT_ELEMENT_XZ boundingVolumes[boundingVolumeIndex2 + DOP18_AXIS_LEFT_DEPTH]
#define DOP18_MIN_POINT_NEXT_ELEMENT_ZX boundingVolumes[boundingVolumeIndex2 + DOP18_AXIS_RIGHT_DEPTH]
#define DOP18_MAX_POINT_NEXT_ELEMENT_X  boundingVolumes[boundingVolumeIndex2 + DOP18_AXIS_X + DOP18_ORIENTATIONS]
#define DOP18_MAX_POINT_NEXT_ELEMENT_Y  boundingVolumes[boundingVolumeIndex2 + DOP18_AXIS_Y + DOP18_ORIENTATIONS]
#define DOP18_MAX_POINT_NEXT_ELEMENT_Z  boundingVolumes[boundingVolumeIndex2 + DOP18_AXIS_Z + DOP18_ORIENTATIONS]
#define DOP18_MAX_POINT_NEXT_ELEMENT_XY boundingVolumes[boundingVolumeIndex2 + DOP18_AXIS_UP_LEFT     + DOP18_ORIENTATIONS]
#define DOP18_MAX_POINT_NEXT_ELEMENT_YX boundingVolumes[boundingVolumeIndex2 + DOP18_AXIS_UP_RIGHT    + DOP18_ORIENTATIONS]
#define DOP18_MAX_POINT_NEXT_ELEMENT_YZ boundingVolumes[boundingVolumeIndex2 + DOP18_AXIS_UP_FRONT    + DOP18_ORIENTATIONS]
#define DOP18_MAX_POINT_NEXT_ELEMENT_ZY boundingVolumes[boundingVolumeIndex2 + DOP18_AXIS_UP_DEPTH    + DOP18_ORIENTATIONS]
#define DOP18_MAX_POINT_NEXT_ELEMENT_XZ boundingVolumes[boundingVolumeIndex2 + DOP18_AXIS_LEFT_DEPTH  + DOP18_ORIENTATIONS]
#define DOP18_MAX_POINT_NEXT_ELEMENT_ZX boundingVolumes[boundingVolumeIndex2 + DOP18_AXIS_RIGHT_DEPTH + DOP18_ORIENTATIONS]

#define DOP18_MAX_POINT  boundingVolumes[boundingVolumeIndex1 + DOP18_ORIENTATIONS + axis]
#define DOP18_minPointX  boundingVolumes[boundingVolumeIndex1 + DOP18_AXIS_X]
#define DOP18_minPointY  boundingVolumes[boundingVolumeIndex1 + DOP18_AXIS_Y]
#define DOP18_minPointZ  boundingVolumes[boundingVolumeIndex1 + DOP18_AXIS_Z]
#define DOP18_minPointXY boundingVolumes[boundingVolumeIndex1 + DOP18_AXIS_UP_LEFT    ]
#define DOP18_minPointYX boundingVolumes[boundingVolumeIndex1 + DOP18_AXIS_UP_RIGHT   ]
#define DOP18_minPointYZ boundingVolumes[boundingVolumeIndex1 + DOP18_AXIS_UP_FRONT   ]
#define DOP18_minPointZY boundingVolumes[boundingVolumeIndex1 + DOP18_AXIS_UP_DEPTH   ]
#define DOP18_minPointXZ boundingVolumes[boundingVolumeIndex1 + DOP18_AXIS_LEFT_DEPTH ]
#define DOP18_minPointZX boundingVolumes[boundingVolumeIndex1 + DOP18_AXIS_RIGHT_DEPTH]
#define DOP18_maxPointX  boundingVolumes[boundingVolumeIndex1 + DOP18_AXIS_X           + DOP18_ORIENTATIONS]
#define DOP18_maxPointY  boundingVolumes[boundingVolumeIndex1 + DOP18_AXIS_Y           + DOP18_ORIENTATIONS]
#define DOP18_maxPointZ  boundingVolumes[boundingVolumeIndex1 + DOP18_AXIS_Z           + DOP18_ORIENTATIONS]
#define DOP18_maxPointXY boundingVolumes[boundingVolumeIndex1 + DOP18_AXIS_UP_LEFT     + DOP18_ORIENTATIONS]
#define DOP18_maxPointYX boundingVolumes[boundingVolumeIndex1 + DOP18_AXIS_UP_RIGHT    + DOP18_ORIENTATIONS]
#define DOP18_maxPointYZ boundingVolumes[boundingVolumeIndex1 + DOP18_AXIS_UP_FRONT    + DOP18_ORIENTATIONS]
#define DOP18_maxPointZY boundingVolumes[boundingVolumeIndex1 + DOP18_AXIS_UP_DEPTH    + DOP18_ORIENTATIONS]
#define DOP18_maxPointXZ boundingVolumes[boundingVolumeIndex1 + DOP18_AXIS_LEFT_DEPTH  + DOP18_ORIENTATIONS]
#define DOP18_maxPointZX boundingVolumes[boundingVolumeIndex1 + DOP18_AXIS_RIGHT_DEPTH + DOP18_ORIENTATIONS]

#define AABB_MIN_POINT_NEXT_ELEMENT boundingVolumes[boundingVolumeIndex2                     + axis]
#define AABB_MAX_POINT              boundingVolumes[boundingVolumeIndex1 + AABB_ORIENTATIONS + axis]

#define AABB_MIN_POINT_NEXT_ELEMENT_X boundingVolumes[boundingVolumeIndex2       ]
#define AABB_MIN_POINT_NEXT_ELEMENT_Y boundingVolumes[boundingVolumeIndex2 +    1]
#define AABB_MIN_POINT_NEXT_ELEMENT_Z boundingVolumes[boundingVolumeIndex2 +    2]
#define AABB_MAX_POINT_NEXT_ELEMENT_X boundingVolumes[boundingVolumeIndex2 +    3]
#define AABB_MAX_POINT_NEXT_ELEMENT_Y boundingVolumes[boundingVolumeIndex2 +    4]
#define AABB_MAX_POINT_NEXT_ELEMENT_Z boundingVolumes[boundingVolumeIndex2 +    5]

#define AABB_minPointX  boundingVolumes[boundingVolumeIndex1     ]
#define AABB_minPointY  boundingVolumes[boundingVolumeIndex1 +  1]
#define AABB_minPointZ  boundingVolumes[boundingVolumeIndex1 +  2]
#define AABB_maxPointX  boundingVolumes[boundingVolumeIndex1 +  3]
#define AABB_maxPointY  boundingVolumes[boundingVolumeIndex1 +  4]
#define AABB_maxPointZ  boundingVolumes[boundingVolumeIndex1 +  5]

__kernel void buildInputElements(
    __global   sp_float* boundingVolume,
    __constant sp_int  * boundingVolumeType,
    __constant sp_uint * indexes,
    __constant sp_uint * inputLength,
    __global   sp_float* output
    )
{
#define axis THREAD_OFFSET

    const sp_uint elementIndex = indexes[THREAD_ID - THREAD_OFFSET];

    switch (*boundingVolumeType)
    {
    case 4: // DOP18
        {
            output[elementIndex] = boundingVolume[elementIndex * DOP18_STRIDE + axis];    
            break;
        }
    case 3: // AABB
        {
            output[elementIndex] = boundingVolume[elementIndex * AABB_STRIDE + axis];
            break;
        }
    case 1: // SPHERE
        {
            Vec3 center;
            center.x = boundingVolume[elementIndex * SPHERE_STRIDE    ];
            center.y = boundingVolume[elementIndex * SPHERE_STRIDE + 1];
            center.z = boundingVolume[elementIndex * SPHERE_STRIDE + 2];
            const sp_float ray = boundingVolume[elementIndex * SPHERE_STRIDE + 3];

            Vec3 axisVector;

            switch (axis)
            {
            case DOP18_AXIS_X:
            {
                vec3_right(axisVector);
                break;
            }
            case DOP18_AXIS_Y:
            {
                vec3_up(axisVector);
                break;
            }
            case DOP18_AXIS_Z:
            {
                vec3_front(axisVector);
                break;
            }
            case DOP18_AXIS_UP_LEFT:
            {
                vec3_up_left(axisVector);
                break;
            }
            case DOP18_AXIS_UP_RIGHT:
            {
                vec3_up_right(axisVector);
                break;
            }
            case DOP18_AXIS_UP_FRONT:
            {
                vec3_up_front(axisVector);
                break;
            }
            case DOP18_AXIS_UP_DEPTH:
            {
                vec3_up_depth(axisVector);
                break;
            }
            case DOP18_AXIS_LEFT_DEPTH:
            {
                vec3_left_depth(axisVector);
                break;
            }
            case DOP18_AXIS_RIGHT_DEPTH:
            {
                vec3_right_depth(axisVector);
                break;
            }
            }

            output[elementIndex] = (vec3_dot_vec3(axisVector, center) / vec3_dot_vec3(axisVector, axisVector)) - ray;

            /*
            Vec3 r;
            vec3_multiply_float(axisVector, ray, &r);
            vec3_add_vec3(center, r, r);
            vec3_diff_vec3(center, r, r);
            */

            break;
        }
    }

#undef axis
}

__kernel void sweepAndPruneSingleAxis(
	__global   sp_float* boundingVolumes,
    __global   sp_float* elements,
    __global   sp_float* rigidBodies3D,
	__constant sp_uint * indexesLength, 
    __global   sp_uint * indexes, 
	__global   sp_uint * outputLength, 
	__global   sp_uint * output)
{
#define axis THREAD_OFFSET
#define index (THREAD_ID - THREAD_OFFSET)

    if (index + 1u > *indexesLength)
        return;

    const sp_uint objIndex1 = indexes[index];
    
    // TODO: REMOVE
#define minY boundingVolumes[objIndex2 * DOP18_STRIDE + DOP18_AXIS_Y]
    if (objIndex1 == ZERO_UINT) // special handler for plane
    {
        for(sp_uint j = 0u; j < *indexesLength; j++) // iterate over next elements
        {
            const sp_uint objIndex2 = indexes[j];
            if (objIndex2 != ZERO_UINT && minY < ZERO_FLOAT) // if minY(objIndex2) < 0, means it is under the plane
            {
                const sp_uint temp = atomic_add(outputLength, 2);
                output[temp    ] = objIndex1;
                output[temp + 1] = objIndex2;
            }
        }

        return;
    }
#undef minY

    const sp_uint boundingVolumeIndex1 = objIndex1 * DOP18_STRIDE;

    for(sp_uint j = index + 1u; j < *indexesLength; j++) // iterate over next elements
    {
        const sp_uint objIndex2 = indexes[j];

        // TODO: REMOVE
        if (objIndex2 == ZERO_UINT) // special handler for plane
            continue;

        const sp_uint boundingVolumeIndex2 = objIndex2 * DOP18_STRIDE;

        if (DOP18_MAX_POINT < DOP18_MIN_POINT_NEXT_ELEMENT)  // if max currernt element < than min of next element, means this element does not collide with nobody else beyond
            return;

        if (   (DOP18_maxPointX  >= DOP18_MIN_POINT_NEXT_ELEMENT_X  && DOP18_minPointX  <= DOP18_MAX_POINT_NEXT_ELEMENT_X)
            && (DOP18_maxPointY  >= DOP18_MIN_POINT_NEXT_ELEMENT_Y  && DOP18_minPointY  <= DOP18_MAX_POINT_NEXT_ELEMENT_Y)
            && (DOP18_maxPointZ  >= DOP18_MIN_POINT_NEXT_ELEMENT_Z  && DOP18_minPointZ  <= DOP18_MAX_POINT_NEXT_ELEMENT_Z)
            && (DOP18_maxPointXY >= DOP18_MIN_POINT_NEXT_ELEMENT_XY && DOP18_minPointXY <= DOP18_MAX_POINT_NEXT_ELEMENT_XY)
            && (DOP18_maxPointYX >= DOP18_MIN_POINT_NEXT_ELEMENT_YX && DOP18_minPointYX <= DOP18_MAX_POINT_NEXT_ELEMENT_YX)
            && (DOP18_maxPointYZ >= DOP18_MIN_POINT_NEXT_ELEMENT_YZ && DOP18_minPointYZ <= DOP18_MAX_POINT_NEXT_ELEMENT_YZ)
            && (DOP18_maxPointZY >= DOP18_MIN_POINT_NEXT_ELEMENT_ZY && DOP18_minPointZY <= DOP18_MAX_POINT_NEXT_ELEMENT_ZY)
            && (DOP18_maxPointXZ >= DOP18_MIN_POINT_NEXT_ELEMENT_XZ && DOP18_minPointXZ <= DOP18_MAX_POINT_NEXT_ELEMENT_XZ)
            && (DOP18_maxPointZX >= DOP18_MIN_POINT_NEXT_ELEMENT_ZX && DOP18_minPointZX <= DOP18_MAX_POINT_NEXT_ELEMENT_ZX)  
        )
        {
            const sp_uint temp = atomic_add(outputLength, 2);
            output[temp    ] = objIndex1;
            output[temp + 1] = objIndex2;
        }
    }

#undef index
#undef axis
}


__kernel void sweepAndPruneSingleAxisAABB(
    __global   sp_float* boundingVolumes,
    __global   sp_float* elements,
    __global   sp_float* rigidBodies3D,
    __constant sp_uint* indexesLength,
    __global   sp_uint* indexes,
    __global   sp_uint* outputLength,
    __global   sp_uint* output)
{
#define axis THREAD_OFFSET
#define index (THREAD_ID - THREAD_OFFSET)

    if (index + 1u > * indexesLength)
        return;

    const sp_uint objIndex1 = indexes[index];

    // TODO: REMOVE
    if (objIndex1 == ZERO_UINT) // special handler for plane
    {
        for(sp_uint j = 0u; j < *indexesLength; j++) // iterate over next elements
        {
            const sp_uint objIndex2 = indexes[j];
            if (objIndex2 != ZERO_UINT && boundingVolumes[objIndex2 * AABB_STRIDE + AABB_AXIS_Y] < ZERO_FLOAT) // if minY(objIndex2) < 0, means it is under the plane
            {
                const sp_uint temp = atomic_add(outputLength, 2);
                output[temp    ] = objIndex1;
                output[temp + 1] = objIndex2;
            }
        }

        return;
    }

    const sp_uint boundingVolumeIndex1 = objIndex1 * AABB_STRIDE;

    for (sp_uint j = index + 1u; j < *indexesLength; j++) // iterate over next elements
    {
        const sp_uint objIndex2 = indexes[j];

        // TODO: REMOVE
        if (objIndex2 == ZERO_UINT) // special handler for plane
            continue;

        const sp_uint boundingVolumeIndex2 = objIndex2 * AABB_STRIDE;

        if (AABB_MAX_POINT < AABB_MIN_POINT_NEXT_ELEMENT)  // if max currernt element < than min of next element, means this element does not collide with nobody else beyond
            return;

        if (   (AABB_maxPointX >= AABB_MIN_POINT_NEXT_ELEMENT_X && AABB_minPointX <= AABB_MAX_POINT_NEXT_ELEMENT_X)
            && (AABB_maxPointY >= AABB_MIN_POINT_NEXT_ELEMENT_Y && AABB_minPointY <= AABB_MAX_POINT_NEXT_ELEMENT_Y)
            && (AABB_maxPointZ >= AABB_MIN_POINT_NEXT_ELEMENT_Z && AABB_minPointZ <= AABB_MAX_POINT_NEXT_ELEMENT_Z)
        )
        {
            const sp_uint temp = atomic_add(outputLength, 2);
            output[temp    ] = objIndex1;
            output[temp + 1] = objIndex2;
        }
    }

#undef index
#undef axis
}


__kernel void sweepAndPruneSingleAxisSphere(
    __global   sp_float* boundingVolumes,
    __global   sp_float* elements,
    __global   sp_float* rigidBodies3D,
    __constant sp_uint * indexesLength,
    __global   sp_uint * indexes,
    __global   sp_uint * outputLength,
    __global   sp_uint * output)
{
#define axis THREAD_OFFSET
#define index (THREAD_ID - THREAD_OFFSET)

    if (index + 1u > *indexesLength)
        return;

    const sp_uint objIndex1 = indexes[index];
    const sp_uint boundingVolumeIndex1 = objIndex1 * SPHERE_STRIDE;

    Vec3 positionObj1;
    positionObj1.x         = boundingVolumes[boundingVolumeIndex1    ];
    positionObj1.y         = boundingVolumes[boundingVolumeIndex1 + 1];
    positionObj1.z         = boundingVolumes[boundingVolumeIndex1 + 2];
    const sp_float rayObj1 = boundingVolumes[boundingVolumeIndex1 + 3];

    const sp_float endObj1 = elements[objIndex1] + (rayObj1 * TWO_FLOAT);

    for (sp_uint j = index + 1u; j < *indexesLength; j++) // iterate over next elements
    {
        const sp_uint objIndex2 = indexes[j];

        if (elements[objIndex2] > endObj1)
            return;

        const sp_uint boundingVolumeIndex2 = objIndex2 * SPHERE_STRIDE;

        Vec3 positionObj2;
        positionObj2.x         = boundingVolumes[boundingVolumeIndex2    ];
        positionObj2.y         = boundingVolumes[boundingVolumeIndex2 + 1];
        positionObj2.z         = boundingVolumes[boundingVolumeIndex2 + 2];
        const sp_float rayObj2 = boundingVolumes[boundingVolumeIndex2 + 3];

        if (rayObj1 + rayObj2 >= distance(positionObj1, positionObj2))
        {
            const sp_uint temp = atomic_add(outputLength, 2);
            output[temp    ] = objIndex1;
            output[temp + 1] = objIndex2;
        }
    }

#undef index
#undef axis
}
