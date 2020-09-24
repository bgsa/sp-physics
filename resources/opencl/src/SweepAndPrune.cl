#include "OpenCLBase.cl"
#include "DOP18.cl"
#include "SpPhysicProperties.cl"

#define MIN_POINT_NEXT_ELEMENT    input[dopIndex2 + axis]
#define MIN_POINT_NEXT_ELEMENT_X  input[dopIndex2     ]
#define MIN_POINT_NEXT_ELEMENT_Y  input[dopIndex2 +  1]
#define MIN_POINT_NEXT_ELEMENT_Z  input[dopIndex2 +  2]
#define MIN_POINT_NEXT_ELEMENT_XY input[dopIndex2 +  3]
#define MIN_POINT_NEXT_ELEMENT_YX input[dopIndex2 +  4]
#define MIN_POINT_NEXT_ELEMENT_YZ input[dopIndex2 +  5]
#define MIN_POINT_NEXT_ELEMENT_ZY input[dopIndex2 +  6]
#define MIN_POINT_NEXT_ELEMENT_XZ input[dopIndex2 +  7]
#define MIN_POINT_NEXT_ELEMENT_ZX input[dopIndex2 +  8]
#define MAX_POINT_NEXT_ELEMENT_X  input[dopIndex2 +  9]
#define MAX_POINT_NEXT_ELEMENT_Y  input[dopIndex2 + 10]
#define MAX_POINT_NEXT_ELEMENT_Z  input[dopIndex2 + 11]
#define MAX_POINT_NEXT_ELEMENT_XY input[dopIndex2 + 12]
#define MAX_POINT_NEXT_ELEMENT_YX input[dopIndex2 + 13]
#define MAX_POINT_NEXT_ELEMENT_YZ input[dopIndex2 + 14]
#define MAX_POINT_NEXT_ELEMENT_ZY input[dopIndex2 + 15]
#define MAX_POINT_NEXT_ELEMENT_XZ input[dopIndex2 + 16]
#define MAX_POINT_NEXT_ELEMENT_ZX input[dopIndex2 + 17]

#define MAX_POINT input[dopIndex1 + DOP18_ORIENTATIONS + axis]
#define minPointX input[dopIndex1      ]
#define minPointY input[dopIndex1  +  1]
#define minPointZ input[dopIndex1  +  2]
#define minPointXY input[dopIndex1 +  3]
#define minPointYX input[dopIndex1 +  4]
#define minPointYZ input[dopIndex1 +  5]
#define minPointZY input[dopIndex1 +  6]
#define minPointXZ input[dopIndex1 +  7]
#define minPointZX input[dopIndex1 +  8]
#define maxPointX input[dopIndex1  +  9]
#define maxPointY input[dopIndex1  + 10]
#define maxPointZ input[dopIndex1  + 11]
#define maxPointXY input[dopIndex1 + 12]
#define maxPointYX input[dopIndex1 + 13]
#define maxPointYZ input[dopIndex1 + 14]
#define maxPointZY input[dopIndex1 + 15]
#define maxPointXZ input[dopIndex1 + 16]
#define maxPointZX input[dopIndex1 + 17]

__kernel void sweepAndPruneSingleAxis(
	__global   sp_float* input,
    __global   sp_float* physicProperties,
	__constant sp_uint * indexesLength, 
    __global   sp_uint * indexes, 
	__global   sp_uint * outputLength, 
	__global   sp_uint * output)
{
    __private const sp_uint axis = THREAD_OFFSET;
    __private const sp_uint index = THREAD_ID - THREAD_OFFSET;

    if (index + 1u > *indexesLength)
        return;

    const sp_uint objIndex1 = indexes[index];
    const sp_uint dopIndex1 = objIndex1 * INPUT_STRIDE;

    const sp_bool isStaticObj1 = SpPhysicProperties_isStatic(physicProperties, objIndex1 * SP_PHYSIC_PROPERTY_SIZE);

    for(sp_uint j = index + 1u; j < *indexesLength; j++) // iterate over next elements
    {
        const sp_uint objIndex2 = indexes[j];
        const sp_uint dopIndex2 = objIndex2 * INPUT_STRIDE;

        if (MAX_POINT < MIN_POINT_NEXT_ELEMENT)  // if max currernt element < than min of next element, means this element does not collide with nobody else beyond
            return;

        if (   (maxPointX  >= MIN_POINT_NEXT_ELEMENT_X  && minPointX  <= MAX_POINT_NEXT_ELEMENT_X)
            && (maxPointY  >= MIN_POINT_NEXT_ELEMENT_Y  && minPointY  <= MAX_POINT_NEXT_ELEMENT_Y)
            && (maxPointZ  >= MIN_POINT_NEXT_ELEMENT_Z  && minPointZ  <= MAX_POINT_NEXT_ELEMENT_Z)
            && (maxPointXY >= MIN_POINT_NEXT_ELEMENT_XY && minPointXY <= MAX_POINT_NEXT_ELEMENT_XY)
            && (maxPointYX >= MIN_POINT_NEXT_ELEMENT_YX && minPointYX <= MAX_POINT_NEXT_ELEMENT_YX)
            && (maxPointYZ >= MIN_POINT_NEXT_ELEMENT_YZ && minPointYZ <= MAX_POINT_NEXT_ELEMENT_YZ)
            && (maxPointZY >= MIN_POINT_NEXT_ELEMENT_ZY && minPointZY <= MAX_POINT_NEXT_ELEMENT_ZY)
            && (maxPointXZ >= MIN_POINT_NEXT_ELEMENT_XZ && minPointXZ <= MAX_POINT_NEXT_ELEMENT_XZ)
            && (maxPointZX >= MIN_POINT_NEXT_ELEMENT_ZX && minPointZX <= MAX_POINT_NEXT_ELEMENT_ZX)  
        )
        {
            const sp_uint isStaticObj2 = SpPhysicProperties_isStatic(physicProperties, objIndex2 * SP_PHYSIC_PROPERTY_SIZE);

            if (isStaticObj1 && isStaticObj2) // if the objects are no static, inclulde on collision
                continue;

            const sp_uint temp = atomic_add(outputLength, 2);
            output[temp    ] = objIndex1;
            output[temp + 1] = objIndex2;
        }
    }
}
