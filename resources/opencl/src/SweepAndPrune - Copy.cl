#include "OpenCLBase.cl"
#include "SpPhysicProperties.cl"

#define MIN_POINT_NEXT_ELEMENT input[indexes[j] * INPUT_STRIDE + INPUT_OFFSET + axis                     ]
#define MAX_POINT_NEXT_ELEMENT input[indexes[j] * INPUT_STRIDE + INPUT_OFFSET + axis + ORIENTATION_LENGTH]

#define MIN_POINT_NEXT_ELEMENT_X  input[indexes[j] * INPUT_STRIDE + INPUT_OFFSET    ]
#define MIN_POINT_NEXT_ELEMENT_Y  input[indexes[j] * INPUT_STRIDE + INPUT_OFFSET + 1]
#define MIN_POINT_NEXT_ELEMENT_Z  input[indexes[j] * INPUT_STRIDE + INPUT_OFFSET + 2]
#define MIN_POINT_NEXT_ELEMENT_XY input[indexes[j] * INPUT_STRIDE + INPUT_OFFSET + 3]
#define MIN_POINT_NEXT_ELEMENT_YX input[indexes[j] * INPUT_STRIDE + INPUT_OFFSET + 4]
#define MIN_POINT_NEXT_ELEMENT_YZ input[indexes[j] * INPUT_STRIDE + INPUT_OFFSET + 5]
#define MIN_POINT_NEXT_ELEMENT_ZY input[indexes[j] * INPUT_STRIDE + INPUT_OFFSET + 6]
#define MIN_POINT_NEXT_ELEMENT_XZ input[indexes[j] * INPUT_STRIDE + INPUT_OFFSET + 7]
#define MIN_POINT_NEXT_ELEMENT_ZX input[indexes[j] * INPUT_STRIDE + INPUT_OFFSET + 8]
#define MAX_POINT_NEXT_ELEMENT_X  input[indexes[j] * INPUT_STRIDE + INPUT_OFFSET +     ORIENTATION_LENGTH]
#define MAX_POINT_NEXT_ELEMENT_Y  input[indexes[j] * INPUT_STRIDE + INPUT_OFFSET + 1 + ORIENTATION_LENGTH]
#define MAX_POINT_NEXT_ELEMENT_Z  input[indexes[j] * INPUT_STRIDE + INPUT_OFFSET + 2 + ORIENTATION_LENGTH]
#define MAX_POINT_NEXT_ELEMENT_XY input[indexes[j] * INPUT_STRIDE + INPUT_OFFSET + 3 + ORIENTATION_LENGTH]
#define MAX_POINT_NEXT_ELEMENT_YX input[indexes[j] * INPUT_STRIDE + INPUT_OFFSET + 4 + ORIENTATION_LENGTH]
#define MAX_POINT_NEXT_ELEMENT_YZ input[indexes[j] * INPUT_STRIDE + INPUT_OFFSET + 5 + ORIENTATION_LENGTH]
#define MAX_POINT_NEXT_ELEMENT_ZY input[indexes[j] * INPUT_STRIDE + INPUT_OFFSET + 6 + ORIENTATION_LENGTH]
#define MAX_POINT_NEXT_ELEMENT_XZ input[indexes[j] * INPUT_STRIDE + INPUT_OFFSET + 7 + ORIENTATION_LENGTH]
#define MAX_POINT_NEXT_ELEMENT_ZX input[indexes[j] * INPUT_STRIDE + INPUT_OFFSET + 8 + ORIENTATION_LENGTH]

__kernel void sweepAndPruneSingleAxis(
	__global   sp_float* input,
    __global   sp_float* physicProperties,
	__constant sp_uint * indexesLength, 
    __global   sp_uint * indexes, 
	__global   sp_uint * outputLength, 
	__global   sp_uint * output)
{

    __private const sp_uint axis = THREAD_OFFSET;
    __private const sp_uint index = THREAD_ID - axis;

    if (index + 1u > *indexesLength)
        return;

    //const sp_float minPoint = input[indexes[index] * INPUT_STRIDE + INPUT_OFFSET + axis];
    const sp_float maxPoint = input[indexes[index] * INPUT_STRIDE + INPUT_OFFSET + axis + ORIENTATION_LENGTH];

    const sp_float minPointX  = input[indexes[index] * INPUT_STRIDE + INPUT_OFFSET    ];
    const sp_float minPointY  = input[indexes[index] * INPUT_STRIDE + INPUT_OFFSET + 1];
    const sp_float minPointZ  = input[indexes[index] * INPUT_STRIDE + INPUT_OFFSET + 2];
    const sp_float minPointXY = input[indexes[index] * INPUT_STRIDE + INPUT_OFFSET + 3];
    const sp_float minPointYX = input[indexes[index] * INPUT_STRIDE + INPUT_OFFSET + 4];
    const sp_float minPointYZ = input[indexes[index] * INPUT_STRIDE + INPUT_OFFSET + 5];
    const sp_float minPointZY = input[indexes[index] * INPUT_STRIDE + INPUT_OFFSET + 6];
    const sp_float minPointXZ = input[indexes[index] * INPUT_STRIDE + INPUT_OFFSET + 7];
    const sp_float minPointZX = input[indexes[index] * INPUT_STRIDE + INPUT_OFFSET + 8];

    const sp_float maxPointX  = input[indexes[index] * INPUT_STRIDE + INPUT_OFFSET +     ORIENTATION_LENGTH];
    const sp_float maxPointY  = input[indexes[index] * INPUT_STRIDE + INPUT_OFFSET + 1 + ORIENTATION_LENGTH];
    const sp_float maxPointZ  = input[indexes[index] * INPUT_STRIDE + INPUT_OFFSET + 2 + ORIENTATION_LENGTH];
    const sp_float maxPointXY = input[indexes[index] * INPUT_STRIDE + INPUT_OFFSET + 3 + ORIENTATION_LENGTH];
    const sp_float maxPointYX = input[indexes[index] * INPUT_STRIDE + INPUT_OFFSET + 4 + ORIENTATION_LENGTH];
    const sp_float maxPointYZ = input[indexes[index] * INPUT_STRIDE + INPUT_OFFSET + 5 + ORIENTATION_LENGTH];
    const sp_float maxPointZY = input[indexes[index] * INPUT_STRIDE + INPUT_OFFSET + 6 + ORIENTATION_LENGTH];
    const sp_float maxPointXZ = input[indexes[index] * INPUT_STRIDE + INPUT_OFFSET + 7 + ORIENTATION_LENGTH];
    const sp_float maxPointZX = input[indexes[index] * INPUT_STRIDE + INPUT_OFFSET + 8 + ORIENTATION_LENGTH];

    const sp_uint stride    = index * SP_PHYSIC_PROPERTY_SIZE;
    const sp_bool isStaticObj1 = SpPhysicProperties_isStatic(physicProperties, stride);
    const sp_bool isRestingObj1 = SpPhysicProperties_isResting(physicProperties, stride);
    const Vec3 VEC3_ZERO = { 0.0f, 0.0f, 0.0f };

    const Vec3 previousPosition = {
        physicProperties[stride + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX],
        physicProperties[stride + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX + 1],
        physicProperties[stride + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX + 2]
    };

    for(sp_uint j = index + 1u; j < *indexesLength; j++) // iterate over next elements
    {
        if (maxPoint < MIN_POINT_NEXT_ELEMENT)  // if max currernt element < than min of next element, means this element does not collide with nobody else
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
            const sp_uint strideObj2 = j * SP_PHYSIC_PROPERTY_SIZE;
            const sp_uint isStaticObj2 = SpPhysicProperties_isStatic(physicProperties, strideObj2);

            if (isStaticObj1 && isStaticObj2) // if the objects are no static, inclulde on collision
                continue;

            const sp_bool isRestingObj2 = SpPhysicProperties_isResting(physicProperties, strideObj2);
            
            if (isRestingObj1 && isRestingObj2) // if one of them are not resting
            {
                const Vec3 previousPositionObj2 = {
                    physicProperties[strideObj2 + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX],
                    physicProperties[strideObj2 + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX + 1],
                    physicProperties[strideObj2 + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX + 2]
                };

                SpPhysicProperties_setPosition(physicProperties, stride, previousPosition);
                SpPhysicProperties_setVelocity(physicProperties, stride, VEC3_ZERO);
                SpPhysicProperties_setAcceleration(physicProperties, stride, VEC3_ZERO);

                SpPhysicProperties_setPosition(physicProperties, strideObj2, previousPositionObj2);
                SpPhysicProperties_setVelocity(physicProperties, strideObj2, VEC3_ZERO);
                SpPhysicProperties_setAcceleration(physicProperties, strideObj2, VEC3_ZERO);

                continue;
            }

            if (isStaticObj1 && isRestingObj2)
            {
                SpPhysicProperties_setVelocity(physicProperties, strideObj2, VEC3_ZERO);
                SpPhysicProperties_setAcceleration(physicProperties, strideObj2, VEC3_ZERO);
                continue;
            }

            if (isStaticObj2 && isRestingObj1)
            {
                SpPhysicProperties_setVelocity(physicProperties, stride, VEC3_ZERO);
                SpPhysicProperties_setAcceleration(physicProperties, stride, VEC3_ZERO);
                continue;
            }

            const sp_uint temp = atomic_add(outputLength, 2);
            output[temp] = indexes[index];
            output[temp + 1] = indexes[j];
        }
    }
}
