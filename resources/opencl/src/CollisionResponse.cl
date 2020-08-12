#include "OpenCLBase.cl"
#include "DOP18.cl"
#include "SpPhysicProperties.cl"

__kernel void handleCollision(
    __constant sp_uint * indexes,
	__constant sp_uint * indexesLength,
    __global   sp_float* dops,
    __global   sp_float* physicProperties,
    __global   sp_uint * outputIndexesLength,
    __global   sp_uint * outputIndexes
)
{
    if (THREAD_ID + 1u > ((*indexesLength) / 2))
        return;

    const sp_uint index1 = indexes[THREAD_ID * 2    ];
    const sp_uint index2 = indexes[THREAD_ID * 2 + 1];
    
    const sp_uint physicIndex1 = index1 * SP_PHYSIC_PROPERTY_SIZE;
    const sp_uint physicIndex2 = index2 * SP_PHYSIC_PROPERTY_SIZE;

    const sp_bool isStaticObj1 = SpPhysicProperties_isStatic(physicProperties, physicIndex1);
    const sp_bool isStaticObj2 = SpPhysicProperties_isStatic(physicProperties, physicIndex2);

    if (isStaticObj1 && isStaticObj2) // if the objects are static, ignre them
        return;

    const Vec3 VEC3_ZERO = { 0.0f, 0.0f, 0.0f };
    const sp_bool isRestingObj1 = SpPhysicProperties_isResting(physicProperties, physicIndex1);
    const sp_bool isRestingObj2 = SpPhysicProperties_isResting(physicProperties, physicIndex2);

    const Vec3 previousPositionObj1 = {
        physicProperties[physicIndex1 + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX],
        physicProperties[physicIndex1 + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX + 1],
        physicProperties[physicIndex1 + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX + 2]
    };

    const Vec3 previousPositionObj2 = {
        physicProperties[physicIndex2 + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX],
        physicProperties[physicIndex2 + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX + 1],
        physicProperties[physicIndex2 + SP_PHYSIC_PROPERTY_PREV_POSITION_INDEX + 2]
    };

    Vec3 translation;
    vec3_minus_vec3(previousPositionObj2, previousPositionObj1, translation);
    translation.x = 0.0; translation.y = 0.0; translation.z = 0.0;

    if (isStaticObj1 && isRestingObj2)
    {
        if (!isStaticObj1 && !isStaticObj2)
            return;

        dop18_translate(dops, index2, translation);
        
        SpPhysicProperties_setPosition(physicProperties, physicIndex2, previousPositionObj2);
        SpPhysicProperties_setVelocity(physicProperties, physicIndex2, VEC3_ZERO);
        SpPhysicProperties_setAcceleration(physicProperties, physicIndex2, VEC3_ZERO);
        return;
    }

    if (isStaticObj2 && isRestingObj1)
    {
        dop18_translate(dops, index1, translation);
        
        SpPhysicProperties_setPosition(physicProperties, physicIndex1, previousPositionObj1);
        SpPhysicProperties_setVelocity(physicProperties, physicIndex1, VEC3_ZERO);
        SpPhysicProperties_setAcceleration(physicProperties, physicIndex1, VEC3_ZERO);
        return;
    }

    if (isRestingObj1 && isRestingObj2) // if one of them are not resting
    {
        dop18_translate(dops, index1, translation);
        
        SpPhysicProperties_setPosition(physicProperties, physicIndex1, previousPositionObj1);
        SpPhysicProperties_setVelocity(physicProperties, physicIndex1, VEC3_ZERO);
        SpPhysicProperties_setAcceleration(physicProperties, physicIndex1, VEC3_ZERO);

        dop18_translate(dops, index2, translation);
        
        SpPhysicProperties_setPosition(physicProperties, physicIndex2, previousPositionObj2);
        SpPhysicProperties_setVelocity(physicProperties, physicIndex2, VEC3_ZERO);
        SpPhysicProperties_setAcceleration(physicProperties, physicIndex2, VEC3_ZERO);

        return;
    }

    if (SpPhysicProperties_areMovingAway(physicProperties, physicIndex1, physicIndex2))
        return;

    const sp_uint temp = atomic_add(outputIndexesLength, 2);
    outputIndexes[temp    ] = index1;
    outputIndexes[temp + 1] = index2;
}
