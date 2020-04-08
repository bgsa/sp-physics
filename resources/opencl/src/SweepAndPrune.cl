#include "OpenCLBase.cl"

#define MIN_POINT_X aabbs[indexes[i] * INPUT_STRIDE + INPUT_OFFSET    ]
#define MIN_POINT_Y aabbs[indexes[i] * INPUT_STRIDE + INPUT_OFFSET + 1]
#define MIN_POINT_Z aabbs[indexes[i] * INPUT_STRIDE + INPUT_OFFSET + 2]
#define MAX_POINT_X aabbs[indexes[i] * INPUT_STRIDE + INPUT_OFFSET + 3]
#define MAX_POINT_Y aabbs[indexes[i] * INPUT_STRIDE + INPUT_OFFSET + 4]
#define MAX_POINT_Z aabbs[indexes[i] * INPUT_STRIDE + INPUT_OFFSET + 5]

#define MIN_POINT_X_NEXT_AABB aabbs[indexes[j] * INPUT_STRIDE + INPUT_OFFSET    ]
#define MIN_POINT_Y_NEXT_AABB aabbs[indexes[j] * INPUT_STRIDE + INPUT_OFFSET + 1]
#define MIN_POINT_Z_NEXT_AABB aabbs[indexes[j] * INPUT_STRIDE + INPUT_OFFSET + 2]
#define MAX_POINT_X_NEXT_AABB aabbs[indexes[j] * INPUT_STRIDE + INPUT_OFFSET + 3]
#define MAX_POINT_Y_NEXT_AABB aabbs[indexes[j] * INPUT_STRIDE + INPUT_OFFSET + 4]
#define MAX_POINT_Z_NEXT_AABB aabbs[indexes[j] * INPUT_STRIDE + INPUT_OFFSET + 5]

__kernel void sweepAndPrune(
	__global   sp_float* aabbs, 
	__constant sp_uint* aabbsCount, 
    __global   sp_uint* indexes, 
	__global   sp_uint* collisionLength, 
	__global   sp_uint* collisions)
{
    __private const sp_uint elementsPerWorkItem = *aabbsCount / THREAD_LENGTH;
    __private const sp_uint inputThreadIndex = THREAD_ID * elementsPerWorkItem;
    __private sp_uint outputIndex = 0;

    for(sp_uint i = inputThreadIndex ; i < inputThreadIndex + elementsPerWorkItem ; i++ )  // compute each element for this thread
    {
        for(sp_uint j = i + 1; j < *aabbsCount; j++) // compute one element
        {
            if (MAX_POINT_X < MIN_POINT_X_NEXT_AABB)  // if maxPoint currernt AABB greater than minPoint of right AABB, they do not collides in X axis
                break;

            if (MAX_POINT_Y >= MIN_POINT_Y_NEXT_AABB && MIN_POINT_Y <= MAX_POINT_Y_NEXT_AABB    //check AABB collision
             && MAX_POINT_Z >= MIN_POINT_Z_NEXT_AABB && MIN_POINT_Z <= MAX_POINT_Z_NEXT_AABB)
            {
                outputIndex = atomic_add(collisionLength, 2);
                collisions[outputIndex] = i;
                collisions[outputIndex + 1] = j;
            }
        }
    }

}
