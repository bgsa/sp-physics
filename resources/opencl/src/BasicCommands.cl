#include "OpenCLBase.cl"

__kernel void initIndexes(
    __constant sp_uint* length,
    __global   sp_uint* indexes
    )
{
    __private sp_uint elementsPerWorkItem = max( (sp_uint)(*length / THREAD_LENGTH) , 1u);
    __private sp_uint inputThreadIndex = THREAD_ID * elementsPerWorkItem;

    for (sp_uint i = 0 ; i < elementsPerWorkItem ; i++)
        indexes[inputThreadIndex + i] = inputThreadIndex + i;
}
