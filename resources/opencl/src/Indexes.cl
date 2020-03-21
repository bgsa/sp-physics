#include "OpenCLBase.cl"

__kernel void create(
    __constant sp_uint* length,
    __global   sp_uint* indexes
    )
{
    __private const sp_uint elementsPerWorkItem = max( (sp_uint)(*length / THREAD_LENGTH) , ONE_UINT);
    __private const sp_uint inputThreadIndex = THREAD_ID * elementsPerWorkItem;

    for (sp_uint i = 0 ; i < elementsPerWorkItem ; i++)
        indexes[inputThreadIndex + i] = inputThreadIndex + i;
}
