#include "OpenCLBase.cl"

__kernel void create(
    __global sp_uint* indexes
)
{
    indexes[THREAD_ID] = THREAD_ID;
}
