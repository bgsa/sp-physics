#include "OpenCLBase.cl"

/// Find the first negative number from ordered list of numbers with negative sign ignored
/// Ex.:  1, 2, 3, -4, -5, -6, ...   
/// Result should be the index 3 starting from index 0
/// Requirements:  INPUT_STRIDE, INPUT_OFFSET
__kernel void findFirstNegative(
    __constant sp_float* input,
    __constant sp_uint*  indexes,
    __constant sp_uint*  indexesLength,
    __global   sp_uint*  output
)
{
    __private const sp_uint index = indexes[THREAD_ID];
    __private const sp_uint nextIndex = indexes[THREAD_ID + 1];

    if (index > *indexesLength) // guard
        return;

    if (input[index * INPUT_STRIDE + INPUT_OFFSET] >= ZERO_FLOAT && input[nextIndex * INPUT_STRIDE + INPUT_OFFSET] < ZERO_FLOAT)  // if has next and the next is lesser than zero, found!
        output[0] = THREAD_ID + 1u;
    else
        if (index == 0u && input[index * INPUT_STRIDE + INPUT_OFFSET] < ZERO_FLOAT)
            output[0] = 0u;
}
