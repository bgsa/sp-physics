#include "OpenCLBase.cl"

// Reverse the values - The first element will be the last (inputLength).
// The elements after inputLength parameter will be copied.
// Ex.: Inputlength: 3 -> Input: { 1, 2, 3, 4, 5 } -> Output: { 3, 2, 1, 4, 5 }
__kernel void reverseOrCopy(
    __global sp_uint* inputValues,
    __global sp_uint* outputValues,
    __global sp_uint* inputLength
    )
{
    if (THREAD_ID < *inputLength)
        outputValues[*inputLength - (*inputLength - THREAD_ID)] = inputValues[*inputLength - THREAD_ID - ONE_UINT];
    else
        outputValues[THREAD_ID] = inputValues[THREAD_ID];
}
