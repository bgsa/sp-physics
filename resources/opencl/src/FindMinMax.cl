#include "OpenCLBase.cl"

#define OFFSET_GLOBAL (INPUT_STRIDE) + (INPUT_OFFSET)

__kernel void findMinMaxByThread(
    __constant sp_float * input,
    __constant sp_uint  * indexes,
    __constant sp_uint  * indexesLength,
    __global   sp_float * output
    )
{
    __private const sp_uint elementsPerWorkItem = max((sp_uint) ( *indexesLength / THREAD_LENGTH ), ONE_UINT );
    __private       sp_float minValue = FLT_MAX;
    __private       sp_float maxValue = -FLT_MAX;

    __private const sp_uint inputIndex = THREAD_ID * elementsPerWorkItem;
    __private const sp_uint outputIndex = multiplyBy2(THREAD_ID);
    __private       sp_uint currentIndex;

    for(sp_uint i = 0 ; i < elementsPerWorkItem ; i++ )
    {
        currentIndex = indexes[inputIndex + i] * OFFSET_GLOBAL;
        minValue = min( minValue , input[ currentIndex ] );
        maxValue = max( maxValue , input[ currentIndex ] );
    }

    output[outputIndex    ] = minValue;
    output[outputIndex + 1] = maxValue;
}


__kernel void findMinMaxParallelReduce(
    __global   sp_float* output
    )
{
    __private const sp_uint outputIndex = multiplyBy2(THREAD_ID);
    __private const sp_uint offset = outputIndex + multiplyBy2(THREAD_LENGTH);

    output[outputIndex  ] = min( output[outputIndex  ] , output[offset]   );
    output[outputIndex  ] = min( output[outputIndex  ] , output[offset+1] );

    output[outputIndex+1] = max( output[outputIndex+1] , output[offset]   );
    output[outputIndex+1] = max( output[outputIndex+1] , output[offset+1] );
}


__kernel void findMinMaxParallelReduce_OneThread(__global sp_float* output)
{
    output[0] = min( output[0] , output[2] );
    output[1] = max( output[1] , output[3] );
}


__kernel void findMinMaxParallelReduce_Odd(
    __constant sp_float* input,
    __constant sp_uint * indexes,
    __global   sp_float* output,
    __constant sp_uint*  indexesLength
    )
{
    __private const sp_uint offset = indexes[*indexesLength] * OFFSET_GLOBAL;

    output[0] = min( output[0] , input[offset] );
    output[1] = max( output[1] , input[offset] );
}
