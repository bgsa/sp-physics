#include "OpenCLBase.cl"

#define OFFSET_GLOBAL INPUT_STRIDE + INPUT_OFFSET
#define BUCKET_LENGTH 10

#ifndef INPUT_LENGTH
    #define INPUT_LENGTH (*inputLength)
#endif

#define UPDATE_GLOBAL_OFFSET_TABLE()\
            offsetTable[globalBucketOffset    ] = bucket[0]; \
            offsetTable[globalBucketOffset + 1] = bucket[1]; \
            offsetTable[globalBucketOffset + 2] = bucket[2]; \
            offsetTable[globalBucketOffset + 3] = bucket[3]; \
            offsetTable[globalBucketOffset + 4] = bucket[4]; \
            offsetTable[globalBucketOffset + 5] = bucket[5]; \
            offsetTable[globalBucketOffset + 6] = bucket[6]; \
            offsetTable[globalBucketOffset + 7] = bucket[7]; \
            offsetTable[globalBucketOffset + 8] = bucket[8]; \
            offsetTable[globalBucketOffset + 9] = bucket[9];

#define INIT_START_INDEXES()\
            startIndex[0] = 0;                                                            \
            startIndex[1] =                 offsetTable[offsetTable_LastBucketIndex    ]; \
            startIndex[2] = startIndex[1] + offsetTable[offsetTable_LastBucketIndex + 1]; \
            startIndex[3] = startIndex[2] + offsetTable[offsetTable_LastBucketIndex + 2]; \
            startIndex[4] = startIndex[3] + offsetTable[offsetTable_LastBucketIndex + 3]; \
            startIndex[5] = startIndex[4] + offsetTable[offsetTable_LastBucketIndex + 4]; \
            startIndex[6] = startIndex[5] + offsetTable[offsetTable_LastBucketIndex + 5]; \
            startIndex[7] = startIndex[6] + offsetTable[offsetTable_LastBucketIndex + 6]; \
            startIndex[8] = startIndex[7] + offsetTable[offsetTable_LastBucketIndex + 7]; \
            startIndex[9] = startIndex[8] + offsetTable[offsetTable_LastBucketIndex + 8];


#define UPDATE_OFFSET_TABLE()\
            nextOffsetTable[offsetTableIndex    ] = previousOffsetTable[offset    ] + previousOffsetTable[offsetTableIndex    ]; \
            nextOffsetTable[offsetTableIndex + 1] = previousOffsetTable[offset + 1] + previousOffsetTable[offsetTableIndex + 1]; \
            nextOffsetTable[offsetTableIndex + 2] = previousOffsetTable[offset + 2] + previousOffsetTable[offsetTableIndex + 2]; \
            nextOffsetTable[offsetTableIndex + 3] = previousOffsetTable[offset + 3] + previousOffsetTable[offsetTableIndex + 3]; \
            nextOffsetTable[offsetTableIndex + 4] = previousOffsetTable[offset + 4] + previousOffsetTable[offsetTableIndex + 4]; \
            nextOffsetTable[offsetTableIndex + 5] = previousOffsetTable[offset + 5] + previousOffsetTable[offsetTableIndex + 5]; \
            nextOffsetTable[offsetTableIndex + 6] = previousOffsetTable[offset + 6] + previousOffsetTable[offsetTableIndex + 6]; \
            nextOffsetTable[offsetTableIndex + 7] = previousOffsetTable[offset + 7] + previousOffsetTable[offsetTableIndex + 7]; \
            nextOffsetTable[offsetTableIndex + 8] = previousOffsetTable[offset + 8] + previousOffsetTable[offsetTableIndex + 8]; \
            nextOffsetTable[offsetTableIndex + 9] = previousOffsetTable[offset + 9] + previousOffsetTable[offsetTableIndex + 9];



#define MANTISSA_LENGTH 1000  // = 10^3  // 3 mantissas digits are taken
#define DIGIT_WITH_POWER_D(doubleValue, power)\
                    (sp_uint) fmod( (doubleValue * MANTISSA_LENGTH) / power, 10.0)

#define DIGIT_WITH_POWER(floatValue, power)\
            DIGIT_WITH_POWER_D( (sp_double) fabs(floatValue), power )

#define DIGIT(floatValue, idx)\
                    DIGIT_WITH_POWER_F( floatValue , pow(10.0, idx) )

__kernel void count(
    __constant sp_float* input,
    __constant sp_uint * indexes,
    __constant sp_uint * inputLength,
    __global   sp_uint * offsetTable
    )
{
    __private const sp_uint   elementsPerWorkItem = max( (sp_uint) (INPUT_LENGTH / THREAD_LENGTH) , ONE_UINT );
    __private const sp_uint   globalBucketOffset = (THREAD_ID - THREAD_OFFSET) * BUCKET_LENGTH;    
    __private const sp_uint   inputThreadIndex = (THREAD_ID - THREAD_OFFSET) * elementsPerWorkItem;
    __private       sp_uint   bucket[BUCKET_LENGTH];
    __private const sp_double power = pow(10.0, THREAD_OFFSET);

    SET_ARRAY_10_ELEMENTS( bucket, ZERO_UINT )

    for (sp_uint i = 0 ; i < elementsPerWorkItem; i++) // make private histogram for expoent of float
        bucket[
            DIGIT_WITH_POWER( input[indexes[inputThreadIndex + i] * OFFSET_GLOBAL] , power )
        ]++;

    UPDATE_GLOBAL_OFFSET_TABLE();
}

__kernel void countNegatives(
    __constant sp_float* input,
    __constant sp_uint * indexes,
    __constant sp_uint * inputLength,
    __global   sp_uint * offsetTable
    )
{
    __private const sp_uint   elementsPerWorkItem = max( (sp_uint) (INPUT_LENGTH / THREAD_LENGTH) , ONE_UINT );
    __private const sp_uint   globalBucketOffset = THREAD_ID * TWO_UINT;
    __private const sp_uint   inputThreadIndex = THREAD_ID * elementsPerWorkItem;
    __private       sp_uint   negatives = ZERO_UINT;
    __private       sp_uint   positives = ZERO_UINT;

    for (sp_uint i = 0 ; i < elementsPerWorkItem; i++) // make private histogram for expoent of float
        if( input[indexes[inputThreadIndex + i] * OFFSET_GLOBAL] < ZERO_FLOAT )
            negatives++;
        else
            positives++;

    offsetTable[globalBucketOffset    ] = negatives;
    offsetTable[globalBucketOffset + 1] = positives;
}


__kernel void prefixScan(
    __global sp_uint * previousOffsetTable,
    __global sp_uint * nextOffsetTable
    )
{
    __private const sp_uint offsetTableIndex = (THREAD_ID - THREAD_OFFSET) * BUCKET_LENGTH;
    __private const sp_uint offset = offsetTableIndex - THREAD_OFFSET;

    if (offsetTableIndex < offset)
    {
        COPY_ARRAY_10_ELEMENTS(previousOffsetTable, nextOffsetTable, offsetTableIndex)
    }
    else 
    {
        UPDATE_OFFSET_TABLE()
    }
}

__kernel void prefixScanNegatives(
    __global sp_uint * previousOffsetTable,
    __global sp_uint * nextOffsetTable
    )
{
    __private const sp_uint offsetTableIndex = (THREAD_ID - THREAD_OFFSET) * TWO_UINT;
    __private const sp_uint offset = offsetTableIndex - THREAD_OFFSET;

    if (offsetTableIndex < offset)
    {
        nextOffsetTable[offsetTableIndex    ] = previousOffsetTable[offsetTableIndex    ];
        nextOffsetTable[offsetTableIndex + 1] = previousOffsetTable[offsetTableIndex + 1];
    }
    else 
    {
        nextOffsetTable[offsetTableIndex    ] = previousOffsetTable[offset    ] + previousOffsetTable[offsetTableIndex    ];
        nextOffsetTable[offsetTableIndex + 1] = previousOffsetTable[offset + 1] + previousOffsetTable[offsetTableIndex + 1];
    }
}


__kernel void prefixScanUp(
    __global sp_uint* offsetTable
    )
{
    __private sp_uint index = (THREAD_ID - THREAD_OFFSET + 1) * THREAD_OFFSET * BUCKET_LENGTH - BUCKET_LENGTH;
    __private sp_uint offset = index - (divideBy2(THREAD_OFFSET) * BUCKET_LENGTH);

    offsetTable[index    ] += offsetTable[offset    ];
    offsetTable[index + 1] += offsetTable[offset + 1];
    offsetTable[index + 2] += offsetTable[offset + 2];
    offsetTable[index + 3] += offsetTable[offset + 3];
    offsetTable[index + 4] += offsetTable[offset + 4];
    offsetTable[index + 5] += offsetTable[offset + 5];
    offsetTable[index + 6] += offsetTable[offset + 6];
    offsetTable[index + 7] += offsetTable[offset + 7];
    offsetTable[index + 8] += offsetTable[offset + 8];
    offsetTable[index + 9] += offsetTable[offset + 9];
}

#define PREFIX_SCAN_DOWN_STRIDE divideBy2(THREAD_OFFSET)
__kernel void prefixScanDown(
    __global sp_uint* offsetTable
    )
{
    __private sp_uint index = ((THREAD_ID - THREAD_OFFSET + 1) * THREAD_OFFSET + PREFIX_SCAN_DOWN_STRIDE) * BUCKET_LENGTH - BUCKET_LENGTH;
    __private sp_uint offset = index - (PREFIX_SCAN_DOWN_STRIDE * BUCKET_LENGTH);
    
    offsetTable[index    ] += offsetTable[offset    ];
    offsetTable[index + 1] += offsetTable[offset + 1];
    offsetTable[index + 2] += offsetTable[offset + 2];
    offsetTable[index + 3] += offsetTable[offset + 3];
    offsetTable[index + 4] += offsetTable[offset + 4];
    offsetTable[index + 5] += offsetTable[offset + 5];
    offsetTable[index + 6] += offsetTable[offset + 6];
    offsetTable[index + 7] += offsetTable[offset + 7];
    offsetTable[index + 8] += offsetTable[offset + 8];
    offsetTable[index + 9] += offsetTable[offset + 9];
}
#undef PREFIX_SCAN_DOWN_STRIDE


__kernel void reorder(
    __constant sp_float* input,
    __constant sp_uint * inputLength,
    __global   sp_uint * offsetTable,
    __constant sp_uint * indexesInput,
    __global   sp_uint * indexesOutput
    )
{
    __private const sp_uint   elementsPerWorkItem = max( (sp_uint) (INPUT_LENGTH / THREAD_LENGTH) , ONE_UINT );
    __private const sp_uint   indexesInputBegin = (THREAD_ID - THREAD_OFFSET) * elementsPerWorkItem;
    __private const sp_uint   offsetTable_Index = (THREAD_ID - THREAD_OFFSET) * BUCKET_LENGTH;
    __private const sp_uint   offsetTable_LastBucketIndex = ( min((sp_int)THREAD_LENGTH, INPUT_LENGTH) * BUCKET_LENGTH) - BUCKET_LENGTH;
    __private const sp_double power = pow(10.0, THREAD_OFFSET);

    __private sp_uint currentDigit;
    __private sp_uint startIndex[BUCKET_LENGTH];
    
    INIT_START_INDEXES()

    for (sp_int i = elementsPerWorkItem - ONE_INT; i >= ZERO_INT; i--)
    {
        currentDigit = DIGIT_WITH_POWER( input[indexesInput[indexesInputBegin + i] * OFFSET_GLOBAL] , power );

        indexesOutput[
                        startIndex[currentDigit] + offsetTable[offsetTable_Index + currentDigit] - 1 // get the global output address where the element is going to be stored
                    ] = indexesInput[indexesInputBegin + i];

        offsetTable[offsetTable_Index + currentDigit]--;    // decrement the offset table to store the others elements before
    }
}


__kernel void reorderNegatives(
    __constant sp_float* input,
    __constant sp_uint * inputLength,
    __global   sp_uint * offsetTable,
    __constant sp_uint * indexesInput,
    __global   sp_uint * indexesOutput
    )
{
    __private const sp_uint elementsPerWorkItem = max( (sp_uint) (INPUT_LENGTH / THREAD_LENGTH) , ONE_UINT );
    __private const sp_uint indexesInputBegin = THREAD_ID * elementsPerWorkItem;
    __private const sp_uint offsetTable_Index = THREAD_ID * TWO_UINT;
    __private const sp_uint offsetTable_LastBucketIndex = ( min((sp_int)THREAD_LENGTH, INPUT_LENGTH) * TWO_UINT) - TWO_UINT;

    __private sp_uint currentSign;
    __private sp_uint startIndex[2];

    startIndex[0] = ZERO_UINT;
    startIndex[1] = offsetTable[offsetTable_LastBucketIndex];
    
    for (sp_int i = elementsPerWorkItem - ONE_INT; i >= ZERO_INT; i--)
    {
        currentSign = input[indexesInput[indexesInputBegin + i] * OFFSET_GLOBAL] < ZERO_FLOAT ? ZERO_UINT : ONE_UINT;

        indexesOutput[
                        startIndex[currentSign] + (--offsetTable[offsetTable_Index + currentSign]) // get the global output address where the element is going to be stored
                    ] = indexesInput[indexesInputBegin + i];
    }
}
