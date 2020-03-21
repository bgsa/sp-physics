#include "OpenCLBase.cl"

#define OFFSET_GLOBAL INPUT_STRIDE + INPUT_OFFSET
#define MAX_DIGITS_DECIMALS 4
#define BUCKET_LENGTH 10

#ifndef INPUT_LENGTH
    #define INPUT_LENGTH (*inputLength)
#endif

#define UPDATE_GLOBAL_OFFSET_TABLE()\
            offsetTable[globalBucketOffset + 0] = bucket[0]; \
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
            startIndex[1] =                 offsetTable[offsetTable_LastBucketIndex];     \
            startIndex[2] = startIndex[1] + offsetTable[offsetTable_LastBucketIndex + 1]; \
            startIndex[3] = startIndex[2] + offsetTable[offsetTable_LastBucketIndex + 2]; \
            startIndex[4] = startIndex[3] + offsetTable[offsetTable_LastBucketIndex + 3]; \
            startIndex[5] = startIndex[4] + offsetTable[offsetTable_LastBucketIndex + 4]; \
            startIndex[6] = startIndex[5] + offsetTable[offsetTable_LastBucketIndex + 5]; \
            startIndex[7] = startIndex[6] + offsetTable[offsetTable_LastBucketIndex + 6]; \
            startIndex[8] = startIndex[7] + offsetTable[offsetTable_LastBucketIndex + 7]; \
            startIndex[9] = startIndex[8] + offsetTable[offsetTable_LastBucketIndex + 8];


#define UPDATE_OFFSET_TABLE()\
            nextOffsetTable[offsetTableIndex] = previousOffsetTable[offsetTableIndex - offset] + previousOffsetTable[offsetTableIndex]; \
            nextOffsetTable[offsetTableIndex + 1] = previousOffsetTable[offsetTableIndex + 1 - offset] + previousOffsetTable[offsetTableIndex + 1]; \
            nextOffsetTable[offsetTableIndex + 2] = previousOffsetTable[offsetTableIndex + 2 - offset] + previousOffsetTable[offsetTableIndex + 2]; \
            nextOffsetTable[offsetTableIndex + 3] = previousOffsetTable[offsetTableIndex + 3 - offset] + previousOffsetTable[offsetTableIndex + 3]; \
            nextOffsetTable[offsetTableIndex + 4] = previousOffsetTable[offsetTableIndex + 4 - offset] + previousOffsetTable[offsetTableIndex + 4]; \
            nextOffsetTable[offsetTableIndex + 5] = previousOffsetTable[offsetTableIndex + 5 - offset] + previousOffsetTable[offsetTableIndex + 5]; \
            nextOffsetTable[offsetTableIndex + 6] = previousOffsetTable[offsetTableIndex + 6 - offset] + previousOffsetTable[offsetTableIndex + 6]; \
            nextOffsetTable[offsetTableIndex + 7] = previousOffsetTable[offsetTableIndex + 7 - offset] + previousOffsetTable[offsetTableIndex + 7]; \
            nextOffsetTable[offsetTableIndex + 8] = previousOffsetTable[offsetTableIndex + 8 - offset] + previousOffsetTable[offsetTableIndex + 8]; \
            nextOffsetTable[offsetTableIndex + 9] = previousOffsetTable[offsetTableIndex + 9 - offset] + previousOffsetTable[offsetTableIndex + 9];


//#define digitMantissa(value, index) \
//    (sp_uint) ( (sp_uint) (  (fabs(((sp_float) ((sp_uint) value)) - value) * 10000)   / pow(10.0, (sp_double) index)) % 10);

sp_uint OVERLOAD digit(sp_float value, sp_uint index)
{
    sp_uint mantissa = (sp_uint) (fabs(((sp_float) ((sp_uint) value)) - value) * 10000);

    return (sp_uint) ( (sp_uint) (mantissa / pow(10.0, (sp_double) index)) % 10);
}

sp_uint OVERLOAD digit(sp_int value, sp_uint index)
{
    return ((sp_int) (value  / pow(10.0, (sp_double) index))) % 10;
}

__kernel void count(
    __constant sp_float* input,
    __constant sp_uint * indexes,
    __constant sp_uint * inputLength,
    __constant sp_uint * digitIndex,
    __constant sp_bool * useExpoent,
    __constant sp_float* minMaxValues,
    __global   sp_uint * offsetTable
    )
{
    __private const sp_uint  elementsPerWorkItem = max( (sp_uint) (INPUT_LENGTH / THREAD_LENGTH) , ONE_UINT );
    __private const sp_uint  globalBucketOffset = BUCKET_LENGTH * THREAD_ID;    
    __private const sp_uint  inputThreadIndex = THREAD_ID * elementsPerWorkItem;
    __private const sp_float minValue = -min(0.0f, minMaxValues[0]);
    __private       sp_uint  bucket[BUCKET_LENGTH];
    
    SET_ARRAY_10_ELEMENTS( bucket, ZERO_UINT )

    if (*useExpoent)
        for (sp_uint i = 0 ; i < elementsPerWorkItem; i++) // make private histogram for expoent of float
        {
            bucket[
                digit((sp_int) (input[indexes[inputThreadIndex + i] * OFFSET_GLOBAL] + minValue), *digitIndex)
            ]++;
        }
    else
        for (sp_uint i = 0 ; i < elementsPerWorkItem ; i++) // make private histogram for mantissa of float
        {
            bucket[
                digit(input[indexes[inputThreadIndex + i] * OFFSET_GLOBAL] + minValue, *digitIndex)
            ]++;
        }

    UPDATE_GLOBAL_OFFSET_TABLE();
}

__kernel void prefixScan(
    __global sp_uint * previousOffsetTable,
    __global sp_uint * nextOffsetTable,
    __global sp_uint * offsetPrefixScan
    )
{
    __private const sp_uint offsetTableIndex = THREAD_ID * BUCKET_LENGTH;
    __private const sp_uint offset = *offsetPrefixScan;

    if (offsetTableIndex < offset)
    {
        COPY_ARRAY_10_ELEMENTS(nextOffsetTable, previousOffsetTable, offsetTableIndex)
    }
    else 
    {
        UPDATE_OFFSET_TABLE()
    }

    barrier(CLK_GLOBAL_MEM_FENCE);
    if (THREAD_ID == 0)
        offsetPrefixScan[0] = multiplyBy2(*offsetPrefixScan);
}

__kernel void reorder(
    __constant sp_float* input,
    __constant sp_uint * inputLength,
    __global   sp_uint * digitIndex_global,
    __global   sp_bool * useExpoent,
    __global   sp_uint * offsetTable,
    __constant sp_float* minMaxValues,
    __constant sp_uint * indexesInput,
    __global   sp_uint * indexesOutput,
    __global   sp_uint * offsetPrefixScan
    )
{
    __private const sp_uint  elementsPerWorkItem = max( (sp_uint) (INPUT_LENGTH / THREAD_LENGTH) , ONE_UINT );
    __private const sp_uint  digitIndex = *digitIndex_global;
    __private const sp_uint  indexesInputBegin = THREAD_ID * elementsPerWorkItem;
    __private const sp_uint  offsetTable_Index = THREAD_ID * BUCKET_LENGTH;
    __private const sp_uint  offsetTable_LastBucketIndex = ( min((sp_int)THREAD_LENGTH, INPUT_LENGTH) * BUCKET_LENGTH) - BUCKET_LENGTH;
    __private const sp_float minValue = -min(0.0f, minMaxValues[0]);

    __private sp_uint globalAddress;
    __private sp_uint currentDigit;
    __private sp_uint startIndex[BUCKET_LENGTH];

    INIT_START_INDEXES()

    if (*useExpoent)
        for (sp_int i = elementsPerWorkItem - 1; i >= 0; i--)
        {
            currentDigit = digit((sp_int) (input[indexesInput[indexesInputBegin + i] * OFFSET_GLOBAL] + minValue), digitIndex);  // get the digit to process            
            
            globalAddress = startIndex[currentDigit] + offsetTable[offsetTable_Index + currentDigit] - 1; // get the global output address where the element is going to be stored

            indexesOutput[globalAddress] = indexesInput[indexesInputBegin + i];

            offsetTable[offsetTable_Index + currentDigit]--;    // decrement the offset table to store the others elements before
        }
    else
        for (sp_int i = elementsPerWorkItem - 1; i >= 0; i--)
        {
            currentDigit = digit(input[indexesInput[indexesInputBegin + i] * OFFSET_GLOBAL] + minValue, digitIndex);  // get the digit to process

            globalAddress = startIndex[currentDigit] + offsetTable[offsetTable_Index + currentDigit] - 1; // get the global output address where the element is going to be stored

            indexesOutput[globalAddress] = indexesInput[indexesInputBegin + i];

            offsetTable[offsetTable_Index + currentDigit]--;    // decrement the offset table to store the others elements before
        }

    barrier(CLK_GLOBAL_MEM_FENCE);
    if (THREAD_ID)
        offsetPrefixScan[0] = BUCKET_LENGTH;

    if (THREAD_ID == 0)
    {
        if ( !useExpoent[0] && digitIndex == MAX_DIGITS_DECIMALS - 1) // if decimal digits is being processed and it already have processed 4 digits ...
        {
            useExpoent[0] = true;
            digitIndex_global[0] = 0;
        }
        else
            digitIndex_global[0] = *digitIndex_global + 1;
    }

}
