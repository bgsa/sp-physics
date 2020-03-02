#include "OpenCLBase.cl"

#define OFFSET_GLOBAL INPUT_STRIDE + INPUT_OFFSET

/*
__kernel void sort(
    __constant float * input,
    __constant size_t* indexes,
    __constant size_t* inputLength,
    __global   size_t* output
    )
{
    if (THREAD_ID + 1 > *inputLength) // guard
        return;

    __private size_t elementsPerWorkItem = max( (int) (*inputLength / THREAD_LENGTH) , 1 );
    __private size_t inputThreadIndex = THREAD_ID * elementsPerWorkItem;
    __private size_t maxElements = min( inputThreadIndex + elementsPerWorkItem, *inputLength ) - 1;
    __private size_t j, temp;
    
    for(size_t i = inputThreadIndex ; i < maxElements ; i++) // interate over all elements for this thread
    {

        if ( input[indexes[i]] < input[indexes[i+1]] )
        {
            output[i] = indexes[i];
        }
        else
        {
            output[i] = indexes[i];
            output[i+1] = indexes[i+1];

            j = i+1;
            do
            {
                j--;

                //swap
                temp = output[j];
                output[j] = output[j+1];
                output[j+1] = temp;

            } while( j > 0 && input[output[j]] < input[output[j-1]] );
        }

    }
    
}
*/

__kernel void sort(
    __global float * input,
    __global size_t* indexes,
    __global size_t* inputLength
    )
{
    if (THREAD_ID + 1 > *inputLength) // guard
        return;

    __private size_t elementsPerWorkItem = max( (int) (*inputLength / THREAD_LENGTH) , 1 );
    __private size_t inputThreadIndex = THREAD_ID * elementsPerWorkItem;
    __private size_t inputLastThreadIndex = min( inputThreadIndex + elementsPerWorkItem, *inputLength ) - 1;
    __private size_t i, j, temp;
    
    // sort elements for this thread
    for(i = inputThreadIndex ; i < inputLastThreadIndex ; i++)
    {
        if ( input[indexes[i]] > input[indexes[i+1]] )
        {
            j = i+1;
            do
            {
                j--;

                //swap
                temp = indexes[j];
                indexes[j] = indexes[j+1];
                indexes[j+1] = temp;

            } while( j > 0 && input[indexes[j]] < input[indexes[j-1]] );
        }
    }

    barrier(CLK_GLOBAL_MEM_FENCE);

for(size_t w = 0 ; w < 9 ; w++)
{

    // sort elements for this thread with the next one
    i = inputLastThreadIndex; // 255
    size_t nextElement = i+1;

    if ( nextElement < *inputLength )
    {
        do
        {
            while( i >= inputThreadIndex && input[indexes[i]] > input[indexes[i+1]] )
            {
                //swap
                temp = indexes[i];
                indexes[i] = indexes[i+1];
                indexes[i+1] = temp;

                i--;
            }

            nextElement++;
            i = nextElement; // 127+1

            barrier(CLK_GLOBAL_MEM_FENCE);

        } while( i < *inputLength && input[indexes[i]] > input[indexes[i+1]] );

barrier(CLK_GLOBAL_MEM_FENCE);

        for(i = inputThreadIndex ; i < inputLastThreadIndex ; i++)
        {
            if ( input[indexes[i]] > input[indexes[i+1]] )
            {
                j = i+1;
                do
                {
                    j--;

                    //swap
                    temp = indexes[j];
                    indexes[j] = indexes[j+1];
                    indexes[j+1] = temp;

                } while( j > 0 && input[indexes[j]] < input[indexes[j-1]] );
            }
        }
    }
    
barrier(CLK_GLOBAL_MEM_FENCE);

}

    
}
