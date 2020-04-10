#define OVERLOAD  __attribute__((overloadable))

#define THREAD_ID           get_global_id(0)
#define THREAD_LENGTH       get_global_size(0)
#define THREAD_LOCAL_ID     get_local_id(0)
#define THREAD_LOCAL_LENGTH get_local_size(0)
#define THREAD_OFFSET       get_global_offset(0)
#define GROUP_ID            get_group_id(0)
#define GROUP_LENGTH        get_num_groups(0)

#define isEven(value) value % 2 == 0
#define isOdd(value)  value % 2 != 0

#define multiplyBy2(value) (value << 1)
#define multiplyBy4(value) (value << 2)
#define multiplyBy8(value) (value << 3)
#define multiplyBy16(value) (value << 4)

#define divideBy2(value) (value >> 1)
#define divideBy4(value) (value >> 2)
#define divideBy8(value) (value >> 3)
#define divideBy16(value) (value >> 4)

//#define ENV_64BITS
#ifdef ENV_64BITS
    #pragma OPENCL EXTENSION cl_khr_int64_base_atomics: enable
#endif

typedef bool           sp_bool;
typedef char           sp_char;
typedef short          sp_short;
typedef int            sp_int;
typedef long           sp_long;
typedef unsigned int   sp_uint;
typedef unsigned short sp_ushort;
typedef unsigned long  sp_ulong;
typedef unsigned int   sp_size;

typedef float          sp_float;
typedef double         sp_double;

#define ZERO_UINT 0u
#define ONE_UINT  1u
#define TWO_UINT  2u

#define SET_ARRAY_10_ELEMENTS(_name, _value)\
            _name[0] = _value;  \
            _name[1] = _value;  \
            _name[2] = _value;  \
            _name[3] = _value;  \
            _name[4] = _value;  \
            _name[5] = _value;  \
            _name[6] = _value;  \
            _name[7] = _value;  \
            _name[8] = _value;  \
            _name[9] = _value;  \

#define COPY_ARRAY_10_ELEMENTS(_sourceArray, _destinyArray, _offset)\
            _destinyArray[_offset]     = _sourceArray[_offset];      \
            _destinyArray[_offset + 1] = _sourceArray[_offset + 1];  \
            _destinyArray[_offset + 2] = _sourceArray[_offset + 2];  \
            _destinyArray[_offset + 3] = _sourceArray[_offset + 3];  \
            _destinyArray[_offset + 4] = _sourceArray[_offset + 4];  \
            _destinyArray[_offset + 5] = _sourceArray[_offset + 5];  \
            _destinyArray[_offset + 6] = _sourceArray[_offset + 6];  \
            _destinyArray[_offset + 7] = _sourceArray[_offset + 7];  \
            _destinyArray[_offset + 8] = _sourceArray[_offset + 8];  \
            _destinyArray[_offset + 9] = _sourceArray[_offset + 9];  \


sp_bool isPowerOf2(sp_uint value)
{
    return value && !(value & (value - 1));
}

sp_uint nextPowOf2(sp_uint value)
{
    sp_uint rval = 1;

    while (rval < value) 
        rval = multiplyBy2(rval);

    return rval;
}
