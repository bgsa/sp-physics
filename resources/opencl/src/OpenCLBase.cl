#define OVERLOAD  __attribute__((overloadable))

#define THREAD_ID           get_global_id(0)
#define THREAD_LENGTH       get_global_size(0)
#define THREAD_LOCAL_ID     get_local_id(0)
#define THREAD_LOCAL_LENGTH get_local_size(0)
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

typedef char           sp_char;
typedef short          sp_short;
typedef int            sp_int;
typedef unsigned int   sp_uint;
typedef unsigned short sp_ushort;
typedef long           sp_long;
typedef unsigned long  sp_ulong;
typedef unsigned int   sp_size;

typedef float          sp_float;
