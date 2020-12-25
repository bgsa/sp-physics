#ifndef SP_BODY_MAPPER_OPENCL_HEADER
#define SP_BODY_MAPPER_OPENCL_HEADER

#include "OpenCLBase.cl"

#define SP_BODY_MAPPER_SIZE   (multiplyBy2(SIZEOF_FLOAT))
#define SP_BODY_MAPPER_LENGTH (2)

#define SP_BODY_MAPPER_TYPE_INDEX  0
#define SP_BODY_MAPPER_INDEX_INDEX 1

#define SP_BODY_MAPPER_TYPE_RIGID 1
#define SP_BODY_MAPPER_TYPE_SOFT  2

#define SpBodyMapper_isRigid(mapper, stride) \
    mapper[stride + SP_BODY_MAPPER_TYPE_INDEX] == SP_BODY_MAPPER_TYPE_RIGID

#define SpBodyMapper_isSoft(mapper, stride) \
    mapper[stride + SP_BODY_MAPPER_TYPE_INDEX] == SP_BODY_MAPPER_TYPE_SOFT

#define SpBodyMapper_getIndex(mapper, stride) \
    mapper[stride + SP_BODY_MAPPER_INDEX_INDEX]


#endif // SP_BODY_MAPPER_OPENCL_HEADER