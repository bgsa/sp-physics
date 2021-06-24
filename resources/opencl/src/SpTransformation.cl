#ifndef SP_TRANSFORMATION_HEADER
#define SP_TRANSFORMATION_HEADER

#include "OpenCLBase.cl"
#include "Vec3.cl"
#include "Quat.cl"

#define SP_TRANSFORMATION_ORIENTATION_OFFSET (0)
#define SP_TRANSFORMATION_POSITION_OFFSET (4)
#define SP_TRANSFORMATION_SCALE_OFFSET (7)
#define SP_TRANSFORMATION_STRIDER (10)

inline void sp_transformation_get_position(__global sp_float* transformations, sp_uint stride, float3* position)
{
	position->x = transformations[stride + SP_TRANSFORMATION_POSITION_OFFSET     ];
	position->y = transformations[stride + SP_TRANSFORMATION_POSITION_OFFSET + 1u];
	position->z = transformations[stride + SP_TRANSFORMATION_POSITION_OFFSET + 2u];
}

inline void sp_transformation_get_orientation(__global sp_float* transformations, sp_uint stride, Quat* orientation)
{
	orientation->w = transformations[stride + SP_TRANSFORMATION_ORIENTATION_OFFSET     ];
	orientation->x = transformations[stride + SP_TRANSFORMATION_ORIENTATION_OFFSET + 1u];
	orientation->y = transformations[stride + SP_TRANSFORMATION_ORIENTATION_OFFSET + 2u];
	orientation->z = transformations[stride + SP_TRANSFORMATION_ORIENTATION_OFFSET + 3u];
}

inline void sp_transformation_get_scale(__global sp_float* transformations, sp_uint stride, float3* scale)
{
	scale->x = transformations[stride + SP_TRANSFORMATION_SCALE_OFFSET     ];
	scale->y = transformations[stride + SP_TRANSFORMATION_SCALE_OFFSET + 1u];
	scale->z = transformations[stride + SP_TRANSFORMATION_SCALE_OFFSET + 2u];
}

inline void sp_transformation_transform(__global sp_float* transformations, sp_uint stride, Vec3 vertex, Vec3* output)
{
	Quat orientation;
	sp_transformation_get_orientation(transformations, stride, &orientation);
	Vec3 translation;
	sp_transformation_get_position(transformations, stride, &translation);
	Vec3 scale;
	sp_transformation_get_scale(transformations, stride, &scale);

	Vec3 vertexScaled;
	vec3_multiply_vec3(scale, vertex, &vertexScaled); // scale

	Vec3 vertexRotated;
	quat_rotate_vec3(orientation, vertexScaled, &vertexRotated); // rotate

	Vec3 vertexTranslated;
	vec3_add_vec3(vertexRotated, translation, vertexTranslated); // translate

	output->x = vertexTranslated.x;
	output->y = vertexTranslated.y;
	output->z = vertexTranslated.z;
}

#endif // DOP18_OPENCL_HEADER