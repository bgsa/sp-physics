#include "OpenCLBase.cl"
#include "Vec3.cl"

#ifndef DOP18_OPENCL_HEADER
#define DOP18_OPENCL_HEADER

#define DOP18_ORIENTATIONS (9)
#define DOP18_STRIDE (18)

#define DOP18_AXIS_X (0)
#define DOP18_AXIS_Y (1)
#define DOP18_AXIS_Z (2)
#define DOP18_AXIS_UP_LEFT (3)
#define DOP18_AXIS_UP_RIGHT (4)
#define DOP18_AXIS_UP_FRONT (5)
#define DOP18_AXIS_UP_DEPTH (6)
#define DOP18_AXIS_LEFT_DEPTH (7)
#define DOP18_AXIS_RIGHT_DEPTH (8)

#define dop18_minPointX(dops, index) dops[index  * DOP18_STRIDE    ]
#define dop18_minPointY(dops, index) dops[index  * DOP18_STRIDE + 1]
#define dop18_minPointZ(dops, index) dops[index  * DOP18_STRIDE + 2]
#define dop18_minPointXY(dops, index) dops[index * DOP18_STRIDE + 3]
#define dop18_minPointYX(dops, index) dops[index * DOP18_STRIDE + 4]
#define dop18_minPointYZ(dops, index) dops[index * DOP18_STRIDE + 5]
#define dop18_minPointZY(dops, index) dops[index * DOP18_STRIDE + 6]
#define dop18_minPointXZ(dops, index) dops[index * DOP18_STRIDE + 7]
#define dop18_minPointZX(dops, index) dops[index * DOP18_STRIDE + 8]
#define dop18_maxPointX(dops, index) dops[index  * DOP18_STRIDE + DOP18_ORIENTATIONS    ]
#define dop18_maxPointY(dops, index) dops[index  * DOP18_STRIDE + DOP18_ORIENTATIONS + 1]
#define dop18_maxPointZ(dops, index) dops[index  * DOP18_STRIDE + DOP18_ORIENTATIONS + 2]
#define dop18_maxPointXY(dops, index) dops[index * DOP18_STRIDE + DOP18_ORIENTATIONS + 3]
#define dop18_maxPointYX(dops, index) dops[index * DOP18_STRIDE + DOP18_ORIENTATIONS + 4]
#define dop18_maxPointYZ(dops, index) dops[index * DOP18_STRIDE + DOP18_ORIENTATIONS + 5]
#define dop18_maxPointZY(dops, index) dops[index * DOP18_STRIDE + DOP18_ORIENTATIONS + 6]
#define dop18_maxPointXZ(dops, index) dops[index * DOP18_STRIDE + DOP18_ORIENTATIONS + 7]
#define dop18_maxPointZX(dops, index) dops[index * DOP18_STRIDE + DOP18_ORIENTATIONS + 8]

#define dop18_translate(dops, index, translation)                                       \
	dops[index * DOP18_STRIDE + DOP18_AXIS_X] += translation.x;                         \
	dops[index * DOP18_STRIDE + DOP18_AXIS_X + DOP18_ORIENTATIONS] += translation.x;    \
	dops[index * DOP18_STRIDE + DOP18_AXIS_Y] += translation.y;                         \
	dops[index * DOP18_STRIDE + DOP18_AXIS_Y + DOP18_ORIENTATIONS] += translation.y;    \
	dops[index * DOP18_STRIDE + DOP18_AXIS_Z] += translation.z;                         \
	dops[index * DOP18_STRIDE + DOP18_AXIS_Z + DOP18_ORIENTATIONS] += translation.z;    \
	dops[index * DOP18_STRIDE + DOP18_AXIS_UP_LEFT] += translation.x - translation.y;   \
	dops[index * DOP18_STRIDE + DOP18_AXIS_UP_LEFT + DOP18_ORIENTATIONS] += translation.x - translation.y; \
	dops[index * DOP18_STRIDE + DOP18_AXIS_UP_RIGHT] += translation.x + translation.y;  \
	dops[index * DOP18_STRIDE + DOP18_AXIS_UP_RIGHT + DOP18_ORIENTATIONS] += translation.x + translation.y; \
	dops[index * DOP18_STRIDE + DOP18_AXIS_UP_FRONT] += translation.y + translation.z;  \
	dops[index * DOP18_STRIDE + DOP18_AXIS_UP_FRONT + DOP18_ORIENTATIONS] += translation.y + translation.z; \
	dops[index * DOP18_STRIDE + DOP18_AXIS_UP_DEPTH] += translation.y - translation.z;  \
	dops[index * DOP18_STRIDE + DOP18_AXIS_UP_DEPTH + DOP18_ORIENTATIONS] += translation.y - translation.z; \
	dops[index * DOP18_STRIDE + DOP18_AXIS_LEFT_DEPTH] += translation.x + translation.z;\
	dops[index * DOP18_STRIDE + DOP18_AXIS_LEFT_DEPTH + DOP18_ORIENTATIONS] += translation.x + translation.z; \
	dops[index * DOP18_STRIDE + DOP18_AXIS_RIGHT_DEPTH] += translation.x - translation.z;\
	dops[index * DOP18_STRIDE + DOP18_AXIS_RIGHT_DEPTH + DOP18_ORIENTATIONS] += translation.x - translation.z;


/*
inline void dop18_translate(__global sp_float* dops, const sp_uint index, const Vec3 translation)
{
	dops[index * DOP18_STRIDE + DOP18_AXIS_X                     ] += translation.x;
	dops[index * DOP18_STRIDE + DOP18_AXIS_X + DOP18_ORIENTATIONS] += translation.x;

	dops[index * DOP18_STRIDE + DOP18_AXIS_Y                     ] += translation.y;
	dops[index * DOP18_STRIDE + DOP18_AXIS_Y + DOP18_ORIENTATIONS] += translation.y;

	dops[index * DOP18_STRIDE + DOP18_AXIS_Z                     ] += translation.z;
	dops[index * DOP18_STRIDE + DOP18_AXIS_Z + DOP18_ORIENTATIONS] += translation.z;

	dops[index * DOP18_STRIDE + DOP18_AXIS_UP_LEFT                     ] += translation.x - translation.y;
	dops[index * DOP18_STRIDE + DOP18_AXIS_UP_LEFT + DOP18_ORIENTATIONS] += translation.x - translation.y;

	dops[index * DOP18_STRIDE + DOP18_AXIS_UP_RIGHT                     ] += translation.x + translation.y;
	dops[index * DOP18_STRIDE + DOP18_AXIS_UP_RIGHT + DOP18_ORIENTATIONS] += translation.x + translation.y;

	dops[index * DOP18_STRIDE + DOP18_AXIS_UP_FRONT                     ] += translation.y + translation.z;
	dops[index * DOP18_STRIDE + DOP18_AXIS_UP_FRONT + DOP18_ORIENTATIONS] += translation.y + translation.z;

	dops[index * DOP18_STRIDE + DOP18_AXIS_UP_DEPTH                     ] += translation.y - translation.z;
	dops[index * DOP18_STRIDE + DOP18_AXIS_UP_DEPTH + DOP18_ORIENTATIONS] += translation.y - translation.z;

	dops[index * DOP18_STRIDE + DOP18_AXIS_LEFT_DEPTH                     ] += translation.x + translation.z;
	dops[index * DOP18_STRIDE + DOP18_AXIS_LEFT_DEPTH + DOP18_ORIENTATIONS] += translation.x + translation.z;

	dops[index * DOP18_STRIDE + DOP18_AXIS_RIGHT_DEPTH                     ] += translation.x - translation.z;
	dops[index * DOP18_STRIDE + DOP18_AXIS_RIGHT_DEPTH + DOP18_ORIENTATIONS] += translation.x - translation.z;
}
*/

#endif // DOP18_OPENCL_HEADER