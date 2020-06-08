#ifndef DOP18_HEADER
#define DOP18_HEADER

#include "SpectrumPhysics.h"
#include "Vec3.h"
#include "BoundingVolume.h"
#include "Plane3D.h"
#include "DetailedCollisionStatus.h"

#define DOP18_ORIENTATIONS (9)
#define DOP18_STRIDER      (20)
#define DOP18_OFFSET       (2)
#define DOP18_SIZE         (80) /* DOP18_ORIENTATIONS * 2 (min,max) * SIZEOF_FLOAT + 8 (2 x heritage) */

#define DOP18_AXIS_X           0
#define DOP18_AXIS_Y           1
#define DOP18_AXIS_Z           2
#define DOP18_AXIS_UP_LEFT     3
#define DOP18_AXIS_UP_RIGHT    4
#define DOP18_AXIS_UP_FRONT    5
#define DOP18_AXIS_UP_DEPTH    6
#define DOP18_AXIS_LEFT_DEPTH  7
#define DOP18_AXIS_RIGHT_DEPTH 8

#define DOP18_PLANES_LEFT_INDEX         0
#define DOP18_PLANES_RIGHT_INDEX        1
#define DOP18_PLANES_UP_INDEX           2
#define DOP18_PLANES_DOWN_INDEX         3
#define DOP18_PLANES_FRONT_INDEX        4
#define DOP18_PLANES_DEPTH_INDEX        5
#define DOP18_PLANES_UP_LEFT_INDEX      6
#define DOP18_PLANES_DOWN_RIGHT_INDEX   7
#define DOP18_PLANES_UP_RIGHT_INDEX     8
#define DOP18_PLANES_DOWN_LEFT_INDEX    9
#define DOP18_PLANES_UP_FRONT_INDEX    10
#define DOP18_PLANES_DOWN_DEPTH_INDEX  11
#define DOP18_PLANES_UP_DEPTH_INDEX    12
#define DOP18_PLANES_DOWN_FRONT_INDEX  13
#define DOP18_PLANES_LEFT_DEPTH_INDEX  14
#define DOP18_PLANES_RIGHT_FRONT_INDEX 15
#define DOP18_PLANES_RIGHT_DEPTH_INDEX 16
#define DOP18_PLANES_LEFT_FRONT_INDEX  17

namespace NAMESPACE_PHYSICS
{
	/// <summary>
	/// Represents a k-DOP with 9 orientations and 18 DOPs
	/// Normal coordinates od planes:
	///
	///     axis-aligned:
	///		{ -1,  0,  0 } - left
	///		{  1,  0,  0 } - right
	///		{  0,  1,  0 } - up
	///		{  0, -1,  0 } - down
	///		{  0,  0, -1 } - front
	///		{  0,  0,  1 } - depth
	///
	///     edges cut-off:
	///		{ -1,  1,  0 } - up-left
	///		{  1, -1,  0 } - down-right
	///		{  1,  1,  0 } - up-right
	///		{ -1, -1,  0 } - down-left
	///		{  0,  1, -1 } - up-front
	///		{  0, -1,  1 } - down-depth
	///		{  0,  1,  1 } - up-depth
	///		{  0, -1, -1 } - down-front
	///		{ -1,  0,  1 } - left-depth
	///		{  1,  0, -1 } - right-front
	///		{  1,  0,  1 } - right-depth
	///		{ -1,  0, -1 } - left-front
	///
	/// </summary>
	class DOP18
		: public BoundingVolume
	{
	private:
		void fixTopDegeneration(const Plane3D* planes);
		void fixBottomDegeneration(const Plane3D* planes);
		void fixLeftDegeneration(const Plane3D* planes);
		void fixRightDegeneration(const Plane3D* planes);
		void fixFrontDegeneration(const Plane3D* planes);
		void fixDepthDegeneration(const Plane3D* planes);

		/// <summary>
		/// This method is not applied to k-DOPs
		/// </summary>
		void rotate(const Vec3& angles) override { }

		/// <summary>
		/// Get the points from planes
		/// </summary>
		void pointsFromPlanes(Vec3* points, sp_size pointOffset,
			const Plane3D& plane1, const Plane3D& plane2,
			const Plane3D& plane3, const Plane3D& plane4)
		{
			Line3D line;
			plane1.intersection(plane2, &line);

			line.intersectionOnRay(plane3, &points[pointOffset]);
			line.intersectionOnRay(plane4, &points[pointOffset + 1]);
		}

	public:
		sp_float min[DOP18_ORIENTATIONS];
		sp_float max[DOP18_ORIENTATIONS];

		/// <summary>
		/// Default constructur - build a unit k-DOP with the center in the origin
		/// </summary>
		API_INTERFACE DOP18();

		/// <summary>
		/// Get the normals of k-DOP planes
		/// </summary>
		API_INTERFACE const Vec3* normals() const
		{
			const static Vec3 normal[18] = {
				{ -1.0f,  0.0f,  0.0f },
				{  1.0f,  0.0f,  0.0f },
				{  0.0f,  1.0f,  0.0f },
				{  0.0f, -1.0f,  0.0f },
				{  0.0f,  0.0f, -1.0f },
				{  0.0f,  0.0f,  1.0f },

				{ -0.5f,  0.5f,  0.0f },
				{  0.5f, -0.5f,  0.0f },
				{  0.5f,  0.5f,  0.0f },
				{ -0.5f, -0.5f,  0.0f },
				{  0.0f,  0.5f, -0.5f },
				{  0.0f, -0.5f,  0.5f },
				{  0.0f,  0.5f,  0.5f },
				{  0.0f, -0.5f, -0.5f },
				{ -0.5f,  0.0f,  0.5f },
				{  0.5f,  0.0f, -0.5f },
				{  0.5f,  0.0f,  0.5f },
				{ -0.5f,  0.0f, -0.5f }
			};

			return &normal[0];
		}

		/// <summary>
		/// Get the k-DOP planes
		/// </summary>
		API_INTERFACE Plane3D* planes() const;

		///<summary>
		/// Get the center of k-DOP bounding volumne
		///</summary>
		API_INTERFACE Vec3 centerOfBoundingVolume() const override;

		/// <summary>
		/// Translate the bounding volume
		/// </summary>
		API_INTERFACE void translate(const Vec3& translation) override;

		/// <summary>
		/// Translate the bounding volume
		/// </summary>
		API_INTERFACE inline void translate(const sp_float x, const sp_float y, const sp_float z)
		{
			translate(Vec3(x, y, z));
		}

		/// <summary>
		/// Scale the bounding volume (only in X, Y and Z)
		/// </summary>
		API_INTERFACE void scale(const Vec3& factor) override;

		/// <summary>
		/// Check collision with another k-DOP
		/// </summary>
		API_INTERFACE CollisionStatus collisionStatus(const DOP18& kDop) const;

		/// <summary>
		/// Check collision with a plane
		/// </summary>
		API_INTERFACE CollisionStatus collisionStatus(const Plane3D& plane) const;

		/// <summary>
		/// Check collision with a point
		/// </summary>
		API_INTERFACE CollisionStatus collisionStatus(const Vec3& point) const;

		/// <summary>
		/// Check collision with a point and return details, just in case of collisions
		/// </summary>
		CollisionStatus collisionStatus(const Vec3& point, sp_bool* minPlane, sp_uint* planeIndex, sp_float* depth) const;

		/// <summary>
		/// Fix degenerated 18-DOP pulling their planes on the right position
		/// </summary>
		API_INTERFACE void fixDegenerations();

		/// <summary>
		/// Returns the type og bounding volume
		/// </summary>
		API_INTERFACE BoundingVolumeType type() const
		{
			return BoundingVolumeType::DOP18;
		}

		/// <summary>
		/// Get the points of front plane
		/// </summary>
		API_INTERFACE inline void pointsFromPlaneFront(Vec3* points, sp_uint* length)
		{
			Plane3D _planeFront = planeFront();
			Plane3D _planeUpFront = planeUpFront();
			Plane3D _planeDownFront = planeDownFront();

			*length = 4u;
			pointsFromPlanes(points, 0, _planeFront, planeLeftFront(), _planeUpFront, _planeDownFront);
			pointsFromPlanes(points, 2, _planeFront, planeRightFront(), _planeUpFront, _planeDownFront);
		}

		/// <summary>
		/// Get the points of depth plane
		/// </summary>
		API_INTERFACE inline void pointsFromPlaneDepth(Vec3* points, sp_uint* length)
		{
			Plane3D _planeDepth = planeDepth();
			Plane3D _planeUpDepth = planeUpDepth();
			Plane3D _planeDownDepth = planeDownDepth();

			*length = 4u;
			pointsFromPlanes(points, 0, _planeDepth, planeLeftDepth(), _planeUpDepth, _planeDownDepth);  // left-depth line and vertexes
			pointsFromPlanes(points, 2, _planeDepth, planeRightDepth(), _planeUpDepth, _planeDownDepth); // right-depth line and vertexes
		}

		/// <summary>
		/// Get the points of up plane
		/// </summary>
		API_INTERFACE inline void pointsFromPlaneUp(Vec3* points, sp_uint* length)
		{
			Plane3D _planeUp = planeUp();
			Plane3D _planeUpFront = planeUpFront();
			Plane3D _planeUpDepth = planeUpDepth();

			*length = 4u;
			pointsFromPlanes(points, 0, _planeUp, planeUpLeft(), _planeUpFront, _planeUpDepth); // left-top line and vertexes
			pointsFromPlanes(points, 2, _planeUp, planeUpRight(), _planeUpFront, _planeUpDepth); // right-top line and vertexes
		}

		/// <summary>
		/// Get the points of down plane
		/// </summary>
		API_INTERFACE inline void pointsFromPlaneDown(Vec3* points, sp_uint* length)
		{
			Plane3D _planeDown = planeDown();
			Plane3D _planeDownFront = planeDownFront();
			Plane3D _planeDownDepth = planeDownDepth();

			*length = 4u;
			pointsFromPlanes(points, 0, _planeDown, planeDownLeft(), _planeDownFront, _planeDownDepth);
			pointsFromPlanes(points, 2, _planeDown, planeDownRight(), _planeDownDepth, _planeDownFront);
		}

		/// <summary>
		/// Get the points of left plane
		/// </summary>
		API_INTERFACE inline void pointsFromPlaneLeft(Vec3* points, sp_uint* length)
		{
			Plane3D _planeLeft = planeLeft();
			Plane3D _planeUpLeft = planeUpLeft();
			Plane3D _planeDownLeft = planeDownLeft();

			*length = 4u;
			pointsFromPlanes(points, 0, _planeLeft, planeLeftFront(), _planeUpLeft, _planeDownLeft);
			pointsFromPlanes(points, 2, _planeLeft, planeLeftDepth(), _planeUpLeft, _planeDownLeft);
		}

		/// <summary>
		/// Get the points of right plane
		/// </summary>
		API_INTERFACE inline void pointsFromPlaneRight(Vec3* points, sp_uint* length)
		{
			Plane3D _planeRight = planeRight();
			Plane3D _planeUpRight = planeUpRight();
			Plane3D _planeDownRight = planeDownRight();

			*length = 4u;
			pointsFromPlanes(points, 0, _planeRight, planeRightFront(), _planeUpRight, _planeDownRight);
			pointsFromPlanes(points, 2, _planeRight, planeRightDepth(), _planeUpRight, _planeDownRight);
		}

		/// <summary>
		/// Get the points of up-left plane
		/// </summary>
		API_INTERFACE inline void pointsFromPlaneUpLeft(Vec3* points, sp_uint* length)
		{
			Plane3D _planeUpLeft = planeUpLeft();
			Plane3D _planeLeftDepth = planeLeftDepth();
			Plane3D _planeLeftFront = planeLeftFront();

			Line3D lineUp;
			_planeUpLeft.intersection(planeUp(), &lineUp);
			lineUp.intersectionOnRay(_planeLeftFront, &points[1]);
			lineUp.intersectionOnRay(_planeLeftDepth, points);

			Line3D lineLeft;
			_planeUpLeft.intersection(planeLeft(), &lineLeft);
			lineLeft.intersectionOnRay(_planeLeftDepth, &points[2]);
			lineLeft.intersectionOnRay(_planeLeftFront, &points[3]);

			*length = 4u;
		}

		/// <summary>
		/// Get the points of up-right plane
		/// </summary>
		API_INTERFACE inline void pointsFromPlaneUpRight(Vec3* points, sp_uint* length)
		{
			Plane3D _planeUpRight = planeUpRight();
			Plane3D _planeRightDepth = planeRightDepth();
			Plane3D _planeRightFront = planeRightFront();

			Line3D lineUp;
			_planeUpRight.intersection(planeUp(), &lineUp);
			lineUp.intersectionOnRay(_planeRightFront, &points[1]);
			lineUp.intersectionOnRay(_planeRightDepth, points);

			Line3D lineRight;
			_planeUpRight.intersection(planeRight(), &lineRight);
			lineRight.intersectionOnRay(_planeRightDepth, &points[2]);
			lineRight.intersectionOnRay(_planeRightFront, &points[3]);

			*length = 4u;
		}

		/// <summary>
		/// Get the points of down-left plane
		/// </summary>
		API_INTERFACE inline void pointsFromPlaneDownLeft(Vec3* points, sp_uint* length)
		{
			Plane3D _planeDownLeft = planeDownLeft();
			Plane3D _planeLeftDepth = planeLeftDepth();
			Plane3D _planeLeftFront = planeLeftFront();

			Line3D lineDown;
			_planeDownLeft.intersection(planeDown(), &lineDown);
			lineDown.intersectionOnRay(_planeLeftDepth, points);
			lineDown.intersectionOnRay(_planeLeftFront, &points[1]);

			Line3D lineLeft;
			_planeDownLeft.intersection(planeLeft(), &lineLeft);
			lineLeft.intersectionOnRay(_planeLeftDepth, &points[2]);
			lineLeft.intersectionOnRay(_planeLeftFront, &points[3]);

			*length = 4u;
		}

		/// <summary>
		/// Get the points of down-right plane
		/// </summary>
		API_INTERFACE inline void pointsFromPlaneDownRight(Vec3* points, sp_uint* length)
		{
			Plane3D _planeDownRight = planeDownRight();
			Plane3D _planeRightDepth = planeRightDepth();
			Plane3D _planeRightFront = planeRightFront();

			Line3D lineDown;
			_planeDownRight.intersection(planeDown(), &lineDown);
			lineDown.intersectionOnRay(_planeRightDepth, points);
			lineDown.intersectionOnRay(_planeRightFront, &points[1]);

			Line3D lineRight;
			_planeDownRight.intersection(planeRight(), &lineRight);
			lineRight.intersectionOnRay(_planeRightFront, &points[2]);
			lineRight.intersectionOnRay(_planeRightDepth, &points[3]);

			*length = 4u;
		}

		/// <summary>
		/// Get the points of up-front plane
		/// </summary>
		API_INTERFACE inline void pointsFromPlaneUpFront(Vec3* points, sp_uint* length)
		{
			Plane3D _planeUpFront = planeUpFront();
			Plane3D _planeLeftFront = planeLeftFront();
			Plane3D _planeRightFront = planeRightFront();
			
			Line3D lineUp;
			_planeUpFront.intersection(planeUp(), &lineUp);
			lineUp.intersectionOnRay(_planeRightFront, points);
			lineUp.intersectionOnRay(_planeLeftFront, &points[1]);

			Line3D lineFront;
			_planeUpFront.intersection(planeFront(), &lineFront);
			lineFront.intersectionOnRay(_planeLeftFront, &points[2]);
			lineFront.intersectionOnRay(_planeRightFront, &points[3]);

			*length = 4u;
		}

		/// <summary>
		/// Get the points of down-front plane
		/// </summary>
		API_INTERFACE inline void pointsFromPlaneDownFront(Vec3* points, sp_uint* length)
		{
			Plane3D _planeDownFront = planeDownFront();
			Plane3D _planeLeftFront = planeLeftFront();
			Plane3D _planeRightFront = planeRightFront();

			Line3D lineUp;
			_planeDownFront.intersection(planeDown(), &lineUp);
			lineUp.intersectionOnRay(_planeRightFront, points);
			lineUp.intersectionOnRay(_planeLeftFront, &points[1]);

			Line3D lineFront;
			_planeDownFront.intersection(planeFront(), &lineFront);
			lineFront.intersectionOnRay(_planeLeftFront, &points[2]);
			lineFront.intersectionOnRay(_planeRightFront, &points[3]);

			*length = 4u;
		}

		/// <summary>
		/// Get the points of up-depth plane
		/// </summary>
		API_INTERFACE inline void pointsFromPlaneUpDepth(Vec3* points, sp_uint* length)
		{
			Plane3D _planeUpDepth = planeUpDepth();
			Plane3D _planeLeftDepth = planeLeftDepth();
			Plane3D _planeRightDepth = planeRightDepth();

			Line3D lineUp;
			_planeUpDepth.intersection(planeUp(), &lineUp);
			lineUp.intersectionOnRay(_planeLeftDepth, points);
			lineUp.intersectionOnRay(_planeRightDepth, &points[1]);

			Line3D lineFront;
			_planeUpDepth.intersection(planeDepth(), &lineFront);
			lineFront.intersectionOnRay(_planeRightDepth, &points[2]);
			lineFront.intersectionOnRay(_planeLeftDepth, &points[3]);

			*length = 4u;
		}

		/// <summary>
		/// Get the points of down-depth plane
		/// </summary>
		API_INTERFACE inline void pointsFromPlaneDownDepth(Vec3* points, sp_uint* length)
		{
			Plane3D _planeDownDepth = planeDownDepth();
			Plane3D _planeLeftDepth = planeLeftDepth();
			Plane3D _planeRightDepth = planeRightDepth();

			Line3D lineDown;
			_planeDownDepth.intersection(planeDown(), &lineDown);
			lineDown.intersectionOnRay(_planeLeftDepth, points);
			lineDown.intersectionOnRay(_planeRightDepth, &points[1]);

			Line3D lineDepth;
			_planeDownDepth.intersection(planeDepth(), &lineDepth);
			lineDepth.intersectionOnRay(_planeRightDepth, &points[2]);
			lineDepth.intersectionOnRay(_planeLeftDepth, &points[3]);

			*length = 4u;
		}

		/// <summary>
		/// Get the points of left-depth plane
		/// </summary>
		API_INTERFACE inline void pointsFromPlaneLeftDepth(Vec3* points, sp_uint* length)
		{
			Plane3D _planeLeftDepth = planeLeftDepth();
			Plane3D _planeUpLeft = planeUpLeft();
			Plane3D _planeDownLeft = planeDownLeft();

			Line3D lineDepth;
			_planeLeftDepth.intersection(planeDepth(), &lineDepth);
			lineDepth.intersectionOnRay(_planeDownLeft, points);
			lineDepth.intersectionOnRay(_planeUpLeft, &points[1]);

			Line3D lineLeft;
			_planeLeftDepth.intersection(planeLeft(), &lineLeft);
			lineLeft.intersectionOnRay(_planeUpLeft, &points[2]);
			lineLeft.intersectionOnRay(_planeDownLeft, &points[3]);

			*length = 4u;
		}

		/// <summary>
		/// Get the points of left-front plane
		/// </summary>
		API_INTERFACE inline void pointsFromPlaneLeftFront(Vec3* points, sp_uint* length)
		{
			Plane3D _planeLeftFront = planeLeftFront();
			Plane3D _planeUpLeft = planeUpLeft();
			Plane3D _planeDownLeft = planeDownLeft();

			Line3D lineDepth;
			_planeLeftFront.intersection(planeFront(), &lineDepth);
			lineDepth.intersectionOnRay(_planeDownLeft, points);
			lineDepth.intersectionOnRay(_planeUpLeft, &points[1]);

			Line3D lineLeft;
			_planeLeftFront.intersection(planeLeft(), &lineLeft);
			lineLeft.intersectionOnRay(_planeUpLeft, &points[2]);
			lineLeft.intersectionOnRay(_planeDownLeft, &points[3]);

			*length = 4u;
		}

		/// <summary>
		/// Get the points of right-front plane
		/// </summary>
		API_INTERFACE inline void pointsFromPlaneRightFront(Vec3* points, sp_uint* length)
		{
			Plane3D _planeRightFront = planeRightFront();
			Plane3D _planeUpRight = planeUpRight();
			Plane3D _planeDownRight = planeDownRight();

			Line3D lineFront;
			_planeRightFront.intersection(planeFront(), &lineFront);
			lineFront.intersectionOnRay(_planeUpRight, points);
			lineFront.intersectionOnRay(_planeDownRight, &points[1]);

			Line3D lineRight;
			_planeRightFront.intersection(planeRight(), &lineRight);
			lineRight.intersectionOnRay(_planeDownRight, &points[2]);
			lineRight.intersectionOnRay(_planeUpRight, &points[3]);

			*length = 4u;
		}

		/// <summary>
		/// Get the points of right-depth plane
		/// </summary>
		API_INTERFACE inline void pointsFromPlaneRightDepth(Vec3* points, sp_uint* length)
		{
			Plane3D _planeRightDepth = planeRightDepth();
			Plane3D _planeUpRight = planeUpRight();
			Plane3D _planeDownRight = planeDownRight();

			Line3D lineDepth;
			_planeRightDepth.intersection(planeDepth(), &lineDepth);
			lineDepth.intersectionOnRay(_planeUpRight, points);
			lineDepth.intersectionOnRay(_planeDownRight, &points[1]);

			Line3D lineRight;
			_planeRightDepth.intersection(planeRight(), &lineRight);
			lineRight.intersectionOnRay(_planeDownRight, &points[2]);
			lineRight.intersectionOnRay(_planeUpRight, &points[3]);

			*length = 4u;
		}

		/// <summary>
		/// Get the left plane of the bounding volume
		/// </summary>
		API_INTERFACE inline Plane3D planeLeft()
		{
			return Plane3D(Vec3(min[DOP18_AXIS_X], ZERO_FLOAT, ZERO_FLOAT), normals()[0]);
		}

		/// <summary>
		/// Get the right plane of the bounding volume
		/// </summary>
		API_INTERFACE inline Plane3D planeRight()
		{
			return Plane3D(Vec3(max[DOP18_AXIS_X], ZERO_FLOAT, ZERO_FLOAT), normals()[1]);
		}

		/// <summary>
		/// Get the up plane of the bounding volume
		/// </summary>
		API_INTERFACE inline Plane3D planeUp()
		{
			return Plane3D(Vec3(ZERO_FLOAT, max[DOP18_AXIS_Y], ZERO_FLOAT), normals()[2]);
		}

		/// <summary>
		/// Get the down plane of the bounding volume
		/// </summary>
		API_INTERFACE inline Plane3D planeDown()
		{
			return Plane3D(Vec3(ZERO_FLOAT, min[DOP18_AXIS_Y], ZERO_FLOAT), normals()[3]);
		}

		/// <summary>
		/// Get the front plane of the bounding volume
		/// </summary>
		API_INTERFACE inline Plane3D planeFront()
		{
			return Plane3D(Vec3(ZERO_FLOAT, ZERO_FLOAT, min[DOP18_AXIS_Z]), normals()[4]);
		}

		/// <summary>
		/// Get the depth plane of the bounding volume
		/// </summary>
		API_INTERFACE inline Plane3D planeDepth()
		{
			return Plane3D(Vec3(ZERO_FLOAT, ZERO_FLOAT, max[DOP18_AXIS_Z]), normals()[5]);
		}

		/// <summary>
		/// Get the up-left plane of the bounding volume
		/// </summary>
		API_INTERFACE inline Plane3D planeUpLeft()
		{
			return Plane3D(Vec3(min[DOP18_AXIS_UP_LEFT], ZERO_FLOAT, ZERO_FLOAT), normals()[6]);
		}

		/// <summary>
		/// Get the down-right plane of the bounding volume
		/// </summary>
		API_INTERFACE inline Plane3D planeDownRight()
		{
			return Plane3D(Vec3(max[DOP18_AXIS_UP_LEFT], ZERO_FLOAT, ZERO_FLOAT), normals()[7]);
		}

		/// <summary>
		/// Get the up-right plane of the bounding volume
		/// </summary>
		API_INTERFACE inline Plane3D planeUpRight()
		{
			return Plane3D(Vec3(max[DOP18_AXIS_UP_RIGHT], ZERO_FLOAT, ZERO_FLOAT), normals()[8]);
		}

		/// <summary>
		/// Get the down-left plane of the bounding volume
		/// </summary>
		API_INTERFACE inline Plane3D planeDownLeft()
		{
			return Plane3D(Vec3(min[DOP18_AXIS_UP_RIGHT], ZERO_FLOAT, ZERO_FLOAT), normals()[9]);
		}

		/// <summary>
		/// Get the up-front plane of the bounding volume
		/// </summary>
		API_INTERFACE inline Plane3D planeUpFront()
		{
			return Plane3D(Vec3(ZERO_FLOAT, max[DOP18_AXIS_UP_FRONT], ZERO_FLOAT), normals()[10]);
		}

		/// <summary>
		/// Get the down-depth plane of the bounding volume
		/// </summary>
		API_INTERFACE inline Plane3D planeDownDepth()
		{
			return Plane3D(Vec3(ZERO_FLOAT, min[DOP18_AXIS_UP_FRONT], ZERO_FLOAT), normals()[11]);
		}

		/// <summary>
		/// Get the up-depth plane of the bounding volume
		/// </summary>
		API_INTERFACE inline Plane3D planeUpDepth()
		{
			return Plane3D(Vec3(ZERO_FLOAT, max[DOP18_AXIS_UP_DEPTH], ZERO_FLOAT), normals()[12]);
		}

		/// <summary>
		/// Get the down-front plane of the bounding volume
		/// </summary>
		API_INTERFACE inline Plane3D planeDownFront()
		{
			return Plane3D(Vec3(ZERO_FLOAT, min[DOP18_AXIS_UP_DEPTH], ZERO_FLOAT), normals()[13]);
		}
		
		/// <summary>
		/// Get the left-depth plane of the bounding volume
		/// </summary>
		API_INTERFACE inline Plane3D planeLeftDepth()
		{
			return Plane3D(Vec3(min[DOP18_AXIS_LEFT_DEPTH], ZERO_FLOAT, ZERO_FLOAT), normals()[14]);
		}

		/// <summary>
		/// Get the right-front plane of the bounding volume
		/// </summary>
		API_INTERFACE inline Plane3D planeRightFront()
		{
			return Plane3D(Vec3(max[DOP18_AXIS_LEFT_DEPTH], ZERO_FLOAT, ZERO_FLOAT), normals()[15]);
		}
		
		/// <summary>
		/// Get the right-depth plane of the bounding volume
		/// </summary>
		API_INTERFACE inline Plane3D planeRightDepth()
		{
			return Plane3D(Vec3(max[DOP18_AXIS_RIGHT_DEPTH], ZERO_FLOAT, ZERO_FLOAT), normals()[16]);
		}

		/// <summary>
		/// Get the left-front plane of the bounding volume
		/// </summary>
		API_INTERFACE inline Plane3D planeLeftFront()
		{
			return Plane3D(Vec3(min[DOP18_AXIS_RIGHT_DEPTH], ZERO_FLOAT, ZERO_FLOAT), normals()[17]);
		}		

		/// <summary>
		/// Auto convertion to void *
		/// </summary>
		API_INTERFACE operator void*() const;

		/// <summary>
		/// Auto convertion to sp_float*
		/// </summary>
		API_INTERFACE operator sp_float*() const;

		/// <summary>
		/// Releases all allocated resouces
		/// </summary>
		API_INTERFACE virtual void dispose() override
		{
		}

		API_INTERFACE virtual const sp_char* toString() override
		{
			return "k-DOP 18";
		}

	};

}

#endif // DOP18_HEADER