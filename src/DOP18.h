#ifndef DOP18_HEADER
#define DOP18_HEADER

#include "SpectrumPhysics.h"
#include "Vec3.h"
#include "BoundingVolume.h"
#include "Plane3D.h"
#include "DetailedCollisionStatus.h"

#define DOP18_ORIENTATIONS 9
#define DOP18_STRIDER 20
#define DOP18_OFFSET 2

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
		API_INTERFACE void rotate(const Vec3& angles) override;

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
		API_INTERFACE const Vec3* normals()
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
		API_INTERFACE Plane3D* planes();

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
		API_INTERFACE CollisionStatus collisionStatus(const DOP18& kDop);

		/// <summary>
		/// Fix degenerated 18-DOP pulling their planes on the right position
		/// </summary>
		API_INTERFACE void fixDegenerations();

		/// <summary>
		/// Returns the type og bounding volume
		/// </summary>
		API_INTERFACE BoundingVolumeType DOP18::type() const;

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