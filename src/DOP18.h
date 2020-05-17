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