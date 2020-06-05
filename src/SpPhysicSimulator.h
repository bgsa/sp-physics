#ifndef SP_PHYSIC_SIMULATOR_HEADER
#define SP_PHYSIC_SIMULATOR_HEADER

#include "SpectrumPhysics.h"
#include "GpuContext.h"
#include "DOP18.h"
#include "SweepAndPrune.h"
#include "SpEventDispatcher.h"
#include "SpPhysicObject.h"
#include "CL/cl.h"
#include "Timer.h"
#include "SpPhysicSettings.h"

namespace NAMESPACE_PHYSICS
{
	class SpPhysicSimulator
	{
	private:
		GpuDevice* gpu;
		SweepAndPrune* sap;

		sp_uint objectsLengthAllocated;
		sp_uint objectsLength;

		cl_mem boundingVolumeBuffer = nullptr;
		DOP18* _boundingVolumes;
		SpPhysicProperties* _physicProperties;

		SpPhysicSimulator() { }

		void findContact(const DOP18& dop, const Vec3& center, const Line3D& direction, sp_uint planeIndexes[6], sp_uint* planeIndex , Vec3* contactPoint)
		{
			Vec3 point;
			Plane3D* dopPlanes = dop.planes();

			dopPlanes[planeIndexes[0]].findIntersection(direction, &point);
			sp_float distance = point.squaredDistance(center);
			*planeIndex = 0;
			*contactPoint = point;

			dopPlanes[planeIndexes[1]].findIntersection(direction, &point);
			sp_float newDistance = point.squaredDistance(center);
			if (newDistance < distance)
			{
				distance = newDistance;
				*planeIndex = 1;
				*contactPoint = point;
			}

			dopPlanes[planeIndexes[2]].findIntersection(direction, &point);
			newDistance = point.squaredDistance(center);
			if (newDistance < distance)
			{
				distance = newDistance;
				*planeIndex = 2;
				*contactPoint = point;
			}

			dopPlanes[planeIndexes[3]].findIntersection(direction, &point);
			newDistance = point.squaredDistance(center);
			if (newDistance < distance)
			{
				distance = newDistance;
				*planeIndex = 3;
				*contactPoint = point;
			}

			dopPlanes[planeIndexes[4]].findIntersection(direction, &point);
			newDistance = point.squaredDistance(center);
			if (newDistance < distance)
			{
				distance = newDistance;
				*planeIndex = 4;
				*contactPoint = point;
			}

			dopPlanes[planeIndexes[5]].findIntersection(direction, &point);
			newDistance = point.squaredDistance(center);
			if (newDistance < distance)
			{
				distance = newDistance;
				*planeIndex = 5;
				*contactPoint = point;
			}

			sp_mem_release(dopPlanes);
		}

		sp_float timeOfCollision(const sp_uint objIndex1, const sp_uint objIndex2, sp_float elapsedTime)
		{
			SpPhysicProperties* obj1Properties = &_physicProperties[objIndex1];
			SpPhysicProperties* obj2Properties = &_physicProperties[objIndex2];

			sp_float previousElapsedTime = elapsedTime;
			sp_float factor = 0.25f;
			elapsedTime *= 0.5f;
			CollisionStatus status;
			do
			{
				backToTime(objIndex1);
				backToTime(objIndex2);

				integrate(objIndex1, elapsedTime);
				integrate(objIndex2, elapsedTime);

				status = _boundingVolumes[objIndex1].collisionStatus(_boundingVolumes[objIndex2]);

				previousElapsedTime = elapsedTime;
				if (status == CollisionStatus::OUTSIDE)
					elapsedTime += elapsedTime * factor;
				else
					elapsedTime -= elapsedTime * factor;
				factor *= 0.65f;
			}
			while (std::abs(elapsedTime - previousElapsedTime) > 0.9f);

			return elapsedTime;
		}

		void handleCollision(const sp_uint objIndex1, const sp_uint objIndex2, sp_float elapsedTime)
		{
			sp_assert(objIndex1 != objIndex2, "InvalidArgumentException");

			sp_float timeOfImpact = timeOfCollision(objIndex1, objIndex2, elapsedTime);

			SpPhysicProperties* obj1Properties = &_physicProperties[objIndex1];
			SpPhysicProperties* obj2Properties = &_physicProperties[objIndex2];
			DOP18 bv1 = _boundingVolumes[objIndex1];

			if (!obj1Properties->isMovable() && !obj2Properties->isMovable())
				return;

			Vec3 centerObj1 = obj1Properties->previousPosition();
			Vec3 centerObj2 = obj2Properties->previousPosition();

			Line3D line(centerObj2, centerObj1);
			Vec3 direction = (centerObj2 - centerObj1).normalize();

			sp_float angleRight = direction.dot({ 1.0f, 0.0f, 0.0f });
			sp_float angleUp = direction.dot({ 0.0f, 1.0f, 0.0f });
			sp_float angleDepth = direction.dot({ 0.0f, 0.0f, 1.0f });

			sp_uint planeIndex;
			Vec3 contactPoint;
			
			if (angleUp >= HALF_PI && angleUp <= HALF_PI) // it the collision happend up ...
			{	
				if (angleRight >= HALF_PI && angleRight <= HALF_PI) // it the collision happend right ...
				{
					if (angleDepth >= HALF_PI && angleDepth <= HALF_PI) // it the collision happend depth ...
					{
						// direction points to up, right and depth
						sp_uint planeIndexes[6] = {
							DOP18_PLANES_RIGHT_INDEX,
							DOP18_PLANES_DEPTH_INDEX,
							DOP18_PLANES_UP_INDEX,
							DOP18_PLANES_UP_RIGHT_INDEX,
							DOP18_PLANES_RIGHT_DEPTH_INDEX,
							DOP18_PLANES_UP_DEPTH_INDEX
						};

						findContact(bv1, centerObj1, line, planeIndexes, &planeIndex, &contactPoint);
					}
					else
					{
						// direction points to up, right and front
						sp_uint planeIndexes[6] = {
							DOP18_PLANES_RIGHT_INDEX,
							DOP18_PLANES_FRONT_INDEX,
							DOP18_PLANES_UP_INDEX,
							DOP18_PLANES_UP_RIGHT_INDEX,
							DOP18_PLANES_RIGHT_FRONT_INDEX,
							DOP18_PLANES_UP_FRONT_INDEX
						};
						findContact(bv1, centerObj1, line, planeIndexes, &planeIndex, &contactPoint);
					}
				}
				else
				{
					if (angleDepth >= HALF_PI && angleDepth <= HALF_PI) // it the collision happend depth ...
					{
						// direction points to up, left and depth
						sp_uint planeIndexes[6] = {
							DOP18_PLANES_LEFT_INDEX,
							DOP18_PLANES_DEPTH_INDEX,
							DOP18_PLANES_UP_INDEX,
							DOP18_PLANES_UP_LEFT_INDEX,
							DOP18_PLANES_LEFT_DEPTH_INDEX,
							DOP18_PLANES_UP_DEPTH_INDEX
						};
						findContact(bv1, centerObj1, line, planeIndexes, &planeIndex, &contactPoint);
					}
					else
					{
						// direction points to up, left and front
						sp_uint planeIndexes[6] = {
							DOP18_PLANES_LEFT_INDEX,
							DOP18_PLANES_FRONT_INDEX,
							DOP18_PLANES_UP_INDEX,
							DOP18_PLANES_UP_LEFT_INDEX,
							DOP18_PLANES_LEFT_FRONT_INDEX,
							DOP18_PLANES_UP_FRONT_INDEX
						};
						findContact(bv1, centerObj1, line, planeIndexes, &planeIndex, &contactPoint);
					}
				}
			}
			else
			{
				if (angleRight >= HALF_PI && angleRight <= HALF_PI) // it the collision happend right ...
				{
					if (angleDepth >= HALF_PI && angleDepth <= HALF_PI) // it the collision happend depth ...
					{
						// direction points to down, right and depth
						sp_uint planeIndexes[6] = {
							DOP18_PLANES_RIGHT_INDEX,
							DOP18_PLANES_DEPTH_INDEX,
							DOP18_PLANES_DOWN_INDEX,
							DOP18_PLANES_DOWN_RIGHT_INDEX,
							DOP18_PLANES_RIGHT_DEPTH_INDEX,
							DOP18_PLANES_DOWN_DEPTH_INDEX
						};
						findContact(bv1, centerObj1, line, planeIndexes, &planeIndex, &contactPoint);
					}
					else
					{
						// direction points to down, right and front
						sp_uint planeIndexes[6] = {
							DOP18_PLANES_RIGHT_INDEX,
							DOP18_PLANES_FRONT_INDEX,
							DOP18_PLANES_DOWN_INDEX,
							DOP18_PLANES_DOWN_RIGHT_INDEX,
							DOP18_PLANES_RIGHT_FRONT_INDEX,
							DOP18_PLANES_DOWN_FRONT_INDEX
						};
						findContact(bv1, centerObj1, line, planeIndexes, &planeIndex, &contactPoint);
					}
				}
				else
				{
					if (angleDepth >= HALF_PI && angleDepth <= HALF_PI)
					{
						// direction points to down, left and depth
						sp_uint planeIndexes[6] = {
							DOP18_PLANES_LEFT_INDEX,
							DOP18_PLANES_DEPTH_INDEX,
							DOP18_PLANES_DOWN_INDEX,
							DOP18_PLANES_DOWN_LEFT_INDEX,
							DOP18_PLANES_LEFT_DEPTH_INDEX,
							DOP18_PLANES_DOWN_DEPTH_INDEX
						};
						findContact(bv1, centerObj1, line, planeIndexes, &planeIndex, &contactPoint);
					}
					else
					{
						// direction points to down, left and front
						sp_uint planeIndexes[6] = {
							DOP18_PLANES_LEFT_INDEX,
							DOP18_PLANES_FRONT_INDEX,
							DOP18_PLANES_DOWN_INDEX,
							DOP18_PLANES_DOWN_LEFT_INDEX,
							DOP18_PLANES_LEFT_FRONT_INDEX,
							DOP18_PLANES_DOWN_FRONT_INDEX
						};
						findContact(bv1, centerObj1, line, planeIndexes, &planeIndex, &contactPoint);
					}
				}
			}

			const sp_float sumMass = obj1Properties->massInverse() + obj2Properties->massInverse();
			const Vec3 relativeVelocity = obj2Properties->velocity() - obj1Properties->velocity();
			const sp_float cor = std::min(obj1Properties->coeficientOfRestitution(), obj2Properties->coeficientOfRestitution());

			if (obj1Properties->isMovable())
			{
				sp_float factor1 
					= (obj1Properties->massInverse() - cor * obj2Properties->massInverse())
					/ sumMass;

				sp_float factor2 
					= ((1.0f + cor) * obj2Properties->massInverse())
					/ sumMass;

				const Vec3 newForceObj1
					= (obj1Properties->velocity() * factor1)
					+ (obj2Properties->velocity() * factor2);

				obj1Properties->_acceleration = 0.0f;
				obj1Properties->_velocity = direction * newForceObj1;
			}

			if (obj2Properties->isMovable())
			{
				sp_float factor1 =
					((1.0f + cor) * obj1Properties->massInverse())
					/ sumMass;

				sp_float factor2 =
					(obj2Properties->massInverse() - cor * obj1Properties->massInverse())
					/ sumMass;

				const Vec3 newForceObj2
					= (obj1Properties->velocity() * factor1)
					+ (obj2Properties->velocity() * factor2);

				obj2Properties->_acceleration = 0.0f;
				obj2Properties->_velocity = direction * newForceObj2;
			}
		}

	public:

		API_INTERFACE static SpPhysicSimulator* instance();

		API_INTERFACE static void init(sp_uint objectsLength);

		API_INTERFACE sp_uint alloc(sp_uint length);

		API_INTERFACE inline BoundingVolume* boundingVolumes(const sp_uint index) const
		{
			return &_boundingVolumes[index];
		}

		API_INTERFACE inline SpPhysicProperties* physicProperties(const sp_uint index) const
		{
			return &_physicProperties[index];
		}

		API_INTERFACE inline void integrate(const sp_uint index, sp_float elapsedTime)
		{
			sp_assert(elapsedTime > ZERO_FLOAT, "InvalidArgumentException");
			sp_assert(index >= ZERO_UINT, "IndexOutOfRangeException");
			sp_assert(index < objectsLength, "IndexOutOfRangeException");

			SpPhysicSettings* settings = SpPhysicSettings::instance();

			SpPhysicProperties* element = &_physicProperties[index];

			element->addForce(settings->gravityForce());

			const sp_float drag = 0.1f; // rho*C*Area - simplified drag for this example
			const Vec3 dragForce = (element->velocity() * element->velocity().abs()) * 0.5f * drag;

			// Velocity Verlet Integration
			elapsedTime = elapsedTime * settings->physicVelocity();

			// Velocity Verlet Integration because regards the velocity
			const Vec3 newPosition = element->position()
				+ element->velocity() * elapsedTime
				+ element->acceleration()  * (elapsedTime * elapsedTime * 0.5f);

			const Vec3 newAcceleration = (element->force() - dragForce) * element->massInverse();

			Vec3 newVelocity = element->velocity()
				+ (element->acceleration() + newAcceleration) * (elapsedTime * 0.5f);

			_boundingVolumes[index].translate(newPosition - element->position()); // Sync with the bounding Volume

			element->_previousAcceleration = element->acceleration();
			element->_acceleration = newAcceleration;

			element->_previousVelocity = element->velocity();
			element->_velocity = newVelocity;

			element->_previousPosition = element->position();
			element->_position = newPosition;

			element->_previousForce = element->force();
			element->_force = 0.0f;
		}

		/// <summary>
		/// Back the object to the state before timestep
		/// </summary>
		API_INTERFACE inline void backToTime(const sp_uint index)
		{
			SpPhysicProperties* element = &_physicProperties[index];
			DOP18* dop = &_boundingVolumes[index];

			dop->translate(element->previousPosition() - element->position());
			element->rollbackState();
		}

		API_INTERFACE void run(const Timer& timer);

		API_INTERFACE void dispose();

		~SpPhysicSimulator();

	};
}

#endif // SP_PHYSIC_SIMULATOR_HEADER