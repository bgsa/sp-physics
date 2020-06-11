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
#include "SpCollisionDetails.h"
#include "SpPhysicSyncronizer.h"

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

		inline void dispatchEvent(const sp_uint objIndex1, const sp_uint objIndex2)
		{
			SpCollisionEvent* evt = sp_mem_new(SpCollisionEvent)();
			evt->indexBody1 = objIndex1;
			evt->indexBody2 = objIndex2;

			SpEventDispatcher::instance()->push(evt);
		}

		void findContact(const DOP18& dop, const Vec3& center, const Line3D& direction, sp_uint planeIndexes[6], sp_uint* planeIndex , Vec3* contactPoint)
		{
			Vec3 point;
			Plane3D* dopPlanes = dop.planes();

			dopPlanes[planeIndexes[0]].intersection(direction, &point);
			sp_float distance = point.squaredDistance(center);
			*planeIndex = 0;
			*contactPoint = point;

			dopPlanes[planeIndexes[1]].intersection(direction, &point);
			sp_float newDistance = point.squaredDistance(center);
			if (newDistance < distance)
			{
				distance = newDistance;
				*planeIndex = 1;
				*contactPoint = point;
			}

			dopPlanes[planeIndexes[2]].intersection(direction, &point);
			newDistance = point.squaredDistance(center);
			if (newDistance < distance)
			{
				distance = newDistance;
				*planeIndex = 2;
				*contactPoint = point;
			}

			dopPlanes[planeIndexes[3]].intersection(direction, &point);
			newDistance = point.squaredDistance(center);
			if (newDistance < distance)
			{
				distance = newDistance;
				*planeIndex = 3;
				*contactPoint = point;
			}

			dopPlanes[planeIndexes[4]].intersection(direction, &point);
			newDistance = point.squaredDistance(center);
			if (newDistance < distance)
			{
				distance = newDistance;
				*planeIndex = 4;
				*contactPoint = point;
			}

			dopPlanes[planeIndexes[5]].intersection(direction, &point);
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

		void collisionDetails(const sp_uint objIndex1, const sp_uint objIndex2, sp_float elapsedTime, SpCollisionDetails* details)
		{
			details->timeOfCollision = timeOfCollision(objIndex1, objIndex2, elapsedTime);

			SpPhysicProperties* obj1Properties = &_physicProperties[objIndex1];
			SpPhysicProperties* obj2Properties = &_physicProperties[objIndex2];

			DOP18 bv1 = _boundingVolumes[objIndex1];
			DOP18 bv2 = _boundingVolumes[objIndex2];

			Vec3 centerObj1 = obj1Properties->position();
			Vec3 centerObj2 = obj2Properties->position();

			Line3D lineOfAction(centerObj2, centerObj1);
			Vec3 direction = lineOfAction.direction().normalize();

			sp_float angleRight = direction.dot({ 1.0f, 0.0f, 0.0f });
			sp_float angleUp = direction.dot({ 0.0f, 1.0f, 0.0f });
			sp_float angleDepth = direction.dot({ 0.0f, 0.0f, 1.0f });

			Vec3 contactPoint;

			if (angleUp >= HALF_PI && angleUp <= HALF_PI) // it the collision happend up ...
			{
				if (angleRight >= HALF_PI && angleRight <= HALF_PI) // it the collision happend right ...
				{
					if (angleDepth >= HALF_PI && angleDepth <= HALF_PI) // it the collision happend depth ...
					{
						details->objectIndexPlane1 = DOP18_PLANES_RIGHT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_LEFT_INDEX;
						sp_float smallestDistace = bv1.planeRight().distance(bv2.planeLeft());

						sp_float newDistace = bv1.planeDepth().distance(bv2.planeFront());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_DEPTH_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_FRONT_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeUp().distance(bv2.planeDown());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_UP_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_DOWN_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeUpRight().distance(bv2.planeDownLeft());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_UP_RIGHT_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_DOWN_LEFT_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeRightDepth().distance(bv2.planeLeftFront());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_RIGHT_DEPTH_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_LEFT_FRONT_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeUpDepth().distance(bv2.planeDownFront());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_UP_DEPTH_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_DOWN_FRONT_INDEX;
							smallestDistace = newDistace;
						}
					}
					else
					{
						details->objectIndexPlane1 = DOP18_PLANES_RIGHT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_LEFT_INDEX;
						sp_float smallestDistace = bv1.planeRight().distance(bv2.planeLeft());

						sp_float newDistace = bv1.planeFront().distance(bv2.planeDepth());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_FRONT_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_DEPTH_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeUp().distance(bv2.planeDown());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_UP_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_DOWN_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeUpRight().distance(bv2.planeDownLeft());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_UP_RIGHT_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_DOWN_LEFT_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeRightFront().distance(bv2.planeLeftDepth());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_RIGHT_FRONT_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_LEFT_DEPTH_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeUpFront().distance(bv2.planeDownDepth());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_UP_FRONT_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_DOWN_DEPTH_INDEX;
							smallestDistace = newDistace;
						}
					}
				}
				else
				{
					if (angleDepth >= HALF_PI && angleDepth <= HALF_PI) // it the collision happend depth ...
					{
						details->objectIndexPlane1 = DOP18_PLANES_LEFT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_RIGHT_INDEX;
						sp_float smallestDistace = bv1.planeLeft().distance(bv2.planeRight());

						sp_float newDistace = bv1.planeDepth().distance(bv2.planeFront());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_DEPTH_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_FRONT_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeUp().distance(bv2.planeDown());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_UP_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_DOWN_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeUpLeft().distance(bv2.planeDownRight());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_UP_LEFT_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_DOWN_RIGHT_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeLeftDepth().distance(bv2.planeRightFront());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_LEFT_DEPTH_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_RIGHT_FRONT_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeLeftDepth().distance(bv2.planeRightFront());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_UP_DEPTH_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_DOWN_FRONT_INDEX;
							smallestDistace = newDistace;
						}
					}
					else
					{
						details->objectIndexPlane1 = DOP18_PLANES_LEFT_INDEX;
						details->objectIndexPlane1 = DOP18_PLANES_RIGHT_INDEX;
						sp_float smallestDistace = bv1.planeLeft().distance(bv2.planeRight());

						sp_float newDistace = bv1.planeFront().distance(bv2.planeDepth());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_FRONT_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_DEPTH_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeUp().distance(bv2.planeDown());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_UP_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_DOWN_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeUpLeft().distance(bv2.planeDownRight());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_UP_LEFT_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_DOWN_RIGHT_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeLeftFront().distance(bv2.planeRightDepth());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_LEFT_FRONT_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_RIGHT_DEPTH_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeUpFront().distance(bv2.planeDownDepth());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_UP_FRONT_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_DOWN_DEPTH_INDEX;
							smallestDistace = newDistace;
						}
					}
				}
			}
			else
			{
				if (angleRight >= HALF_PI && angleRight <= HALF_PI) // it the collision happend right ...
				{
					if (angleDepth >= HALF_PI && angleDepth <= HALF_PI) // it the collision happend depth ...
					{
						details->objectIndexPlane1 = DOP18_PLANES_RIGHT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_LEFT_INDEX;
						sp_float smallestDistace = bv1.planeRight().distance(bv2.planeLeft());

						sp_float newDistace = bv1.planeDepth().distance(bv2.planeFront());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_DEPTH_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_FRONT_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeDown().distance(bv2.planeUp());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_DOWN_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_UP_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeDownRight().distance(bv2.planeUpLeft());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_DOWN_RIGHT_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_UP_LEFT_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeRightDepth().distance(bv2.planeLeftFront());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_RIGHT_DEPTH_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_LEFT_FRONT_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeDownDepth().distance(bv2.planeUpFront());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_DOWN_DEPTH_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_UP_FRONT_INDEX;
							smallestDistace = newDistace;
						}
					}
					else
					{
						details->objectIndexPlane1 = DOP18_PLANES_RIGHT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_LEFT_INDEX;
						sp_float smallestDistace = bv1.planeRight().distance(bv2.planeLeft());

						sp_float newDistace = bv1.planeFront().distance(bv2.planeDepth());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_FRONT_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_DEPTH_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeDown().distance(bv2.planeUp());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_DOWN_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_UP_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeDownRight().distance(bv2.planeUpLeft());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_DOWN_RIGHT_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_UP_LEFT_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeRightFront().distance(bv2.planeLeftDepth());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_RIGHT_FRONT_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_LEFT_DEPTH_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeRightFront().distance(bv2.planeLeftDepth());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_DOWN_FRONT_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_UP_DEPTH_INDEX;
							smallestDistace = newDistace;
						}
					}
				}
				else
				{
					if (angleDepth >= HALF_PI && angleDepth <= HALF_PI)
					{
						details->objectIndexPlane1 = DOP18_PLANES_LEFT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_RIGHT_INDEX;
						sp_float smallestDistace = bv1.planeLeft().distance(bv2.planeRight());

						sp_float newDistace = bv1.planeDepth().distance(bv2.planeFront());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_DEPTH_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_FRONT_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeDown().distance(bv2.planeUp());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_DOWN_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_UP_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeDownLeft().distance(bv2.planeUpRight());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_DOWN_LEFT_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_UP_RIGHT_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeLeftDepth().distance(bv2.planeRightFront());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_LEFT_DEPTH_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_RIGHT_FRONT_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeDownDepth().distance(bv2.planeUpFront());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_DOWN_DEPTH_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_UP_FRONT_INDEX;
							smallestDistace = newDistace;
						}
					}
					else
					{
						details->objectIndexPlane1 = DOP18_PLANES_LEFT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_RIGHT_INDEX;
						sp_float smallestDistace = bv1.planeLeft().distance(bv2.planeRight());

						sp_float newDistace = bv1.planeFront().distance(bv2.planeDepth());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_FRONT_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_DEPTH_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeDown().distance(bv2.planeUp());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_DOWN_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_UP_INDEX;
							smallestDistace = newDistace;

							Plane3D bv1PlaneLeftDepth = bv1.planeLeftDepth();
							Plane3D bv1PlaneLeftFront = bv1.planeLeftFront();
							Plane3D bv1PlaneRightDepth = bv1.planeRightDepth();
							Plane3D bv1PlaneRightFront = bv1.planeRightFront();

							Plane3D bv2PlaneLeftDepth = bv2.planeLeftDepth();
							Plane3D bv2PlaneLeftFront = bv2.planeLeftFront();
							Plane3D bv2PlaneRightDepth = bv2.planeRightDepth();
							Plane3D bv2PlaneRightFront = bv2.planeRightFront();

							Vec2 vertexes[4] = {
								Vec2(bv1.min[DOP18_AXIS_X], bv1.min[DOP18_AXIS_Z]),
								Vec2(bv1.max[DOP18_AXIS_X], bv1.min[DOP18_AXIS_Z]),
								Vec2(bv1.max[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Z]),
								Vec2(bv1.min[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Z])
							};

							const Vec3 temp0 = -bv1PlaneLeftFront.closestPointOnThePlane(Vec3(vertexes[0].x, bv1PlaneLeftFront.point.y, vertexes[0].y));
							const Vec3 temp1 = -bv1PlaneRightDepth.closestPointOnThePlane(Vec3(vertexes[2].x, bv1PlaneRightDepth.point.y, vertexes[2].y));
							const Vec3 temp2 = bv1PlaneRightFront.closestPointOnThePlane(Vec3(vertexes[1].x, bv1PlaneRightFront.point.y, vertexes[1].y));
							const Vec3 temp3 = bv1PlaneLeftDepth.closestPointOnThePlane(Vec3(vertexes[3].x, bv1PlaneLeftDepth.point.y, vertexes[3].y));

							vertexes[0].x = temp0.z;
							vertexes[0].y = temp0.x;

							vertexes[1].x = temp1.z;
							vertexes[1].y = temp2.x;
														
							vertexes[2].x = temp2.z;
							vertexes[2].y = temp1.x;
							
							vertexes[3].x = temp3.z;
							vertexes[3].y = temp3.x;

							vertexes[0] = (vertexes[0] + vertexes[1] + vertexes[2] + vertexes[3]) * 0.25f;
							details->contactPoint = Vec3(vertexes[0].x, bv1.planeDown().point.y, vertexes[0].y);
						}

						newDistace = bv1.planeDownLeft().distance(bv2.planeUpRight());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_DOWN_LEFT_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_UP_RIGHT_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeLeftFront().distance(bv2.planeRightDepth());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_LEFT_FRONT_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_RIGHT_DEPTH_INDEX;
							smallestDistace = newDistace;
						}

						newDistace = bv1.planeDownFront().distance(bv2.planeUpDepth());
						if (newDistace < smallestDistace)
						{
							details->objectIndexPlane1 = DOP18_PLANES_DOWN_FRONT_INDEX;
							details->objectIndexPlane2 = DOP18_PLANES_UP_DEPTH_INDEX;
							smallestDistace = newDistace;
						}
					}
				}
			}
		}

		void handleCollisionResponse(sp_uint objIndex1, sp_uint objIndex2, const SpCollisionDetails& details)
		{
			SpPhysicProperties* obj1Properties = &_physicProperties[objIndex1];
			SpPhysicProperties* obj2Properties = &_physicProperties[objIndex2];

			const sp_float sumMass = obj1Properties->massInverse() + obj2Properties->massInverse();
			const Vec3 relativeVelocity = obj2Properties->velocity() - obj1Properties->velocity();
			const sp_float cor = std::min(obj1Properties->coeficientOfRestitution(), obj2Properties->coeficientOfRestitution());

			if (obj1Properties->isDynamic())
			{
				const Line3D lineOfAction(obj1Properties->position(), details.contactPoint);

				if (obj2Properties->isDynamic())
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

					obj1Properties->_velocity = lineOfAction.direction() * newForceObj1;
				}
				else
					obj1Properties->_velocity = (lineOfAction.direction()  * obj1Properties->velocity()) * cor;

				obj1Properties->_acceleration = ZERO_FLOAT;
			}

			if (obj2Properties->isDynamic())
			{
				const Line3D lineOfAction(obj2Properties->position(), details.contactPoint);

				if (obj1Properties->isDynamic())
				{
					sp_float factor1 =
						((1.0f + cor) * obj1Properties->massInverse())
						/ sumMass;

					sp_float factor2 =
						(obj2Properties->massInverse() - cor * obj1Properties->massInverse())
						/ sumMass;

					const Vec3 newForceObj2
						= (relativeVelocity * factor1)
						+ (relativeVelocity * factor2);

					obj2Properties->_velocity = lineOfAction.direction() * newForceObj2;
				}
				else
					obj2Properties->_velocity = (lineOfAction.direction() * relativeVelocity) * cor;

				obj2Properties->_acceleration = ZERO_FLOAT;
			}
		}

		void handleCollision(const sp_uint objIndex1, const sp_uint objIndex2, sp_float elapsedTime)
		{
			sp_assert(objIndex1 != objIndex2, "InvalidArgumentException");

			SpPhysicProperties* obj1Properties = &_physicProperties[objIndex1];
			SpPhysicProperties* obj2Properties = &_physicProperties[objIndex2];

			const sp_bool isObj1Static = obj1Properties->isStatic();
			const sp_bool isObj2Static = obj2Properties->isStatic();

			if (isObj1Static && isObj1Static)
				return;

			const sp_bool isObj1Resting = obj1Properties->isResting();
			const sp_bool isObj2Resting = obj2Properties->isResting();

			if (isObj1Resting && isObj2Resting)
			{
				obj1Properties->_acceleration = Vec3(ZERO_FLOAT);
				obj1Properties->_velocity = Vec3(ZERO_FLOAT);
				obj2Properties->_acceleration = Vec3(ZERO_FLOAT);
				obj2Properties->_velocity = Vec3(ZERO_FLOAT);
				return;
			}

			if (isObj1Static && isObj2Resting)
			{
				obj2Properties->_acceleration = Vec3(ZERO_FLOAT);
				obj2Properties->_velocity = Vec3(ZERO_FLOAT);
				return;
			}
			
			if (isObj2Static && isObj1Resting)
			{
				obj1Properties->_acceleration = Vec3(ZERO_FLOAT);
				obj1Properties->_velocity = Vec3(ZERO_FLOAT);
				return;
			}

			SpCollisionDetails details;
			collisionDetails(objIndex1, objIndex2, elapsedTime, &details);

			handleCollisionResponse(objIndex1, objIndex2, details);

			dispatchEvent(objIndex1, objIndex2);
		}

	public:

		SpPhysicSyncronizer* syncronizer;

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

			const Vec3 newVelocity = element->velocity()
				+ (element->acceleration() + newAcceleration) * (elapsedTime * 0.5f);

			const Vec3 translation = newPosition - element->position();
			_boundingVolumes[index].translate(translation); // Sync with the bounding Volume
			syncronizer->sync(index, translation);

			element->_previousAcceleration = element->acceleration();
			element->_acceleration = newAcceleration;

			element->_previousVelocity = element->velocity();
			element->_velocity = newVelocity;

			element->_previousPosition = element->position();
			element->_position = newPosition;

			element->_previousForce = element->force();
			element->_force = ZERO_FLOAT;
		}

		/// <summary>
		/// Back the object to the state before timestep
		/// </summary>
		API_INTERFACE inline void backToTime(const sp_uint index)
		{
			SpPhysicProperties* element = &_physicProperties[index];
			DOP18* dop = &_boundingVolumes[index];

			const Vec3 translation = element->previousPosition() - element->position();

			dop->translate(translation);
			syncronizer->sync(index, translation);

			element->rollbackState();
		}

		API_INTERFACE void run(const Timer& timer);

		API_INTERFACE void dispose();

		~SpPhysicSimulator();

	};
}

#endif // SP_PHYSIC_SIMULATOR_HEADER