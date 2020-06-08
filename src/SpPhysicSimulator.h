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

			if (!obj1Properties->isMovable() && !obj2Properties->isMovable())
				return;

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

							Vec3 planeVertexes1[8]; // get vertexes of plane 1
							sp_uint planeVertexes1Length;
							bv1.pointsFromPlaneDown(planeVertexes1, &planeVertexes1Length);

							Vec3 planeVertexes2[8]; // get vertexes of plane 2
							sp_uint planeVertexes2Length;
							bv2.pointsFromPlaneUp(planeVertexes2, &planeVertexes2Length);

							SpArray<Vec3> vertexesReliesOnPlane1(8u);
							Plane3D bv1PlaneDepth = bv1.planeDepth();
							Plane3D bv1PlaneFront = bv1.planeFront();
							Plane3D bv1PlaneLeft = bv1.planeLeft();
							Plane3D bv1PlaneRight = bv1.planeRight();
							Plane3D bv1PlaneLeftDepth = bv1.planeLeftDepth();
							Plane3D bv1PlaneLeftFront = bv1.planeLeftFront();
							Plane3D bv1PlaneRightDepth = bv1.planeRightDepth();
							Plane3D bv1PlaneRightFront = bv1.planeRightFront();

							for (sp_uint i = 0; i < planeVertexes2Length; i++)
							{
								if  (  bv1PlaneDepth.orientation(planeVertexes2[i]) == Orientation::RIGHT
									&& bv1PlaneFront.orientation(planeVertexes2[i]) == Orientation::RIGHT
									&& bv1PlaneLeft.orientation(planeVertexes2[i]) == Orientation::RIGHT
									&& bv1PlaneRight.orientation(planeVertexes2[i]) == Orientation::RIGHT
									&& bv1PlaneLeftDepth.orientation(planeVertexes2[i]) == Orientation::RIGHT
									&& bv1PlaneLeftFront.orientation(planeVertexes2[i]) == Orientation::RIGHT
									&& bv1PlaneRightDepth.orientation(planeVertexes2[i]) == Orientation::RIGHT
									&& bv1PlaneRightFront.orientation(planeVertexes2[i]) == Orientation::RIGHT 
								)
									vertexesReliesOnPlane1.add(planeVertexes2[i]);
							}

							// if all vertexes of obj 2 are inside obj 1, so ...
							if (vertexesReliesOnPlane1.length() == planeVertexes2Length)
							{
								details->contactPoint = planeVertexes2[0];

								for (sp_uint i = 0; i < planeVertexes2Length; i++)
									details->contactPoint 
										= (details->contactPoint + planeVertexes2[0])
										* 0.5f;
							}

							SpArray<Vec3> vertexesReliesOnPlane2(8u);
							Plane3D bv2PlaneDepth = bv2.planeDepth();
							Plane3D bv2PlaneFront = bv2.planeFront();
							Plane3D bv2PlaneLeft = bv2.planeLeft();
							Plane3D bv2PlaneRight = bv2.planeRight();
							Plane3D bv2PlaneLeftDepth = bv2.planeLeftDepth();
							Plane3D bv2PlaneLeftFront = bv2.planeLeftFront();
							Plane3D bv2PlaneRightDepth = bv2.planeRightDepth();
							Plane3D bv2PlaneRightFront = bv2.planeRightFront();

							for (sp_uint i = 0u; i < planeVertexes1Length; i++)
							{
								if (   bv2PlaneDepth.orientation(planeVertexes1[i]) == Orientation::RIGHT
									&& bv2PlaneFront.orientation(planeVertexes1[i]) == Orientation::RIGHT
									&& bv2PlaneLeft.orientation(planeVertexes1[i]) == Orientation::RIGHT
									&& bv2PlaneRight.orientation(planeVertexes1[i]) == Orientation::RIGHT
									&& bv2PlaneLeftDepth.orientation(planeVertexes1[i]) == Orientation::RIGHT
									&& bv2PlaneLeftFront.orientation(planeVertexes1[i]) == Orientation::RIGHT
									&& bv2PlaneRightDepth.orientation(planeVertexes1[i]) == Orientation::RIGHT
									&& bv2PlaneRightFront.orientation(planeVertexes1[i]) == Orientation::RIGHT
								)
									vertexesReliesOnPlane2.add(planeVertexes2[i]);
							}

							// if all vertexes of obj 1 are inside obj 2, so ...
							if (vertexesReliesOnPlane2.length() == planeVertexes1Length)
							{
								details->contactPoint = planeVertexes1[0];

								for (sp_uint i = 1u; i < planeVertexes1Length; i++)
									details->contactPoint
									= (details->contactPoint + planeVertexes1[i])
									* 0.5f;
							}

							// SENÂO pegar todos os vértices do 2 que estão no 1
							//       pegar cada aresta do 2 e verificar colisão com as arestas do 1
							// Pegar o ponto médio de todos os vértices encontrados. Este é o ponto.

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

			if (obj1Properties->isMovable())
			{
				obj1Properties->_acceleration = ZERO_FLOAT;

				const Line3D lineOfAction(obj1Properties->position(), details.contactPoint);

				if (obj2Properties->isMovable())
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
			}

			if (obj2Properties->isMovable())
			{
				obj2Properties->_acceleration = ZERO_FLOAT;

				const Line3D lineOfAction(obj2Properties->position(), details.contactPoint);

				if (obj1Properties->isMovable())
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
			}
		}

		void handleCollision(const sp_uint objIndex1, const sp_uint objIndex2, sp_float elapsedTime)
		{
			sp_assert(objIndex1 != objIndex2, "InvalidArgumentException");

			SpPhysicProperties* obj1Properties = &_physicProperties[objIndex1];
			SpPhysicProperties* obj2Properties = &_physicProperties[objIndex2];

			if (!obj1Properties->isMovable() && !obj2Properties->isMovable())
				return;

			SpCollisionDetails details;
			collisionDetails(objIndex1, objIndex2, elapsedTime, &details);

			handleCollisionResponse(objIndex1, objIndex2, details);
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