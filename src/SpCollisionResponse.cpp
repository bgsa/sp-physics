#include "SpCollisionResponse.h"

namespace NAMESPACE_PHYSICS
{

	void SpCollisionResponse::addFriction(SpRigidBody3D* obj1Properties, SpRigidBody3D* obj2Properties, const Vec3& relativeVel, const Vec3& collisionNormal, const sp_bool obj2IsPositiveNormal, const Vec3& rayToContactObj1, const Vec3& rayToContactObj2, const sp_float& j, SpCollisionDetails* details)
	{
		const sp_float invMassSum = obj1Properties->massInverse() + obj2Properties->massInverse();

		Vec3 tangent = relativeVel - (collisionNormal * relativeVel.dot(collisionNormal));

		if (NAMESPACE_FOUNDATION::isCloseEnough(length(tangent), ZERO_FLOAT, SP_EPSILON_TWO_DIGITS))
			return;

		normalize(tangent);

		Vec3 temp1, temp2, d2, d3, temp;
		cross(rayToContactObj1, tangent, &temp1);
		cross(rayToContactObj2, tangent, &temp2);

		obj1Properties->inertialTensorInverse().multiply(temp1, temp);
		cross(temp, rayToContactObj1, &d2);

		obj2Properties->inertialTensorInverse().multiply(temp2, temp);
		cross(temp, rayToContactObj2, &d3);

		const sp_float denominator = invMassSum + tangent.dot(d2 + d3);

		sp_float jt = -(relativeVel.dot(tangent)) / denominator;

		if (NAMESPACE_FOUNDATION::isCloseEnough(jt, ZERO_FLOAT))
			return;

		jt /= details->contactPointsLength;

		const sp_float friction = sqrtf(obj1Properties->coeficientOfFriction() * obj2Properties->coeficientOfFriction());

		if (jt > j * friction) 
			jt = j * friction;
		else if (jt < -j * friction) 
			jt = -j * friction;

		const Vec3 tangentImpuse = tangent * jt;

		if (!obj1Properties->isResting())
		{
			if (obj2IsPositiveNormal)
			{
				obj1Properties->currentState._velocity -= tangentImpuse * obj1Properties->massInverse() * obj1Properties->damping();
				cross(rayToContactObj1, tangentImpuse, &temp1);

				obj1Properties->inertialTensorInverse().multiply(temp1, temp);
				obj1Properties->currentState._angularVelocity -= temp * obj1Properties->angularDamping();
			}
			else
			{
				obj1Properties->currentState._velocity += tangentImpuse * obj1Properties->massInverse() * obj1Properties->damping();
				cross(rayToContactObj1, tangentImpuse, &temp1);

				obj1Properties->inertialTensorInverse().multiply(temp1, temp);
				obj1Properties->currentState._angularVelocity += temp * obj1Properties->angularDamping();
			}
		}

		if (!obj2Properties->isResting())
		{
			if (obj2IsPositiveNormal)
			{
				obj2Properties->currentState._velocity += tangentImpuse * obj2Properties->massInverse() * obj2Properties->damping();
				cross(rayToContactObj2, tangentImpuse, &temp1);

				obj2Properties->inertialTensorInverse().multiply(temp1, temp);
				obj2Properties->currentState._angularVelocity += temp * obj2Properties->angularDamping();
			}
			else
			{
				obj2Properties->currentState._velocity -= tangentImpuse * obj2Properties->massInverse() * obj2Properties->damping();
				cross(rayToContactObj2, tangentImpuse, &temp1);

				obj2Properties->inertialTensorInverse().multiply(temp1, temp);
				obj2Properties->currentState._angularVelocity -= temp * obj2Properties->angularDamping();
			}
		}
	}

	void SpCollisionResponse::handleCollisionResponse(SpCollisionDetails* details)
	{
		SpWorld* world = SpWorldManagerInstance->current();

		const sp_float physicVelocity = SpPhysicSettings::instance()->physicVelocity();
		SpRigidBody3D* obj1Properties = world->rigidBody3D(details->objIndex1);
		SpRigidBody3D* obj2Properties = world->rigidBody3D(details->objIndex2);

		const sp_float invMassSum = obj1Properties->massInverse() + obj2Properties->massInverse();
		const sp_float cor = std::min(obj1Properties->coeficientOfRestitution(), obj2Properties->coeficientOfRestitution());
		const Vec3 collisionNormal = details->collisionNormal;

		const Vec3 centerObj1 = world->transforms(details->objIndex1)->position;
		const Vec3 centerObj2 = world->transforms(details->objIndex2)->position;

		Plane contactFace(details->centerContactPoint, collisionNormal);

		Vec3 rayToContactObj1, rayToContactObj2, angularCrossContactRayObj1, angularCrossContactRayObj2;
		diff(details->centerContactPoint, centerObj1, rayToContactObj1);
		diff(details->centerContactPoint, centerObj2, rayToContactObj2);

		// get relative velocity
		cross(rayToContactObj1, obj1Properties->currentState.angularVelocity(), &angularCrossContactRayObj1);
		cross(rayToContactObj2, obj2Properties->currentState.angularVelocity(), &angularCrossContactRayObj2);

		Vec3 pointVelocityObj1, pointVelocityObj2;
		add(obj1Properties->currentState.velocity(), angularCrossContactRayObj1, pointVelocityObj1);
		add(obj2Properties->currentState.velocity(), angularCrossContactRayObj2, pointVelocityObj2);

		Vec3 relativeVel;
		sp_float relativeVelocityAtNormal;

		const sp_bool obj2IsPositiveNormal = contactFace.distance(centerObj2) > ZERO_FLOAT;
		
		if (obj2IsPositiveNormal)
			diff(pointVelocityObj1, pointVelocityObj2, relativeVel);
		else
			diff(pointVelocityObj2, pointVelocityObj1, relativeVel);

		relativeVelocityAtNormal = relativeVel.dot(collisionNormal);

		if (relativeVelocityAtNormal < ZERO_FLOAT) // check if objects are moving away
		{
			details->ignoreCollision = true;
			return;
		}

		const sp_float numerator = -(ONE_FLOAT + cor) * relativeVelocityAtNormal;

		cross(rayToContactObj1, collisionNormal, &angularCrossContactRayObj1);
		cross(rayToContactObj2, collisionNormal, &angularCrossContactRayObj2);

		Vec3 d2, temp;
		obj1Properties->inertialTensorInverse().multiply(angularCrossContactRayObj1, temp);
		cross(temp, rayToContactObj1, &d2);

		Vec3 d3;
		obj2Properties->inertialTensorInverse().multiply(angularCrossContactRayObj2, temp);
		cross(temp, rayToContactObj2, &d3);

		const sp_float denominator = invMassSum + collisionNormal.dot(d2 + d3);
		sp_float j;

		if (denominator == ZERO_FLOAT)
			j = ZERO_FLOAT;
		else
			j = (numerator / denominator) / (sp_float)details->contactPointsLength;

		
		if (obj1Properties->isResting())
		{
			obj1Properties->currentState._position = obj1Properties->previousState._position;
			obj1Properties->currentState._orientation = obj1Properties->previousState._orientation;
			obj1Properties->currentState._velocity = Vec3Zeros;
			obj1Properties->currentState._angularVelocity = Vec3Zeros;
		}
		else
		{
			Vec3 angularImpulse1;
			cross(rayToContactObj1, collisionNormal, &angularImpulse1);

			if (obj2IsPositiveNormal)
			{
				obj1Properties->currentState._velocity = (collisionNormal * j) * obj1Properties->massInverse() * obj1Properties->damping();
				obj1Properties->currentState._angularVelocity = angularImpulse1 * obj1Properties->angularDamping();
			}
			else
			{
				obj1Properties->currentState._velocity = (collisionNormal * -j) * obj1Properties->massInverse() * obj1Properties->damping();
				obj1Properties->currentState._angularVelocity = -angularImpulse1 * obj1Properties->angularDamping();
			}
		}

		/*
		Vec3 n;
		normalize(rayToContactObj2, &n);
		sp_float angle = collisionNormal.dot(-n);
		sp_bool isSloped = angle != 1.0f;
		*/

		if (obj2Properties->isResting())
		{
			obj2Properties->currentState._position = obj2Properties->previousState._position;
			obj2Properties->currentState._orientation = obj2Properties->previousState._orientation;
			obj2Properties->currentState._velocity = Vec3Zeros;
			obj2Properties->currentState._angularVelocity = Vec3Zeros;
		}
		else
		{
			Vec3 angularImpulse2;
			cross(rayToContactObj2, collisionNormal, &angularImpulse2);
			
			if (obj2IsPositiveNormal)
			{
				obj2Properties->currentState._velocity = (collisionNormal * -j) * obj2Properties->massInverse() * obj2Properties->damping();
				obj2Properties->currentState._angularVelocity = -angularImpulse2 * obj2Properties->angularDamping();
			}
			else
			{
				obj2Properties->currentState._velocity = (collisionNormal * j) * obj2Properties->massInverse() * obj2Properties->damping();
				obj2Properties->currentState._angularVelocity = angularImpulse2 * obj2Properties->angularDamping();
			}
		}

		addFriction(obj1Properties, obj2Properties, relativeVel, collisionNormal, obj2IsPositiveNormal, rayToContactObj1, rayToContactObj2, j, details);

		obj1Properties->currentState._acceleration = Vec3Zeros;
		obj1Properties->currentState._torque = Vec3Zeros;

		obj2Properties->currentState._acceleration = Vec3Zeros;
		obj2Properties->currentState._torque = Vec3Zeros;
	}

}
