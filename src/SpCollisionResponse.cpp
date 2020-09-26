#include "SpCollisionResponse.h"

namespace NAMESPACE_PHYSICS
{

	void SpCollisionResponse::addFriction(SpPhysicProperties* obj1Properties, SpPhysicProperties* obj2Properties, const Vec3& relativeVel, const Vec3& collisionNormal, const Vec3& rayToContactObj1, const Vec3& rayToContactObj2, const sp_float& j, SpCollisionDetails* details)
	{
		const sp_float invMassSum = obj1Properties->massInverse() + obj2Properties->massInverse();

		Vec3 tangent = relativeVel - (collisionNormal * relativeVel.dot(collisionNormal));

		if (isCloseEnough(length(tangent), ZERO_FLOAT))
			return;

		normalize(&tangent);

		Vec3 temp1, temp2;
		cross(tangent, rayToContactObj1, &temp1);
		cross(tangent, rayToContactObj2, &temp2);

		Vec3 d2;
		cross(obj1Properties->inertialTensorInverse() * temp1, rayToContactObj1, &d2);
		
		Vec3 d3;
		cross(obj2Properties->inertialTensorInverse() * temp2, rayToContactObj2, &d3);
		
		sp_float denominator = invMassSum + tangent.dot(d2 + d3);

		if (denominator == ZERO_FLOAT)
			return;

		sp_float jt = (-relativeVel.dot(tangent)) / denominator;

		if (isCloseEnough(jt, ZERO_FLOAT))
			return;
		else
			jt /= details->contactPointsLength;

		const sp_float friction = sqrtf(obj1Properties->coeficientOfFriction() * obj2Properties->coeficientOfFriction());

		/* commented ?!
		if (jt > j * friction) {
			jt = j * friction;
		}
		else if (jt < -j * friction) {
			jt = -j * friction;
		}
		*/
		jt *= friction;

		sp_log_info1s("DENOMINADOR: ");
		sp_log_info1f(denominator);
		sp_log_newline();

		sp_log_info1s("CONTACTS: ");
		sp_log_info1f(details->contactPointsLength);
		sp_log_newline();
		
		sp_log_info1s("JT: ");
		sp_log_info1f(jt);
		sp_log_newline();

		Vec3 tangentImpuse = tangent * jt;

		cross(rayToContactObj1, tangentImpuse, &temp1);
		cross(rayToContactObj2, tangentImpuse, &temp2);

		obj1Properties->currentState._velocity += tangentImpuse;
		obj1Properties->currentState._angularVelocity += tangentImpuse * jt;

		obj2Properties->currentState._velocity -= tangentImpuse;
		obj2Properties->currentState._angularVelocity += temp2 * -jt;
	}

	void SpCollisionResponse::handleCollisionResponse(SpCollisionDetails* details)
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();

		const sp_float physicVelocity = SpPhysicSettings::instance()->physicVelocity();
		SpPhysicProperties* obj1Properties = simulator->physicProperties(details->objIndex1);
		SpPhysicProperties* obj2Properties = simulator->physicProperties(details->objIndex2);

		const sp_float invMassSum = obj1Properties->massInverse() + obj2Properties->massInverse();
		const sp_float cor = std::min(obj1Properties->coeficientOfRestitution(), obj2Properties->coeficientOfRestitution());

		const Vec3 centerObj1 = simulator->transforms(details->objIndex1)->position;
		const Vec3 centerObj2 = simulator->transforms(details->objIndex2)->position;

		const Vec3 rayToContactObj1 = (details->centerContactPoint - centerObj1);
		const Vec3 rayToContactObj2 = (details->centerContactPoint - centerObj2);

		const Vec3 collisionNormal = details->collisionNormalObj1;


		Vec3 angularCrossContactRayObj1, angularCrossContactRayObj2;
		cross(obj1Properties->currentState.angularVelocity(), rayToContactObj1, &angularCrossContactRayObj1);
		cross(obj2Properties->currentState.angularVelocity(), rayToContactObj2, &angularCrossContactRayObj2);

		Vec3 pointVelocityObj1, pointVelocityObj2;
		add(obj1Properties->currentState.velocity(), angularCrossContactRayObj1, &pointVelocityObj1);
		add(obj2Properties->currentState.velocity(), angularCrossContactRayObj2, &pointVelocityObj2);

		const sp_float relativeVel = collisionNormal.dot(pointVelocityObj1 - pointVelocityObj2);

		//Since object 1 is on the positive side of the normal 
		if (relativeVel < ZERO_FLOAT) // check objects are moving away
		{
			details->ignoreCollision = true;
			return;
		}

		const sp_float numerator = -(ONE_FLOAT + cor) * relativeVel;

		cross(rayToContactObj1, collisionNormal, &angularCrossContactRayObj1);
		cross(rayToContactObj2, collisionNormal, &angularCrossContactRayObj2);

		Vec3 d2;
		cross(obj1Properties->inertialTensorInverse() * angularCrossContactRayObj1, rayToContactObj1, &d2);

		Vec3 d3;
		cross(obj2Properties->inertialTensorInverse() * angularCrossContactRayObj2, rayToContactObj2, &d3);

		sp_float denominator = invMassSum + collisionNormal.dot(d2 + d3);
		sp_float j;

		if (denominator == ZERO_FLOAT)
			j = ZERO_FLOAT;
		else
			j = (numerator / denominator) / (sp_float)details->contactPointsLength;

		cross(rayToContactObj1, collisionNormal, &angularCrossContactRayObj1);
		cross(rayToContactObj2, collisionNormal, &angularCrossContactRayObj2);

		// TODO: REMOVER
		sp_log_info1s("COLISION RESPONSE:::::::::::::::    PREVIOUS VEL: ");
		sp_log_newline();
		sp_log_info1s("PREVIOUS VEL: ");
		sp_log_info3f(obj2Properties->currentState._velocity.x, obj2Properties->currentState._velocity.y, obj2Properties->currentState._velocity.z);
		sp_log_info1s("    CONTACT: ");
		sp_log_info3f(details->centerContactPoint.x, details->centerContactPoint.y, details->centerContactPoint.z);
		sp_log_info1s("    POS: ");
		sp_log_info3f(centerObj2.x, centerObj2.y, centerObj2.z);
		sp_log_info1s("    DENOMINADOR: ");
		sp_log_info1f(denominator);
		sp_log_newline();
		
		// se denominador < 1, o impulso será muito alto positivo para uma colisão mto baixa
		Vec3 impulse;
		if (denominator < HALF_FLOAT)
		{
			impulse = ZERO_FLOAT;
			sp_log_info1s("IMPULSO ZERADO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
			sp_log_info1f(denominator);
			sp_log_newline();
		}
		else
			impulse = collisionNormal * j;

		obj1Properties->currentState._velocity = impulse;
		obj1Properties->currentState._angularVelocity = angularCrossContactRayObj1 * j;

		obj2Properties->currentState._velocity = -impulse;
		obj2Properties->currentState._angularVelocity = angularCrossContactRayObj2 * -j;

		addFriction(obj1Properties, obj2Properties, relativeVel, collisionNormal , rayToContactObj1, rayToContactObj2, j, details);

		obj1Properties->currentState._acceleration = ZERO_FLOAT;
		obj1Properties->currentState._torque = ZERO_FLOAT;

		obj2Properties->currentState._acceleration = ZERO_FLOAT;
		obj2Properties->currentState._torque = ZERO_FLOAT;
	}

}
