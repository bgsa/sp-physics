#include "SpCollisionResponse.h"

namespace NAMESPACE_PHYSICS
{

	void SpCollisionResponse::addFriction(SpPhysicProperties* obj1Properties, SpPhysicProperties* obj2Properties, const Vec3& relativeVel, const Vec3& collisionNormal, const Vec3& rayToContactObj1, const Vec3& rayToContactObj2, const sp_float& j)
	{
		const sp_float invMassSum = obj1Properties->massInverse() + obj2Properties->massInverse();

		Vec3 tangent = relativeVel - (collisionNormal * relativeVel.dot(collisionNormal));

		if (isCloseEnough((tangent.x + tangent.y + tangent.z), ZERO_FLOAT))
			return;

		tangent = tangent.normalize();
		sp_float numerator = -relativeVel.dot(tangent);
		Vec3 d2 = (obj1Properties->inertialTensorInverse() * tangent.cross(rayToContactObj1)).cross(rayToContactObj1);
		Vec3 d3 = (obj2Properties->inertialTensorInverse() * tangent.cross(rayToContactObj2)).cross(rayToContactObj2);
		sp_float denominator = invMassSum + tangent.dot(d2 + d3);

		if (denominator == 0.0f)
			return;

		sp_float jt = numerator / denominator;

		if (isCloseEnough(jt, 0.0f))
			return;

		const sp_float friction = sqrtf(obj1Properties->coeficientOfFriction() * obj2Properties->coeficientOfFriction());
		if (jt > j * friction) {
			jt = j * friction;
		}
		else if (jt < -j * friction) {
			jt = -j * friction;
		}

		const Vec3 tangentImpuse = tangent * jt;

		obj1Properties->currentState._velocity = obj1Properties->currentState.velocity() - tangentImpuse * obj1Properties->massInverse();
		obj1Properties->currentState._angularVelocity = obj1Properties->currentState.angularVelocity() - obj1Properties->inertialTensorInverse() * rayToContactObj1.cross(tangentImpuse);

		obj2Properties->currentState._velocity = obj2Properties->currentState.velocity() + tangentImpuse * obj2Properties->massInverse();
		obj2Properties->currentState._angularVelocity = obj2Properties->currentState.angularVelocity() + obj2Properties->inertialTensorInverse() * rayToContactObj2.cross(tangentImpuse);
	}

	void SpCollisionResponse::handleCollisionResponse(SpCollisionDetails* details)
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();

		const sp_float physicVelocity = SpPhysicSettings::instance()->physicVelocity();
		SpPhysicProperties* obj1Properties = simulator->physicProperties(details->objIndex1);
		SpPhysicProperties* obj2Properties = simulator->physicProperties(details->objIndex2);

		/*
		if (obj1Properties->isStatic())
		{
			const Vec3 center = simulator->transforms(details->objIndex2)->position;
			handleCollisionResponseWithStatic(details, obj2Properties, center);
		}
		else if (obj2Properties->isStatic())
		{
			const Vec3 center = simulator->transforms(details->objIndex1)->position;
			handleCollisionResponseWithStatic(details, obj1Properties, center);
		}
		else
		{
		*/
			const sp_float invMassSum = obj1Properties->massInverse() + obj2Properties->massInverse();
			const sp_float cor = std::min(obj1Properties->coeficientOfRestitution(), obj2Properties->coeficientOfRestitution());

			const Vec3 centerObj1 = simulator->transforms(details->objIndex1)->position;
			const Vec3 centerObj2 = simulator->transforms(details->objIndex2)->position;

			const Vec3 rayToContactObj1 = (details->centerContactPoint - centerObj1);
			const Vec3 rayToContactObj2 = (details->centerContactPoint - centerObj2);

			//const Vec3 collisionNormal = (details->contactPointsObj1[0] - centerObj1).normalize();
			const Vec3 collisionNormal = details->collisionNormalObj2;

			const Vec3 relativeVel = (obj2Properties->currentState.velocity() + obj2Properties->currentState.angularVelocity().cross(rayToContactObj2))
				- (obj1Properties->currentState.velocity() + obj2Properties->currentState.angularVelocity().cross(rayToContactObj1));

			sp_float numerator = (-(1.0f + cor) * relativeVel.dot(collisionNormal));

			Vec3 d2 = (obj1Properties->inertialTensorInverse() * rayToContactObj1.cross(collisionNormal))
				.cross(rayToContactObj1);

			Vec3 d3 = (obj2Properties->inertialTensorInverse() * rayToContactObj2.cross(collisionNormal))
				.cross(rayToContactObj2);

			sp_float denominator = invMassSum + collisionNormal.dot(d2 + d3);

			sp_float j = denominator == ZERO_FLOAT ? ZERO_FLOAT
				: numerator / denominator;

			if (details->contactPointsLength > ZERO_FLOAT && j != ZERO_FLOAT)
				j /= (sp_float)details->contactPointsLength;

			const Vec3 impulse = collisionNormal * j;

			obj1Properties->currentState._velocity = -impulse * obj1Properties->massInverse();
			obj1Properties->currentState._angularVelocity = obj1Properties->inertialTensorInverse() * rayToContactObj1.cross(-impulse);

			obj2Properties->currentState._velocity = impulse * obj2Properties->massInverse();
			obj2Properties->currentState._angularVelocity = obj2Properties->inertialTensorInverse() * rayToContactObj2.cross(impulse);

			//addFriction(obj1Properties, obj2Properties, relativeVel, collisionNormal , rayToContactObj1, rayToContactObj2, j);

			obj1Properties->currentState._acceleration = ZERO_FLOAT;
			obj1Properties->currentState._torque = ZERO_FLOAT;

			obj2Properties->currentState._acceleration = ZERO_FLOAT;
			obj2Properties->currentState._torque = ZERO_FLOAT;
		//}
	}

}
