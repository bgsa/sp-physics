#include "SpCollisionResponseShapeMatching.h"

namespace NAMESPACE_PHYSICS
{

	void SpCollisionResponseShapeMatching::initShape(const sp_uint objIndex, SpRigidBodyShapeMatch* shape)
	{
		const sp_uint meshIndex = SpWorldManagerInstance->current()->collisionFeatures(objIndex)->meshIndex;
		SpMesh* mesh = SpWorldManagerInstance->current()->mesh(meshIndex);
		SpTransform* transformation = SpWorldManagerInstance->current()->transforms(objIndex);
		
		shape->objectIndex = objIndex;
		shape->initialPosition = transformation->position;
		shape->particlesLength = mesh->vertexLength();
		shape->particles = ALLOC_NEW_ARRAY(Vec3, shape->particlesLength);
		shape->initialParticles = ALLOC_NEW_ARRAY(Vec3, shape->particlesLength);

		for (sp_uint i = 0; i < shape->particlesLength; i++)
		{
			transformation->transform(mesh->vertexesMesh->get(i)->value(), &shape->particles[i]);

			// fill the initial shape with local coordinates
			diff(shape->particles[i], transformation->position, shape->initialParticles[i]);
		}
	}

	sp_bool SpCollisionResponseShapeMatching::shapeMatch(SpRigidBodyShapeMatch* shape)
	{
		Vec3 centerOfMassAfterCollision;
		shape->centerOfMass(centerOfMassAfterCollision);
	
		const sp_float mass = ONE_FLOAT / SpWorldManagerInstance->current()->rigidBody3D(shape->objectIndex)->massInverse();

		Mat3 matrixApq;
		std::memcpy(matrixApq.values(), Mat3Zeros, sizeof(Mat3));

		const Vec3* particles = shape->particles;

		for (sp_uint i = 0u; i < shape->particlesLength; i++)
		{
			Vec3 relativePosition;
			diff(particles[i], centerOfMassAfterCollision, relativePosition);

			Mat3 tempMatrix;
			multiply(relativePosition, shape->initialParticles[i], tempMatrix);
			multiply(tempMatrix, mass, tempMatrix);
			matrixApq += tempMatrix;
		}

		Mat3 symetricMatrix;
		matrixApq.symmetric(symetricMatrix);

		Mat3 sqrtSymetric;
		if (!sqrtm(symetricMatrix, sqrtSymetric, 40))
			return false;

		if (sqrtSymetric.isSingular()) // matriz cannot be inverted
			return false;

		Mat3 sqrtSymetricInv;
		inverse(sqrtSymetric, sqrtSymetricInv);
		//transpose(sqrtSymetric, sqrtSymetricInv);

		Mat3 rotationMatrix;
		multiply(matrixApq, sqrtSymetricInv, rotationMatrix);

		for (sp_uint i = 0u; i < shape->particlesLength; i++)
		{
			Vec3 newPosition;
			rotationMatrix.multiply(shape->initialParticles[i], newPosition);

			add(newPosition, centerOfMassAfterCollision, shape->particles[i]);
		}

		return true;
	}

	sp_bool SpCollisionResponseShapeMatching::updateFromShape(const SpRigidBodyShapeMatch* shape)
	{
		SpRigidBody3D* rigidBody = SpWorldManagerInstance->current()->rigidBody3D(shape->objectIndex);
		const Vec3* particles = shape->particles;

		// find new center of mass...
		Vec3 centerOfMassAfterCollision;
		shape->centerOfMass(centerOfMassAfterCollision);

		const sp_float mass = ONE_FLOAT / rigidBody->massInverse();

		Mat3 matrixApq;
		std::memcpy(&matrixApq, &Mat3Zeros, sizeof(Mat3));

		for (sp_uint i = 0u; i < shape->particlesLength; i++)
		{
			Vec3 relativePosition;
			diff(particles[i], centerOfMassAfterCollision, relativePosition);

			Mat3 tempMatrix;
			multiply(relativePosition, shape->initialParticles[i], tempMatrix);
			multiply(tempMatrix, mass, tempMatrix);
			matrixApq += tempMatrix;
		}

		Mat3 temp;
		matrixApq.transpose(temp);

		Mat3 symetricMatrix;
		multiply(temp, matrixApq, symetricMatrix);

		if (!sqrtm(symetricMatrix, temp, 40))
			return false;

		if (temp.isSingular()) // matrix cannot be inverted
			return false;

		Mat3 sqrtSymetricInv;
		inverse(temp, sqrtSymetricInv);

		Mat3 rotationMatrix;
		multiply(matrixApq, sqrtSymetricInv, rotationMatrix);

		Quat newOrientation;
		rotationMatrix.convert(newOrientation);

		SpTransform* transformation = SpWorldManagerInstance->current()->transforms(shape->objectIndex);
		transformation->orientation *= newOrientation;
		rigidBody->currentState.orientation(transformation->orientation);

		transformation->position = centerOfMassAfterCollision;
		rigidBody->currentState.position(centerOfMassAfterCollision);

		return true;
	}

	sp_bool SpCollisionResponseShapeMatching::hasPlaneCollision(SpRigidBodyShapeMatch* shape, SpCollisionDetails* collisionManifold, sp_uint& pointsLength, SpVertexMesh** points)
	{
		SpWorldManagerInstance->current()
			->mesh(SpWorldManagerInstance->current()->collisionFeatures(shape->objectIndex)->meshIndex)
			->supportAll(Vec3Down, shape->particles, pointsLength, points);

		sp_assert(pointsLength > ZERO_UINT, "ApplicationException");

		const sp_float pointY = shape->particles[points[0]->index()].y;

		if (pointY < ZERO_FLOAT)
		{
			collisionManifold->collisionNormal = Vec3Down;
			collisionManifold->depth = -pointY;
			return true;
		}

		return false;
	}

	sp_bool SpCollisionResponseShapeMatching::hasCollision(SpRigidBodyShapeMatch* shape1, SpRigidBodyShapeMatch* shape2, SpCollisionDetails* collisionManifold, sp_uint& mesh1PointsLength, SpVertexMesh** mesh1Points, sp_uint& mesh2PointsLength, SpVertexMesh** mesh2Points)
	{
		if (shape1->objectIndex == ZERO_UINT)
			return hasPlaneCollision(shape2, collisionManifold, mesh2PointsLength, mesh2Points);
		
		if (shape2->objectIndex == ZERO_UINT)
			return hasPlaneCollision(shape1, collisionManifold, mesh1PointsLength, mesh1Points);


		SpWorld* world = SpWorldManagerInstance->current();
		const SpMesh* mesh1 = world->mesh(world->collisionFeatures(shape1->objectIndex)->meshIndex);
		const SpMesh* mesh2 = world->mesh(world->collisionFeatures(shape2->objectIndex)->meshIndex);

		Vec3 tetrahedron[4];
		if (!gjk(mesh1, shape1->particles, mesh2, shape2->particles, tetrahedron))
			return false;

		if (!epa(tetrahedron, mesh1, shape1->particles, mesh2, shape2->particles, collisionManifold->collisionNormal, collisionManifold->depth))
			return false;

		/*
		if (collisionManifold->depth >= 2.0f)
			collisionManifold->depth = 2.0f;
		*/

		mesh1->supportAll(-collisionManifold->collisionNormal, shape1->particles, mesh1PointsLength, mesh1Points, SP_EPSILON_TWO_DIGITS);
		mesh2->supportAll(collisionManifold->collisionNormal, shape2->particles, mesh2PointsLength, mesh2Points, SP_EPSILON_TWO_DIGITS);

		if (mesh1PointsLength == ONE_UINT)
		{
			collisionManifold->type = mesh2PointsLength == TWO_UINT
				? SpCollisionType::VertexEdge
				: SpCollisionType::VertexFace;
		}
		else if (mesh1PointsLength == TWO_UINT)
		{
			switch (mesh2PointsLength)
			{
			case ONE_UINT:
				collisionManifold->type = SpCollisionType::VertexFace;
				break;

			case TWO_UINT:
				collisionManifold->type = SpCollisionType::EdgeEdge; 
				break;

			default:
				collisionManifold->type = SpCollisionType::EdgeFace;
				break;
			}
		}
		else
		{
			switch (mesh2PointsLength)
			{
			case ONE_UINT:
				collisionManifold->type = SpCollisionType::VertexFace;
				break;

			case TWO_UINT:
				collisionManifold->type = SpCollisionType::EdgeFace;
				break;

			default:
				collisionManifold->type = SpCollisionType::FaceFace;
				break;
			}
		}

		return true;
	}

	void SpCollisionResponseShapeMatching::solve(SpRigidBodyShapeMatch* shape1, SpRigidBodyShapeMatch* shape2)
	{
		SpCollisionDetails collisionManifold;

		sp_uint mesh1PointsLength = ZERO_UINT, mesh2PointsLength = ZERO_UINT;
		SpVertexMesh* mesh1Points[12], *mesh2Points[12];
		if (!hasCollision(shape1, shape2, &collisionManifold, mesh1PointsLength, mesh1Points, mesh2PointsLength, mesh2Points))
			return;

		updateParticles(shape1, shape2, collisionManifold, mesh1PointsLength, mesh1Points, mesh2PointsLength, mesh2Points);
	}

	void SpCollisionResponseShapeMatching::updateParticlesShape(SpRigidBodyShapeMatch* shape, const Vec3& collisionNormal, const sp_float depth, sp_uint& pointsLength, SpVertexMesh** points)
	{
		for (sp_uint i = 0u; i < pointsLength; i++)
		{
			const sp_uint pointIndex = points[i]->index();

			sp_assert(pointIndex < shape->particlesLength, "ApplicationException");

			const Vec3 newPosition = (-collisionNormal * depth) + shape->particles[pointIndex];

			shape->particles[pointIndex] = newPosition;
		}
	}

	void SpCollisionResponseShapeMatching::updateParticles(SpRigidBodyShapeMatch* shape1, SpRigidBodyShapeMatch* shape2, const SpCollisionDetails& collisionManifold, sp_uint& mesh1PointsLength, SpVertexMesh** mesh1Points, sp_uint& mesh2PointsLength, SpVertexMesh** mesh2Points)
	{
		const sp_float depth = fabsf(collisionManifold.depth);
		Vec3 normalToObj1;

		if (SpWorldManagerInstance->current()->rigidBody3D(shape1->objectIndex)->isDynamic())
		{
			Vec3 directionObj1;
			shape1->centerOfMass(directionObj1);
			diff(shape1->particles[mesh1Points[0]->index()], directionObj1, directionObj1);
			normalize(directionObj1);

			if (directionObj1.dot(collisionManifold.collisionNormal) > ZERO_FLOAT)
				normalToObj1 = collisionManifold.collisionNormal;
			else
				normalToObj1 = -collisionManifold.collisionNormal;

			updateParticlesShape(shape1, normalToObj1, depth, mesh1PointsLength, mesh1Points);
			shapeMatch(shape1);
			shape1->isDirty = true;
		}
		else
		{
			Vec3 directionObj2;
			shape2->centerOfMass(directionObj2);
			diff(shape2->particles[mesh2Points[0]->index()], directionObj2, directionObj2);
			normalize(directionObj2);

			if (directionObj2.dot(collisionManifold.collisionNormal) > ZERO_FLOAT)
				normalToObj1 = -collisionManifold.collisionNormal;
			else
				normalToObj1 = collisionManifold.collisionNormal;
		}

		if (SpWorldManagerInstance->current()->rigidBody3D(shape2->objectIndex)->isDynamic())
		{
			updateParticlesShape(shape2, -normalToObj1, depth, mesh2PointsLength, mesh2Points);
			shapeMatch(shape2);
			shape2->isDirty = true;
		}
	}
	
}