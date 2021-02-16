#include "SpCollisionResponseShapeMatching.h"

namespace NAMESPACE_PHYSICS
{

	void SpCollisionResponseShapeMatching::initShape(const sp_uint objIndex, SpRigidBodyShapeMatch* shape)
	{
		const sp_uint meshIndex = SpPhysicSimulator::instance()->collisionFeatures(objIndex)->meshIndex;
		SpMesh* mesh = SpPhysicSimulator::instance()->mesh(meshIndex);
		SpTransform* transformation = SpPhysicSimulator::instance()->transforms(objIndex);
		SpPhysicProperties* physicProperties = SpPhysicSimulator::instance()->physicProperties(objIndex);

		shape->objectIndex = objIndex;
		shape->initialPosition = transformation->position;
		shape->particlesLength = mesh->vertexLength();
		shape->particles = ALLOC_NEW_ARRAY(Vec3, shape->particlesLength);
		shape->initialParticles = ALLOC_NEW_ARRAY(Vec3, shape->particlesLength);

		for (sp_uint i = 0; i < shape->particlesLength; i++)
		{
			transformation->transform(mesh->vertexesMesh->get(i)->value(), &shape->particles[i]);

			// fill the initial shape with local coordinates
			diff(shape->particles[i], transformation->position, &shape->initialParticles[i]);
		}
	}

	Eigen::EigenSolver<Eigen::MatrixXf> solver;

	void SpCollisionResponseShapeMatching::shapeMatch(SpRigidBodyShapeMatch* shape)
	{
		SpPhysicProperties* physicProperties = SpPhysicSimulator::instance()->physicProperties(shape->objectIndex);
		const Vec3* particles = shape->particles;
		
		// find new center of mass...
		Vec3 centerOfMassAfterCollision;
		shape->centerOfMass(centerOfMassAfterCollision);
	
		Mat3 matrixApq = Mat3Zeros;
		const sp_float mass = ONE_FLOAT / physicProperties->massInverse();
		
		for (sp_uint i = 0u; i < shape->particlesLength; i++)
		{
			Vec3 relativePosition;
			diff(particles[i], centerOfMassAfterCollision, &relativePosition);

			Mat3 tempMatrix;
			multiply(relativePosition, shape->initialParticles[i], &tempMatrix);
			multiply(tempMatrix, mass, &tempMatrix);
			matrixApq += tempMatrix;
		}
		
		Mat3 symetricMatrix;
		multiply(matrixApq.transpose(), matrixApq, symetricMatrix);

		sp_assert(symetricMatrix.isSymetric(), "ApplicationException");

		/*
		Mat3 diagonalizedMatrix;
		symetricMatrix.diagonalize(&diagonalizedMatrix, &iterations, SP_EPSILON_TWO_DIGITS);
		Vec3 eigenValues;
		diagonalizedMatrix.primaryDiagonal(&eigenValues);
		*/

		Mat3 sqrtSymetric;
		sqrtm(symetricMatrix, sqrtSymetric);

		Mat3 sqrtSymetricInv;
		inverse(sqrtSymetric, sqrtSymetricInv);

		Mat3 rotationMatrix;
		multiply(matrixApq, sqrtSymetricInv, rotationMatrix);

		//SystemOfLinearEquations system;
		//sp_log_debug1s("Rotation: \n");
		//sp_log_debug1s(system.printMatrix(rotationMatrix, 3, 3).c_str());

		Eigen::MatrixXf matrxApqEigen = Eigen::Map<Eigen::MatrixXf>(matrixApq, 3, 3);
		Eigen::MatrixXf symetricMatrixEgen = Eigen::Map<Eigen::MatrixXf>(symetricMatrix, 3, 3);
		Eigen::MatrixXf inverseMatrixS = symetricMatrixEgen.sqrt().inverse();

		Eigen::Map<Eigen::MatrixXf>(rotationMatrix, 3, 3) = matrxApqEigen * inverseMatrixS;
		//sp_log_debug1s("Rotation Eigen: \n");
		//sp_log_debug1s(system.printMatrix(rotationMatrix, 3, 3).c_str());

		for (sp_uint i = 0u; i < shape->particlesLength; i++)
		{
			Vec3 newPosition;
			rotationMatrix.multiply(shape->initialParticles[i], newPosition);

			shape->particles[i] = newPosition + centerOfMassAfterCollision;
		}
	}

	void SpCollisionResponseShapeMatching::updateFromShape(const sp_uint objIndex, const SpCollisionDetails* details, const SpRigidBodyShapeMatch* shape)
	{
		SpPhysicProperties* physicProperties = SpPhysicSimulator::instance()->physicProperties(shape->objectIndex);
		const Vec3* particles = shape->particles;

		// find new center of mass...
		Vec3 centerOfMassAfterCollision;
		shape->centerOfMass(centerOfMassAfterCollision);

		Mat3 matrixApq = Mat3Zeros;
		const sp_float mass = ONE_FLOAT / physicProperties->massInverse();

		for (sp_uint i = 0u; i < shape->particlesLength; i++)
		{
			Vec3 relativePosition;
			diff(particles[i], centerOfMassAfterCollision, &relativePosition);

			Mat3 tempMatrix;
			multiply(relativePosition, shape->initialParticles[i], &tempMatrix);
			multiply(tempMatrix, mass, &tempMatrix);
			matrixApq += tempMatrix;
		}

		Mat3 symetricMatrix;
		multiply(matrixApq.transpose(), matrixApq, symetricMatrix);

		Mat3 sqrtSymetric;
		sqrtm(symetricMatrix, sqrtSymetric, 10);

		Mat3 sqrtSymetricInv;
		inverse(sqrtSymetric, sqrtSymetricInv);

		Mat3 MyrotationMatrix;
		multiply(matrixApq, sqrtSymetricInv, MyrotationMatrix);

		/*
		SystemOfLinearEquations system;
		sp_log_debug1s("Rotation: \n");
		sp_log_debug1s(system.printMatrix(MyrotationMatrix, 3, 3).c_str());

		Eigen::MatrixXf symetricMatrixEgen = Eigen::Map<Eigen::MatrixXf>(symetricMatrix, 3, 3);
		Eigen::MatrixXf inverseMatrixS = symetricMatrixEgen.sqrt().inverse();
		Eigen::MatrixXf matrxApqEigen = Eigen::Map<Eigen::MatrixXf>(matrixApq, 3, 3);		
		Mat3 rotationMatrix;
		Eigen::Map<Eigen::MatrixXf>(rotationMatrix, 3, 3) = matrxApqEigen * inverseMatrixS;

		sp_log_debug1s("Rotation Eigen: \n");
		sp_log_debug1s(system.printMatrix(rotationMatrix, 3, 3).c_str());
		
		for (sp_uint i = 0; i < 9; i++)
			if (!NAMESPACE_FOUNDATION::isCloseEnough(MyrotationMatrix[i], rotationMatrix[i]))
				int a = 1;
		*/

		Quat newOrientation;
		MyrotationMatrix.convert(newOrientation);

		SpTransform* transformation = SpPhysicSimulator::instance()->transforms(shape->objectIndex);
		transformation->orientation *= newOrientation;
		physicProperties->currentState.orientation(transformation->orientation);

		transformation->position = centerOfMassAfterCollision;
		physicProperties->currentState.position(transformation->position);
	}

	sp_bool SpCollisionResponseShapeMatching::hasPlaneCollision(SpRigidBodyShapeMatch* shape, SpCollisionDetails* collisionManifold, sp_uint& pointsLength, SpVertexMesh** points)
	{
		SpPhysicSimulator::instance()
			->mesh(SpPhysicSimulator::instance()->collisionFeatures(shape->objectIndex)->meshIndex)
			->supportAll(Vec3Down, shape->particles, pointsLength, points);

		const sp_float pointY = shape->particles[points[0]->index()].y;

		if (pointY < ZERO_FLOAT)
		{
			collisionManifold->collisionNormal = Vec3Down;
			collisionManifold->depth = -pointY;
			return true;
		}

		return false;

		/*

		Vec3* particles = shape2->particles;
		sp_uint particleIndex = SP_UINT_MAX;
		for (sp_uint i = 0; i < shape2->particlesLength; i++)
			if (particles[i].y < ZERO_FLOAT)
			{
				particleIndex = i;
				break;
			}

		if (particleIndex == SP_UINT_MAX)
			return false;

		SpVertexMesh* vertexMesh = mesh->vertexesMesh->get(particleIndex);

		collisionManifold->edgeObjectIndex = shape2->objectIndex;

		collisionManifold->faceObjectIndex = shape1->objectIndex;
		collisionManifold->faceIndex = 0u;
		collisionManifold->collisionNormal = Vec3Up;

		// find penetrated edge to fill collision manifold
		sp_float offset = ZERO_FLOAT;	
		while (collisionManifold->edgeIndex == SP_UINT_MAX)
		{
			for (sp_uint i = 0; i < vertexMesh->edgeLength(); i++)
			{
				SpEdgeMesh* e = vertexMesh->edges(i);
				const sp_float v1_y = particles[e->vertexIndex1].y;
				const sp_float v2_y = particles[e->vertexIndex2].y;

				// check this edge cross the plane
				if ((v1_y + offset <= ZERO_FLOAT && v2_y + offset > ZERO_FLOAT)
					|| (v2_y + offset <= ZERO_FLOAT && v1_y + offset > ZERO_FLOAT))
				{
					collisionManifold->edgeIndex = e->index();
					collisionManifold->depth = -(v1_y < ZERO_FLOAT ? v1_y : v2_y);
					break;
				}
			}
			offset += HALF_FLOAT;
		}

		sp_assert(collisionManifold->edgeIndex != SP_UINT_MAX, "ApplicationException");

		return true;
		*/
	}

	sp_bool SpCollisionResponseShapeMatching::hasCollision(SpRigidBodyShapeMatch* shape1, SpRigidBodyShapeMatch* shape2, SpCollisionDetails* collisionManifold, sp_uint& mesh1PointsLength, SpVertexMesh** mesh1Points, sp_uint& mesh2PointsLength, SpVertexMesh** mesh2Points)
	{
		if (shape1->objectIndex == ZERO_UINT)
			return hasPlaneCollision(shape2, collisionManifold, mesh2PointsLength, mesh2Points);
		
		if (shape2->objectIndex == ZERO_UINT)
			return hasPlaneCollision(shape1, collisionManifold, mesh1PointsLength, mesh1Points);


		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		const SpMesh* mesh1 = simulator->mesh(simulator->collisionFeatures(shape1->objectIndex)->meshIndex);
		const SpMesh* mesh2 = simulator->mesh(simulator->collisionFeatures(shape2->objectIndex)->meshIndex);

		Vec3 tetrahedron[4];
		if (!gjk(mesh1, shape1->particles, mesh2, shape2->particles, tetrahedron))
			return false;

		epa(tetrahedron, mesh1, shape1->particles, mesh2, shape2->particles, collisionManifold->collisionNormal, collisionManifold->depth);

		mesh1->supportAll(-collisionManifold->collisionNormal, shape1->particles, mesh1PointsLength, mesh1Points, SP_EPSILON_TWO_DIGITS);
		mesh2->supportAll(collisionManifold->collisionNormal, shape2->particles, mesh2PointsLength, mesh2Points, SP_EPSILON_TWO_DIGITS);

		/*
		sp_char text[5000];
		sp_uint textLength, textLength2 = ZERO_UINT;
		Matlab::convert(*mesh1, shape1->particles, "mesh1", "red", text, &textLength);
		textLength --;
		Matlab::convert(*mesh2, shape2->particles, "mesh2", "green", &text[textLength], &textLength2);
		textLength += textLength2;
		Matlab::display(-10, 10, -10, 10, -10, 10, &text[textLength -1u], &textLength2);
		textLength += textLength2;
		Matlab::drawSphere(Vec3Zeros, 0.1f, &text[textLength - 1u], &textLength2);
		textLength += textLength2;
		*/

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
		const sp_float coeficientOfRestitution = 0.5f;

		for (sp_uint i = 0u; i < pointsLength; i++)
		{
			const sp_uint pointIndex = points[i]->index();

			const Vec3 newPosition = (-collisionNormal * depth) + shape->particles[pointIndex];
		
			shape->particles[pointIndex] = newPosition;
		}
	}

	void SpCollisionResponseShapeMatching::updateParticles(SpRigidBodyShapeMatch* shape1, SpRigidBodyShapeMatch* shape2, const SpCollisionDetails& collisionManifold, sp_uint& mesh1PointsLength, SpVertexMesh** mesh1Points, sp_uint& mesh2PointsLength, SpVertexMesh** mesh2Points)
	{
		const sp_float depth = fabsf(collisionManifold.depth);
		Vec3 normalToObj1;

		if (SpPhysicSimulator::instance()->physicProperties(shape1->objectIndex)->isDynamic())
		{
			Vec3 directionObj1;
			shape1->centerOfMass(directionObj1);
			diff(shape1->particles[mesh1Points[0]->index()], directionObj1, &directionObj1);
			normalize(&directionObj1);

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
			diff(shape2->particles[mesh2Points[0]->index()], directionObj2, &directionObj2);
			normalize(&directionObj2);

			if (directionObj2.dot(collisionManifold.collisionNormal) > ZERO_FLOAT)
				normalToObj1 = -collisionManifold.collisionNormal;
			else
				normalToObj1 = collisionManifold.collisionNormal;
		}

		if (SpPhysicSimulator::instance()->physicProperties(shape2->objectIndex)->isDynamic())
		{
			updateParticlesShape(shape2, -normalToObj1, depth, mesh2PointsLength, mesh2Points);
			shapeMatch(shape2);
			shape2->isDirty = true;
		}
	}
	
}