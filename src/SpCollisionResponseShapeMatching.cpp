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
			transformation->transform(mesh->vertexesMesh->get(i)->value(), shape->particles[i]);

			// fill the initial shape with local coordinates
			diff(shape->particles[i], transformation->position, shape->initialParticles[i]);
		}
	}

	void SpCollisionResponseShapeMatching::run(const SweepAndPruneResult& sapResult)
	{
		SpWorld* world = SpWorldManagerInstance->current();
		SpRigidBodyShapeMatch** shapes = ALLOC_NEW_ARRAY(SpRigidBodyShapeMatch*, world->objectsLength());
		std::memset(shapes, ZERO_INT, SIZEOF_WORD * world->objectsLength());

		// init shapes
		for (sp_uint i = 0; i < sapResult.length; i++)
		{
			const sp_uint obj1 = sapResult.indexes[multiplyBy2(i)];
			const sp_uint obj2 = sapResult.indexes[multiplyBy2(i) + 1u];

			if (shapes[obj1] == nullptr)
			{
				shapes[obj1] = ALLOC_NEW(SpRigidBodyShapeMatch)();
				initShape(obj1, shapes[obj1]);
			}

			if (shapes[obj2] == nullptr)
			{
				shapes[obj2] = ALLOC_NEW(SpRigidBodyShapeMatch)();
				initShape(obj2, shapes[obj2]);
			}
		}

		// first iteration shape matching
		for (sp_uint i = 0u; i < sapResult.length; i++)
			firstSolve(
				shapes[sapResult.indexes[multiplyBy2(i)]],
				shapes[sapResult.indexes[multiplyBy2(i) + 1u]]
			);
		// many shape match iterations
		for (sp_uint iterations = 1u; iterations < 10u; iterations++)
		{
			for (sp_uint i = 0u; i < sapResult.length; i++)
				solve(
					shapes[sapResult.indexes[multiplyBy2(i)]],
					shapes[sapResult.indexes[multiplyBy2(i) + 1u]]
				);
		}

		for (sp_uint i = 0; i < sapResult.length; i++)
		{
			sp_uint obj1 = sapResult.indexes[multiplyBy2(i)];
			sp_uint obj2 = sapResult.indexes[multiplyBy2(i) + 1];

			if (shapes[obj1]->isDirty && world->rigidBody3D(obj1)->isDynamic())
			{
				updateFromShape(shapes[obj1]);
				shapes[obj1]->isDirty = false;
			}

			if (shapes[obj2]->isDirty && world->rigidBody3D(obj2)->isDynamic())
			{
				updateFromShape(shapes[obj2]);
				shapes[obj2]->isDirty = false;
			}
		}
	}

	sp_bool getShapeMatchingDetails(SpRigidBodyShapeMatch* shape, Vec3& newPosition, Mat3& rotation)
	{
		shape->centerOfMass(newPosition);

		Mat3 matrixApq;
		std::memcpy(matrixApq.values(), Mat3Zeros, sizeof(Mat3));

		const Vec3* particles = shape->particles;

		for (sp_uint i = 0u; i < shape->particlesLength; i++)
		{
			Vec3 relativePosition;
			diff(particles[i], newPosition, relativePosition);

			Mat3 tempMatrix;
			multiply(relativePosition, shape->initialParticles[i], tempMatrix);
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

		//multiply(matrixApq, sqrtSymetricInv, rotation);
		multiply(sqrtSymetricInv, matrixApq, rotation);

		// check sqrtSymetric is almost singular and bad conditioned.
		if (!NAMESPACE_FOUNDATION::isCloseEnough(rotation.m11, ONE_FLOAT, SP_EPSILON_ONE_DIGITS)
			|| !NAMESPACE_FOUNDATION::isCloseEnough(rotation.m22, ONE_FLOAT, SP_EPSILON_ONE_DIGITS)
			|| !NAMESPACE_FOUNDATION::isCloseEnough(rotation.m33, ONE_FLOAT, SP_EPSILON_ONE_DIGITS))
			return false; 

		return true;
	}

	sp_bool SpCollisionResponseShapeMatching::shapeMatch(SpRigidBodyShapeMatch* shape)
	{
		Mat3 rotation;
		Vec3 centerOfMassAfterCollision;
		if (!getShapeMatchingDetails(shape, centerOfMassAfterCollision, rotation))
			return false;

		for (sp_uint i = 0u; i < shape->particlesLength; i++)
		{
			Vec3 newPosition;
			rotation.multiply(shape->initialParticles[i], newPosition);

			add(newPosition, centerOfMassAfterCollision, shape->particles[i]);
		}

		return true;
	}

	void SpCollisionResponseShapeMatching::updateFromShape(SpRigidBodyShapeMatch* shape)
	{
		SpRigidBody3D* rigidBody = SpWorldManagerInstance->current()->rigidBody3D(shape->objectIndex);
		SpTransform* transformation = SpWorldManagerInstance->current()->transforms(shape->objectIndex);

		Mat3 rotation;
		Vec3 centerOfMassAfterCollision;

		if (getShapeMatchingDetails(shape, centerOfMassAfterCollision, rotation))
		{
			Quat newOrientation;
			rotation.convert(newOrientation);

			Quat finalOrientation;
			multiply(newOrientation, transformation->orientation, finalOrientation);
			normalize(finalOrientation, transformation->orientation);

			/*
			if (shape->objectIndex == 1)
			{
				Vec3 previousAngles;
				eulerAnglesXYZ(transformation->orientation, previousAngles);

				std::cout << "----------------------------" << std::endl;
				std::cout << "PREV OBJ " << shape->objectIndex << std::endl;
				std::cout << "X: " << degree(previousAngles.x) << std::endl;
				std::cout << "Y: " << degree(previousAngles.y) << std::endl;
				std::cout << "Z: " << degree(previousAngles.z) << std::endl;

				Vec3 angles;
				eulerAnglesXYZ(newOrientation, angles);

				std::cout << "NEW OBJ " << shape->objectIndex << std::endl;
				std::cout << "X: " << degree(angles.x) << std::endl;
				std::cout << "Y: " << degree(angles.y) << std::endl;
				std::cout << "Z: " << degree(angles.z) << std::endl;
			}
			*/

			rigidBody->currentState.orientation(transformation->orientation);

			transformation->position = centerOfMassAfterCollision;
			rigidBody->currentState.position(centerOfMassAfterCollision);
		}
		else
		{
			transformation->position = centerOfMassAfterCollision;
			rigidBody->currentState.position(centerOfMassAfterCollision);
		}
	}

	sp_bool SpCollisionResponseShapeMatching::hasPlaneCollision(SpRigidBodyShapeMatch* shape, SpCollisionDetails* collisionManifold)
	{
		const SpVertexMesh* supportPointMesh = SpWorldManagerInstance->current()
			->mesh(SpWorldManagerInstance->current()->collisionFeatures(shape->objectIndex)->meshIndex)
			->support(Vec3Down, shape->particles);

		const sp_float pointY = shape->particles[supportPointMesh->index()].y;

		if (pointY < ZERO_FLOAT)
		{
			collisionManifold->collisionNormal = Vec3Down;
			collisionManifold->depth = -pointY;
			return true;
		}

		return false;
	}

	sp_bool SpCollisionResponseShapeMatching::hasCollision(SpRigidBodyShapeMatch* shape1, SpRigidBodyShapeMatch* shape2, SpCollisionDetails* collisionManifold)
	{
		if (shape1->objectIndex == ZERO_UINT)
			return hasPlaneCollision(shape2, collisionManifold);
		
		if (shape2->objectIndex == ZERO_UINT)
			return hasPlaneCollision(shape1, collisionManifold);

		SpWorld* world = SpWorldManagerInstance->current();
		const SpMesh* mesh1 = world->mesh(world->collisionFeatures(shape1->objectIndex)->meshIndex);
		const SpMesh* mesh2 = world->mesh(world->collisionFeatures(shape2->objectIndex)->meshIndex);

		Timer t;
		t.start();

		const sp_uint maxIterations = mesh1->vertexLength() * mesh2->vertexLength();
		sp_uint gjkIterations = sp_max((sp_uint)(maxIterations * SpPhysicSettings::instance()->gjkPrecision()), (sp_uint)10);
		sp_uint epaIterations = sp_max((sp_uint)(maxIterations * SpPhysicSettings::instance()->epaPrecision()), (sp_uint)10);

		sp_uint iterations = 0;
		static sp_uint mIterations = 0;
		static sp_uint avgIterations = 0;
		static sp_size gjkErro = 0;
		static sp_size gjkAcerto = 0;
		static sp_uint gjkCounter = 0;

		Vec3 tetrahedron[4];
		if (!gjk(mesh1, shape1->particles, mesh2, shape2->particles, tetrahedron, gjkIterations, iterations))
		{
			gjkCounter++;
			mIterations = sp_max(mIterations, iterations);
			avgIterations += iterations;

			if (gjk(mesh1, shape1->particles, mesh2, shape2->particles, tetrahedron, 576, iterations))
				gjkErro++; // gjkIterations itera��es n�o foram suficientes
			else
				gjkAcerto++;

			((sp_float*)SpGlobalPropertiesInscance->get(ID_gjkEpaTime))[0] += t.elapsedTime();
			((sp_uint*)SpGlobalPropertiesInscance->get(ID_gjkEpaCount))[0] ++;
			return false;
		}

		if (iterations > 100 || mIterations > 100)
			int a = 1;

		mIterations = sp_max(mIterations, iterations);
		avgIterations += iterations;
		gjkAcerto++;
		gjkCounter++;

		if (gjkCounter == 10000000)
		{
			int a = 1;
		}

		static sp_size epaErro = 0;
		static sp_size epaAcerto = 0;
		static sp_size epaGlobalErro = 0;
		static sp_size epaMIterations = 0;
		static sp_size epaAverage = 0;
		static sp_size epaCouter = 0;
		iterations = 0;
		sp_float depth;

		if (epa(tetrahedron, mesh1, shape1->particles, mesh2, shape2->particles, collisionManifold->collisionNormal, collisionManifold->depth, maxIterations, iterations))
			depth = collisionManifold->depth;
		else
			epaGlobalErro++;

		collisionManifold->depth = 0.0f;
		epaCouter++;
		if (iterations != 576)
			epaMIterations = sp_max((sp_uint)epaMIterations, iterations);

		epaAverage += iterations;

		if (!epa(tetrahedron, mesh1, shape1->particles, mesh2, shape2->particles, collisionManifold->collisionNormal, collisionManifold->depth, epaIterations, iterations))
		{
			((sp_float*)SpGlobalPropertiesInscance->get(ID_gjkEpaTime))[0] += t.elapsedTime();
			((sp_uint*)SpGlobalPropertiesInscance->get(ID_gjkEpaCount))[0] ++;
			return false;
		}

		if (depth != collisionManifold->depth)
		{
			if (iterations != 576)
				epaMIterations = sp_max((sp_uint)epaMIterations, iterations);

			epaErro++;
		}
		else
			epaAcerto++;

		if (epaCouter == 10000000)
		{
			int a = 1;
		}

		((sp_float*)SpGlobalPropertiesInscance->get(ID_gjkEpaTime))[0] += t.elapsedTime();
		((sp_uint*)SpGlobalPropertiesInscance->get(ID_gjkEpaCount))[0] ++;
		return true;
	}

	void SpCollisionResponseShapeMatching::firstSolve(SpRigidBodyShapeMatch* shape1, SpRigidBodyShapeMatch* shape2)
	{
		SpCollisionDetails collisionManifold;

		if (!hasCollision(shape1, shape2, &collisionManifold))
			return;

		((sp_uint*)SpGlobalPropertiesInscance->get(ID_paresReais))[0] ++;

		updateParticles(shape1, shape2, collisionManifold);
	}

	void SpCollisionResponseShapeMatching::solve(SpRigidBodyShapeMatch* shape1, SpRigidBodyShapeMatch* shape2)
	{
		SpCollisionDetails collisionManifold;

		if (!hasCollision(shape1, shape2, &collisionManifold))
			return;

		updateParticles(shape1, shape2, collisionManifold);
	}

	void SpCollisionResponseShapeMatching::updateParticlesShape(SpRigidBodyShapeMatch* shape, const Plane& plane)
	{
		for (sp_uint i = 0u; i < shape->particlesLength; i++)
		{
			const sp_float distance = plane.distance(shape->particles[i]);

			if (distance < ZERO_FLOAT)
			{
				Vec3 newPosition;
				plane.project(shape->particles[i], &newPosition);

				shape->particles[i] = newPosition;
			}
		}
	}

	void SpCollisionResponseShapeMatching::updateParticles(SpRigidBodyShapeMatch* shape1, SpRigidBodyShapeMatch* shape2, const SpCollisionDetails& collisionManifold)
	{
		const sp_float depth = sp_abs(collisionManifold.depth);
		const SpWorld* world = SpWorldManagerInstance->current();

		Vec3 positionObj1, positionObj2, direction, normalToObj2;
		shape1->centerOfMass(positionObj1);

		if (shape2->objectIndex == 0) // if plane, ...
		{
			positionObj2 = Vec3(positionObj1.x, ZERO_FLOAT, positionObj1.z);
			normalToObj2 = Vec3Down;
		}
		else
		{
			shape2->centerOfMass(positionObj2);

			if (shape1->objectIndex == 0) // if plane, ...
				normalToObj2 = Vec3Up;
			else
			{
				diff(positionObj2, positionObj1, direction);
				normalize(direction);

				if (direction.dot(collisionManifold.collisionNormal) > ZERO_FLOAT)
					normalToObj2 = collisionManifold.collisionNormal;
				else
					normalToObj2 = -collisionManifold.collisionNormal;

				//normalToObj2 = direction;
			}
		}

		if (SpWorldManagerInstance->current()->rigidBody3D(shape1->objectIndex)->isDynamic())
		{
			const SpMesh* mesh2 = world->mesh(world->collisionFeatures(shape2->objectIndex)->meshIndex);
			SpVertexMesh* supportPointMesh = mesh2->support(-normalToObj2, shape2->particles);
			const Vec3 supportPoint = shape2->particles[supportPointMesh->index()];

			Plane plane(supportPoint, -normalToObj2);
			updateParticlesShape(shape1, plane);

			Timer t;
			t.start();
			//if (shapeMatch(shape1))
			shapeMatch(shape1);
			((sp_float*)SpGlobalPropertiesInscance->get(ID_shapeMatchingTime))[0] += t.elapsedTime();
			((sp_uint*)SpGlobalPropertiesInscance->get(ID_shapeMatchingCount))[0] ++;

			shape1->isDirty = true;
		}

		if (SpWorldManagerInstance->current()->rigidBody3D(shape2->objectIndex)->isDynamic())
		{
			const SpMesh* mesh1 = world->mesh(world->collisionFeatures(shape1->objectIndex)->meshIndex);
			SpVertexMesh* supportPointMesh = mesh1->support(normalToObj2, shape1->particles);
			const Vec3 supportPoint = shape1->particles[supportPointMesh->index()];

			Plane plane(supportPoint, normalToObj2);
			updateParticlesShape(shape2, plane);

			Timer t;
			t.start();
			//if (shapeMatch(shape2))
			shapeMatch(shape2);
			((sp_float*)SpGlobalPropertiesInscance->get(ID_shapeMatchingTime))[0] += t.elapsedTime();
			((sp_uint*)SpGlobalPropertiesInscance->get(ID_shapeMatchingCount))[0] ++;

			shape2->isDirty = true;
		}
	}
	
}