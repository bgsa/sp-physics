#include "SpectrumPhysicsTest.h"
#include <SpCollisionDetector.h>
#include "Asserts.h"
#include "SpMapleExporter.h"

#define CLASS_NAME SpCollisionDetectorTest

namespace NAMESPACE_PHYSICS_TEST
{

	SP_TEST_CLASS(CLASS_NAME)
	{
	private:

		void createMeshePlane()
		{
			SpPhysicSimulator* simulator = SpPhysicSimulator::instance();

			const sp_uint vertexesLength = 8u;
			Vec3 vertexes[vertexesLength] = {
				Vec3(-1.0f, -1.0f, 1.0f), Vec3(-1.0f, -1.0f, -1.0f),
				Vec3(-1.0f, 1.0f, -1.0f), Vec3(-1.0f, 1.0f, 1.0f),
				Vec3(1.0f, -1.0f, 1.0f), Vec3(1.0f, 1.0f, 1.0f),
				Vec3(1.0f, 1.0f, -1.0f), Vec3(1.0f, -1.0f, -1.0f)
			};

			const sp_uint facesLength = 12u;
			SpPoint3<sp_uint> facesIndex[facesLength] = {
				SpPoint3<sp_uint>(1, 0, 3), SpPoint3<sp_uint>(3, 2, 1), // left faces ok
				SpPoint3<sp_uint>(4, 7, 6), SpPoint3<sp_uint>(6, 5, 4), // right faces ok
				SpPoint3<sp_uint>(1, 2, 6), SpPoint3<sp_uint>(6, 7, 1), // back faces ok 
				SpPoint3<sp_uint>(0, 4, 5), SpPoint3<sp_uint>(5, 3, 0), // front faces ok
				SpPoint3<sp_uint>(3, 5, 6), SpPoint3<sp_uint>(6, 2, 3), // top face ok
				SpPoint3<sp_uint>(1, 7, 4), SpPoint3<sp_uint>(4, 0, 1)  // bottom face ok
			};

			SpMesh* mesh = sp_mem_new(SpMesh)();
			mesh->vertexesMesh = sp_mem_new(SpArray<SpVertexMesh*>)(vertexesLength);
			mesh->faces = sp_mem_new(SpArray<SpFaceMesh*>)(facesLength);

			for (sp_uint i = 0; i < vertexesLength; i++)
				mesh->vertexesMesh->add(sp_mem_new(SpVertexMesh)(mesh, i, vertexes[i]));

			for (sp_uint i = 0; i < facesLength; i++)
				mesh->faces->add(sp_mem_new(SpFaceMesh)(mesh, i, facesIndex[i].x, facesIndex[i].y, facesIndex[i].z));

			mesh->init();

			SpPhysicSimulator::instance()->mesh(0u, mesh);
		}

		void createCubeMeshes(const sp_uint index)
		{
			SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
			
			const sp_uint vertexesLength = 8u;
			Vec3 vertexes[vertexesLength] = {
				Vec3(-1.0f, -1.0f, 1.0f), Vec3(-1.0f, -1.0f, -1.0f),
				Vec3(-1.0f, 1.0f, -1.0f), Vec3(-1.0f, 1.0f, 1.0f),
				Vec3(1.0f, -1.0f, 1.0f), Vec3(1.0f, 1.0f, 1.0f),
				Vec3(1.0f, 1.0f, -1.0f), Vec3(1.0f, -1.0f, -1.0f)
			};

			const sp_uint facesLength = 12u;
			SpPoint3<sp_uint> facesIndex[facesLength] = {
				SpPoint3<sp_uint>(1, 0, 3), SpPoint3<sp_uint>(3, 2, 1), // left faces ok
				SpPoint3<sp_uint>(4, 7, 6), SpPoint3<sp_uint>(6, 5, 4), // right faces ok
				SpPoint3<sp_uint>(1, 2, 6), SpPoint3<sp_uint>(6, 7, 1), // back faces ok 
				SpPoint3<sp_uint>(0, 4, 5), SpPoint3<sp_uint>(5, 3, 0), // front faces ok
				SpPoint3<sp_uint>(3, 5, 6), SpPoint3<sp_uint>(6, 2, 3), // top face ok
				SpPoint3<sp_uint>(1, 7, 4), SpPoint3<sp_uint>(4, 0, 1)  // bottom face ok
			};

			SpMesh* mesh = sp_mem_new(SpMesh)();
			mesh->vertexesMesh = sp_mem_new(SpArray<SpVertexMesh*>)(vertexesLength);
			mesh->faces = sp_mem_new(SpArray<SpFaceMesh*>)(facesLength);
				
			for (sp_uint i = 0; i < vertexesLength; i++)
				mesh->vertexesMesh->add(sp_mem_new(SpVertexMesh)(mesh, i, vertexes[i]));

			for (sp_uint i = 0; i < facesLength; i++)
				mesh->faces->add(sp_mem_new(SpFaceMesh)(mesh, i, facesIndex[i].x, facesIndex[i].y, facesIndex[i].z));

			mesh->init();

			simulator->mesh(index, mesh);
		}

		void createMeshes2(const sp_uint index)
		{
			const sp_uint vertexesLength = 24;
			SpArray<Vec3> vertexes(vertexesLength);
			vertexes.add({ -0.179303f, -1.000000f, -0.179303f });
			vertexes.add({ 0.179303f, -1.000000f, -0.179303f });
			vertexes.add({ 0.179303f, -1.000000f,  0.179303f });
			vertexes.add({ -0.179303f, -1.000000f,  0.179303f });
			vertexes.add({ -1.000000f, -0.179303f,  0.179303f });
			vertexes.add({ -1.000000f,  0.179303f,  0.179303f });
			vertexes.add({ -1.000000f,  0.179303f, -0.179303f });
			vertexes.add({ -1.000000f, -0.179303f, -0.179303f });
			vertexes.add({ 0.179303f, -0.179303f,  1.000000f });
			vertexes.add({ 0.179303f,  0.179303f,  1.000000f });
			vertexes.add({ -0.179303f,  0.179303f,  1.000000f });
			vertexes.add({ -0.179303f, -0.179303f,  1.000000f });
			vertexes.add({ 0.179303f,  1.000000f, -0.179303f });
			vertexes.add({ -0.179303f,  1.000000f, -0.179303f });
			vertexes.add({ -0.179303f,  1.000000f,  0.179303f });
			vertexes.add({ 0.179303f,  1.000000f,  0.179303f });
			vertexes.add({ 1.000000f, -0.179303f, -0.179303f });
			vertexes.add({ 1.000000f,  0.179303f, -0.179303f });
			vertexes.add({ 1.000000f,  0.179303f,  0.179303f });
			vertexes.add({ 1.000000f, -0.179303f,  0.179303f });
			vertexes.add({ 0.179303f,  0.179303f, -1.000000f });
			vertexes.add({ 0.179303f, -0.179303f, -1.000000f });
			vertexes.add({ -0.179303f,  0.179303f, -1.000000f });
			vertexes.add({ -0.179303f, -0.179303f, -1.000000f });

			const sp_uint facesLength = 44;
			SpPoint3<sp_uint>* facesIndexes = sp_mem_new_array(SpPoint3<sp_uint>, facesLength);
			facesIndexes[0] = SpPoint3<sp_uint>(1, 2, 3);
			facesIndexes[1] = SpPoint3<sp_uint>(3, 4, 1);
			facesIndexes[2] = SpPoint3<sp_uint>(5, 6, 7);
			facesIndexes[3] = SpPoint3<sp_uint>(7, 8, 5);
			facesIndexes[4] = SpPoint3<sp_uint>(9, 10, 11);
			facesIndexes[5] = SpPoint3<sp_uint>(11, 12, 9);
			facesIndexes[6] = SpPoint3<sp_uint>(13, 14, 15);
			facesIndexes[7] = SpPoint3<sp_uint>(15, 16, 13);
			facesIndexes[8] = SpPoint3<sp_uint>(17, 18, 19);
			facesIndexes[9] = SpPoint3<sp_uint>(19, 20, 17);
			facesIndexes[10] = SpPoint3<sp_uint>(21, 13, 18);
			facesIndexes[11] = SpPoint3<sp_uint>(2, 22, 17);
			facesIndexes[12] = SpPoint3<sp_uint>(19, 16, 10);
			facesIndexes[13] = SpPoint3<sp_uint>(20, 9, 3);
			facesIndexes[14] = SpPoint3<sp_uint>(23, 7, 14);
			facesIndexes[15] = SpPoint3<sp_uint>(8, 24, 1);
			facesIndexes[16] = SpPoint3<sp_uint>(6, 11, 15);
			facesIndexes[17] = SpPoint3<sp_uint>(4, 12, 5);
			facesIndexes[18] = SpPoint3<sp_uint>(1, 4, 5);
			facesIndexes[19] = SpPoint3<sp_uint>(5, 8, 1);
			facesIndexes[20] = SpPoint3<sp_uint>(2, 1, 24);
			facesIndexes[21] = SpPoint3<sp_uint>(24, 22, 2);
			facesIndexes[22] = SpPoint3<sp_uint>(18, 17, 22);
			facesIndexes[23] = SpPoint3<sp_uint>(22, 21, 18);
			facesIndexes[24] = SpPoint3<sp_uint>(12, 11, 6);
			facesIndexes[25] = SpPoint3<sp_uint>(6, 5, 12);
			facesIndexes[26] = SpPoint3<sp_uint>(10, 9, 20);
			facesIndexes[27] = SpPoint3<sp_uint>(20, 19, 10);
			facesIndexes[28] = SpPoint3<sp_uint>(23, 24, 8);
			facesIndexes[29] = SpPoint3<sp_uint>(8, 7, 23);
			facesIndexes[30] = SpPoint3<sp_uint>(16, 15, 11);
			facesIndexes[31] = SpPoint3<sp_uint>(11, 10, 16);
			facesIndexes[32] = SpPoint3<sp_uint>(13, 16, 19);
			facesIndexes[33] = SpPoint3<sp_uint>(19, 18, 13);
			facesIndexes[34] = SpPoint3<sp_uint>(4, 3, 9);
			facesIndexes[35] = SpPoint3<sp_uint>(9, 12, 4);
			facesIndexes[36] = SpPoint3<sp_uint>(15, 14, 7);
			facesIndexes[37] = SpPoint3<sp_uint>(7, 6, 15);
			facesIndexes[38] = SpPoint3<sp_uint>(14, 13, 21);
			facesIndexes[39] = SpPoint3<sp_uint>(21, 23, 14);
			facesIndexes[40] = SpPoint3<sp_uint>(3, 2, 17);
			facesIndexes[41] = SpPoint3<sp_uint>(17, 20, 3);
			facesIndexes[42] = SpPoint3<sp_uint>(24, 23, 21);
			facesIndexes[43] = SpPoint3<sp_uint>(21, 22, 24);

			SpMesh* mesh = sp_mem_new(SpMesh)();

			mesh->vertexesMesh = sp_mem_new(SpArray<SpVertexMesh*>)(vertexesLength);
			for (sp_uint i = 0; i < vertexesLength; i++)
				mesh->vertexesMesh->add(sp_mem_new(SpVertexMesh)(mesh, i, vertexes.get(i)));


			mesh->faces = sp_mem_new(SpArray<SpFaceMesh*>)(facesLength);
			for (sp_uint i = 0; i < facesLength; i++)
				mesh->faces->add(sp_mem_new(SpFaceMesh)(mesh, i, facesIndexes[i].x - 1, facesIndexes[i].y - 1, facesIndexes[i].z - 1));

			mesh->init();

			SpPhysicSimulator::instance()->mesh(index, mesh);
		}

		void resetObject(const sp_uint index)
		{
			SpPhysicSimulator* simulator = SpPhysicSimulator::instance();

			simulator->transforms(index)->reset();
		
			simulator->rigidBodies(index)->currentState.reset();
			simulator->rigidBodies(index)->previousState.reset();
			simulator->rigidBodies(index)->mass(8.0f);
		}

		void setupObject(const sp_uint index, const Vec3& position, const Quat& orientation, 
			const Vec3& velocity, const Vec3& angularVelocity)
		{
			SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
			SpRigidBody* properties = simulator->rigidBodies(index);

			properties->currentState.position(position);
			properties->previousState.position(position);
			simulator->transforms(index)->position = position;

			properties->currentState.orientation(orientation);
			properties->previousState.orientation(orientation);
			simulator->transforms(index)->orientation = orientation;

			properties->currentState.velocity(velocity);
			properties->previousState.velocity(velocity);

			properties->currentState.angularVelocity(angularVelocity);
			properties->previousState.angularVelocity(angularVelocity);
		}

		SpCollisionDetails newCollisionDetails()
		{
			SpCollisionDetails details;
			details.objIndex1 = ZERO_UINT;
			details.objIndex2 = ONE_UINT;
			details.timeStep = 30.0f;
			details.type = SpCollisionType::None;
			details.vertexIndexObj1 = SP_UINT_MAX;
			details.vertexIndexObj2 = SP_UINT_MAX;
			details.ignoreCollision = false;

			SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
			details.cacheObj1 = ALLOC_NEW(SpMeshCache)(simulator->mesh(0u)->vertexesMesh->length());
			details.cacheObj2 = ALLOC_NEW(SpMeshCache)(simulator->mesh(1u)->vertexesMesh->length());

			return details;
		}

	public:
		SP_TEST_METHOD_DEF(areMovingAway);
		SP_TEST_METHOD_DEF(collisionStatus);
		SP_TEST_METHOD_DEF(filterCollision);
		SP_TEST_METHOD_DEF(collisionDetails_FaceFace_1);
		SP_TEST_METHOD_DEF(collisionDetails_FaceFace_2);
		//SP_TEST_METHOD_DEF(collisionDetails_inside);
		SP_TEST_METHOD_DEF(collisionDetails_FaceFace_4);
		SP_TEST_METHOD_DEF(collisionDetails_FaceFace_5);
		SP_TEST_METHOD_DEF(collisionDetails_FaceFace_6);
		SP_TEST_METHOD_DEF(collisionDetails_EdgeFace_1);
		SP_TEST_METHOD_DEF(collisionDetails_EdgeEdge_1);
		SP_TEST_METHOD_DEF(collisionDetails_VertexFace);
		SP_TEST_METHOD_DEF(collisionDetails_multi); 
		SP_TEST_METHOD_DEF(collisionDetails_multi2); 
		SP_TEST_METHOD_DEF(collisionDetails_1);
		SP_TEST_METHOD_DEF(collisionDetails_2);
		SP_TEST_METHOD_DEF(collisionDetails_3);
		SP_TEST_METHOD_DEF(hasCollision);
	};

	SP_TEST_METHOD(CLASS_NAME, hasCollision)
	{
		TestPhysic::lock();

		createMeshePlane();
		createCubeMeshes(1u);

		resetObject(0u);
		resetObject(1u);

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpMesh* mesh1 = simulator->mesh(0u);
		SpMesh* mesh2 = simulator->mesh(1u);
		
		SpCollisionDetectorCache cache;
		const SpCollisionDetector collisionDetector;

		SpCollisionDetails details = newCollisionDetails();
		details.cacheObj1->update(mesh1, simulator->transforms(0u));
		details.cacheObj2->update(mesh2, simulator->transforms(1u));
		simulator->rigidBodies(1u)->currentState.position(Vec3(0.0f, 1.0f, 0.0f));
		simulator->transforms(1u)->position = Vec3(0.0f, 1.0f, 0.0f);
		sp_bool result = collisionDetector.hasCollision(0u, 1u, &details, &cache);
		Assert::IsTrue(result, L"Wrong value.", LINE_INFO());

		details = newCollisionDetails();
		details.cacheObj1->update(mesh1, simulator->transforms(0u));
		details.cacheObj2->update(mesh2, simulator->transforms(1u));
		simulator->rigidBodies(1u)->currentState.position(Vec3(0.0f, 0.5f, 0.0f));
		simulator->transforms(1u)->position = Vec3(0.0f, 0.5f, 0.0f);
		result = collisionDetector.hasCollision(0u, 1u, &details, &cache);
		Assert::IsTrue(result, L"Wrong value.", LINE_INFO());

		details = newCollisionDetails();
		details.cacheObj1->update(mesh1, simulator->transforms(0u));
		details.cacheObj2->update(mesh2, simulator->transforms(1u));
		simulator->rigidBodies(1u)->currentState.position(Vec3(0.0f, 1.5f, 0.0f));
		simulator->transforms(1u)->position = Vec3(0.0f, 1.5f, 0.0f);
		result = collisionDetector.hasCollision(0u, 1u, &details, &cache);
		Assert::IsTrue(result, L"Wrong value.", LINE_INFO());

		details = newCollisionDetails();
		details.cacheObj1->update(mesh1, simulator->transforms(0u));
		details.cacheObj2->update(mesh2, simulator->transforms(1u));
		simulator->rigidBodies(1u)->currentState.position(Vec3(0.0f, 2.0f, 0.0f));
		simulator->transforms(1u)->position = Vec3(0.0f, 2.0f, 0.0f);
		result = collisionDetector.hasCollision(0u, 1u, &details, &cache);
		Assert::IsTrue(result, L"Wrong value.", LINE_INFO());

		details = newCollisionDetails();
		details.cacheObj1->update(mesh1, simulator->transforms(0u));
		details.cacheObj2->update(mesh2, simulator->transforms(1u));
		simulator->rigidBodies(1u)->currentState.position(Vec3(0.0f, 2.5f, 0.0f));
		simulator->transforms(1u)->position = Vec3(0.0f, 2.5f, 0.0f);
		result = collisionDetector.hasCollision(0u, 1u, &details, &cache);
		Assert::IsFalse(result, L"Wrong value.", LINE_INFO());
		
		details = newCollisionDetails();
		details.cacheObj1->update(mesh1, simulator->transforms(0u));
		details.cacheObj2->update(mesh2, simulator->transforms(1u));
		simulator->rigidBodies(1u)->currentState.position(Vec3(3000.0f, 0.0f, 0.0f));
		simulator->transforms(1u)->position = Vec3(3000.0f, 0.0f, 0.0f);
		result = collisionDetector.hasCollision(0u, 1u, &details, &cache);
		Assert::IsFalse(result, L"Wrong value.", LINE_INFO());

		TestPhysic::unlock();
	}

	SP_TEST_METHOD(CLASS_NAME, collisionDetails_multi)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;
		SpCollisionDetails details;

		createMeshePlane();
		createMeshes2(1u);

		resetObject(0u);
		resetObject(1u);

		SpRigidBody* propertiesObj1 = simulator->rigidBodies(0u);
		SpRigidBody* propertiesObj2 = simulator->rigidBodies(1u);

		details = newCollisionDetails();

		propertiesObj1->mass(ZERO_FLOAT); // static object
		simulator->transforms(0u)->scaleVector = Vec3(100.0f, 1.0f, 100.0f);

		const Vec3 defaultPositionObj2 = Vec3(0.0f, 4.0f, 0.0f);
		const Vec3 defaultVelocityObj2 = Vec3(0.0f, -20.0f, 0.0f);
		const Quat defaultOrientationObj2 = Quat(Vec3(0.0f, 0.0f, 0.0f));
		const Quat defaultAngVelocityObj2 = Vec3(0.0f, 0.0f, 0.0f);

		for (sp_float posX = -50.0f; posX < 50.0f; posX++)
		{
			for (sp_float posZ = -50.0f; posZ < 50.0f; posZ++)
			{
				for (sp_float posY = 5.0f; posY > 2.0f; posY--)
				{
					
					// test collision with all X axis orientations 
					for (sp_float angle = 0; angle < 360.0f; angle++)
					{
						details = newCollisionDetails();

						setupObject(1u,
							Vec3(posX, posY, posZ),
							Quat::createRotationAxisX(degreesToRadians(angle)),
							defaultVelocityObj2,
							defaultAngVelocityObj2
						);

						collisionDetector.collisionDetails(&details);

						if (details.type == SpCollisionType::None)
							Assert::IsTrue(false, L"ERRO", LINE_INFO());
					}

					// test collision with all Y axis orientations 
					for (sp_float angle = 0; angle < 360.0f; angle++)
					{
						details = newCollisionDetails();

						setupObject(1u,
							Vec3(posX, posY, posZ),
							Quat::createRotationAxisY(degreesToRadians(angle)),
							defaultVelocityObj2,
							defaultAngVelocityObj2
						);

						collisionDetector.collisionDetails(&details);

						if (details.type == SpCollisionType::None)
							Assert::IsTrue(false, L"ERRO", LINE_INFO());
					}

					// test collision with all Z axis orientations 
					for (sp_float angle = 0; angle < 360.0f; angle++)
					{
						details = newCollisionDetails();

						setupObject(1u,
							Vec3(posX, posY, posZ),
							Quat::createRotationAxisZ(degreesToRadians(angle)),
							defaultVelocityObj2,
							defaultAngVelocityObj2
						);

						collisionDetector.collisionDetails(&details);

						if (details.type == SpCollisionType::None)
							Assert::IsTrue(false, L"ERRO", LINE_INFO());
					}

					// test collision with X and Z axis orientations 
					for (sp_float angle = 0; angle < 360.0f; angle++)
					{
						details = newCollisionDetails();

						setupObject(1u,
							Vec3(posX, posY, posZ),
							Quat::createRotate(degreesToRadians(angle), Vec3(1.0f, 0.0f, 1.0f)),
							defaultVelocityObj2,
							defaultAngVelocityObj2
						);

						collisionDetector.collisionDetails(&details);

						if (details.type == SpCollisionType::None)
							Assert::IsTrue(false, L"ERRO", LINE_INFO());
					}

				}
			}
		}

		/*
		Assert::IsFalse(details.ignoreCollision, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.type == SpCollisionType::FaceFace, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.timeOfCollision >= ZERO_FLOAT && details.timeOfCollision <= details.timeStep, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.contactPointsLength == 4u, L"Wrong value.", LINE_INFO());
		Asserts::isCloseEnough(details.collisionNormalObj1, Vec3(1.0f, 0.0f, 0.0f), ERROR_MARGIN_PHYSIC);
		Asserts::isCloseEnough(details.collisionNormalObj2, Vec3(-1.0f, 0.0f, 0.0f), ERROR_MARGIN_PHYSIC);

		Assert::IsTrue(isCloseEnough(details.contactPoints[0], Vec3(1.0f, -1.0f, -1.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.contactPoints[1], Vec3(1.0f, -1.0f, 1.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.contactPoints[2], Vec3(1.0f, 1.0f, -1.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.contactPoints[3], Vec3(1.0f, 1.0f, 1.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());

		Assert::IsTrue(isCloseEnough(details.centerContactPoint, Vec3(1.0f, 0.0f, 0.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());
		*/

		TestPhysic::unlock();
	}

	SP_TEST_METHOD(CLASS_NAME, collisionDetails_multi2)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;
		SpCollisionDetails details;

		createMeshes2(0u);
		createMeshes2(1u);

		resetObject(0u);
		resetObject(1u);

		SpRigidBody* propertiesObj1 = simulator->rigidBodies(0u);
		SpRigidBody* propertiesObj2 = simulator->rigidBodies(1u);

		details = newCollisionDetails();

		propertiesObj1->mass(ZERO_FLOAT); // static object
		
		const Vec3 defaultVelocityObj2 = Vec3(0.0f, -25.0f, 0.0f);
		const Quat defaultAngVelocityObj2 = Vec3(0.0f, 0.0f, 0.0f);

		for (sp_float posX = -0.9f; posX < 0.9f; posX += 0.2f)
		{
			for (sp_float posZ = -0.9f; posZ < 0.9f; posZ += 0.2f)
			{
				for (sp_float posY = 4.0f; posY > 2.0f; posY--)
				{

					// test collision with all X axis orientations 
					for (sp_float angle = 0; angle < 360.0f; angle++)
					{
						details = newCollisionDetails();

						setupObject(1u,
							Vec3(posX, posY, posZ),
							Quat::createRotationAxisX(degreesToRadians(angle)),
							defaultVelocityObj2,
							defaultAngVelocityObj2
						);

						collisionDetector.collisionDetails(&details);

						if (details.type == SpCollisionType::None)
							Assert::IsTrue(false, L"ERRO", LINE_INFO());
					}

					// test collision with all Y axis orientations 
					for (sp_float angle = 0; angle < 360.0f; angle++)
					{
						details = newCollisionDetails();

						setupObject(1u,
							Vec3(posX, posY, posZ),
							Quat::createRotationAxisY(degreesToRadians(angle)),
							defaultVelocityObj2,
							defaultAngVelocityObj2
						);

						collisionDetector.collisionDetails(&details);

						if (details.type == SpCollisionType::None)
							Assert::IsTrue(false, L"ERRO", LINE_INFO());
					}

					// test collision with all Z axis orientations 
					for (sp_float angle = 0; angle < 360.0f; angle++)
					{
						details = newCollisionDetails();

						setupObject(1u,
							Vec3(posX, posY, posZ),
							Quat::createRotationAxisZ(degreesToRadians(angle)),
							defaultVelocityObj2,
							defaultAngVelocityObj2
						);

						collisionDetector.collisionDetails(&details);

						if (details.type == SpCollisionType::None)
							Assert::IsTrue(false, L"ERRO", LINE_INFO());
					}

					// test collision with X and Z axis orientations 
					for (sp_float angle = 0; angle < 360.0f; angle++)
					{
						details = newCollisionDetails();

						setupObject(1u,
							Vec3(posX, posY, posZ),
							Quat::createRotate(degreesToRadians(angle), Vec3(1.0f, 0.0f, 1.0f)),
							defaultVelocityObj2,
							defaultAngVelocityObj2
						);

						collisionDetector.collisionDetails(&details);

						if (details.type == SpCollisionType::None)
							Assert::IsTrue(false, L"ERRO", LINE_INFO());
					}

				}
			}
		}

		TestPhysic::unlock();
	}

	SP_TEST_METHOD(CLASS_NAME, collisionDetails_1)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;
		SpCollisionDetails details;

		createMeshePlane();
		createCubeMeshes(1u);

		resetObject(0u);
		resetObject(1u);

		SpRigidBody* propertiesObj1 = simulator->rigidBodies(0u);
		SpRigidBody* propertiesObj2 = simulator->rigidBodies(1u);

		details = newCollisionDetails();

		propertiesObj1->mass(ZERO_FLOAT); // static object
		simulator->transforms(0u)->scaleVector = Vec3(100.0f, 1.0f, 100.0f);

		const Vec3 defaultPositionObj2 = Vec3(0.0f, 4.0f, 0.0f);
		const Vec3 defaultVelocityObj2 = Vec3(0.0f, -20.0f, 0.0f);
		const Quat defaultOrientationObj2 = Quat(Vec3(0.0f, 0.0f, 0.0f));
		const Quat defaultAngVelocityObj2 = Vec3(0.0f, 0.0f, 0.0f);

		sp_float posX = -50.0f;
		sp_float posY = 5.0f;
		sp_float posZ = -44.0f;
		sp_float angle = 88.0f;

		details = newCollisionDetails();

		setupObject(1u,
			Vec3(posX, posY, posZ),
			Quat::createRotationAxisX(degreesToRadians(angle)),
			defaultVelocityObj2,
			defaultAngVelocityObj2
		);

		collisionDetector.collisionDetails(&details);

		Assert::IsTrue(details.type == SpCollisionType::EdgeFace, L"ERRO", LINE_INFO());

		TestPhysic::unlock();
	}

	SP_TEST_METHOD(CLASS_NAME, collisionDetails_2)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;
		SpCollisionDetails details;

		createMeshePlane();
		createCubeMeshes(1u);

		resetObject(0u);
		resetObject(1u);

		SpRigidBody* propertiesObj1 = simulator->rigidBodies(0u);
		SpRigidBody* propertiesObj2 = simulator->rigidBodies(1u);

		details = newCollisionDetails();

		propertiesObj1->mass(ZERO_FLOAT); // static object
		simulator->transforms(0u)->scaleVector = Vec3(100.0f, 1.0f, 100.0f);

		const Vec3 defaultVelocityObj2 = Vec3(0.0f, -25.0f, 0.0f);
		const Quat defaultAngVelocityObj2 = Vec3(0.0f, 0.0f, 0.0f);

		sp_float posX = 0.9f;
		sp_float posY = 4.0f;
		sp_float posZ = 0.9f;
		sp_float angle = 66.0f;

		details = newCollisionDetails();

		setupObject(1u,
			Vec3(posX, posY, posZ),
			Quat::createRotationAxisX(degreesToRadians(angle)),
			defaultVelocityObj2,
			defaultAngVelocityObj2
		);

		collisionDetector.collisionDetails(&details);

		Assert::IsTrue(details.type == SpCollisionType::EdgeFace, L"ERRO", LINE_INFO());

		TestPhysic::unlock();
	}

	SP_TEST_METHOD(CLASS_NAME, collisionDetails_3)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;
		SpCollisionDetails details;

		createMeshePlane();
		createCubeMeshes(1u);

		resetObject(0u);
		resetObject(1u);

		SpRigidBody* propertiesObj1 = simulator->rigidBodies(0u);
		SpRigidBody* propertiesObj2 = simulator->rigidBodies(1u);

		details = newCollisionDetails();

		propertiesObj1->mass(ZERO_FLOAT); // static object
		simulator->transforms(0u)->scaleVector = Vec3(100.0f, 1.0f, 100.0f);

		const Vec3 defaultVelocityObj2 = Vec3(0.0f, -25.0f, 0.0f);
		const Quat defaultAngVelocityObj2 = Vec3(0.0f, 0.0f, 0.0f);

		sp_float posX = -0.9f;
		sp_float posY = 4.0f;
		sp_float posZ = -0.1f;
		sp_float angle = 71.0f;

		details = newCollisionDetails();

		setupObject(1u,
			Vec3(posX, posY, posZ),
			Quat::createRotate(degreesToRadians(angle), Vec3(1.0f, 0.0f, 1.0f)),
			defaultVelocityObj2,
			defaultAngVelocityObj2
		);

		collisionDetector.collisionDetails(&details);

		Assert::IsTrue(details.type == SpCollisionType::EdgeFace, L"ERRO", LINE_INFO());

		TestPhysic::unlock();
	}

	SP_TEST_METHOD(CLASS_NAME, areMovingAway)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;

		resetObject(0u);
		resetObject(1u);

		simulator->rigidBodies(0u)->currentState.position(Vec3(1.0f, 0.0f, 0.0f));
		simulator->rigidBodies(1u)->currentState.position(Vec3(0.0f, 0.0f, 0.0f));

		simulator->rigidBodies(0u)->currentState.velocity(Vec3(10.0f, 0.0f, 0.0f));
		simulator->rigidBodies(1u)->currentState.velocity(Vec3(-10.0f, 0.0f, 0.0f));

		sp_bool result = collisionDetector.areMovingAway(0u, 1u);

		Assert::IsTrue(result, L"Wrong value.", LINE_INFO());

		simulator->rigidBodies(0u)->currentState.velocity(Vec3(10.0f, 0.0f, 0.0f));
		simulator->rigidBodies(1u)->currentState.velocity(Vec3(1.0f, 0.0f, 0.0f));

		result = collisionDetector.areMovingAway(0u, 1u);
		
		Assert::IsFalse(result, L"Wrong value.", LINE_INFO());

		TestPhysic::unlock();
	}

	SP_TEST_METHOD(CLASS_NAME, collisionStatus)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;
		SpCollisionDetails details;

		createMeshePlane();
		createCubeMeshes(1u);

		resetObject(0u);
		resetObject(1u);

		Vec3 contactPoint;
		SpCollisionDetectorCache cache;

		details = newCollisionDetails();
		simulator->translate(0u, Vec3(1.0f, 0.0f, 0.0f));
		CollisionStatus result = collisionDetector.collisionStatus(&contactPoint, &cache, &details);
		Assert::IsTrue(result == CollisionStatus::INSIDE, L"Wrong value.", LINE_INFO());

		resetObject(0u);
		details = newCollisionDetails();
		simulator->translate(0u, Vec3(2.5f, 0.0f, 0.0f));
		cache.clear();
		result = collisionDetector.collisionStatus(&contactPoint, &cache, &details);
		Assert::IsTrue(result == CollisionStatus::OUTSIDE, L"Wrong value.", LINE_INFO());

		resetObject(0u);
		details = newCollisionDetails();
		simulator->translate(0u, Vec3(-1.5f, 2.0f, 0.0f));
		cache.clear();
		result = collisionDetector.collisionStatus(&contactPoint, &cache, &details);
		Assert::IsTrue(result == CollisionStatus::INSIDE, L"Wrong value.", LINE_INFO());

		resetObject(0u);
		details = newCollisionDetails();
		simulator->translate(0u, Vec3(0.0f, 2.2f, 0.0f));
		cache.clear();
		result = collisionDetector.collisionStatus(&contactPoint, &cache, &details);
		Assert::IsTrue(result == CollisionStatus::OUTSIDE, L"Wrong value.", LINE_INFO());

		resetObject(0u);
		details = newCollisionDetails();
		cache.clear(); 
		simulator->translate(0u, Vec3(0.0f, 0.0f, 2.0f));
		cache.clear();
		result = collisionDetector.collisionStatus(&contactPoint, &cache, &details);
		Assert::IsTrue(result == CollisionStatus::INSIDE, L"Wrong value.", LINE_INFO());

		resetObject(0u);
		details = newCollisionDetails();
		cache.clear();
		simulator->translate(0u, Vec3(0.0f, 0.0f, 0.2f));
		result = collisionDetector.collisionStatus(&contactPoint, &cache, &details);
		Assert::IsTrue(result == CollisionStatus::INSIDE, L"Wrong value.", LINE_INFO());
		 
		/* check the object is inside the other
		details = newCollisionDetails();
		resetObject(0u);
		simulator->scale(0u, Vec3(0.2f, 0.2f, 0.2f));
		result = collisionDetector.collisionStatus(&contactPoint, &searchOnObj1, &edgeIndex, &faceIndex, &details);
		Assert::IsTrue(result == CollisionStatus::INSIDE, L"Wrong value.", LINE_INFO());
		*/

		TestPhysic::unlock();
	}

	SP_TEST_METHOD(CLASS_NAME, filterCollision)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;

		createMeshePlane();
		createCubeMeshes(1u);

		SpRigidBody* propertiesObj1 = simulator->rigidBodies(0u);
		SpRigidBody* propertiesObj2 = simulator->rigidBodies(1u);

		SpCollisionDetails details = newCollisionDetails();
		resetObject(0u);
		resetObject(1u);
		propertiesObj1->mass(ZERO_FLOAT);
		propertiesObj2->mass(ZERO_FLOAT);
		collisionDetector.filterCollision(&details);
		Assert::IsTrue(details.ignoreCollision, L"Wrong value.", LINE_INFO());

		details = newCollisionDetails();
		resetObject(0u);
		resetObject(1u);
		propertiesObj1->mass(8.0f);
		propertiesObj1->currentState.position(Vec3(10.0f, 0.0f, 0.0f));
		propertiesObj1->previousState.position(Vec3(10.0f, 0.0f, 0.0f));
		propertiesObj1->currentState.acceleration(Vec3(0.0f, -1.225f, 0.0f));
		propertiesObj2->mass(ZERO_FLOAT);
		propertiesObj2->currentState.acceleration(Vec3(0.0f, -1.225f, 0.0f));
		collisionDetector.filterCollision(&details);
		Assert::IsTrue(details.ignoreCollision, L"Wrong value.", LINE_INFO());

		details = newCollisionDetails();
		resetObject(0u);
		resetObject(1u);
		propertiesObj1->mass(ZERO_FLOAT);
		propertiesObj1->currentState.position(Vec3Zeros);
		propertiesObj1->previousState.position(Vec3Zeros);
		propertiesObj2->mass(8.0f);
		propertiesObj2->currentState.position(Vec3(10.0f, 0.0f, 0.0f));
		propertiesObj2->previousState.position(Vec3(10.0f, 0.0f, 0.0f));
		collisionDetector.filterCollision(&details);
		Assert::IsTrue(details.ignoreCollision, L"Wrong value.", LINE_INFO());

		details = newCollisionDetails();
		resetObject(0u);
		resetObject(1u);
		propertiesObj1->mass(8.0f);
		propertiesObj2->mass(8.0f);
		collisionDetector.filterCollision(&details);
		Assert::IsTrue(details.ignoreCollision, L"Wrong value.", LINE_INFO());

		details = newCollisionDetails();
		propertiesObj1->currentState.position(Vec3(2.0f, 0.0f, 0.0f));
		propertiesObj1->currentState.velocity(Vec3(-1.0f, 0.0f, 0.0f));
		propertiesObj2->currentState.position(Vec3(1.0f, 0.0f, 0.0f));
		propertiesObj2->currentState.velocity(Vec3(1.0f, 0.0f, 0.0f));
		collisionDetector.filterCollision(&details);
		Assert::IsFalse(details.ignoreCollision, L"Wrong value.", LINE_INFO());

		TestPhysic::unlock();
	}

	SP_TEST_METHOD(CLASS_NAME, collisionDetails_FaceFace_1)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;
		SpCollisionDetails details;

		createMeshePlane();
		createCubeMeshes(1u);

		resetObject(0u);
		resetObject(1u);

		SpRigidBody* propertiesObj1 = simulator->rigidBodies(0u);
		SpRigidBody* propertiesObj2 = simulator->rigidBodies(1u);

		details = newCollisionDetails();
		
		propertiesObj1->mass(ZERO_FLOAT); // static object
		
		propertiesObj2->previousState.position(Vec3(4.0f, 0.0f, 0.0f));
		simulator->translate(1u, Vec3(1.5f, 0.0f, 0.0f));
		propertiesObj2->currentState.velocity(Vec3(-20.0f, 0.0f, 0.0f));
		propertiesObj2->previousState.velocity(Vec3(-20.0f, 0.0f, 0.0f));

		PerformanceCounter counter;

		collisionDetector.collisionDetails(&details);
		
		// 4273200
		counter.logElapsedTime("SpCollisionDetector::colisionDetails: ");

		Assert::IsFalse(details.ignoreCollision, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.type == SpCollisionType::FaceFace, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.timeOfCollision >= ZERO_FLOAT && details.timeOfCollision <= details.timeStep, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.contactPointsLength == 4u, L"Wrong value.", LINE_INFO());
		Asserts::isCloseEnough(details.collisionNormal, Vec3(1.0f, 0.0f, 0.0f), ERROR_MARGIN_PHYSIC);

		Assert::IsTrue(isCloseEnough(details.contactPoints[0], Vec3(1.0f, -1.0f, -1.0f ), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.contactPoints[1], Vec3(1.0f, -1.0f, 1.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.contactPoints[2], Vec3(1.0f, 1.0f, -1.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.contactPoints[3], Vec3(1.0f, 1.0f, 1.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());

		Assert::IsTrue(isCloseEnough(details.centerContactPoint, Vec3(1.0f, 0.0f, 0.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());

		TestPhysic::unlock();
	}

	SP_TEST_METHOD(CLASS_NAME, collisionDetails_FaceFace_2)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;
		SpCollisionDetails details;

		createMeshePlane();
		createCubeMeshes(1u);
		resetObject(0u);
		resetObject(1u);

		SpRigidBody* propertiesObj1 = simulator->rigidBodies(0u);
		SpRigidBody* propertiesObj2 = simulator->rigidBodies(1u);

		details = newCollisionDetails();
		
		propertiesObj1->mass(ZERO_FLOAT);

		simulator->transforms(1u)->position = Vec3(4.0f, 0.5f, -0.5f);
		propertiesObj2->previousState.position(Vec3(4.0f, 0.5f, -0.5f));
		propertiesObj2->currentState.position(Vec3(4.0f, 0.5f, -0.5f));
		propertiesObj2->currentState.velocity(Vec3(-20.0f, 0.0f, 0.0f));
		propertiesObj2->previousState.velocity(Vec3(-20.0f, 0.0f, 0.0f));
		
		collisionDetector.collisionDetails(&details);
		
		Assert::IsFalse(details.ignoreCollision, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.type == SpCollisionType::FaceFace, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.timeOfCollision >= ZERO_FLOAT && details.timeOfCollision <= details.timeStep, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.contactPointsLength == 4u, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.collisionNormal, Vec3(1.0f, 0.0f, 0.0f), ERROR_MARGIN_PHYSIC), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.contactPoints[0], Vec3(1.0f, -0.5f, 0.5f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.contactPoints[1], Vec3(1.0f, 1.0f, -1.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.contactPoints[2], Vec3(1.0f, -0.5f, -1.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.contactPoints[3], Vec3(1.0f, 1.0f, 0.5f ), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());

		Assert::IsTrue(isCloseEnough(details.centerContactPoint, Vec3(1.0f, 0.25f, -0.25f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());

		TestPhysic::unlock();
	}

	/* Test INSIDE Face-Face 
	SP_TEST_METHOD(CLASS_NAME, collisionDetails_inside)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;
		SpCollisionDetails details;

		createMeshePlane();
		createMeshes();
		resetObject(0u);
		resetObject(1u);

		SpPhysicProperties* propertiesObj1 = simulator->physicProperties(0u);
		SpPhysicProperties* propertiesObj2 = simulator->physicProperties(1u);

		details = newCollisionDetails();

		propertiesObj1->mass(ZERO_FLOAT);

		simulator->transforms(1u)->scaleVector = Vec3(0.5f, 0.5f, 0.5f);
		propertiesObj2->previousState.position(Vec3(4.0f, 0.0f, 0.0f));
		propertiesObj2->currentState.position(Vec3(4.0f, 0.0f, 0.0f));
		propertiesObj2->currentState.velocity(Vec3(-20.0f, 0.0f, 0.0f));
		propertiesObj2->previousState.velocity(Vec3(-20.0f, 0.0f, 0.0f));

		collisionDetector.collisionDetails(&details);

		Assert::IsFalse(details.ignoreCollision, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.type == SpCollisionType::FaceFace, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.timeOfCollision >= ZERO_FLOAT && details.timeOfCollision <= details.timeStep, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.contactPointsLengthObj1 == 4u, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.contactPointsLengthObj2 == 4u, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.collisionNormalObj1.isCloseEnough(Vec3(1.0f, 0.0f, 0.0f)), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.collisionNormalObj2.isCloseEnough(Vec3(-1.0f, 0.0f, 0.0f)), L"Wrong value.", LINE_INFO());

		Assert::IsTrue(details.contactPointsObj1[0].isCloseEnough(Vec3(1.0f, 0.5f, 0.5f), 0.009f), L"Wrong value", LINE_INFO());
		Assert::IsTrue(details.contactPointsObj1[1].isCloseEnough(Vec3(1.0f, -0.5f, -0.5f), 0.009f), L"Wrong value", LINE_INFO());
		Assert::IsTrue(details.contactPointsObj1[2].isCloseEnough(Vec3(1.0f, -0.5f, 0.5f), 0.009f), L"Wrong value", LINE_INFO());
		Assert::IsTrue(details.contactPointsObj1[3].isCloseEnough(Vec3(1.0f, 0.5f, -0.5f), 0.009f), L"Wrong value", LINE_INFO());

		Assert::IsTrue(details.contactPointsObj2[0].isCloseEnough(Vec3(1.0f, 0.5f, 0.5f), 0.009f), L"Wrong value", LINE_INFO());
		Assert::IsTrue(details.contactPointsObj2[1].isCloseEnough(Vec3(1.0f, -0.5f, -0.5f), 0.009f), L"Wrong value", LINE_INFO());
		Assert::IsTrue(details.contactPointsObj2[2].isCloseEnough(Vec3(1.0f, -0.5f, 0.5f), 0.009f), L"Wrong value", LINE_INFO());
		Assert::IsTrue(details.contactPointsObj2[3].isCloseEnough(Vec3(1.0f, 0.5f, -0.5f), 0.009f), L"Wrong value", LINE_INFO());

		Assert::IsTrue(details.centerContactPoint.isCloseEnough(Vec3(1.0f, 0.0f, 0.0f), 0.009f), L"Wrong value", LINE_INFO());

		propertiesObj2->previousState.velocity(Vec3(ZERO_FLOAT));
		propertiesObj2->currentState.velocity(Vec3(ZERO_FLOAT));
		simulator->position(1u, Vec3(ZERO_FLOAT));

		TestPhysic::unlock();
	}
	*/

	SP_TEST_METHOD(CLASS_NAME, collisionDetails_FaceFace_4)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;
		SpCollisionDetails details;

		createMeshePlane();
		createCubeMeshes(1u);

		resetObject(0u);
		resetObject(1u);

		SpRigidBody* propertiesObj1 = simulator->rigidBodies(0u);
		SpRigidBody* propertiesObj2 = simulator->rigidBodies(1u);

		details = newCollisionDetails();

		propertiesObj1->mass(ZERO_FLOAT); // static object
		simulator->transforms(0u)->scaleVector = Vec3(100.0f, 1.0f, 100.0f);

		simulator->transforms(1u)->position = Vec3(0.0f, 4.0f, 0.0f);
		propertiesObj1->currentState.position(Vec3(0.0f, 4.0f, 0.0f));
		propertiesObj2->previousState.position(Vec3(0.0f, 4.0f, 0.0f));
		propertiesObj2->currentState.velocity(Vec3(0.0f, -20.0f, 0.0f));
		propertiesObj2->previousState.velocity(Vec3(0.0f, -20.0f, 0.0f));

		collisionDetector.collisionDetails(&details);

		Assert::IsFalse(details.ignoreCollision, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.type == SpCollisionType::FaceFace, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.timeOfCollision >= ZERO_FLOAT && details.timeOfCollision <= details.timeStep, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.contactPointsLength == 4u, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.collisionNormal, Vec3(0.0f, 1.0f, 0.0f), ERROR_MARGIN_PHYSIC), L"Wrong value.", LINE_INFO());
		
		Assert::IsTrue(isCloseEnough(details.contactPoints[0], Vec3(1.0f, 1.0f, 1.0f ), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.contactPoints[1], Vec3(-1.0f, 1.0f, 1.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.contactPoints[2], Vec3(-1.0f, 1.0f, -1.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.contactPoints[3], Vec3(1.0f, 1.0f, -1.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());

		Assert::IsTrue(isCloseEnough(details.centerContactPoint, Vec3(0.0f, 1.0f, 0.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());

		TestPhysic::unlock();
	}
	
	SP_TEST_METHOD(CLASS_NAME, collisionDetails_FaceFace_5)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;
		SpCollisionDetails details;

		// build mesh 1
		const sp_uint vertexesLength = 3u;
		Vec3 vertexes[vertexesLength] = { 
			Vec3(-1.0f, 0.0f, 0.0f), 
			Vec3(1.0f, 0.0f, 0.0f), 
			Vec3(0.0f, 1.0f, 0.0f) 
		};
		SpPoint3<sp_uint> facesIndex(0, 1, 2);

		SpMesh* mesh = sp_mem_new(SpMesh)();
		mesh->vertexesMesh = sp_mem_new(SpArray<SpVertexMesh*>)(vertexesLength);
		mesh->faces = sp_mem_new(SpArray<SpFaceMesh*>)(1u);

		for (sp_uint i = 0; i < vertexesLength; i++)
			mesh->vertexesMesh->add(sp_mem_new(SpVertexMesh)(mesh, i, vertexes[i]));

		mesh->faces->add(sp_mem_new(SpFaceMesh)(mesh, 0, facesIndex.x, facesIndex.y, facesIndex.z));
		mesh->init();
		simulator->mesh(0u, mesh);
		Vec3 positionObj1;
		midpoint(vertexes[0], vertexes[1], vertexes[2], &positionObj1);

		// build mesh 2
		Vec3 vertexes2[vertexesLength] = { 
			Vec3(-1.0f, 0.5f, 0.0f), 
			Vec3(1.0f, 0.5f, 0.0f), 
			Vec3(0.0f, -0.5f, 0.0f) 
		};
		mesh = sp_mem_new(SpMesh)();
		mesh->vertexesMesh = sp_mem_new(SpArray<SpVertexMesh*>)(vertexesLength);
		mesh->faces = sp_mem_new(SpArray<SpFaceMesh*>)(1u);

		for (sp_uint i = 0; i < vertexesLength; i++)
			mesh->vertexesMesh->add(sp_mem_new(SpVertexMesh)(mesh, i, vertexes2[i]));

		mesh->faces->add(sp_mem_new(SpFaceMesh)(mesh, 0, facesIndex.x, facesIndex.y, facesIndex.z));
		mesh->init();
		simulator->mesh(1u, mesh);
		simulator->collisionFeatures(1u, 1u);
		Vec3 positionObj2;
		midpoint(vertexes2[0], vertexes2[1], vertexes2[2], &positionObj2);

		resetObject(0u);
		resetObject(1u);

		details = newCollisionDetails();

		// ilustrated example
		//         /\
		//        /  \
		//     __/____\__
		//     \/      \/
		//     /\      /\
		//    /__\____/__\
		//        \  /
		//         \/

		collisionDetector.collisionDetails(&details);

		Assert::IsFalse(details.ignoreCollision, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.type == SpCollisionType::FaceFace, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.timeOfCollision >= ZERO_FLOAT && details.timeOfCollision <= details.timeStep, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.contactPointsLength == 6u, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.collisionNormal, Vec3(0.0f, 0.0f, 1.0f), ERROR_MARGIN_PHYSIC), L"Wrong value.", LINE_INFO());
		
		Assert::IsTrue(isCloseEnough(details.contactPoints[0], Vec3(-0.5f, 0.0f, 0.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.contactPoints[1], Vec3(0.5f, 0.0f, 0.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.contactPoints[2], Vec3(-0.5f, 0.5f, 0.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.contactPoints[3], Vec3(-0.75f, 0.25f, 0.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.contactPoints[4], Vec3(0.5f, 0.5f, 0.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.contactPoints[5], Vec3(0.75f, 0.25f, 0.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());

		Assert::IsTrue(isCloseEnough(details.centerContactPoint, Vec3(0.0f, 0.25f, 0.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());

		TestPhysic::unlock();
	}

	SP_TEST_METHOD(CLASS_NAME, collisionDetails_FaceFace_6)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;
		SpCollisionDetails details;

		// build mesh 1
		const sp_uint vertexesLength = 3u;
		Vec3 vertexes[vertexesLength] = { Vec3(-1.0f, 0.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f) };
		SpPoint3<sp_uint> facesIndex(0, 1, 2);

		SpMesh* mesh = sp_mem_new(SpMesh)();
		mesh->vertexesMesh = sp_mem_new(SpArray<SpVertexMesh*>)(vertexesLength);
		mesh->faces = sp_mem_new(SpArray<SpFaceMesh*>)(1u);

		for (sp_uint i = 0; i < vertexesLength; i++)
			mesh->vertexesMesh->add(sp_mem_new(SpVertexMesh)(mesh, i, vertexes[i]));

		mesh->faces->add(sp_mem_new(SpFaceMesh)(mesh, 0, facesIndex.x, facesIndex.y, facesIndex.z));
		mesh->init();
		simulator->mesh(0u, mesh);

		// build mesh 2
		Vec3 vertexes2[vertexesLength] = { Vec3(1.0f, -0.5f, 0.0f), Vec3(-1.0f, -0.5f, 0.0f), Vec3(0.0f, 0.5f, 0.0f) };
		mesh = sp_mem_new(SpMesh)();
		mesh->vertexesMesh = sp_mem_new(SpArray<SpVertexMesh*>)(vertexesLength);
		mesh->faces = sp_mem_new(SpArray<SpFaceMesh*>)(1u);

		for (sp_uint i = 0; i < vertexesLength; i++)
			mesh->vertexesMesh->add(sp_mem_new(SpVertexMesh)(mesh, i, vertexes2[i]));

		mesh->faces->add(sp_mem_new(SpFaceMesh)(mesh, 0, facesIndex.x, facesIndex.y, facesIndex.z));
		mesh->init();
		simulator->mesh(1u, mesh);
		simulator->collisionFeatures(1u, 1u);


		resetObject(0u);
		resetObject(1u);

		SpRigidBody* propertiesObj1 = simulator->rigidBodies(0u);
		SpRigidBody* propertiesObj2 = simulator->rigidBodies(1u);

		details = newCollisionDetails();

		collisionDetector.collisionDetails(&details);

		Assert::IsFalse(details.ignoreCollision, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.type == SpCollisionType::FaceFace, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.timeOfCollision >= ZERO_FLOAT && details.timeOfCollision <= details.timeStep, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.contactPointsLength == 3u, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.collisionNormal, Vec3(0.0f, 0.0f, 1.0f), ERROR_MARGIN_PHYSIC), L"Wrong value.", LINE_INFO());
		
		Assert::IsTrue(isCloseEnough(details.contactPoints[0], Vec3(0.0f, 0.5f, 0.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.contactPoints[1], Vec3(-0.5f, 0.0f, 0.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.contactPoints[2], Vec3(0.5f, 0.0f, 0.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());

		Assert::IsTrue(isCloseEnough(details.centerContactPoint, Vec3(0.0f, 0.15f, 0.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());

		TestPhysic::unlock();
	}

	SP_TEST_METHOD(CLASS_NAME, collisionDetails_EdgeFace_1)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;
		SpCollisionDetails details;

		createMeshePlane();
		createCubeMeshes(1u);
		resetObject(0u);
		resetObject(1u);

		SpRigidBody* propertiesObj1 = simulator->rigidBodies(0u);
		SpRigidBody* propertiesObj2 = simulator->rigidBodies(1u);

		details = newCollisionDetails();
		
		propertiesObj1->mass(ZERO_FLOAT); // static object

		Quat orientationObj2 = Quat::createRotationAxisZ(degreesToRadians(45));
		simulator->transforms(1u)->position = Vec3(5.0f, 0.5f, -0.5f);
		propertiesObj2->currentState.position(Vec3(5.0f, 0.5f, -0.5f));
		propertiesObj2->previousState.position(Vec3(5.0f, 0.5f, -0.5f));
		propertiesObj2->currentState.velocity(Vec3(-20.0f, 0.0f, 0.0f));
		propertiesObj2->previousState.velocity(Vec3(-20.0f, 0.0f, 0.0f));
		simulator->transforms(1u)->orientation = orientationObj2;
		propertiesObj2->currentState.orientation(orientationObj2);
		propertiesObj2->previousState.orientation(orientationObj2);

		collisionDetector.collisionDetails(&details);

		Assert::IsFalse(details.ignoreCollision, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.type == SpCollisionType::EdgeFace, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.timeOfCollision >= ZERO_FLOAT && details.timeOfCollision <= details.timeStep, L"Wrong value.", LINE_INFO());

		Assert::IsTrue(isCloseEnough(details.collisionNormal, Vec3(1.0f, 0.0f, 0.0f), ERROR_MARGIN_PHYSIC), L"Wrong value.", LINE_INFO());
		
		Assert::IsTrue(details.contactPointsLength == 2u, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.contactPoints[0], Vec3(1.0f, 0.5f, -1.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.contactPoints[1], Vec3(1.0f, 0.5f, 0.5f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());

		Assert::IsTrue(isCloseEnough(details.centerContactPoint, Vec3(1.0f, 0.5f, -0.25f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());

		TestPhysic::unlock();
	}

	SP_TEST_METHOD(CLASS_NAME, collisionDetails_EdgeEdge_1)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;
		SpCollisionDetails details;

		createMeshePlane();
		createCubeMeshes(1u);
		resetObject(0u);
		resetObject(1u);

		SpRigidBody* propertiesObj1 = simulator->rigidBodies(0u);
		SpRigidBody* propertiesObj2 = simulator->rigidBodies(1u);

		details = newCollisionDetails();
		
		Quat orientationObj1 = Quat::createRotationAxisY(degreesToRadians(45));
		simulator->transforms(0u)->orientation = orientationObj1;
		simulator->rigidBodies(0u)->currentState.orientation(orientationObj1);
		simulator->rigidBodies(0u)->previousState.orientation(orientationObj1);
		propertiesObj1->mass(ZERO_FLOAT); // static object
		
		Quat orientationObj2 = Quat::createRotationAxisZ(degreesToRadians(45));
		simulator->transforms(1u)->orientation = orientationObj2;
		simulator->rigidBodies(1u)->currentState.orientation(orientationObj2);
		simulator->rigidBodies(1u)->previousState.orientation(orientationObj2);
		simulator->transforms(1u)->position = Vec3(4.0f, 0.0f, 0.0f);
		propertiesObj2->currentState.position(Vec3(4.0f, 0.0f, 0.0f));
		propertiesObj2->previousState.position(Vec3(4.0f, 0.0f, 0.0f));
		propertiesObj2->currentState.velocity(Vec3(-20.0f, 0.0f, 0.0f));
		propertiesObj2->previousState.velocity(Vec3(-20.0f, 0.0f, 0.0f));
		
		collisionDetector.collisionDetails(&details);
		
		Assert::IsFalse(details.ignoreCollision, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.type == SpCollisionType::EdgeEdge, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.timeOfCollision >= ZERO_FLOAT && details.timeOfCollision <= details.timeStep, L"Wrong value.", LINE_INFO());

		Assert::IsTrue(isCloseEnough(details.collisionNormal, Vec3(-1.0f, 0.0f, 0.0f), ERROR_MARGIN_PHYSIC), L"Wrong value.", LINE_INFO());
		
		Assert::IsTrue(details.contactPointsLength == 1u, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.contactPoints[0], Vec3(1.414214f, 0.0f, 0.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.centerContactPoint, Vec3(1.413911f, 0.0f, 0.0f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());

		TestPhysic::unlock();
	}

	SP_TEST_METHOD(CLASS_NAME, collisionDetails_VertexFace)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;
		SpCollisionDetails details;

		createMeshePlane();
		createCubeMeshes(1u);
		resetObject(0u);
		resetObject(1u);

		SpRigidBody* propertiesObj1 = simulator->rigidBodies(0u);
		SpRigidBody* propertiesObj2 = simulator->rigidBodies(1u);

		details = newCollisionDetails();

		propertiesObj1->mass(ZERO_FLOAT); // static object

		Quat orientationObj2 = Quat::createRotate(degreesToRadians(45), Vec3(0.0f, 1.0f, 1.0f));
		
		simulator->transforms(1u)->orientation = orientationObj2;
		propertiesObj2->currentState.orientation(orientationObj2);
		propertiesObj2->previousState.orientation(orientationObj2);
		simulator->transforms(1u)->position = Vec3(4.0f, 0.0f, 0.0f);
		propertiesObj2->previousState.position(Vec3(4.0f, 0.0f, 0.0f));
		propertiesObj2->currentState.velocity(Vec3(-20.0f, 0.0f, 0.0f));
		propertiesObj2->previousState.velocity(Vec3(-20.0f, 0.0f, 0.0f));

		collisionDetector.collisionDetails(&details);

		sp_char text[16000];
		sp_uint index = ZERO_UINT;
		Maple::convert(*simulator->mesh(simulator->collisionFeatures(0u)->meshIndex), *simulator->transforms(0u), "mesh1", "red", text, &index);
		Maple::convert(*simulator->mesh(simulator->collisionFeatures(1u)->meshIndex), *simulator->transforms(1u), "mesh2", "blue", text, &index);
		Maple::display("mesh1", "mesh2", text, &index);

		Assert::IsFalse(details.ignoreCollision, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.type == SpCollisionType::VertexFace, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.timeOfCollision >= ZERO_FLOAT && details.timeOfCollision <= details.timeStep, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.contactPointsLength == 1u, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(details.collisionNormal, Vec3(1.0f, 0.0f, 0.0f), ERROR_MARGIN_PHYSIC), L"Wrong value.", LINE_INFO());
		
		Assert::IsTrue(isCloseEnough(details.contactPoints[0], Vec3(1.0f, -0.127f, 0.127f), ERROR_MARGIN_PHYSIC), L"Wrong value", LINE_INFO());
		
		TestPhysic::unlock();
	}

}

#undef CLASS_NAME
