#include "SpectrumPhysicsTest.h"
#include <SpCollisionDetector.h>

#define CLASS_NAME SpCollisionDetectorTest

namespace NAMESPACE_PHYSICS_TEST
{

	SP_TEST_CLASS(CLASS_NAME)
	{
	private:

		void createMeshes()
		{
			SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
			
			const sp_uint vertexesLength = 8u;
			Vec3 vertexes[vertexesLength] = {
				Vec3(-1.0f, -1.0f, -1.0f), Vec3(-1.0f, -1.0f, 1.0f),
				Vec3(-1.0f, 1.0f, 1.0f), Vec3(-1.0f, 1.0f, -1.0f),
				Vec3(1.0f, -1.0f, -1.0f), Vec3(1.0f, 1.0f, -1.0f),
				Vec3(1.0f, 1.0f, 1.0f), Vec3(1.0f, -1.0f, 1.0f)
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

			for (sp_uint i = 0; i < 2; i++)
			{
				SpMesh* mesh = sp_mem_new(SpMesh)();
				mesh->vertexesMesh = sp_mem_new(SpArray<SpVertexMesh*>)(vertexesLength);
				mesh->faces = sp_mem_new(SpArray<SpFaceMesh*>)(facesLength);
				
				for (sp_uint i = 0; i < vertexesLength; i++)
					mesh->vertexesMesh->add(sp_mem_new(SpVertexMesh)(mesh, i, vertexes[i]));

				for (sp_uint i = 0; i < facesLength; i++)
					mesh->faces->add(sp_mem_new(SpFaceMesh)(mesh, i, facesIndex[i].x, facesIndex[i].y, facesIndex[i].z));

				mesh->init();

				SpPhysicSimulator::instance()->mesh(i, mesh);
			}
		}

		void resetObject(const sp_uint index)
		{
			SpPhysicSimulator* simulator = SpPhysicSimulator::instance();

			simulator->transforms(index)->reset();
		
			simulator->physicProperties(index)->currentState.reset();
			simulator->physicProperties(index)->previousState.reset();
			simulator->physicProperties(index)->mass(8.0f);
			simulator->physicProperties(index)->integratedTime(ZERO_FLOAT);
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
			
			return details;
		}

	public:
		SP_TEST_METHOD_DEF(areMovingAway);
		SP_TEST_METHOD_DEF(collisionStatus);
		SP_TEST_METHOD_DEF(filterCollision);
		SP_TEST_METHOD_DEF(collisionDetails_FaceFace_Test1);
		SP_TEST_METHOD_DEF(collisionDetails_FaceFace_Test2);
		//SP_TEST_METHOD_DEF(collisionDetails_FaceFace_Test3);
		SP_TEST_METHOD_DEF(collisionDetails_FaceFace_Test4);
		SP_TEST_METHOD_DEF(collisionDetails_EdgeFace_Test1);
		SP_TEST_METHOD_DEF(collisionDetails_EdgeEdge_Test1);
		SP_TEST_METHOD_DEF(collisionDetails_VertexFace_Test1);
	};

	SP_TEST_METHOD(CLASS_NAME, areMovingAway)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;

		resetObject(0u);
		resetObject(1u);

		simulator->physicProperties(0u)->currentState.position(Vec3(1.0f, 0.0f, 0.0f));
		simulator->physicProperties(1u)->currentState.position(Vec3(0.0f, 0.0f, 0.0f));

		simulator->physicProperties(0u)->currentState.velocity(Vec3(10.0f, 0.0f, 0.0f));
		simulator->physicProperties(1u)->currentState.velocity(Vec3(-10.0f, 0.0f, 0.0f));

		sp_bool result = collisionDetector.areMovingAway(0u, 1u);

		Assert::IsTrue(result, L"Wrong value.", LINE_INFO());

		simulator->physicProperties(0u)->currentState.velocity(Vec3(10.0f, 0.0f, 0.0f));
		simulator->physicProperties(1u)->currentState.velocity(Vec3(1.0f, 0.0f, 0.0f));

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

		createMeshes();

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

		createMeshes();

		SpPhysicProperties* propertiesObj1 = simulator->physicProperties(0u);
		SpPhysicProperties* propertiesObj2 = simulator->physicProperties(1u);

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
		propertiesObj1->currentState.position(ZERO_FLOAT);
		propertiesObj1->previousState.position(ZERO_FLOAT);
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

	SP_TEST_METHOD(CLASS_NAME, collisionDetails_FaceFace_Test1)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;
		SpCollisionDetails details;

		createMeshes();

		resetObject(0u);
		resetObject(1u);

		SpPhysicProperties* propertiesObj1 = simulator->physicProperties(0u);
		SpPhysicProperties* propertiesObj2 = simulator->physicProperties(1u);

		details = newCollisionDetails();
		
		propertiesObj1->mass(ZERO_FLOAT); // static object
		
		propertiesObj2->previousState.position(Vec3(4.0f, 0.0f, 0.0f));
		simulator->translate(1u, Vec3(1.5f, 0.0f, 0.0f));
		propertiesObj2->currentState.velocity(Vec3(-20.0f, 0.0f, 0.0f));
		propertiesObj2->previousState.velocity(Vec3(-20.0f, 0.0f, 0.0f));

		collisionDetector.collisionDetails(&details);
		
		Assert::IsFalse(details.ignoreCollision, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.type == SpCollisionType::FaceFace, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.timeOfCollision >= ZERO_FLOAT && details.timeOfCollision <= details.timeStep, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.contactPointsLength == 4u, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.collisionNormalObj1.isCloseEnough(Vec3(1.0f, 0.0f, 0.0f)), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.collisionNormalObj2.isCloseEnough(Vec3(-1.0f, 0.0f, 0.0f)), L"Wrong value.", LINE_INFO());

		Assert::IsTrue(details.contactPoints[0].isCloseEnough(Vec3(1.0f, -1.0f, -1.0f), 0.009f), L"Wrong value", LINE_INFO());
		Assert::IsTrue(details.contactPoints[1].isCloseEnough(Vec3(1.0f, -1.0f, 1.0f), 0.009f), L"Wrong value", LINE_INFO());
		Assert::IsTrue(details.contactPoints[2].isCloseEnough(Vec3(1.0f, 1.0f, 1.0f), 0.009f), L"Wrong value", LINE_INFO());
		Assert::IsTrue(details.contactPoints[3].isCloseEnough(Vec3(1.0f, 1.0f, -1.0f), 0.009f), L"Wrong value", LINE_INFO());

		Assert::IsTrue(details.centerContactPoint.isCloseEnough(Vec3(1.0f, 0.0f, 0.0f), 0.009f), L"Wrong value", LINE_INFO());

		TestPhysic::unlock();
	}

	SP_TEST_METHOD(CLASS_NAME, collisionDetails_FaceFace_Test2)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;
		SpCollisionDetails details;

		createMeshes();
		resetObject(0u);
		resetObject(1u);

		SpPhysicProperties* propertiesObj1 = simulator->physicProperties(0u);
		SpPhysicProperties* propertiesObj2 = simulator->physicProperties(1u);

		details = newCollisionDetails();
		
		propertiesObj1->mass(ZERO_FLOAT);

		simulator->transforms(1u)->position = Vec3(1.5f, 0.5f, -0.5f);
		propertiesObj2->previousState.position(Vec3(4.0f, 0.5f, -0.5f));
		propertiesObj2->currentState.position(Vec3(4.0f, 0.5f, -0.5f));
		propertiesObj2->currentState.velocity(Vec3(-20.0f, 0.0f, 0.0f));
		propertiesObj2->previousState.velocity(Vec3(-20.0f, 0.0f, 0.0f));
		
		collisionDetector.collisionDetails(&details);
		
		Assert::IsFalse(details.ignoreCollision, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.type == SpCollisionType::FaceFace, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.timeOfCollision >= ZERO_FLOAT && details.timeOfCollision <= details.timeStep, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.contactPointsLength == 4u, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.collisionNormalObj1.isCloseEnough(Vec3(1.0f, 0.0f, 0.0f)), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.collisionNormalObj2.isCloseEnough(Vec3(-1.0f, 0.0f, 0.0f)), L"Wrong value.", LINE_INFO());

		Assert::IsTrue(details.contactPoints[0].isCloseEnough(Vec3(1.0f, -0.5f, 0.5f), 0.009f), L"Wrong value", LINE_INFO());
		Assert::IsTrue(details.contactPoints[1].isCloseEnough(Vec3(1.0f, 1.0f, -1.0f), 0.009f), L"Wrong value", LINE_INFO());
		Assert::IsTrue(details.contactPoints[2].isCloseEnough(Vec3(1.0f, 1.0f, 0.5f), 0.009f), L"Wrong value", LINE_INFO());
		Assert::IsTrue(details.contactPoints[3].isCloseEnough(Vec3(1.0f, -0.5f, -1.0f), 0.009f), L"Wrong value", LINE_INFO());

		Assert::IsTrue(details.centerContactPoint.isCloseEnough(Vec3(1.0f, 0.25f, -0.25f), 0.009f), L"Wrong value", LINE_INFO());

		propertiesObj2->previousState.velocity(Vec3(ZERO_FLOAT));
		propertiesObj2->currentState.velocity(Vec3(ZERO_FLOAT));
		simulator->position(1u, Vec3(ZERO_FLOAT));

		TestPhysic::unlock();
	}

	/* Test INSIDE Face-Face 
	SP_TEST_METHOD(CLASS_NAME, collisionDetails_FaceFace_Test3)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;
		SpCollisionDetails details;

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

	SP_TEST_METHOD(CLASS_NAME, collisionDetails_FaceFace_Test4)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;
		SpCollisionDetails details;

		createMeshes();

		resetObject(0u);
		resetObject(1u);

		SpPhysicProperties* propertiesObj1 = simulator->physicProperties(0u);
		SpPhysicProperties* propertiesObj2 = simulator->physicProperties(1u);

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
		Assert::IsTrue(details.collisionNormalObj1.isCloseEnough(Vec3(0.0f, 1.0f, 0.0f)), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.collisionNormalObj2.isCloseEnough(Vec3(0.0f, -1.0f, 0.0f)), L"Wrong value.", LINE_INFO());

		Assert::IsTrue(details.contactPoints[0].isCloseEnough(Vec3(1.0f, -1.0f, -1.0f), 0.009f), L"Wrong value", LINE_INFO());
		Assert::IsTrue(details.contactPoints[1].isCloseEnough(Vec3(1.0f, -1.0f, 1.0f), 0.009f), L"Wrong value", LINE_INFO());
		Assert::IsTrue(details.contactPoints[2].isCloseEnough(Vec3(1.0f, 1.0f, 1.0f), 0.009f), L"Wrong value", LINE_INFO());
		Assert::IsTrue(details.contactPoints[3].isCloseEnough(Vec3(1.0f, 1.0f, -1.0f), 0.009f), L"Wrong value", LINE_INFO());

		Assert::IsTrue(details.centerContactPoint.isCloseEnough(Vec3(1.0f, 0.0f, 0.0f), 0.009f), L"Wrong value", LINE_INFO());

		TestPhysic::unlock();
	}

	SP_TEST_METHOD(CLASS_NAME, collisionDetails_EdgeFace_Test1)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;
		SpCollisionDetails details;

		createMeshes();
		resetObject(0u);
		resetObject(1u);

		SpPhysicProperties* propertiesObj1 = simulator->physicProperties(0u);
		SpPhysicProperties* propertiesObj2 = simulator->physicProperties(1u);

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
		
		Assert::IsTrue(details.contactPointsLength == 2u, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.contactPointsLength == 2u, L"Wrong value.", LINE_INFO());
	
		Assert::IsTrue(details.collisionNormalObj1.isCloseEnough(Vec3(1.0f, 0.0f, 0.0f), 0.009f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.collisionNormalObj2.isCloseEnough(Vec3(-1.0f, 0.0f, 0.0f), 0.009f), L"Wrong value.", LINE_INFO());

		Assert::IsTrue(details.contactPoints[0].isCloseEnough(Vec3(1.0f, 0.5f, -1.0f), 0.009f), L"Wrong value", LINE_INFO());
		Assert::IsTrue(details.contactPoints[1].isCloseEnough(Vec3(1.0f, 0.5f, 0.5f), 0.009f), L"Wrong value", LINE_INFO());

		Assert::IsTrue(details.centerContactPoint.isCloseEnough(Vec3(1.0f, 0.5f, -0.25f), 0.009f), L"Wrong value", LINE_INFO());

		TestPhysic::unlock();
	}

	SP_TEST_METHOD(CLASS_NAME, collisionDetails_EdgeEdge_Test1)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;
		SpCollisionDetails details;

		createMeshes();
		resetObject(0u);
		resetObject(1u);

		SpPhysicProperties* propertiesObj1 = simulator->physicProperties(0u);
		SpPhysicProperties* propertiesObj2 = simulator->physicProperties(1u);

		details = newCollisionDetails();

		Quat orientationObj1 = Quat::createRotationAxisY(degreesToRadians(45));
		simulator->transforms(0u)->orientation = orientationObj1;
		simulator->physicProperties(0u)->currentState.orientation(orientationObj1);
		simulator->physicProperties(0u)->previousState.orientation(orientationObj1);
		propertiesObj1->mass(ZERO_FLOAT); // static object
		
		Quat orientationObj2 = Quat::createRotationAxisZ(degreesToRadians(45));
		simulator->transforms(1u)->orientation = orientationObj2;
		simulator->physicProperties(1u)->currentState.orientation(orientationObj2);
		simulator->physicProperties(1u)->previousState.orientation(orientationObj2);
		simulator->transforms(1u)->position = Vec3(4.0f, 0.0f, 0.0f);
		propertiesObj2->currentState.position(Vec3(4.0f, 0.0f, 0.0f));
		propertiesObj2->previousState.position(Vec3(4.0f, 0.0f, 0.0f));
		propertiesObj2->currentState.velocity(Vec3(-20.0f, 0.0f, 0.0f));
		propertiesObj2->previousState.velocity(Vec3(-20.0f, 0.0f, 0.0f));
		
		collisionDetector.collisionDetails(&details);
		
		Vec3 vertexes[30];
		simulator->mesh(1)->convert(vertexes, *simulator->transforms(1u));

		Assert::IsFalse(details.ignoreCollision, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.type == SpCollisionType::EdgeEdge, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.timeOfCollision >= ZERO_FLOAT && details.timeOfCollision <= details.timeStep, L"Wrong value.", LINE_INFO());

		Assert::IsTrue(details.collisionNormalObj1.isCloseEnough(Vec3(-1.0f, 0.0f, 0.0f), 0.009f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.collisionNormalObj2.isCloseEnough(Vec3(1.0f, 0.0f, 0.0f), 0.009f), L"Wrong value.", LINE_INFO());

		Assert::IsTrue(details.contactPointsLength == 1u, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.contactPoints[0].isCloseEnough(Vec3(1.414214f, 0.0f, 0.0f), 0.009f), L"Wrong value", LINE_INFO());
		Assert::IsTrue(details.centerContactPoint.isCloseEnough(Vec3(1.413911f, 0.0f, 0.0f), 0.009f), L"Wrong value", LINE_INFO());

		TestPhysic::unlock();
	}

	SP_TEST_METHOD(CLASS_NAME, collisionDetails_VertexFace_Test1)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;
		SpCollisionDetails details;

		createMeshes();
		resetObject(0u);
		resetObject(1u);

		SpPhysicProperties* propertiesObj1 = simulator->physicProperties(0u);
		SpPhysicProperties* propertiesObj2 = simulator->physicProperties(1u);

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

		Assert::IsFalse(details.ignoreCollision, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.type == SpCollisionType::PointFace, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.timeOfCollision >= ZERO_FLOAT && details.timeOfCollision <= details.timeStep, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.contactPointsLength == 1u, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.collisionNormalObj1.isCloseEnough(Vec3(1.0f, 0.0f, 0.0f), 0.009f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(details.collisionNormalObj2.isCloseEnough(Vec3(-1.0f, 0.0f, 0.0f), 0.009f), L"Wrong value.", LINE_INFO());

		Assert::IsTrue(details.contactPoints[0].isCloseEnough(Vec3(0.9f, -0.127f, 0.127f), 0.009f), L"Wrong value", LINE_INFO());
		
		TestPhysic::unlock();
	}

}

#undef CLASS_NAME
