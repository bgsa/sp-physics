#include "SpectrumPhysicsTest.h"
#include <SpCollisionResponse.h>

#define CLASS_NAME SpCollisionResponseTest

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
		
			simulator->rigidBodies(index)->currentState.reset();
			simulator->rigidBodies(index)->previousState.reset();
			simulator->rigidBodies(index)->mass(8.0f);
		}

		SpCollisionDetails newCollisionDetails()
		{
			SpCollisionDetails details;
			details.objIndex1 = ZERO_UINT;
			details.objIndex2 = ONE_UINT;
			details.timeStep = 30.0f;

			return details;
		}

	public:
		SP_TEST_METHOD_DEF(handleCollisionResponse);
	};

	SP_TEST_METHOD(CLASS_NAME, handleCollisionResponse)
	{
		TestPhysic::lock();
		
		createMeshes();
		resetObject(0u);
		resetObject(1u);

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionResponse response;
		SpCollisionDetector detector;
		SpCollisionDetails details = newCollisionDetails();

		SpRigidBody* propertiesObj1 = simulator->rigidBodies(0u);
		SpRigidBody* propertiesObj2 = simulator->rigidBodies(1u);

		details = newCollisionDetails();

		propertiesObj1->mass(ZERO_FLOAT); // static object

		simulator->transforms(1u)->position = Vec3(5.0f, 0.0f, 0.0f);
		propertiesObj2->currentState.position(Vec3(5.0f, 0.0f, 0.0f));
		propertiesObj2->previousState.position(Vec3(5.0f, 0.0f, 0.0f));
		propertiesObj2->currentState.velocity(Vec3(-20.0f, 0.0f, 0.0f));
		propertiesObj2->previousState.velocity(Vec3(-20.0f, 0.0f, 0.0f));

		detector.collisionDetails(&details);

		response.handleCollisionResponse(&details);

		Assert::IsTrue(isCloseEnough(propertiesObj2->currentState.velocity(), Vec3(7.6f, 0.0f, 0.0f), 0.009f) , L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(propertiesObj2->previousState.velocity(), Vec3(-20.0f, 0.0f, 0.0f), 0.009f), L"Wrong value.", LINE_INFO());
		
		Assert::IsTrue(isCloseEnough(propertiesObj2->currentState.angularVelocity(), Vec3(0.0f, 0.0f, 0.0f), 0.009f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(propertiesObj2->previousState.angularVelocity(), Vec3(0.0f, 0.0f, 0.0f), 0.009f), L"Wrong value.", LINE_INFO());
		
		Assert::IsTrue(propertiesObj2->currentState.orientation().isCloseEnough(QUAT_UNIT, 0.009f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(propertiesObj2->previousState.orientation().isCloseEnough(QUAT_UNIT, 0.009f), L"Wrong value.", LINE_INFO());

		TestPhysic::unlock();
	}

}

#undef CLASS_NAME
