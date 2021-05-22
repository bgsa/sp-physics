#include "SpectrumPhysicsTest.h"
#include <SpCollisionResponseGPU.h>
#include "SpPhysicSimulator.h"

#define CLASS_NAME SpCollisionResponseGPUTest

namespace NAMESPACE_PHYSICS_TEST
{

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		SP_TEST_METHOD_DEF(handleCollisionResponseGPU);
	};

	SP_TEST_METHOD(CLASS_NAME, handleCollisionResponseGPU)
	{
		TestPhysic::lock();

		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice();
	
		SpCollisionResponseGPU response;
		response.init(gpu, nullptr);
	
		SpWorld* world = SpWorldManagerInstance->current();

		SpRigidBody3D* propertiesObj1 = world->rigidBody3D(0u);
		SpRigidBody3D* propertiesObj2 = world->rigidBody3D(1u);

		propertiesObj1->mass(ZERO_FLOAT); // static object

		world->transforms(1u)->position = Vec3(5.0f, 0.0f, 0.0f);
		propertiesObj2->mass(8.0f); // static object
		propertiesObj2->currentState.position(Vec3(35.000000f, -0.454432f, 15.000000f));
		propertiesObj2->previousState.position(Vec3(35.000000f, 0.197773f, 15.000000f));
		propertiesObj2->currentState.orientation(QUAT_UNIT);
		propertiesObj2->previousState.orientation(QUAT_UNIT);

		sp_uint indexes[4] = { 0u, 0u, 0u, 1u };
		cl_mem indexesGPU = gpu->createBuffer(indexes, 4 * sizeof(sp_uint), CL_MEM_USE_HOST_PTR | CL_MEM_READ_WRITE);

		sp_uint indexesLength = 4u;
		cl_mem indexesLengthGPU = gpu->createBuffer(&indexesLength, sizeof(sp_uint), CL_MEM_USE_HOST_PTR | CL_MEM_READ_WRITE);

		cl_mem rigidBodies3DGPU = gpu->createBuffer(propertiesObj1, sizeof(SpRigidBody3D) * 2u, CL_MEM_USE_HOST_PTR | CL_MEM_READ_WRITE);

		cl_mem outputIndexesGPU = gpu->createBuffer(4 * sizeof(sp_uint), CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_WRITE);
		
		sp_uint outputIndexesLength = ZERO_UINT;
		cl_mem outputIndexesLengthGPU = gpu->createBuffer(&outputIndexesLength, sizeof(sp_uint), CL_MEM_USE_HOST_PTR | CL_MEM_READ_WRITE);

		response.setParameters(indexesGPU, indexesLengthGPU, indexesLength, rigidBodies3DGPU, outputIndexesGPU, outputIndexesLengthGPU, 4 * sizeof(sp_uint));

		cl_event evt;
		response.execute(ZERO_UINT, NULL, &evt);
		gpu->releaseEvent(evt);

		sp_uint length;
		sp_uint* indexesResult = ALLOC_ARRAY(sp_uint, 4);

		response.fetchCollisionLength(&length, ZERO_UINT, NULL, &evt);
		gpu->releaseEvent(evt);

		response.fetchCollisions(indexesResult, ZERO_UINT, NULL, &evt);
		gpu->releaseEvent(evt);

		SpRigidBody3D* resultProperties = ALLOC_NEW_ARRAY(SpRigidBody3D, 2u);
		gpu->commandManager->readBuffer(rigidBodies3DGPU, sizeof(SpRigidBody3D) * 2u, resultProperties);

		Assert::IsTrue(length == 1u, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(indexesResult[0] == 0u, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(indexesResult[1] == 1u, L"Wrong value.", LINE_INFO());
		
		TestPhysic::unlock();
	}

}

#undef CLASS_NAME
