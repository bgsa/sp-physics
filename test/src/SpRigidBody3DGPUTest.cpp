#ifdef OPENCL_ENABLED

#include "SpectrumPhysicsTest.h"
#include <SweepAndPrune.h>
#include "Randomizer.h"
#include "DOP18.h"
#include "SpRigidBody3D.h"

#define CLASS_NAME SpRigidBody3DGPUTest

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		SP_TEST_METHOD_DEF(fetch);
		SP_TEST_METHOD_DEF(isResting);
		SP_TEST_METHOD_DEF(isStatic);
	};

	SP_TEST_METHOD(CLASS_NAME, fetch)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice();

		const sp_uint sizePhysicProperties = sizeof(SpRigidBody3D);
		const sp_uint length = 3u;
		SpRigidBody3D* rigidBodies = ALLOC_NEW_ARRAY(SpRigidBody3D, length);
		for (sp_uint i = 0; i < length; i++)
		{
			sp_float index = (sp_float) (i * sizeof(SpRigidBody3D));
			
			rigidBodies[i].currentState.position(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;
			rigidBodies[i].currentState.velocity(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;
			rigidBodies[i].currentState.acceleration(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;
			rigidBodies[i].currentState.addForce(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;
			rigidBodies[i].currentState.orientation(Vec3(index, index + 1.0f, index + 2.0f));
			index += 4;
			rigidBodies[i].currentState.angularVelocity(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;
			rigidBodies[i].currentState.torque(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;

			rigidBodies[i].previousState.position(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;
			rigidBodies[i].previousState.velocity(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;
			rigidBodies[i].previousState.acceleration(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;
			rigidBodies[i].previousState.addForce(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;
			rigidBodies[i].previousState.orientation(Vec3(index, index + 1.0f, index + 2.0f));
			index += 4;
			rigidBodies[i].previousState.angularVelocity(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;
			rigidBodies[i].previousState.torque(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;

			rigidBodies[i].mass(index);
			index += 1;
			rigidBodies[i].damping(index);
			index += 1;
			rigidBodies[i].angularDamping(index);
			index += 1;
			rigidBodies[i].coeficientOfRestitution(index);
			index += 1;
			rigidBodies[i].coeficientOfFriction(index);
		}
		
		cl_mem physcPropertiesGpu = gpu->createBuffer(rigidBodies, sizePhysicProperties * length, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem outputGpu = gpu->createBuffer(sizePhysicProperties, CL_MEM_READ_ONLY | CL_MEM_ALLOC_HOST_PTR);

		SpDirectory* filename = SpDirectory::currentDirectory()
			->add(SP_DIRECTORY_OPENCL_SOURCE)
			->add("SpRigidBody3D.cl");

		SP_FILE file;
		SpString* source = file.readTextFile(filename->name()->data());

		cl_program program;
		gpu->commandManager->buildProgram(source->data(), SIZEOF_CHAR * source->length(), nullptr, &program);
		
		sp_mem_delete(source, SpString);

		GpuCommand* command = gpu->commandManager
			->createCommand()
			->setInputParameter(physcPropertiesGpu, sizePhysicProperties * length)
			->setInputParameter(outputGpu, sizePhysicProperties)
			->buildFromProgram(program, "fetch");

		const sp_size globalWorkSize[3] = { 1, 0, 0 };
		const sp_size localWorkSize[3] = { 1, 0, 0 };

		const sp_size elementIndex = 2u;
		sp_float* result = (sp_float*)ALLOC_SIZE(sizePhysicProperties);
		cl_event evt, evt2;
		command
			->execute(1, globalWorkSize, localWorkSize, &elementIndex, ZERO_UINT, NULL, &evt)
			->fetchInOutParameter<sp_float>(1u, result, ONE_UINT, &evt, &evt2);
		gpu->releaseEvent(evt);
		gpu->releaseEvent(evt2);

		const sp_float* expected = (sp_float*)&rigidBodies[elementIndex];
		const sp_uint floatsInProperties = sizePhysicProperties / sizeof(sp_float);

		for (sp_uint i = 0; i < floatsInProperties; i++)
			Assert::AreEqual(expected[i], result[i], L"wrong value", LINE_INFO());

		gpu->releaseBuffer(physcPropertiesGpu);
		gpu->releaseBuffer(outputGpu);
		ALLOC_RELEASE(rigidBodies);
	}

	SP_TEST_METHOD(CLASS_NAME, isResting)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice();

		const sp_uint length = 3u;
		SpRigidBody3D* rigidBodies3D = ALLOC_NEW_ARRAY(SpRigidBody3D, length);
		for (sp_uint i = 0; i < length; i++)
		{
			sp_float index = (sp_float)(i * sizeof(SpRigidBody3D));

			rigidBodies3D[i].currentState.position(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;
			rigidBodies3D[i].currentState.velocity(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;
			rigidBodies3D[i].currentState.acceleration(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;
			rigidBodies3D[i].currentState.addForce(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;
			rigidBodies3D[i].currentState.orientation(Vec3(index, index + 1.0f, index + 2.0f));
			index += 4;
			rigidBodies3D[i].currentState.angularVelocity(Vec3(index, index + 1.0f, index + 2.0f));
			index += 6 + 21;
			rigidBodies3D[i].mass(index);
			index += 1;
			rigidBodies3D[i].damping(index);
			index += 1;
			rigidBodies3D[i].angularDamping(index);
			index += 1;
			rigidBodies3D[i].coeficientOfRestitution(index);
			index += 1;
			rigidBodies3D[i].coeficientOfFriction(index);
		}
		//rigidBodies3D[2].position(rigidBodies3D[2].previousPosition());
		//rigidBodies3D[2].velocity(rigidBodies3D[2].previousVelocity());
		//rigidBodies3D[2].acceleration(rigidBodies3D[2].previousAcceleration());
		
		cl_mem physcPropertiesGpu = gpu->createBuffer(rigidBodies3D, sizeof(SpRigidBody3D) * length, CL_MEM_READ_ONLY, true);
		cl_mem outputGpu = gpu->createBuffer(sizeof(SpRigidBody3D), CL_MEM_READ_ONLY);

		SpDirectory* filename = SpDirectory::currentDirectory()
			->add(SP_DIRECTORY_OPENCL_SOURCE)
			->add("SpRigidBody3D.cl");

		SP_FILE file;
		SpString* source = file.readTextFile(filename->name()->data());

		cl_program program; 
		gpu->commandManager->buildProgram(source->data(), SIZEOF_CHAR * source->length(), nullptr, &program);

		sp_mem_delete(source, SpString);

		GpuCommand* command = gpu->commandManager
			->createCommand()
			->setInputParameter(physcPropertiesGpu, sizeof(SpRigidBody3D) * length)
			->setInputParameter(outputGpu, sizeof(sp_bool))
			->buildFromProgram(program, "isResting");

		const sp_size globalWorkSize[3] = { 1, 0, 0 };
		const sp_size localWorkSize[3] = { 1, 0, 0 };

		const sp_size elementIndex = 2u;
		sp_bool result;
		cl_event evt, evt2;
		command
			->execute(1, globalWorkSize, localWorkSize, &elementIndex, ZERO_UINT, NULL, &evt)
			->fetchInOutParameter<sp_bool>(1u, &result, ONE_UINT, &evt, &evt2);
		gpu->releaseEvent(evt);
		gpu->releaseEvent(evt2);

		const sp_bool expected = rigidBodies3D[elementIndex].isResting();

		Assert::AreEqual(expected, result, L"wrong value", LINE_INFO());

		gpu->releaseBuffer(physcPropertiesGpu);
		gpu->releaseBuffer(outputGpu);
		ALLOC_RELEASE(rigidBodies3D);
	}

	SP_TEST_METHOD(CLASS_NAME, isStatic)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice();

		SpRigidBody3D* rigidBodies3D = ALLOC_NEW_ARRAY(SpRigidBody3D, 2u);
		rigidBodies3D[0].mass(ZERO_FLOAT);
		rigidBodies3D[1].mass(8.0f);

		cl_mem rigidBodies3DGpu = gpu->createBuffer(rigidBodies3D, sizeof(SpRigidBody3D) * 2u, CL_MEM_READ_ONLY, true);
		cl_mem outputGpu = gpu->createBuffer(sizeof(SpRigidBody3D), CL_MEM_READ_ONLY);

		SpDirectory* filename = SpDirectory::currentDirectory()
			->add(SP_DIRECTORY_OPENCL_SOURCE)
			->add("SpRigidBody3D.cl");

		SP_FILE file;
		SpString* source = file.readTextFile(filename->name()->data());

		cl_program program;
		gpu->commandManager->buildProgram(source->data(), SIZEOF_CHAR * source->length(), nullptr, &program);
		
		sp_mem_delete(source, SpString);

		GpuCommand* command = gpu->commandManager
			->createCommand()
			->setInputParameter(rigidBodies3DGpu, sizeof(SpRigidBody3D) * 2u)
			->setInputParameter(outputGpu, sizeof(sp_bool))
			->buildFromProgram(program, "isStatic");

		const sp_size globalWorkSize[3] = { 1, 0, 0 };
		const sp_size localWorkSize[3] = { 1, 0, 0 };

		sp_uint index = 0u;
		sp_bool result;
		cl_event evt, evt2;
		command
			->execute(1, globalWorkSize, localWorkSize, &index, ZERO_UINT, NULL, &evt)
			->fetchInOutParameter<sp_bool>(1u, &result, ONE_UINT, &evt, &evt2);
		gpu->releaseEvent(evt);
		gpu->releaseEvent(evt2);

		Assert::IsTrue(result, L"wrong value", LINE_INFO());

		index = 1u;
		command
			->execute(1, globalWorkSize, localWorkSize, &index, ZERO_UINT, NULL, &evt)
			->fetchInOutParameter<sp_bool>(1u, &result, ONE_UINT, &evt, &evt2);
		gpu->releaseEvent(evt);
		gpu->releaseEvent(evt2);

		Assert::IsFalse(result, L"wrong value", LINE_INFO());

		gpu->releaseBuffer(rigidBodies3DGpu);
		gpu->releaseBuffer(outputGpu);
		ALLOC_RELEASE(rigidBodies3D);
	}

}

#undef CLASS_NAME

#endif // OPENGL_ENABLED
