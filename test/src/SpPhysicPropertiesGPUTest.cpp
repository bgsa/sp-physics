#ifdef OPENCL_ENABLED

#include "SpectrumPhysicsTest.h"
#include <SweepAndPrune.h>
#include "Randomizer.h"
#include "DOP18.h"
#include "SpPhysicProperties.h"

#define CLASS_NAME SpPhysicPropertiesGPUTest

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		SP_TEST_METHOD_DEF(fetch);
		SP_TEST_METHOD_DEF(isResting);
	};

	SP_TEST_METHOD(CLASS_NAME, fetch)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice();

		const sp_uint length = 3u;
		SpPhysicProperties* physicProperties = ALLOC_NEW_ARRAY(SpPhysicProperties, length);
		for (sp_uint i = 0; i < length; i++)
		{
			sp_float index = (sp_float) (i * sizeof(SpPhysicProperties));
			
			physicProperties[i].position(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;
			physicProperties[i].velocity(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;
			physicProperties[i].acceleration(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;
			physicProperties[i].addForce(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;
			physicProperties[i].orientation(Vec3(index, index + 1.0f, index + 2.0f));
			index += 4;
			physicProperties[i].addImpulseAngular(Vec3(index, index + 1.0f, index + 2.0f), Vec3(index, index + 1.0f, index + 2.0f));
			index += 6 + 21;
			physicProperties[i].mass(index);
			index += 1;
			physicProperties[i].damping(index);
			index += 1;
			physicProperties[i].angularDamping(index);
			index += 1;
			physicProperties[i].coeficientOfRestitution(index);
			index += 1;
			physicProperties[i].coeficientOfFriction(index);
		}
		physicProperties[2].position(physicProperties[2].previousPosition());
		physicProperties[2].acceleration(physicProperties[2].previousAcceleration());

		cl_mem physcPropertiesGpu = gpu->createBuffer(physicProperties, sizeof(SpPhysicProperties) * length, CL_MEM_READ_ONLY, true);
		cl_mem outputGpu = gpu->createBuffer(sizeof(SpPhysicProperties), CL_MEM_READ_ONLY);

		SpDirectory* filename = SpDirectory::currentDirectory()
			->add(SP_DIRECTORY_OPENCL_SOURCE)
			->add("SpPhysicProperties.cl");

		SP_FILE file;
		SpString* source = file.readTextFile(filename->name()->data());

		sp_uint sapIndex = gpu->commandManager->cacheProgram(source->data(), SIZEOF_CHAR * source->length(), nullptr);
		cl_program sapProgram = gpu->commandManager->cachedPrograms[sapIndex];

		sp_mem_delete(source, SpString);

		GpuCommand* command = gpu->commandManager
			->createCommand()
			->setInputParameter(physcPropertiesGpu, sizeof(SpPhysicProperties) * length)
			->setInputParameter(outputGpu, sizeof(SpPhysicProperties))
			->buildFromProgram(sapProgram, "fetch");

		const sp_size globalWorkSize[3] = { 1, 0, 0 };
		const sp_size localWorkSize[3] = { 1, 0, 0 };

		const sp_size elementIndex = 2u;
		sp_float* result = (sp_float*)ALLOC_SIZE(sizeof(SpPhysicProperties));
		command
			->execute(1, globalWorkSize, localWorkSize, &elementIndex)
			->fetchInOutParameter<sp_float>(1u, result);

		const sp_float* expected = (sp_float*)&physicProperties[elementIndex];
		const sp_uint floatsInProperties = sizeof(SpPhysicProperties) / sizeof(sp_float);

		for (sp_uint i = 0; i < floatsInProperties; i++)
			Assert::AreEqual(expected[i], result[i], L"wrong value", LINE_INFO());

		gpu->releaseBuffer(physcPropertiesGpu);
		gpu->releaseBuffer(outputGpu);
		ALLOC_RELEASE(physicProperties);
	}

	SP_TEST_METHOD(CLASS_NAME, isResting)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice();

		const sp_uint length = 3u;
		SpPhysicProperties* physicProperties = ALLOC_NEW_ARRAY(SpPhysicProperties, length);
		for (sp_uint i = 0; i < length; i++)
		{
			sp_float index = (sp_float)(i * sizeof(SpPhysicProperties));

			physicProperties[i].position(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;
			physicProperties[i].velocity(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;
			physicProperties[i].acceleration(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;
			physicProperties[i].addForce(Vec3(index, index + 1.0f, index + 2.0f));
			index += 3;
			physicProperties[i].orientation(Vec3(index, index + 1.0f, index + 2.0f));
			index += 4;
			physicProperties[i].angularVelocity(Vec3(index, index + 1.0f, index + 2.0f));
			index += 6 + 21;
			physicProperties[i].mass(index);
			index += 1;
			physicProperties[i].damping(index);
			index += 1;
			physicProperties[i].angularDamping(index);
			index += 1;
			physicProperties[i].coeficientOfRestitution(index);
			index += 1;
			physicProperties[i].coeficientOfFriction(index);
		}
		//physicProperties[2].position(physicProperties[2].previousPosition());
		//physicProperties[2].velocity(physicProperties[2].previousVelocity());
		//physicProperties[2].acceleration(physicProperties[2].previousAcceleration());
		
		cl_mem physcPropertiesGpu = gpu->createBuffer(physicProperties, sizeof(SpPhysicProperties) * length, CL_MEM_READ_ONLY, true);
		cl_mem outputGpu = gpu->createBuffer(sizeof(SpPhysicProperties), CL_MEM_READ_ONLY);

		SpDirectory* filename = SpDirectory::currentDirectory()
			->add(SP_DIRECTORY_OPENCL_SOURCE)
			->add("SpPhysicProperties.cl");

		SP_FILE file;
		SpString* source = file.readTextFile(filename->name()->data());

		sp_uint sapIndex = gpu->commandManager->cacheProgram(source->data(), SIZEOF_CHAR * source->length(), nullptr);
		cl_program sapProgram = gpu->commandManager->cachedPrograms[sapIndex];

		sp_mem_delete(source, SpString);

		GpuCommand* command = gpu->commandManager
			->createCommand()
			->setInputParameter(physcPropertiesGpu, sizeof(SpPhysicProperties) * length)
			->setInputParameter(outputGpu, sizeof(sp_bool))
			->buildFromProgram(sapProgram, "isResting");

		const sp_size globalWorkSize[3] = { 1, 0, 0 };
		const sp_size localWorkSize[3] = { 1, 0, 0 };

		const sp_size elementIndex = 2u;
		sp_bool result;
		command
			->execute(1, globalWorkSize, localWorkSize, &elementIndex)
			->fetchInOutParameter<sp_bool>(1u, &result);

		const sp_bool expected = physicProperties[elementIndex].isResting();

		Assert::AreEqual(expected, result, L"wrong value", LINE_INFO());

		gpu->releaseBuffer(physcPropertiesGpu);
		gpu->releaseBuffer(outputGpu);
		ALLOC_RELEASE(physicProperties);
	}

}

#undef CLASS_NAME

#endif // OPENGL_ENABLED
