#include "SpectrumPhysicsTest.h"
#include <SpPhysicSimulator.h>
#include "ObjModel.h"
#include "SpMatlabExporter.h"

#define CLASS_NAME SpDOP18FactoryTest

namespace NAMESPACE_PHYSICS_TEST
{

	SP_TEST_CLASS(CLASS_NAME)
	{
	private:

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

			SpWorldManagerInstance->current()->mesh(index, mesh);
		}

		void resetObject(const sp_uint index)
		{
			SpWorld* world = SpWorldManagerInstance->current();

			world->transforms(index)->reset();

			world->rigidBody3D(index)->currentState.reset();
			world->rigidBody3D(index)->previousState.reset();
			world->rigidBody3D(index)->mass(8.0f);
		}

	public:
		SP_TEST_METHOD_DEF(build);

#ifdef OPENCL_ENABLED
		SP_TEST_METHOD_DEF(buildDOP18GPU);
		SP_TEST_METHOD_DEF(buildDOP18GPU_2);
		SP_TEST_METHOD_DEF(buildDOP18GPU_3);
		SP_TEST_METHOD_DEF(buildDOP18GPU_4);
#endif
	};

#ifdef OPENCL_ENABLED
	SP_TEST_METHOD(CLASS_NAME, buildDOP18GPU)
	{
		TestPhysic::lock();

		sp_float meshCache[24*3] = {
			0.997765303f, -37.0503998f, -0.798262239f,
			1.32016361f, -36.5010834f, -1.12797058f,
			1.82751310f, -36.4946022f, -0.621070802f,
			1.50511503f, -37.0439148f, -0.291362464f,
			-0.128041685f, -37.2458153f, 1.34582543f,
			-0.519256234f, -36.7847252f, 1.73149133f,
			-1.02660596f, -36.7912064f, 1.22459149f,
			-0.635391235f, -37.2522964f, 0.838925719f,
			2.09329700f, -35.4245224f, 1.42163301f,
			1.70208240f, -34.9634323f, 1.80729878f,
			1.37968433f, -35.5127449f, 2.13700724f,
			1.77089894f, -35.9738388f, 1.75134158f,
			-0.861699462f, -33.9295044f, 1.02294576f,
			-1.18409765f, -34.4788170f, 1.35265422f,
			-0.676748037f, -34.4723358f, 1.85955393f,
			-0.354349941f, -33.9230194f, 1.52984560f,
			1.16267180f, -34.1886940f, -0.999907970f,
			0.771457195f, -33.7276039f, -0.614242136f,
			1.27880681f, -33.7211227f, -0.107342362f,
			1.67002141f, -34.1822128f, -0.493008137f,
			-1.12748349f, -34.9995804f, -1.01975811f,
			-0.736268878f, -35.4606743f, -1.40542376f,
			-1.44988167f, -35.5488968f, -0.690049767f,
			-1.05866706f, -36.0099869f,	-1.07571542f
		};

		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		ObjModel model;
		model.load("resources\\models\\rocks\\obj.obj");

		SpMesh* mesh1 = sp_mem_new(SpMesh)();
		model.buildMesh(mesh1);
		SpWorldManagerInstance->current()->mesh(0, mesh1);
		SpWorldManagerInstance->current()->collisionFeatures(0, 0);
		SpWorldManagerInstance->current()->transforms(0)->position = Vec3(0.321708f, -35.486710f, 0.365792f);
		SpWorldManagerInstance->current()->transforms(0)->orientation = Quat(0.836536825f, 0.158000097f, 0.348789155f, 0.391903371f);
		SpWorldManagerInstance->current()->transforms(0)->scaleVector = Vec3(2.0f, 2.0f, 2.0f);

		sp_char filename[512];
		currentDirectory(filename, 512);
		directoryAddPath(filename, std::strlen(filename), SP_DIRECTORY_OPENCL_SOURCE, SP_DIRECTORY_OPENCL_SOURCE_LENGTH);
		directoryAddPath(filename, std::strlen(filename), "BoundingVolumeFactory.cl", 24);

		SP_FILE file;
		file.open(filename, std::ios::in);
		const sp_size fileSize = file.length();
		sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
		file.read(source, fileSize);
		file.close();

		cl_program program;
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, nullptr, &program);

		ALLOC_RELEASE(source);

		const sp_size globalWorkSize[3] = { 1, 0, 0 };
		const sp_size localWorkSize[3] = { 1, 0, 0 };

		sp_uint meshesLength = 1u;
		sp_uint meshCacheIndex = 0u;
		sp_uint meshCacheVertexesLength = mesh1->vertexLength();

		cl_mem meshCacheLengthGPU = gpu->createBuffer(&meshesLength, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem meshCacheIndexesGPU = gpu->createBuffer(&meshCacheIndex, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem meshCacheVertexesLengthGPU = gpu->createBuffer(&meshCacheVertexesLength, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem meshCacheGPU = gpu->createBuffer(meshCache, sizeof(Vec3) * mesh1->vertexLength(), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem transformsGPU = gpu->createBuffer(SpWorldManagerInstance->current()->transforms(0), sizeof(SpTransform), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem dop18GPU = gpu->createBuffer(sizeof(DOP18), CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		GpuCommand* command = gpu->commandManager
			->createCommand()
			->setInputParameter(meshCacheLengthGPU, sizeof(sp_uint))
			->setInputParameter(meshCacheIndexesGPU, sizeof(sp_uint))
			->setInputParameter(meshCacheVertexesLengthGPU, sizeof(sp_uint))
			->setInputParameter(meshCacheGPU, sizeof(Vec3) * mesh1->vertexLength())
			->setInputParameter(transformsGPU, sizeof(SpTransform))
			->setInputParameter(dop18GPU, sizeof(DOP18))
			->buildFromProgram(program, "buildDOP18");

		command->execute(1, globalWorkSize, localWorkSize, 0, 0, NULL, NULL);

		sp_float bv[18];
		command->fetchInOutParameter(bv, 5);

		sp_float expected[18] = {
			-1.44988167f, -37.2522964f, -1.40542376f, -2.41890478f, -2.40097809f, -2.36195230f, -2.37334776f, -2.51303363f, -2.17095995f,
			2.09329700f, -33.7211227f, 2.13700724f, 3.06232023f, 3.04439354f, 3.09353590f, 3.10493112f, 3.15644836f, 2.81437588f
		};

		sp_char text[2000];
		sp_uint index = ZERO_UINT;
		Matlab::convert(*mesh1, (Vec3*)meshCache, "mesh1", "red", text, index);
		Matlab::display(-50, 50, -50, 50, -50, 50, text, index);
		text[index] = END_OF_STRING;

		for (sp_uint i = 0; i < 18; i++)
			Assert::IsTrue(isCloseEnough(expected[i], bv[i]), L"Wrong value.", LINE_INFO());

		TestPhysic::unlock();
	}

	SP_TEST_METHOD(CLASS_NAME, buildDOP18GPU_2)
	{
		TestPhysic::lock();

		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		ObjModel model;
		model.load("resources\\models\\rocks\\obj.obj");

		SpMesh* mesh1 = sp_mem_new(SpMesh)();
		model.buildMesh(mesh1);
		Vec3 position(-2.0f, -3.0f, -4.0f);
		Quat orientation;
		Mat3 rotation = Mat3::createRotate(radians(60.0f), 0.0f, 0.0f, 1.0f);
		rotation.convert(orientation);

		SpWorldManagerInstance->current()->mesh(0, mesh1);
		SpWorldManagerInstance->current()->collisionFeatures(0, 0);
		SpWorldManagerInstance->current()->transforms(0)->position = position;
		SpWorldManagerInstance->current()->transforms(0)->orientation = orientation;
		SpWorldManagerInstance->current()->transforms(0)->scaleVector = Vec3(2.0f, 2.0f, 2.0f);

		sp_float meshCache[24 * 3];
		SpTransform* transformation = SpWorldManagerInstance->current()->transforms(0);
		mesh1->convert((Vec3*)meshCache, *transformation);

		sp_char filename[512];
		currentDirectory(filename, 512);
		directoryAddPath(filename, std::strlen(filename), SP_DIRECTORY_OPENCL_SOURCE, SP_DIRECTORY_OPENCL_SOURCE_LENGTH);
		directoryAddPath(filename, std::strlen(filename), "BoundingVolumeFactory.cl", 24);

		SP_FILE file;
		file.open(filename, std::ios::in);
		const sp_size fileSize = file.length();
		sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
		file.read(source, fileSize);
		file.close();

		cl_program program;
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, nullptr, &program);

		ALLOC_RELEASE(source);

		const sp_size globalWorkSize[3] = { 1, 0, 0 };
		const sp_size localWorkSize[3] = { 1, 0, 0 };

		sp_uint meshesLength = 1u;
		sp_uint meshCacheIndex = 0u;
		sp_uint meshCacheVertexesLength = mesh1->vertexLength();

		cl_mem meshCacheLengthGPU = gpu->createBuffer(&meshesLength, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem meshCacheIndexesGPU = gpu->createBuffer(&meshCacheIndex, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem meshCacheVertexesLengthGPU = gpu->createBuffer(&meshCacheVertexesLength, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem meshCacheGPU = gpu->createBuffer(meshCache, sizeof(Vec3) * mesh1->vertexLength(), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem transformsGPU = gpu->createBuffer(SpWorldManagerInstance->current()->transforms(0), sizeof(SpTransform), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem dop18GPU = gpu->createBuffer(sizeof(DOP18), CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		GpuCommand* command = gpu->commandManager
			->createCommand()
			->setInputParameter(meshCacheLengthGPU, sizeof(sp_uint))
			->setInputParameter(meshCacheIndexesGPU, sizeof(sp_uint))
			->setInputParameter(meshCacheVertexesLengthGPU, sizeof(sp_uint))
			->setInputParameter(meshCacheGPU, sizeof(Vec3) * mesh1->vertexLength())
			->setInputParameter(transformsGPU, sizeof(SpTransform))
			->setInputParameter(dop18GPU, sizeof(DOP18))
			->buildFromProgram(program, "buildDOP18");

		command->execute(1, globalWorkSize, localWorkSize, 0, 0, NULL, NULL);

		sp_float bv[18];
		command->fetchInOutParameter(bv, 5);

		sp_float expected[18] = {
			-3.91134977f, -4.91134977f, -6.00000000f, -4.86330795f, -4.86330795f, -6.48986292f, -6.48986292f, -4.48986292f, -4.48986292f,
			-0.08865010f, -1.08865011f, -2.00000000f,  0.86330795f,  0.86330795f, -1.51013708f, -1.51013708f,  0.48986291f,  0.48986291f
		};

		/* MatLab Viewer
		Vec3 n;
		normalize(DOP18_NORMALS[DOP18_PLANES_UP_LEFT_INDEX], n);
		Plane planeUpLeft(Vec3(bv[DOP18_AXIS_UP_LEFT], position.y, position.z), n);

		normalize(DOP18_NORMALS[DOP18_PLANES_DOWN_RIGHT_INDEX], n);
		Plane planeDownRight(Vec3(bv[DOP18_ORIENTATIONS + DOP18_AXIS_UP_LEFT], position.y, position.z), n);

		normalize(DOP18_NORMALS[DOP18_PLANES_UP_RIGHT_INDEX], n);
		Plane planeUpRight(Vec3(bv[DOP18_ORIENTATIONS + DOP18_AXIS_UP_RIGHT], position.y, position.z), n);

		normalize(DOP18_NORMALS[DOP18_PLANES_DOWN_LEFT_INDEX], n);
		Plane planeDownLeft(Vec3(bv[DOP18_AXIS_UP_RIGHT], position.y, position.z), n);

		normalize(DOP18_NORMALS[DOP18_PLANES_UP_FRONT_INDEX], n);
		Plane planeUpFront(Vec3(position.x, position.y, bv[DOP18_AXIS_UP_FRONT]), n);
		
		normalize(DOP18_NORMALS[DOP18_PLANES_DOWN_DEPTH_INDEX], n);
		Plane planeDownDepth(Vec3(position.x, position.y, bv[DOP18_ORIENTATIONS + DOP18_AXIS_UP_FRONT]), n);

		normalize(DOP18_NORMALS[DOP18_PLANES_UP_DEPTH_INDEX], n);
		Plane planeUpDepth(Vec3(position.x, position.y, bv[DOP18_AXIS_UP_DEPTH]), n);

		normalize(DOP18_NORMALS[DOP18_PLANES_DOWN_FRONT_INDEX], n);
		Plane planeDownFront(Vec3(position.x, position.y, bv[DOP18_ORIENTATIONS + DOP18_AXIS_UP_DEPTH]), n);

		normalize(DOP18_NORMALS[DOP18_PLANES_LEFT_DEPTH_INDEX], n);
		Plane planeLeftDepth(Vec3(bv[DOP18_AXIS_LEFT_DEPTH], position.y, position.z), n);

		normalize(DOP18_NORMALS[DOP18_PLANES_RIGHT_FRONT_INDEX], n);
		Plane planeRightFront(Vec3(bv[DOP18_ORIENTATIONS + DOP18_AXIS_LEFT_DEPTH], position.y, position.z), n);

		normalize(DOP18_NORMALS[DOP18_PLANES_RIGHT_DEPTH_INDEX], n);
		Plane planeRightDepth(Vec3(bv[DOP18_AXIS_RIGHT_DEPTH], position.y, position.z), n);

		normalize(DOP18_NORMALS[DOP18_PLANES_LEFT_FRONT_INDEX], n);
		Plane planeLeftFront(Vec3(bv[DOP18_ORIENTATIONS + DOP18_AXIS_RIGHT_DEPTH], position.y, position.z), n);

		sp_char text[5000];
		sp_uint index = ZERO_UINT;
		Matlab::convert(*mesh1, (Vec3*)meshCache, "mesh1", "red", text, index);

		//Matlab::convert(planeUpLeft, "planeUpLeft", "green", text, index);
		//Matlab::convert(planeUpRight, "planeUpRight", "green", text, index);
		//Matlab::convert(planeDownLeft, "planeDownLeft", "green", text, index);
		//Matlab::convert(planeDownRight, "planeDownRight", "green", text, index);
		// 
		//Matlab::convert(planeUpFront, "planeUpFront", "green", text, index);
		//Matlab::convert(planeDownDepth, "planeDownDepth", "green", text, index);
		//Matlab::convert(planeUpDepth, "planeUpDepth", "green", text, index);
		//Matlab::convert(planeDownFront, "planeDownFront", "green", text, index);

		Matlab::convert(planeLeftDepth, "planeLeftDepth", "green", text, index);
		Matlab::convert(planeRightFront, "planeRightFront", "green", text, index);
		Matlab::convert(planeRightDepth, "planeRightDepth", "green", text, index);
		Matlab::convert(planeLeftFront, "planeLeftFront", "green", text, index);

		Matlab::display(position.x - 5.0f, position.x + 5.0f, position.y - 5.0f, position.y + 5.0f, position.z - 5.0f, position.z + 5.0f, text, index);
		text[index] = END_OF_STRING;
		*/

		for (sp_uint i = 0; i < 18; i++)
			Assert::IsTrue(isCloseEnough(expected[i], bv[i]), L"Wrong value.", LINE_INFO());

		TestPhysic::unlock();
	}

	SP_TEST_METHOD(CLASS_NAME, buildDOP18GPU_3)
	{
		TestPhysic::lock();

		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		ObjModel model;
		model.load("resources\\models\\rocks\\obj.obj");

		SpMesh* mesh1 = sp_mem_new(SpMesh)();
		model.buildMesh(mesh1);
		Vec3 position(-2.0f, -3.0f, -4.0f);
		Quat orientation;
		Mat3 rotation = Mat3::createRotate(radians(60.0f), 1.0f, 1.0f, 0.0f);
		rotation.convert(orientation);

		SpWorldManagerInstance->current()->mesh(0, mesh1);
		SpWorldManagerInstance->current()->collisionFeatures(0, 0);
		SpWorldManagerInstance->current()->transforms(0)->position = position;
		SpWorldManagerInstance->current()->transforms(0)->orientation = orientation;
		SpWorldManagerInstance->current()->transforms(0)->scaleVector = Vec3(2.0f, 2.0f, 2.0f);

		sp_float meshCache[24 * 3];
		SpTransform* transformation = SpWorldManagerInstance->current()->transforms(0);
		mesh1->convert((Vec3*)meshCache, *transformation);

		sp_char filename[512];
		currentDirectory(filename, 512);
		directoryAddPath(filename, std::strlen(filename), SP_DIRECTORY_OPENCL_SOURCE, SP_DIRECTORY_OPENCL_SOURCE_LENGTH);
		directoryAddPath(filename, std::strlen(filename), "BoundingVolumeFactory.cl", 24);

		SP_FILE file;
		file.open(filename, std::ios::in);
		const sp_size fileSize = file.length();
		sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
		file.read(source, fileSize);
		file.close();

		cl_program program;
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, nullptr, &program);

		ALLOC_RELEASE(source);

		const sp_size globalWorkSize[3] = { 1, 0, 0 };
		const sp_size localWorkSize[3] = { 1, 0, 0 };

		sp_uint meshesLength = 1u;
		sp_uint meshCacheIndex = 0u;
		sp_uint meshCacheVertexesLength = mesh1->vertexLength();

		cl_mem meshCacheLengthGPU = gpu->createBuffer(&meshesLength, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem meshCacheIndexesGPU = gpu->createBuffer(&meshCacheIndex, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem meshCacheVertexesLengthGPU = gpu->createBuffer(&meshCacheVertexesLength, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem meshCacheGPU = gpu->createBuffer(meshCache, sizeof(Vec3) * mesh1->vertexLength(), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem transformsGPU = gpu->createBuffer(SpWorldManagerInstance->current()->transforms(0), sizeof(SpTransform), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem dop18GPU = gpu->createBuffer(sizeof(DOP18), CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		GpuCommand* command = gpu->commandManager
			->createCommand()
			->setInputParameter(meshCacheLengthGPU, sizeof(sp_uint))
			->setInputParameter(meshCacheIndexesGPU, sizeof(sp_uint))
			->setInputParameter(meshCacheVertexesLengthGPU, sizeof(sp_uint))
			->setInputParameter(meshCacheGPU, sizeof(Vec3) * mesh1->vertexLength())
			->setInputParameter(transformsGPU, sizeof(SpTransform))
			->setInputParameter(dop18GPU, sizeof(DOP18))
			->buildFromProgram(program, "buildDOP18");

		command->execute(1, globalWorkSize, localWorkSize, 0, 0, NULL, NULL);

		sp_float bv[18];
		command->fetchInOutParameter(bv, 5);

		sp_float expected[18] = {
			-3.80925512f, -4.80925512f, -5.62364531f, -4.80808830f, -4.35860348f, -6.58335400f, -6.89498663f, -4.89498663f, -4.58335400f,
			-0.19074499f, -1.19074500f, -2.37635469f,  0.80808830f,  0.35860323f, -1.41664624f, -1.10501337f,  0.89498662f,  0.58335375f
		};

		/*
		// MatLab Viewer
		Vec3 n;
		normalize(DOP18_NORMALS[DOP18_PLANES_UP_LEFT_INDEX], n);
		Plane planeUpLeft(Vec3(bv[DOP18_AXIS_UP_LEFT], position.y, position.z), n);

		normalize(DOP18_NORMALS[DOP18_PLANES_DOWN_RIGHT_INDEX], n);
		Plane planeDownRight(Vec3(bv[DOP18_ORIENTATIONS + DOP18_AXIS_UP_LEFT], position.y, position.z), n);

		normalize(DOP18_NORMALS[DOP18_PLANES_UP_RIGHT_INDEX], n);
		Plane planeUpRight(Vec3(bv[DOP18_AXIS_UP_RIGHT], position.y, position.z), n);

		normalize(DOP18_NORMALS[DOP18_PLANES_DOWN_LEFT_INDEX], n);
		Plane planeDownLeft(Vec3(bv[DOP18_ORIENTATIONS + DOP18_AXIS_UP_RIGHT], position.y, position.z), n);

		normalize(DOP18_NORMALS[DOP18_PLANES_UP_FRONT_INDEX], n);
		Plane planeUpFront(Vec3(position.x, position.y, bv[DOP18_AXIS_UP_FRONT]), n);

		normalize(DOP18_NORMALS[DOP18_PLANES_DOWN_DEPTH_INDEX], n);
		Plane planeDownDepth(Vec3(position.x, position.y, bv[DOP18_ORIENTATIONS + DOP18_AXIS_UP_FRONT]), n);

		normalize(DOP18_NORMALS[DOP18_PLANES_UP_DEPTH_INDEX], n);
		Plane planeUpDepth(Vec3(position.x, position.y, bv[DOP18_AXIS_UP_DEPTH]), n);

		normalize(DOP18_NORMALS[DOP18_PLANES_DOWN_FRONT_INDEX], n);
		Plane planeDownFront(Vec3(position.x, position.y, bv[DOP18_ORIENTATIONS + DOP18_AXIS_UP_DEPTH]), n);

		normalize(DOP18_NORMALS[DOP18_PLANES_LEFT_DEPTH_INDEX], n);
		Plane planeLeftDepth(Vec3(bv[DOP18_AXIS_LEFT_DEPTH], position.y, position.z), n);

		normalize(DOP18_NORMALS[DOP18_PLANES_RIGHT_FRONT_INDEX], n);
		Plane planeRightFront(Vec3(bv[DOP18_ORIENTATIONS + DOP18_AXIS_LEFT_DEPTH], position.y, position.z), n);

		normalize(DOP18_NORMALS[DOP18_PLANES_RIGHT_DEPTH_INDEX], n);
		Plane planeRightDepth(Vec3(bv[DOP18_AXIS_RIGHT_DEPTH], position.y, position.z), n);

		normalize(DOP18_NORMALS[DOP18_PLANES_LEFT_FRONT_INDEX], n);
		Plane planeLeftFront(Vec3(bv[DOP18_ORIENTATIONS + DOP18_AXIS_RIGHT_DEPTH], position.y, position.z), n);

		sp_char text[5000];
		sp_uint index = ZERO_UINT;
		Matlab::convert(*mesh1, (Vec3*)meshCache, "mesh1", "red", text, index);

		//Matlab::convert(planeUpLeft, "planeUpLeft", "green", text, index);
		//Matlab::convert(planeUpRight, "planeUpRight", "green", text, index);
		//Matlab::convert(planeDownLeft, "planeDownLeft", "green", text, index);
		//Matlab::convert(planeDownRight, "planeDownRight", "green", text, index);
		
		//Matlab::convert(planeUpFront, "planeUpFront", "green", text, index);
		//Matlab::convert(planeDownDepth, "planeDownDepth", "green", text, index);
		//Matlab::convert(planeUpDepth, "planeUpDepth", "green", text, index);
		//Matlab::convert(planeDownFront, "planeDownFront", "green", text, index);

		Matlab::convert(planeLeftDepth, "planeLeftDepth", "green", text, index);
		Matlab::convert(planeRightFront, "planeRightFront", "green", text, index);
		Matlab::convert(planeRightDepth, "planeRightDepth", "green", text, index);
		Matlab::convert(planeLeftFront, "planeLeftFront", "green", text, index);

		Matlab::display(position.x - 5.0f, position.x + 5.0f, position.y - 5.0f, position.y + 5.0f, position.z - 5.0f, position.z + 5.0f, text, index);
		text[index] = END_OF_STRING;
		*/

		for (sp_uint i = 0; i < 18; i++)
			Assert::IsTrue(isCloseEnough(expected[i], bv[i]), L"Wrong value.", LINE_INFO());

		TestPhysic::unlock();
	}

	SP_TEST_METHOD(CLASS_NAME, buildDOP18GPU_4)
	{
		TestPhysic::lock();

		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		ObjModel model;
		model.load("resources\\models\\rocks\\obj.obj");

		SpMesh* mesh1 = sp_mem_new(SpMesh)();
		model.buildMesh(mesh1);
		Vec3 position = Vec3Zeros;
		Quat orientation = Quat::createRotationAxisX(radians(90));
		Vec3 scale = Vec3(2.0f, 6.0f, 2.0f);

		SpWorldManagerInstance->current()->mesh(0, mesh1);
		SpWorldManagerInstance->current()->collisionFeatures(0, 0);
		SpWorldManagerInstance->current()->transforms(0)->position = position;
		SpWorldManagerInstance->current()->transforms(0)->orientation = orientation;
		SpWorldManagerInstance->current()->transforms(0)->scaleVector = scale;

		sp_float meshCache[24 * 3];
		SpTransform* transformation = SpWorldManagerInstance->current()->transforms(0);
		mesh1->convert((Vec3*)meshCache, *transformation);

		sp_char filename[512];
		currentDirectory(filename, 512);
		directoryAddPath(filename, std::strlen(filename), SP_DIRECTORY_OPENCL_SOURCE, SP_DIRECTORY_OPENCL_SOURCE_LENGTH);
		directoryAddPath(filename, std::strlen(filename), "BoundingVolumeFactory.cl", 24);

		SP_FILE file;
		file.open(filename, std::ios::in);
		const sp_size fileSize = file.length();
		sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
		file.read(source, fileSize);
		file.close();

		cl_program program;
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, nullptr, &program);

		ALLOC_RELEASE(source);

		const sp_size globalWorkSize[3] = { 1, 0, 0 };
		const sp_size localWorkSize[3] = { 1, 0, 0 };

		sp_uint meshesLength = 1u;
		sp_uint meshCacheIndex = 0u;
		sp_uint meshCacheVertexesLength = mesh1->vertexLength();

		cl_mem meshCacheLengthGPU = gpu->createBuffer(&meshesLength, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem meshCacheIndexesGPU = gpu->createBuffer(&meshCacheIndex, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem meshCacheVertexesLengthGPU = gpu->createBuffer(&meshCacheVertexesLength, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem meshCacheGPU = gpu->createBuffer(meshCache, sizeof(Vec3) * mesh1->vertexLength(), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem transformsGPU = gpu->createBuffer(SpWorldManagerInstance->current()->transforms(0), sizeof(SpTransform), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem dop18GPU = gpu->createBuffer(sizeof(DOP18), CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		GpuCommand* command = gpu->commandManager
			->createCommand()
			->setInputParameter(meshCacheLengthGPU, sizeof(sp_uint))
			->setInputParameter(meshCacheIndexesGPU, sizeof(sp_uint))
			->setInputParameter(meshCacheVertexesLengthGPU, sizeof(sp_uint))
			->setInputParameter(meshCacheGPU, sizeof(Vec3) * mesh1->vertexLength())
			->setInputParameter(transformsGPU, sizeof(SpTransform))
			->setInputParameter(dop18GPU, sizeof(DOP18))
			->buildFromProgram(program, "buildDOP18");

		command->execute(1, globalWorkSize, localWorkSize, 0, 0, NULL, NULL);

		sp_float bv[18];
		command->fetchInOutParameter(bv, 5);

		Assert::IsTrue(isCloseEnough(-6.0f, bv[DOP18_AXIS_Z]), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(6.0f, bv[DOP18_AXIS_Z + DOP18_ORIENTATIONS]), L"Wrong value.", LINE_INFO());

		TestPhysic::unlock();
	}
#endif

	SP_TEST_METHOD(CLASS_NAME, build)
	{
		TestPhysic::lock();

		SpWorld* world = SpWorldManagerInstance->current();

		createMeshes2(0u);
		resetObject(0u);

		SpTransform* transform = world->transforms(0u);
		transform->scaleVector *= 2.0f;
		transform->position = Vec3(2.0f, 0.0f, 0.0f);

		SpMesh* mesh = world->mesh(0u);
		SpMeshCache* cache = ALLOC_NEW(SpMeshCache)(mesh->vertexesMesh->length());
		cache->update(mesh, transform);

		DOP18* result = world->boundingVolumes(0u);

		SpDOP18Factory dop18Factory;
		dop18Factory.build(mesh, cache, *transform, result);
		
		Assert::IsTrue(isCloseEnough(result->max[0], 4.0f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->max[1], 2.0f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->max[2], 2.0f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->max[DOP18_AXIS_UP_LEFT], 4.3586f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->max[DOP18_AXIS_UP_RIGHT], 4.3586f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->max[DOP18_AXIS_UP_FRONT], 2.3586f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->max[DOP18_AXIS_UP_DEPTH], 2.3586f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->max[DOP18_AXIS_LEFT_DEPTH], 4.3586f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->max[DOP18_AXIS_RIGHT_DEPTH], 4.3586f), L"Wrong value.", LINE_INFO());

		Assert::IsTrue(isCloseEnough(result->min[0], 0.0f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->min[1], -2.0f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->min[2], -2.0f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->min[DOP18_AXIS_UP_LEFT], -0.3586f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->min[DOP18_AXIS_UP_RIGHT], -0.3586f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->min[DOP18_AXIS_UP_FRONT], -2.3586f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->min[DOP18_AXIS_UP_DEPTH], -2.3586f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->min[DOP18_AXIS_LEFT_DEPTH], -0.3586f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->min[DOP18_AXIS_RIGHT_DEPTH], -0.3586f), L"Wrong value.", LINE_INFO());

		TestPhysic::unlock();
	}

}

#undef CLASS_NAME
