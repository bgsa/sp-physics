#include "SpectrumPhysicsTest.h"
#include <ObjModel.h>
#include <SpWorldManager.h>

#define CLASS_NAME SpMeshCacheTest

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

#ifdef OPENCL_ENABLED
		SP_TEST_METHOD_DEF(updateGPU_1);
#endif

	};

#ifdef OPENCL_ENABLED
	SP_TEST_METHOD(CLASS_NAME, updateGPU_1)
	{
		GpuContext* context = GpuContext::init();
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

		SpEdgeMesh* lastEdge = mesh1->edges->data()[mesh1->edges->length() - 1u];
		sp_size lastMemoryAddress = (sp_size)&lastEdge->faces.data()[lastEdge->faces.length() - 1u];
		sp_uint mesh1Size = lastMemoryAddress - ((sp_size)mesh1);

		SpDirectory* filename = SpDirectory::currentDirectory()
			->add(SP_DIRECTORY_OPENCL_SOURCE)
			->add("SpMeshCache.cl");

		SP_FILE file;
		file.open(filename->name()->data(), std::ios::in);
		const sp_size fileSize = file.length();
		sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
		file.read(source, fileSize);
		file.close();

		sp_mem_delete(filename, SpDirectory);

		cl_program program;
		gpu->commandManager->buildProgram(source, SIZEOF_CHAR * fileSize, nullptr, &program);

		ALLOC_RELEASE(source);

		const sp_size globalWorkSize[3] = { 1, 0, 0 };
		const sp_size localWorkSize[3] = { 1, 0, 0 };

		sp_uint indexesLength = 1u;
		sp_uint meshIndex = 0u;
		sp_uint vertexesLength = mesh1->vertexLength();
		sp_uint vertexMeshIndex = 36928u;

		cl_mem meshesGPU = gpu->createBuffer(mesh1, mesh1Size, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem indexesLengthGPU = gpu->createBuffer(&indexesLength, SIZEOF_UINT, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem meshIndexGPU = gpu->createBuffer(&vertexMeshIndex, SIZEOF_UINT, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem meshIndexesLengthGPU = gpu->createBuffer(&vertexesLength, SIZEOF_UINT, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem transformsGPU = gpu->createBuffer(SpWorldManagerInstance->current()->transforms(0), sizeof(SpTransform), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem meshCacheIndexesGPU = gpu->createBuffer(&meshIndex, SIZEOF_UINT, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		sp_uint meshCacheSize = sizeof(Vec3) * mesh1->vertexLength() * sizeof(sp_float);
		cl_mem meshCacheGPU = gpu->createBuffer(meshCacheSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		GpuCommand* command = gpu->commandManager
			->createCommand()
			->setInputParameter(indexesLengthGPU, SIZEOF_UINT)
			->setInputParameter(meshesGPU, mesh1Size)
			->setInputParameter(meshIndexGPU, SIZEOF_UINT)
			->setInputParameter(meshIndexesLengthGPU, SIZEOF_UINT)
			->setInputParameter(transformsGPU, sizeof(SpTransform))
			->setInputParameter(meshCacheIndexesGPU, SIZEOF_UINT)
			->setInputParameter(meshCacheGPU, meshCacheSize)
			->buildFromProgram(program, "update");

		command->execute(1, globalWorkSize, localWorkSize, 0, 0, NULL, NULL);

		sp_float* vertexes = ALLOC_ARRAY(sp_float, mesh1->vertexLength());
		command->fetchInOutParameter(vertexes, 6);

		//for (int i = 0; i < 3; i++)
		//	Assert::AreEqual(1, 1, L"Wrong value.", LINE_INFO());

		/*
BV (176)
-1.44988
-37.2523
-1.40542
-2.4189
-2.40098
-2.36195
-2.37335
-2.51303
-2.17096
2.0933
1.17549e-38
2.13701
3.06232
3.04439
3.09354
3.10493
3.15645
2.81438

MeshCache:
0x22B5FEF0       0.997765303  Œm.?
0x22B5FEF4       -37.0503998  œ3.Â
0x22B5FEF8      -0.798262239  êZL¿
0x22B5FEFC        1.32016361  .û¨?
0x22B5FF00       -36.5010834  ...Â
0x22B5FF04       -1.12797058  Wa.¿
0x22B5FF08        1.82751310  óëé?
0x22B5FF0C       -36.4946022  yú.Â
0x22B5FF10      -0.621070802  .þ.¿
0x22B5FF14        1.50511503  œ§À?
0x22B5FF18       -37.0439148  ø,.Â
0x22B5FF1C      -0.291362464  v-..
0x22B5FF20      -0.128041685  \...
0x22B5FF24       -37.2458153  ·û.Â
0x22B5FF28        1.34582543  .D¬?
0x22B5FF2C      -0.519256234  úí.¿
0x22B5FF30       -36.7847252  .#.Â
0x22B5FF34        1.73149133  .¡Ý?
0x22B5FF38       -1.02660596  Ógƒ¿
0x22B5FF3C       -36.7912064  2*.Â
0x22B5FF40        1.22459149  j¿œ?
0x22B5FF44      -0.635391235  .©"¿
0x22B5FF48       -37.2522964  Z..Â
0x22B5FF4C       0.838925719  ÖÃV?
0x22B5FF50        2.09329700  ”ø.@
0x22B5FF54       -35.4245224  ¶..Â
0x22B5FF58        1.42163301  .øµ?
0x22B5FF5C        1.70208240  ÖÝÙ?
0x22B5FF60       -34.9634323  ŽÚ.Â
0x22B5FF64        1.80729878  ‘Uç?
0x22B5FF68        1.37968433  .™°?
0x22B5FF6C       -35.5127449  ...Â
0x22B5FF70        2.13700724  ºÄ.@
0x22B5FF74        1.77089894  Ñ¬â?
0x22B5FF78       -35.9738388  6å.Â
0x22B5FF7C        1.75134158  ö+à?
0x22B5FF80      -0.861699462  V˜\¿
0x22B5FF84       -33.9295044  Ð·.Â
0x22B5FF88        1.02294576  ãï.?
0x22B5FF8C       -1.18409765  ƒ.—¿
0x22B5FF90       -34.4788170  Oê.Â
0x22B5FF94        1.35265422  Æ#.?
0x22B5FF98      -0.676748037  \?-¿
0x22B5FF9C       -34.4723358  ¬ã.Â
0x22B5FFA0        1.85955393  Ý.î?
0x22B5FFA4      -0.354349941  [mµ.
0x22B5FFA8       -33.9230194  ,±.Â
0x22B5FFAC        1.52984560  ûÑÃ?
0x22B5FFB0        1.16267180  nÒ”?
0x22B5FFB4       -34.1886940  9Á.Â
0x22B5FFB8      -0.999907970  øù.¿
0x22B5FFBC       0.771457195  8~E?
0x22B5FFC0       -33.7276039  .é.Â
0x22B5FFC4      -0.614242136  ù>.¿
0x22B5FFC8        1.27880681  ñ¯£?
0x22B5FFCC       -33.7211227  nâ.Â
0x22B5FFD0      -0.107342362  PÖÛ.
0x22B5FFD4        1.67002141  CÃÕ?
0x22B5FFD8       -34.1822128  –º.Â
0x22B5FFDC      -0.493008137  .kü.
0x22B5FFE0       -1.12748349  aQ.¿
0x22B5FFE4       -34.9995804  ’ÿ.Â
0x22B5FFE8       -1.01975811  o..¿
0x22B5FFEC      -0.736268878  .|<¿
0x22B5FFF0       -35.4606743  »×.Â
0x22B5FFF4       -1.40542376  íä.¿
0x22B5FFF8       -1.44988167  ...¿
0x22B5FFFC       -35.5488968  .2.Â
0x22B60000      -0.690049767  .§0¿
0x22B60004       -1.05866706  g..¿
0x22B60008       -36.0099869  :..Â
0x22B6000C       -1.07571542  .±.¿
		*/

		ALLOC_RELEASE(vertexes);
	}
#endif
	
}

#undef CLASS_NAME
