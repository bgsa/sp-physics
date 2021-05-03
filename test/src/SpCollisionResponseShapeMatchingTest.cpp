#include "SpectrumPhysicsTest.h"
#include <SpCollisionResponseShapeMatching.h>
#include <ObjModel.h>

#define CLASS_NAME SpCollisionResponseShapeMatchingTest

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		SP_TEST_METHOD_DEF(test1);
		SP_TEST_METHOD_DEF(test2);
	};

	SP_TEST_METHOD(CLASS_NAME, test1)
	{
		SpMesh* mesh0 = sp_mem_new(SpMesh)();
		mesh0->vertexesMesh = sp_mem_new(SpArray<SpVertexMesh*>)(4);
		mesh0->faces = sp_mem_new(SpArray<SpFaceMesh*>)(2);
		mesh0->vertexesMesh->add(sp_mem_new(SpVertexMesh)(mesh0, 0, Vec3(-50.0f, 0.0f, 50.0f)));
		mesh0->vertexesMesh->add(sp_mem_new(SpVertexMesh)(mesh0, 1, Vec3(50.0f, 0.0f, 50.0f)));
		mesh0->vertexesMesh->add(sp_mem_new(SpVertexMesh)(mesh0, 2, Vec3(50.0f, 0.0f, -50.0f)));
		mesh0->vertexesMesh->add(sp_mem_new(SpVertexMesh)(mesh0, 3, Vec3(-50.0f, 0.0f, -50.0f)));
		mesh0->faces->add(sp_mem_new(SpFaceMesh)(mesh0, 0, 0, 1, 2));
		mesh0->faces->add(sp_mem_new(SpFaceMesh)(mesh0, 1, 2, 3, 0));
		mesh0->init();
		SpWorldManagerInstance->current()->mesh(0, mesh0);
		SpWorldManagerInstance->current()->collisionFeatures(0, 0);
		SpWorldManagerInstance->current()->transforms(0)->position = Vec3Zeros;
		SpWorldManagerInstance->current()->transforms(0)->orientation = QUAT_UNIT;

		ObjModel model;
		model.load("resources\\models\\rocks\\obj.obj");

		SpMesh* mesh1 = sp_mem_new(SpMesh)();
		model.buildMesh(mesh1);
		SpWorldManagerInstance->current()->mesh(1, mesh1);
		SpWorldManagerInstance->current()->collisionFeatures(1, 1);
		SpWorldManagerInstance->current()->transforms(1)->position = Vec3(0.0f, 0.869930f, 0.197710f);
		SpWorldManagerInstance->current()->transforms(1)->orientation = Quat(0.999994f, 0.003392f, 0.00000f, 0.00000f);
		SpWorldManagerInstance->current()->transforms(1)->scaleVector = Vec3(2.0f, 2.0f, 2.0f);
		SpWorldManagerInstance->current()->rigidBody3D(1)->mass(1.1f);

		SpMesh* mesh2 = sp_mem_new(SpMesh)();
		model.buildMesh(mesh2);
		SpWorldManagerInstance->current()->mesh(2, mesh2);
		SpWorldManagerInstance->current()->collisionFeatures(2, 2);
		SpWorldManagerInstance->current()->transforms(2)->position = Vec3(0.0f, 4.538721f, -0.17933f);
		SpWorldManagerInstance->current()->transforms(2)->orientation = Quat(0.85078f, 0.292525f, 0.402729f, 0.168555f);
		SpWorldManagerInstance->current()->transforms(2)->scaleVector = Vec3(2.0f, 2.0f, 2.0f);
		SpWorldManagerInstance->current()->rigidBody3D(2)->mass(1.1f);

		/*
		//shape 1
		Particles :
		[0] : -1.53368, -1.01790, -0.30933,
		[1] : -0.83250, -1.16867, -0.31040,
		[2] : -0.82978, -1.16112, 0.406763,
		[3] : -1.53097, -1.01035, 0.407830,
		[4] : -2.79068, 0.939323, 0.392056,
		[5] : -2.63993, 1.640468, 0.384096,
		[6] : -2.64265, 1.632910, -0.33307,
		[7] : -2.79340, 0.931765, -0.32511,
		[8] : -0.47856, 0.460799, 2.029838,
		[9] : -0.32781, 1.161945, 2.021878,
		[10] : -1.02900, 1.312715, 2.022945,
		[11] : -1.17975, 0.611570, 2.030905,
		[12] : 0.008252, 2.741714, -0.35479,
		[13] : -0.69293, 2.892485, -0.35372,
		[14] : -0.69021, 2.900042, 0.363438,
		[15] : 0.010968, 2.749272, 0.362371,
		[16] : 1.117215, 0.090896, -0.33106,
		[17] : 1.267965, 0.792042, -0.33902,
		[18] : 1.270681, 0.799599, 0.378145,
		[19] : 1.119931, 0.098454, 0.386105,
		[20] : -0.34296, 1.119795, -1.97787,
		[21] : -0.49371, 0.418650, -1.96991,
		[22] : -1.04415, 1.270565, -1.97680,
		[23] : -1.19490, 0.569420, -1.96884,

		//shape 2
		Particles:
		[0]: 1.231079, 2.091722, -0.35762,
		[1]: 1.550547, 2.381223, -0.93079,
		[2]: 2.114935, 2.559969, -0.52593,
		[3]: 1.795467, 2.270467, 0.047235,
		[4]: 0.363427, 3.052866, 1.698132,
		[5]: 0.057160, 3.684239, 1.846328,
		[6]: -0.50722, 3.505493, 1.441471,
		[7]: -0.20096, 2.874120, 1.293275,
		[8]: 2.705666, 4.413988, 0.739768,
		[9]: 2.399399, 5.045361, 0.887964,
		[10]: 2.079931, 4.755859, 1.461133,
		[11]: 2.386199, 4.124486, 1.312937,
		[12]: -0.15754, 5.902486, -0.10428,
		[13]: -0.47701, 5.612985, 0.468888,
		[14]: 0.087371, 5.791730, 0.873745,
		[15]: 0.406839, 6.081232, 0.300576,
		[16]: 1.580759, 4.488715, -1.90337,
		[17]: 1.274492, 5.120088, -1.75517,
		[18]: 1.838880, 5.298834, -1.35032,
		[19]: 2.145147, 4.667460, -1.49851,
		[20]: -0.74828, 4.048468, -1.36998,
		[21]: -0.44201, 3.417095, -1.51817,
		[22]: -1.06774, 3.758966, -0.79681,
		[23]: -0.76148, 3.127593, -0.94501,
		*/

		SpCollisionResponseShapeMatching shapeMatching;

		SpRigidBodyShapeMatch shape0, shape1, shape2;
		shapeMatching.initShape(0, &shape0);
		shapeMatching.initShape(1, &shape1);
		shapeMatching.initShape(2, &shape2);

		sp_log_debug1snl("Initial Particles 1");
		for (sp_uint i = 0; i < shape1.particlesLength; i++)
		{
			sp_log_debug1s("Particle["); sp_log_debug1u(i); sp_log_debug1s("]: ");
			sp_log_debugXf(shape1.initialParticles[i], 3);  sp_log_newline();
		}
		sp_log_debug1snl("Initial Particles 2");
		for (sp_uint i = 0; i < shape2.particlesLength; i++)
		{
			sp_log_debug1s("Particle["); sp_log_debug1u(i); sp_log_debug1s("]: ");
			sp_log_debugXf(shape2.initialParticles[i], 3);  sp_log_newline();
		}

		shapeMatching.solve(&shape1, &shape0);
		shapeMatching.solve(&shape1, &shape2);

		// Assert::AreEqual(Vec3(-0.5f, -0.5f, -0.5f)[i], aabb.minPoint[i], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, test2)
	{
		/*
		Init Shape 8
		Position: -0.99112, 0.591480, 3.231639,
		Orientation : 0.926000, 0.147400, 0.346865, 0.021980,
		Particles :
		[0] : -1.61888, -1.27852, 2.626485,
		[1] : -1.07494, -1.17598, 2.170400,
		[2] : -0.60956, -1.36083, 2.683864,
		[3] : -1.15350, -1.46337, 3.139949,
		[4] : -2.29731, -0.12954, 4.656836,
		[5] : -2.25317, 0.555807, 4.863559,
		[6] : -2.71855, 0.740658, 4.350095,
		[7] : -2.76269, 0.055304, 4.143372,
		[8] : 0.556513, -0.21540, 4.332068,
		[9] : 0.600656, 0.469954, 4.538792,
		[10] : 0.056719, 0.367419, 4.994876,
		[11] : 0.012576, -0.31793, 4.788153,
		[12] : -0.82875, 2.646335, 3.323328,
		[13] : -1.37269, 2.543800, 3.779413,
		[14] : -0.90731, 2.358949, 4.292877,
		[15] : -0.36337, 2.461483, 3.836793,
		[16] : 0.270917, 0.627154, 1.599718,
		[17] : 0.315060, 1.312508, 1.806442,
		[18] : 0.780439, 1.127657, 2.319906,
		[19] : 0.736296, 0.442303, 2.113183,
		[20] : -1.99483, 1.500896, 1.675125,
		[21] : -2.03897, 0.815542, 1.468401,
		[22] : -2.53877, 1.398361, 2.131209,
		[23] : -2.58291, 0.713007, 1.924485,

		Init Shape 12
		Position : -1.65231, 0.281097, 2.293869,
		Orientation : 0.870596, -0.36217, 0.279794, -0.18056,
		Particles :
		[0] : -2.37644, -1.06828, 3.676621,
		[1] : -1.81829, -1.43913, 3.421020,
		[2] : -1.37507, -1.05930, 3.837779,
		[3] : -1.93323, -0.68846, 4.093381,
		[4] : -3.02721, 1.264000, 3.477394,
		[5] : -2.94708, 1.746286, 2.952635,
		[6] : -3.39030, 1.366463, 2.535875,
		[7] : -3.47043, 0.884176, 3.060634,
		[8] : -0.17736, 0.913696, 3.590614,
		[9] : -0.09723, 1.395982, 3.065855,
		[10] : -0.65538, 1.766829, 3.321456,
		[11] : -0.73551, 1.284543, 3.846216,
		[12] : -1.37139, 1.250653, 0.494358,
		[13] : -1.92954, 1.621500, 0.749960,
		[14] : -1.48633, 2.001323, 1.166719,
		[15] : -0.92817, 1.630476, 0.911117,
		[16] : -0.35753, -1.18409, 1.635104,
		[17] : -0.27740, -0.70180, 1.110345,
		[18] : 0.165809, -0.32198, 1.527105,
		[19] : 0.085679, -0.80427, 2.051864,
		[20] : -2.56910, -0.72235, 0.741523,
		[21] : -2.64923, -1.20463, 1.266283,
		[22] : -3.12725, -0.35150, 0.997125,
		[23] : -3.20738, -0.83378, 1.521884,

		Solving collision pair(8, 12)
		Collision Normal : -0.04389, 0.150395, 0.987835,
		Collision Depth : 2.361339
		Particles Update 8
		Particle[0] : -1.79060, -0.69012, 6.491259,
		Particle[1] : -1.26681, -0.51858, 6.488416,
		Particle[2] : -0.78129, -0.77244, 6.548638,
		Particle[3] : -1.30508, -0.94397, 6.551480,
		Particle[4] : -2.37215, 0.126867, 6.341035,
		Particle[5] : -2.31461, 0.766315, 6.246238,
		Particle[6] : -2.80013, 1.020171, 6.186015,
		Particle[7] : -2.85767, 0.380722, 6.280813,
		Particle[8] : 0.461540, 0.110018, 6.469509,
		Particle[9] : 0.519080, 0.749467, 6.374712,
		Particle[10] : -0.00471, 0.577927, 6.377555,
		Particle[11] : -0.06225, -0.06152, 6.472352,
		Particle[12] : -0.94589, 3.047716, 5.959717,
		Particle[13] : -1.46969, 2.876177, 5.962560,
		Particle[14] : -0.98417, 2.622321, 6.022782,
		Particle[15] : -0.46037, 2.793860, 6.019939,
		Particle[16] : 0.063626, 1.337422, 6.264960,
		Particle[17] : 0.121166, 1.976871, 6.170163,
		Particle[18] : 0.606685, 1.723015, 6.230385,
		Particle[19] : 0.549144, 1.083567, 6.325182,
		Particle[20] : -2.18872, 2.165258, 6.038846,
		Particle[21] : -2.24626, 1.525810, 6.133643,
		Particle[22] : -2.71252, 1.993719, 6.041689,
		Particle[23] : -2.77006, 1.354271, 6.136485,

		Shape Match 8
		Particles :
		[0] - 1.68502, 1.772566, 19.28509,
		[1] - 1.91432, 3.453073, 30.14941,
		[2] - 1.24334, 1.930590, 22.36809,
		[3] - 1.01405, 0.250082, 11.50376,
		[4] - 0.21133, -4.25220, -25.3184,
		[5] - 0.08987, -4.53898, -30.5436,
		[6] - 0.76084, -3.01650, -22.7623,
		[7] - 0.88230, -2.72972, -17.5371,
		[8] 0.570191, -2.21004, -7.39844,
		[9] 0.691649, -2.49682, -12.6236,
		[10] 0.920943, -4.17733, -23.4880,
		[11] 0.799485, -3.89055, -18.2627,
		[12] - 1.23693, 1.853655, 1.007438,
		[13] - 1.00763, 0.173147, -9.85689,
		[14] - 0.33666, -1.34933, -17.6382,
		[15] - 0.56596, 0.331172, -6.77389,
		[16] - 2.16111, 6.642724, 43.05488,
		[17] - 2.03965, 6.355944, 37.82963,
		[18] - 1.36868, 4.833460, 30.04830,
		[19] - 1.49013, 5.120240, 35.27355,
		[20] - 3.05047, 5.994293, 30.77397,
		[21] - 3.17192, 6.281074, 35.99921,
		[22] - 2.82117, 4.313786, 19.90964,
		[23] - 2.94263, 4.600566, 25.13489,

		Particles Update 12
		Particle[0]: -0.76228, -6.59907, -32.6512,
		Particle[1] : -0.21873, -6.91989, -32.5782,
		Particle[2] : 0.244195, -6.60763, -32.6051,
		Particle[3] : -0.29935, -6.28681, -32.6781,
		Particle[4] : -1.40505, -4.29423, -33.0306,
		Particle[5] : -1.34463, -3.74439, -33.1117,
		Particle[6] : -1.80756, -4.05665, -33.0847,
		Particle[7] : -1.86798, -4.60650, -33.0037,
		Particle[8] : 1.441909, -4.63462, -32.8523,
		Particle[9] : 1.502323, -4.08478, -32.9333,
		Particle[10] : 0.958772, -3.76396, -33.0063,
		Particle[11] : 0.898358, -4.31380, -32.9253,
		Particle[12] : 0.118203, -3.85334, -33.0301,
		Particle[13] : -0.42534, -3.53252, -33.1031,
		Particle[14] : 0.037583, -3.22025, -33.1300,
		Particle[15] : 0.581134, -3.54107, -33.0570,
		Particle[16] : 1.163485, -6.39575, -32.5965,
		Particle[17] : 1.223899, -5.84591, -32.6776,
		Particle[18] : 1.686829, -5.53364, -32.7045,
		Particle[19] : 1.626415, -6.08349, -32.6235,
		Particle[20] : -1.07951, -5.82634, -32.7829,
		Particle[21] : -1.13992, -6.37618, -32.7019,
		Particle[22] : -1.62306, -5.50552, -32.8559,
		Particle[23] : -1.68347, -6.05536, -32.7749,

		Shape Match 12
		Particles :
		[0] - 0.76226, -6.59963, -32.6558,
		[1] - 0.21864, -6.92052, -32.5837,
		[2] 0.244353, -6.60821, -32.6103,
		[3] - 0.29927, -6.28732, -32.6824,
		[4] - 1.40521, -4.29412, -33.0289,
		[5] - 1.34483, -3.74407, -33.1082,
		[6] - 1.80782, -4.05638, -33.0816,
		[7] - 1.86821, -4.60643, -33.0023,
		[8] 1.442140, -4.63464, -32.8527,
		[9] 1.502523, -4.08460, -32.9320,
		[10] 0.958896, -3.76371, -33.0041,
		[11] 0.898513, -4.31376, -32.9248,
		[12] 0.118122, -3.85282, -33.0258,
		[13] - 0.42550, -3.53194, -33.0979,
		[14] 0.037489, -3.21963, -33.1245,
		[15] 0.581116, -3.54051, -33.0524,
		[16] 1.163682, -6.39607, -32.6000,
		[17] 1.224065, -5.84602, -32.6793,
		[18] 1.687059, -5.53371, -32.7059,
		[19] 1.626676, -6.08376, -32.6266,
		[20] - 1.07966, -5.82639, -32.7834,
		[21] - 1.14004, -6.37643, -32.7041,
		[22] - 1.62329, -5.50550, -32.8555,
		[23] - 1.68367, -6.05555, -32.7762,
		*/

		SpMesh* mesh0 = sp_mem_new(SpMesh)();
		mesh0->vertexesMesh = sp_mem_new(SpArray<SpVertexMesh*>)(4);
		mesh0->faces = sp_mem_new(SpArray<SpFaceMesh*>)(2);
		mesh0->vertexesMesh->add(sp_mem_new(SpVertexMesh)(mesh0, 0, Vec3(-50.0f, 0.0f, 50.0f)));
		mesh0->vertexesMesh->add(sp_mem_new(SpVertexMesh)(mesh0, 1, Vec3(50.0f, 0.0f, 50.0f)));
		mesh0->vertexesMesh->add(sp_mem_new(SpVertexMesh)(mesh0, 2, Vec3(50.0f, 0.0f, -50.0f)));
		mesh0->vertexesMesh->add(sp_mem_new(SpVertexMesh)(mesh0, 3, Vec3(-50.0f, 0.0f, -50.0f)));
		mesh0->faces->add(sp_mem_new(SpFaceMesh)(mesh0, 0, 0, 1, 2));
		mesh0->faces->add(sp_mem_new(SpFaceMesh)(mesh0, 1, 2, 3, 0));
		mesh0->init();
		SpWorldManagerInstance->current()->mesh(0, mesh0);
		SpWorldManagerInstance->current()->collisionFeatures(0, 0);
		SpWorldManagerInstance->current()->transforms(0)->position = Vec3Zeros;
		SpWorldManagerInstance->current()->transforms(0)->orientation = QUAT_UNIT;

		ObjModel model;
		model.load("resources\\models\\rocks\\obj.obj");

		SpMesh* mesh8 = sp_mem_new(SpMesh)();
		model.buildMesh(mesh8);
		SpWorldManagerInstance->current()->mesh(8, mesh8);
		SpWorldManagerInstance->current()->collisionFeatures(8, 8);
		SpWorldManagerInstance->current()->transforms(8)->position = Vec3(-0.99112f, 0.591480f, 3.231639f);
		SpWorldManagerInstance->current()->transforms(8)->orientation = Quat(0.926f, 0.1474f, 0.346865f, 0.02198f);
		SpWorldManagerInstance->current()->transforms(8)->scaleVector = Vec3(2.0f, 2.0f, 2.0f);
		SpWorldManagerInstance->current()->rigidBody3D(8)->mass(1.0f);

		SpMesh* mesh12 = sp_mem_new(SpMesh)();
		model.buildMesh(mesh12);
		SpWorldManagerInstance->current()->mesh(12, mesh12);
		SpWorldManagerInstance->current()->collisionFeatures(12, 12);
		SpWorldManagerInstance->current()->transforms(12)->position = Vec3(-1.65231f, 0.281097f, 2.293869f);
		SpWorldManagerInstance->current()->transforms(12)->orientation = Quat(0.870596f, -0.36217f, 0.279794f, -0.18056f);
		SpWorldManagerInstance->current()->transforms(12)->scaleVector = Vec3(2.0f, 2.0f, 2.0f);
		SpWorldManagerInstance->current()->rigidBody3D(12)->mass(1.0f);

		SpCollisionResponseShapeMatching shapeMatching;

		SpRigidBodyShapeMatch shape0, shape8, shape12;
		shapeMatching.initShape(0, &shape0);
		shapeMatching.initShape(8, &shape8);
		shapeMatching.initShape(12, &shape12);

		sp_log_debug1snl("Initial Particles 8");
		for (sp_uint i = 0; i < shape8.particlesLength; i++)
		{
			sp_log_debug1s("Particle["); sp_log_debug1u(i); sp_log_debug1s("]: ");
			sp_log_debugXf(shape8.initialParticles[i], 3);  sp_log_newline();
		}
		sp_log_debug1snl("Initial Particles 12");
		for (sp_uint i = 0; i < shape12.particlesLength; i++)
		{
			sp_log_debug1s("Particle["); sp_log_debug1u(i); sp_log_debug1s("]: ");
			sp_log_debugXf(shape12.initialParticles[i], 3);  sp_log_newline();
		}

		shapeMatching.solve(&shape8, &shape12);
	}
	
}

#undef CLASS_NAME
