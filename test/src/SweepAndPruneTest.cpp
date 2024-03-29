#include "SpectrumPhysicsTest.h"
#include <SweepAndPrune.h>
#include "Randomizer.h"
#include "DOP18.h"
#include "SpRigidBody3D.h"

#define CLASS_NAME SweepAndPruneTest

sp_int comparatorXAxisForQuickSort1(const void* a, const void* b)
{
	AABB* obj1 = (AABB*) a;
	AABB* obj2 = (AABB*) b;

	if (obj1->minPoint.x < obj2->minPoint.x)
		return -1;
	else
		if (obj1->minPoint.x > obj2->minPoint.x)
			return 1;

	return 0;
}

AABB* getRandomAABBs(sp_uint count, sp_uint spaceSize = 1000u)
{
	Randomizer randomizerSize(0, 30);
	Randomizer randomizerLocation(0, spaceSize);

	AABB* aabbs = ALLOC_NEW_ARRAY(AABB, count);

	for (sp_uint i = 0; i < count; i++)
	{
		sp_int xMin = randomizerSize.randInt();
		sp_int yMin = randomizerSize.randInt();
		sp_int zMin = randomizerSize.randInt();

		sp_int xMax = randomizerSize.randInt();
		sp_int yMax = randomizerSize.randInt();
		sp_int zMax = randomizerSize.randInt();

		sp_int locationX = randomizerLocation.randInt();
		sp_int locationY = randomizerLocation.randInt();
		sp_int locationZ = randomizerLocation.randInt();

		if (xMin == xMax)
			xMax++;

		if (yMin == yMax)
			yMax++;

		if (zMin == zMax)
			zMax++;

		if (xMin > xMax)
			std::swap(xMin, xMax);

		if (yMin > yMax)
			std::swap(yMin, yMax);

		if (zMin > zMax)
			std::swap(zMin, zMax);

		aabbs[i] = AABB({ sp_float(xMin + locationX), sp_float(yMin + locationY), sp_float(zMin + locationZ) }
						, { sp_float(xMax + locationX), sp_float(yMax + locationY), sp_float(zMax + locationZ) });
	}

	return aabbs;
}

DOP18* getRandomKDOPs(sp_uint length, sp_uint spaceSize = 10000u)
{
	Randomizer randomizer(0, spaceSize);

	DOP18* kdops = ALLOC_NEW_ARRAY(DOP18, length);

	for (sp_uint i = 0; i < length; i++)
	{
		sp_float x = randomizer.randInt() / 100.0f;
		sp_float y = randomizer.randInt() / 100.0f;
		sp_float z = randomizer.randInt() / 100.0f;

		kdops[i].scale(Vec3(3.0f, 3.0f, 3.0f));
		kdops[i].translate({ x, 0.5f, z });
	}

	return kdops;
}

DOP18* get64KDOPs()
{
	DOP18* kdops = ALLOC_NEW_ARRAY(DOP18, 64);
	kdops[0].min[0] = 27.080000f;
	kdops[0].min[1] = -1.000000f;
	kdops[0].min[2] = 14.120000f;
	kdops[0].min[3] = 25.330000f;
	kdops[0].min[4] = 26.330000f;
	kdops[0].min[5] = -17.869999f;
	kdops[0].min[6] = 13.369999f;
	kdops[0].min[7] = 10.210000f;
	kdops[0].min[8] = 41.450001f;
	kdops[0].max[0] = 30.080000f;
	kdops[0].max[1] = 2.000000f;
	kdops[0].max[2] = 17.119999f;
	kdops[0].max[3] = 30.830000f;
	kdops[0].max[4] = 31.830000f;
	kdops[0].max[5] = -12.370000f;
	kdops[0].max[6] = 18.869999f;
	kdops[0].max[7] = 15.710000f;
	kdops[0].max[8] = 46.950001f;
	kdops[1].min[0] = 91.750000f;
	kdops[1].min[1] = -1.000000f;
	kdops[1].min[2] = 2.210000f;
	kdops[1].min[3] = 90.000000f;
	kdops[1].min[4] = 91.000000f;
	kdops[1].min[5] = -5.960000f;
	kdops[1].min[6] = 1.460000f;
	kdops[1].min[7] = 86.790001f;
	kdops[1].min[8] = 94.209999f;
	kdops[1].max[0] = 94.750000f;
	kdops[1].max[1] = 2.000000f;
	kdops[1].max[2] = 5.210000f;
	kdops[1].max[3] = 95.500000f;
	kdops[1].max[4] = 96.500000f;
	kdops[1].max[5] = -0.460000f;
	kdops[1].max[6] = 6.960000f;
	kdops[1].max[7] = 92.290001f;
	kdops[1].max[8] = 99.709999f;
	kdops[2].min[0] = 44.240002f;
	kdops[2].min[1] = -1.000000f;
	kdops[2].min[2] = 82.279999f;
	kdops[2].min[3] = 42.490002f;
	kdops[2].min[4] = 43.490002f;
	kdops[2].min[5] = -86.029999f;
	kdops[2].min[6] = 81.529999f;
	kdops[2].min[7] = -40.789997f;
	kdops[2].min[8] = 126.770004f;
	kdops[2].max[0] = 47.240002f;
	kdops[2].max[1] = 2.000000f;
	kdops[2].max[2] = 85.279999f;
	kdops[2].max[3] = 47.990002f;
	kdops[2].max[4] = 48.990002f;
	kdops[2].max[5] = -80.529999f;
	kdops[2].max[6] = 87.029999f;
	kdops[2].max[7] = -35.289997f;
	kdops[2].max[8] = 132.270004f;
	kdops[3].min[0] = 60.549999f;
	kdops[3].min[1] = -1.000000f;
	kdops[3].min[2] = 92.720001f;
	kdops[3].min[3] = 58.799999f;
	kdops[3].min[4] = 59.799999f;
	kdops[3].min[5] = -96.470001f;
	kdops[3].min[6] = 91.970001f;
	kdops[3].min[7] = -34.920002f;
	kdops[3].min[8] = 153.520004f;
	kdops[3].max[0] = 63.549999f;
	kdops[3].max[1] = 2.000000f;
	kdops[3].max[2] = 95.720001f;
	kdops[3].max[3] = 64.300003f;
	kdops[3].max[4] = 65.300003f;
	kdops[3].max[5] = -90.970001f;
	kdops[3].max[6] = 97.470001f;
	kdops[3].max[7] = -29.420002f;
	kdops[3].max[8] = 159.020004f;
	kdops[4].min[0] = 26.170000f;
	kdops[4].min[1] = -1.000000f;
	kdops[4].min[2] = 81.779999f;
	kdops[4].min[3] = 24.420000f;
	kdops[4].min[4] = 25.420000f;
	kdops[4].min[5] = -85.529999f;
	kdops[4].min[6] = 81.029999f;
	kdops[4].min[7] = -58.360001f;
	kdops[4].min[8] = 108.199997f;
	kdops[4].max[0] = 29.170000f;
	kdops[4].max[1] = 2.000000f;
	kdops[4].max[2] = 84.779999f;
	kdops[4].max[3] = 29.920000f;
	kdops[4].max[4] = 30.920000f;
	kdops[4].max[5] = -80.029999f;
	kdops[4].max[6] = 86.529999f;
	kdops[4].max[7] = -52.860001f;
	kdops[4].max[8] = 113.699997f;
	kdops[5].min[0] = 40.270000f;
	kdops[5].min[1] = -1.000000f;
	kdops[5].min[2] = 58.139999f;
	kdops[5].min[3] = 38.520000f;
	kdops[5].min[4] = 39.520000f;
	kdops[5].min[5] = -61.889999f;
	kdops[5].min[6] = 57.389999f;
	kdops[5].min[7] = -20.619999f;
	kdops[5].min[8] = 98.660004f;
	kdops[5].max[0] = 43.270000f;
	kdops[5].max[1] = 2.000000f;
	kdops[5].max[2] = 61.139999f;
	kdops[5].max[3] = 44.020000f;
	kdops[5].max[4] = 45.020000f;
	kdops[5].max[5] = -56.389999f;
	kdops[5].max[6] = 62.889999f;
	kdops[5].max[7] = -15.119999f;
	kdops[5].max[8] = 104.160004f;
	kdops[6].min[0] = 71.900002f;
	kdops[6].min[1] = -1.000000f;
	kdops[6].min[2] = 24.670000f;
	kdops[6].min[3] = 70.150002f;
	kdops[6].min[4] = 71.150002f;
	kdops[6].min[5] = -28.420000f;
	kdops[6].min[6] = 23.920000f;
	kdops[6].min[7] = 44.480003f;
	kdops[6].min[8] = 96.820000f;
	kdops[6].max[0] = 74.900002f;
	kdops[6].max[1] = 2.000000f;
	kdops[6].max[2] = 27.670000f;
	kdops[6].max[3] = 75.650002f;
	kdops[6].max[4] = 76.650002f;
	kdops[6].max[5] = -22.920000f;
	kdops[6].max[6] = 29.420000f;
	kdops[6].max[7] = 49.980003f;
	kdops[6].max[8] = 102.320000f;
	kdops[7].min[0] = 41.349998f;
	kdops[7].min[1] = -1.000000f;
	kdops[7].min[2] = 27.920000f;
	kdops[7].min[3] = 39.599998f;
	kdops[7].min[4] = 40.599998f;
	kdops[7].min[5] = -31.670000f;
	kdops[7].min[6] = 27.170000f;
	kdops[7].min[7] = 10.679998f;
	kdops[7].min[8] = 69.519997f;
	kdops[7].max[0] = 44.349998f;
	kdops[7].max[1] = 2.000000f;
	kdops[7].max[2] = 30.920000f;
	kdops[7].max[3] = 45.099998f;
	kdops[7].max[4] = 46.099998f;
	kdops[7].max[5] = -26.170000f;
	kdops[7].max[6] = 32.669998f;
	kdops[7].max[7] = 16.179998f;
	kdops[7].max[8] = 75.019997f;
	kdops[8].min[0] = 11.040000f;
	kdops[8].min[1] = -1.000000f;
	kdops[8].min[2] = 44.430000f;
	kdops[8].min[3] = 9.290000f;
	kdops[8].min[4] = 10.290000f;
	kdops[8].min[5] = -48.180000f;
	kdops[8].min[6] = 43.680000f;
	kdops[8].min[7] = -36.139999f;
	kdops[8].min[8] = 55.720001f;
	kdops[8].max[0] = 14.040000f;
	kdops[8].max[1] = 2.000000f;
	kdops[8].max[2] = 47.430000f;
	kdops[8].max[3] = 14.790000f;
	kdops[8].max[4] = 15.790000f;
	kdops[8].max[5] = -42.680000f;
	kdops[8].max[6] = 49.180000f;
	kdops[8].max[7] = -30.639999f;
	kdops[8].max[8] = 61.220001f;
	kdops[9].min[0] = 82.269997f;
	kdops[9].min[1] = -1.000000f;
	kdops[9].min[2] = 85.849998f;
	kdops[9].min[3] = 80.519997f;
	kdops[9].min[4] = 81.519997f;
	kdops[9].min[5] = -89.599998f;
	kdops[9].min[6] = 85.099998f;
	kdops[9].min[7] = -6.330002f;
	kdops[9].min[8] = 168.369995f;
	kdops[9].max[0] = 85.269997f;
	kdops[9].max[1] = 2.000000f;
	kdops[9].max[2] = 88.849998f;
	kdops[9].max[3] = 86.019997f;
	kdops[9].max[4] = 87.019997f;
	kdops[9].max[5] = -84.099998f;
	kdops[9].max[6] = 90.599998f;
	kdops[9].max[7] = -0.830002f;
	kdops[9].max[8] = 173.869995f;
	kdops[10].min[0] = 95.980003f;
	kdops[10].min[1] = -1.000000f;
	kdops[10].min[2] = 48.490002f;
	kdops[10].min[3] = 94.230003f;
	kdops[10].min[4] = 95.230003f;
	kdops[10].min[5] = -52.240002f;
	kdops[10].min[6] = 47.740002f;
	kdops[10].min[7] = 44.740002f;
	kdops[10].min[8] = 144.720001f;
	kdops[10].max[0] = 98.980003f;
	kdops[10].max[1] = 2.000000f;
	kdops[10].max[2] = 51.490002f;
	kdops[10].max[3] = 99.730003f;
	kdops[10].max[4] = 100.730003f;
	kdops[10].max[5] = -46.740002f;
	kdops[10].max[6] = 53.240002f;
	kdops[10].max[7] = 50.240002f;
	kdops[10].max[8] = 150.220001f;
	kdops[11].min[0] = 36.490002f;
	kdops[11].min[1] = -1.000000f;
	kdops[11].min[2] = 46.160000f;
	kdops[11].min[3] = 34.740002f;
	kdops[11].min[4] = 35.740002f;
	kdops[11].min[5] = -49.910000f;
	kdops[11].min[6] = 45.410000f;
	kdops[11].min[7] = -12.419998f;
	kdops[11].min[8] = 82.900002f;
	kdops[11].max[0] = 39.490002f;
	kdops[11].max[1] = 2.000000f;
	kdops[11].max[2] = 49.160000f;
	kdops[11].max[3] = 40.240002f;
	kdops[11].max[4] = 41.240002f;
	kdops[11].max[5] = -44.410000f;
	kdops[11].max[6] = 50.910000f;
	kdops[11].max[7] = -6.919998f;
	kdops[11].max[8] = 88.400002f;
	kdops[12].min[0] = 56.560001f;
	kdops[12].min[1] = -1.000000f;
	kdops[12].min[2] = 89.230003f;
	kdops[12].min[3] = 54.810001f;
	kdops[12].min[4] = 55.810001f;
	kdops[12].min[5] = -92.980003f;
	kdops[12].min[6] = 88.480003f;
	kdops[12].min[7] = -35.420002f;
	kdops[12].min[8] = 146.040009f;
	kdops[12].max[0] = 59.560001f;
	kdops[12].max[1] = 2.000000f;
	kdops[12].max[2] = 92.230003f;
	kdops[12].max[3] = 60.310001f;
	kdops[12].max[4] = 61.310001f;
	kdops[12].max[5] = -87.480003f;
	kdops[12].max[6] = 93.980003f;
	kdops[12].max[7] = -29.920002f;
	kdops[12].max[8] = 151.540009f;
	kdops[13].min[0] = 61.939999f;
	kdops[13].min[1] = -1.000000f;
	kdops[13].min[2] = 55.029999f;
	kdops[13].min[3] = 60.189999f;
	kdops[13].min[4] = 61.189999f;
	kdops[13].min[5] = -58.779999f;
	kdops[13].min[6] = 54.279999f;
	kdops[13].min[7] = 4.160000f;
	kdops[13].min[8] = 117.220001f;
	kdops[13].max[0] = 64.940002f;
	kdops[13].max[1] = 2.000000f;
	kdops[13].max[2] = 58.029999f;
	kdops[13].max[3] = 65.690002f;
	kdops[13].max[4] = 66.690002f;
	kdops[13].max[5] = -53.279999f;
	kdops[13].max[6] = 59.779999f;
	kdops[13].max[7] = 9.660000f;
	kdops[13].max[8] = 122.720001f;
	kdops[14].min[0] = 91.410004f;
	kdops[14].min[1] = -1.000000f;
	kdops[14].min[2] = 11.300000f;
	kdops[14].min[3] = 89.660004f;
	kdops[14].min[4] = 90.660004f;
	kdops[14].min[5] = -15.050000f;
	kdops[14].min[6] = 10.550000f;
	kdops[14].min[7] = 77.360001f;
	kdops[14].min[8] = 102.960007f;
	kdops[14].max[0] = 94.410004f;
	kdops[14].max[1] = 2.000000f;
	kdops[14].max[2] = 14.300000f;
	kdops[14].max[3] = 95.160004f;
	kdops[14].max[4] = 96.160004f;
	kdops[14].max[5] = -9.550000f;
	kdops[14].max[6] = 16.049999f;
	kdops[14].max[7] = 82.860001f;
	kdops[14].max[8] = 108.460007f;
	kdops[15].min[0] = 25.379999f;
	kdops[15].min[1] = -1.000000f;
	kdops[15].min[2] = 61.389999f;
	kdops[15].min[3] = 23.629999f;
	kdops[15].min[4] = 24.629999f;
	kdops[15].min[5] = -65.139999f;
	kdops[15].min[6] = 60.639999f;
	kdops[15].min[7] = -38.760002f;
	kdops[15].min[8] = 87.019997f;
	kdops[15].max[0] = 28.379999f;
	kdops[15].max[1] = 2.000000f;
	kdops[15].max[2] = 64.389999f;
	kdops[15].max[3] = 29.129999f;
	kdops[15].max[4] = 30.129999f;
	kdops[15].max[5] = -59.639999f;
	kdops[15].max[6] = 66.139999f;
	kdops[15].max[7] = -33.260002f;
	kdops[15].max[8] = 92.519997f;
	kdops[16].min[0] = 77.419998f;
	kdops[16].min[1] = -1.000000f;
	kdops[16].min[2] = 93.750000f;
	kdops[16].min[3] = 75.669998f;
	kdops[16].min[4] = 76.669998f;
	kdops[16].min[5] = -97.500000f;
	kdops[16].min[6] = 93.000000f;
	kdops[16].min[7] = -19.080002f;
	kdops[16].min[8] = 171.419998f;
	kdops[16].max[0] = 80.419998f;
	kdops[16].max[1] = 2.000000f;
	kdops[16].max[2] = 96.750000f;
	kdops[16].max[3] = 81.169998f;
	kdops[16].max[4] = 82.169998f;
	kdops[16].max[5] = -92.000000f;
	kdops[16].max[6] = 98.500000f;
	kdops[16].max[7] = -13.580002f;
	kdops[16].max[8] = 176.919998f;
	kdops[17].min[0] = 25.469999f;
	kdops[17].min[1] = -1.000000f;
	kdops[17].min[2] = 95.220001f;
	kdops[17].min[3] = 23.719999f;
	kdops[17].min[4] = 24.719999f;
	kdops[17].min[5] = -98.970001f;
	kdops[17].min[6] = 94.470001f;
	kdops[17].min[7] = -72.500000f;
	kdops[17].min[8] = 120.940002f;
	kdops[17].max[0] = 28.469999f;
	kdops[17].max[1] = 2.000000f;
	kdops[17].max[2] = 98.220001f;
	kdops[17].max[3] = 29.219999f;
	kdops[17].max[4] = 30.219999f;
	kdops[17].max[5] = -93.470001f;
	kdops[17].max[6] = 99.970001f;
	kdops[17].max[7] = -67.000000f;
	kdops[17].max[8] = 126.440002f;
	kdops[18].min[0] = 21.930000f;
	kdops[18].min[1] = -1.000000f;
	kdops[18].min[2] = 39.830002f;
	kdops[18].min[3] = 20.180000f;
	kdops[18].min[4] = 21.180000f;
	kdops[18].min[5] = -43.580002f;
	kdops[18].min[6] = 39.080002f;
	kdops[18].min[7] = -20.650002f;
	kdops[18].min[8] = 62.010002f;
	kdops[18].max[0] = 24.930000f;
	kdops[18].max[1] = 2.000000f;
	kdops[18].max[2] = 42.830002f;
	kdops[18].max[3] = 25.680000f;
	kdops[18].max[4] = 26.680000f;
	kdops[18].max[5] = -38.080002f;
	kdops[18].max[6] = 44.580002f;
	kdops[18].max[7] = -15.150002f;
	kdops[18].max[8] = 67.510002f;
	kdops[19].min[0] = 62.119999f;
	kdops[19].min[1] = -1.000000f;
	kdops[19].min[2] = -1.250000f;
	kdops[19].min[3] = 60.369999f;
	kdops[19].min[4] = 61.369995f;
	kdops[19].min[5] = -2.500000f;
	kdops[19].min[6] = -2.000000f;
	kdops[19].min[7] = 60.619999f;
	kdops[19].min[8] = 61.119999f;
	kdops[19].max[0] = 65.119995f;
	kdops[19].max[1] = 2.000000f;
	kdops[19].max[2] = 1.750000f;
	kdops[19].max[3] = 65.869995f;
	kdops[19].max[4] = 66.869995f;
	kdops[19].max[5] = 3.000000f;
	kdops[19].max[6] = 3.500000f;
	kdops[19].max[7] = 66.119995f;
	kdops[19].max[8] = 66.619995f;
	kdops[20].min[0] = 81.839996f;
	kdops[20].min[1] = -1.000000f;
	kdops[20].min[2] = 59.779999f;
	kdops[20].min[3] = 80.089996f;
	kdops[20].min[4] = 81.089996f;
	kdops[20].min[5] = -63.529999f;
	kdops[20].min[6] = 59.029999f;
	kdops[20].min[7] = 19.309998f;
	kdops[20].min[8] = 141.869995f;
	kdops[20].max[0] = 84.839996f;
	kdops[20].max[1] = 2.000000f;
	kdops[20].max[2] = 62.779999f;
	kdops[20].max[3] = 85.589996f;
	kdops[20].max[4] = 86.589996f;
	kdops[20].max[5] = -58.029999f;
	kdops[20].max[6] = 64.529999f;
	kdops[20].max[7] = 24.809998f;
	kdops[20].max[8] = 147.369995f;
	kdops[21].min[0] = 74.739998f;
	kdops[21].min[1] = -1.000000f;
	kdops[21].min[2] = 25.370001f;
	kdops[21].min[3] = 72.989998f;
	kdops[21].min[4] = 73.989998f;
	kdops[21].min[5] = -29.120001f;
	kdops[21].min[6] = 24.620001f;
	kdops[21].min[7] = 46.619995f;
	kdops[21].min[8] = 100.360001f;
	kdops[21].max[0] = 77.739998f;
	kdops[21].max[1] = 2.000000f;
	kdops[21].max[2] = 28.370001f;
	kdops[21].max[3] = 78.489998f;
	kdops[21].max[4] = 79.489998f;
	kdops[21].max[5] = -23.620001f;
	kdops[21].max[6] = 30.120001f;
	kdops[21].max[7] = 52.119995f;
	kdops[21].max[8] = 105.860001f;
	kdops[22].min[0] = 48.430000f;
	kdops[22].min[1] = -1.000000f;
	kdops[22].min[2] = 16.990000f;
	kdops[22].min[3] = 46.680000f;
	kdops[22].min[4] = 47.680000f;
	kdops[22].min[5] = -20.740000f;
	kdops[22].min[6] = 16.240000f;
	kdops[22].min[7] = 28.690001f;
	kdops[22].min[8] = 65.669998f;
	kdops[22].max[0] = 51.430000f;
	kdops[22].max[1] = 2.000000f;
	kdops[22].max[2] = 19.990000f;
	kdops[22].max[3] = 52.180000f;
	kdops[22].max[4] = 53.180000f;
	kdops[22].max[5] = -15.240000f;
	kdops[22].max[6] = 21.740000f;
	kdops[22].max[7] = 34.190002f;
	kdops[22].max[8] = 71.169998f;
	kdops[23].min[0] = 94.250000f;
	kdops[23].min[1] = -1.000000f;
	kdops[23].min[2] = 64.209999f;
	kdops[23].min[3] = 92.500000f;
	kdops[23].min[4] = 93.500000f;
	kdops[23].min[5] = -67.959999f;
	kdops[23].min[6] = 63.459999f;
	kdops[23].min[7] = 27.290001f;
	kdops[23].min[8] = 158.709991f;
	kdops[23].max[0] = 97.250000f;
	kdops[23].max[1] = 2.000000f;
	kdops[23].max[2] = 67.209999f;
	kdops[23].max[3] = 98.000000f;
	kdops[23].max[4] = 99.000000f;
	kdops[23].max[5] = -62.459999f;
	kdops[23].max[6] = 68.959999f;
	kdops[23].max[7] = 32.790001f;
	kdops[23].max[8] = 164.209991f;
	kdops[24].min[0] = 21.330000f;
	kdops[24].min[1] = -1.000000f;
	kdops[24].min[2] = 47.110001f;
	kdops[24].min[3] = 19.580000f;
	kdops[24].min[4] = 20.580000f;
	kdops[24].min[5] = -50.860001f;
	kdops[24].min[6] = 46.360001f;
	kdops[24].min[7] = -28.530001f;
	kdops[24].min[8] = 68.690002f;
	kdops[24].max[0] = 24.330000f;
	kdops[24].max[1] = 2.000000f;
	kdops[24].max[2] = 50.110001f;
	kdops[24].max[3] = 25.080000f;
	kdops[24].max[4] = 26.080000f;
	kdops[24].max[5] = -45.360001f;
	kdops[24].max[6] = 51.860001f;
	kdops[24].max[7] = -23.030001f;
	kdops[24].max[8] = 74.190002f;
	kdops[25].min[0] = 16.080000f;
	kdops[25].min[1] = -1.000000f;
	kdops[25].min[2] = 13.520000f;
	kdops[25].min[3] = 14.330000f;
	kdops[25].min[4] = 15.330000f;
	kdops[25].min[5] = -17.270000f;
	kdops[25].min[6] = 12.770000f;
	kdops[25].min[7] = -0.190001f;
	kdops[25].min[8] = 29.849998f;
	kdops[25].max[0] = 19.080000f;
	kdops[25].max[1] = 2.000000f;
	kdops[25].max[2] = 16.520000f;
	kdops[25].max[3] = 19.830000f;
	kdops[25].max[4] = 20.830000f;
	kdops[25].max[5] = -11.770000f;
	kdops[25].max[6] = 18.270000f;
	kdops[25].max[7] = 5.309999f;
	kdops[25].max[8] = 35.349998f;
	kdops[26].min[0] = 38.799999f;
	kdops[26].min[1] = -1.000000f;
	kdops[26].min[2] = 12.740000f;
	kdops[26].min[3] = 37.049999f;
	kdops[26].min[4] = 38.049999f;
	kdops[26].min[5] = -16.490000f;
	kdops[26].min[6] = 11.990000f;
	kdops[26].min[7] = 23.309999f;
	kdops[26].min[8] = 51.790001f;
	kdops[26].max[0] = 41.799999f;
	kdops[26].max[1] = 2.000000f;
	kdops[26].max[2] = 15.740000f;
	kdops[26].max[3] = 42.549999f;
	kdops[26].max[4] = 43.549999f;
	kdops[26].max[5] = -10.990000f;
	kdops[26].max[6] = 17.490000f;
	kdops[26].max[7] = 28.809999f;
	kdops[26].max[8] = 57.290001f;
	kdops[27].min[0] = 11.430000f;
	kdops[27].min[1] = -1.000000f;
	kdops[27].min[2] = 43.060001f;
	kdops[27].min[3] = 9.680000f;
	kdops[27].min[4] = 10.680000f;
	kdops[27].min[5] = -46.810001f;
	kdops[27].min[6] = 42.310001f;
	kdops[27].min[7] = -34.380001f;
	kdops[27].min[8] = 54.740002f;
	kdops[27].max[0] = 14.430000f;
	kdops[27].max[1] = 2.000000f;
	kdops[27].max[2] = 46.060001f;
	kdops[27].max[3] = 15.180000f;
	kdops[27].max[4] = 16.180000f;
	kdops[27].max[5] = -41.310001f;
	kdops[27].max[6] = 47.810001f;
	kdops[27].max[7] = -28.880001f;
	kdops[27].max[8] = 60.240002f;
	kdops[28].min[0] = 18.879999f;
	kdops[28].min[1] = -1.000000f;
	kdops[28].min[2] = 27.160000f;
	kdops[28].min[3] = 17.129999f;
	kdops[28].min[4] = 18.129999f;
	kdops[28].min[5] = -30.910000f;
	kdops[28].min[6] = 26.410000f;
	kdops[28].min[7] = -11.030001f;
	kdops[28].min[8] = 46.290001f;
	kdops[28].max[0] = 21.879999f;
	kdops[28].max[1] = 2.000000f;
	kdops[28].max[2] = 30.160000f;
	kdops[28].max[3] = 22.629999f;
	kdops[28].max[4] = 23.629999f;
	kdops[28].max[5] = -25.410000f;
	kdops[28].max[6] = 31.910000f;
	kdops[28].max[7] = -5.530001f;
	kdops[28].max[8] = 51.790001f;
	kdops[29].min[0] = 56.810001f;
	kdops[29].min[1] = -1.000000f;
	kdops[29].min[2] = 35.070000f;
	kdops[29].min[3] = 55.060001f;
	kdops[29].min[4] = 56.060001f;
	kdops[29].min[5] = -38.820000f;
	kdops[29].min[6] = 34.320000f;
	kdops[29].min[7] = 18.990002f;
	kdops[29].min[8] = 92.130005f;
	kdops[29].max[0] = 59.810001f;
	kdops[29].max[1] = 2.000000f;
	kdops[29].max[2] = 38.070000f;
	kdops[29].max[3] = 60.560001f;
	kdops[29].max[4] = 61.560001f;
	kdops[29].max[5] = -33.320000f;
	kdops[29].max[6] = 39.820000f;
	kdops[29].max[7] = 24.490002f;
	kdops[29].max[8] = 97.630005f;
	kdops[30].min[0] = 92.970001f;
	kdops[30].min[1] = -1.000000f;
	kdops[30].min[2] = 97.089996f;
	kdops[30].min[3] = 91.220001f;
	kdops[30].min[4] = 92.220001f;
	kdops[30].min[5] = -100.839996f;
	kdops[30].min[6] = 96.339996f;
	kdops[30].min[7] = -6.869995f;
	kdops[30].min[8] = 190.309998f;
	kdops[30].max[0] = 95.970001f;
	kdops[30].max[1] = 2.000000f;
	kdops[30].max[2] = 100.089996f;
	kdops[30].max[3] = 96.720001f;
	kdops[30].max[4] = 97.720001f;
	kdops[30].max[5] = -95.339996f;
	kdops[30].max[6] = 101.839996f;
	kdops[30].max[7] = -1.369995f;
	kdops[30].max[8] = 195.809998f;
	kdops[31].min[0] = 72.599998f;
	kdops[31].min[1] = -1.000000f;
	kdops[31].min[2] = 60.230000f;
	kdops[31].min[3] = 70.849998f;
	kdops[31].min[4] = 71.849998f;
	kdops[31].min[5] = -63.980000f;
	kdops[31].min[6] = 59.480000f;
	kdops[31].min[7] = 9.619999f;
	kdops[31].min[8] = 133.080002f;
	kdops[31].max[0] = 75.599998f;
	kdops[31].max[1] = 2.000000f;
	kdops[31].max[2] = 63.230000f;
	kdops[31].max[3] = 76.349998f;
	kdops[31].max[4] = 77.349998f;
	kdops[31].max[5] = -58.480000f;
	kdops[31].max[6] = 64.979996f;
	kdops[31].max[7] = 15.119999f;
	kdops[31].max[8] = 138.580002f;
	kdops[32].min[0] = 69.919998f;
	kdops[32].min[1] = -1.000000f;
	kdops[32].min[2] = 46.369999f;
	kdops[32].min[3] = 68.169998f;
	kdops[32].min[4] = 69.169998f;
	kdops[32].min[5] = -50.119999f;
	kdops[32].min[6] = 45.619999f;
	kdops[32].min[7] = 20.799999f;
	kdops[32].min[8] = 116.539993f;
	kdops[32].max[0] = 72.919998f;
	kdops[32].max[1] = 2.000000f;
	kdops[32].max[2] = 49.369999f;
	kdops[32].max[3] = 73.669998f;
	kdops[32].max[4] = 74.669998f;
	kdops[32].max[5] = -44.619999f;
	kdops[32].max[6] = 51.119999f;
	kdops[32].max[7] = 26.299999f;
	kdops[32].max[8] = 122.039993f;
	kdops[33].min[0] = 13.260000f;
	kdops[33].min[1] = -1.000000f;
	kdops[33].min[2] = 6.150000f;
	kdops[33].min[3] = 11.510000f;
	kdops[33].min[4] = 12.510000f;
	kdops[33].min[5] = -9.900000f;
	kdops[33].min[6] = 5.400000f;
	kdops[33].min[7] = 4.360000f;
	kdops[33].min[8] = 19.660000f;
	kdops[33].max[0] = 16.260000f;
	kdops[33].max[1] = 2.000000f;
	kdops[33].max[2] = 9.150000f;
	kdops[33].max[3] = 17.010000f;
	kdops[33].max[4] = 18.010000f;
	kdops[33].max[5] = -4.400000f;
	kdops[33].max[6] = 10.900000f;
	kdops[33].max[7] = 9.860001f;
	kdops[33].max[8] = 25.160000f;
	kdops[34].min[0] = 80.209999f;
	kdops[34].min[1] = -1.000000f;
	kdops[34].min[2] = 9.380000f;
	kdops[34].min[3] = 78.459999f;
	kdops[34].min[4] = 79.459999f;
	kdops[34].min[5] = -13.130000f;
	kdops[34].min[6] = 8.630000f;
	kdops[34].min[7] = 68.080002f;
	kdops[34].min[8] = 89.839996f;
	kdops[34].max[0] = 83.209999f;
	kdops[34].max[1] = 2.000000f;
	kdops[34].max[2] = 12.380000f;
	kdops[34].max[3] = 83.959999f;
	kdops[34].max[4] = 84.959999f;
	kdops[34].max[5] = -7.630000f;
	kdops[34].max[6] = 14.130000f;
	kdops[34].max[7] = 73.580002f;
	kdops[34].max[8] = 95.339996f;
	kdops[35].min[0] = 33.500000f;
	kdops[35].min[1] = -1.000000f;
	kdops[35].min[2] = 88.959999f;
	kdops[35].min[3] = 31.750000f;
	kdops[35].min[4] = 32.750000f;
	kdops[35].min[5] = -92.709999f;
	kdops[35].min[6] = 88.209999f;
	kdops[35].min[7] = -58.209999f;
	kdops[35].min[8] = 122.709999f;
	kdops[35].max[0] = 36.500000f;
	kdops[35].max[1] = 2.000000f;
	kdops[35].max[2] = 91.959999f;
	kdops[35].max[3] = 37.250000f;
	kdops[35].max[4] = 38.250000f;
	kdops[35].max[5] = -87.209999f;
	kdops[35].max[6] = 93.709999f;
	kdops[35].max[7] = -52.709999f;
	kdops[35].max[8] = 128.209991f;
	kdops[36].min[0] = 38.919998f;
	kdops[36].min[1] = -1.000000f;
	kdops[36].min[2] = 38.029999f;
	kdops[36].min[3] = 37.169998f;
	kdops[36].min[4] = 38.169998f;
	kdops[36].min[5] = -41.779999f;
	kdops[36].min[6] = 37.279999f;
	kdops[36].min[7] = -1.860001f;
	kdops[36].min[8] = 77.199997f;
	kdops[36].max[0] = 41.919998f;
	kdops[36].max[1] = 2.000000f;
	kdops[36].max[2] = 41.029999f;
	kdops[36].max[3] = 42.669998f;
	kdops[36].max[4] = 43.669998f;
	kdops[36].max[5] = -36.279999f;
	kdops[36].max[6] = 42.779999f;
	kdops[36].max[7] = 3.639999f;
	kdops[36].max[8] = 82.699997f;
	kdops[37].min[0] = 47.200001f;
	kdops[37].min[1] = -1.000000f;
	kdops[37].min[2] = 20.590000f;
	kdops[37].min[3] = 45.450001f;
	kdops[37].min[4] = 46.450001f;
	kdops[37].min[5] = -24.340000f;
	kdops[37].min[6] = 19.840000f;
	kdops[37].min[7] = 23.860001f;
	kdops[37].min[8] = 68.040001f;
	kdops[37].max[0] = 50.200001f;
	kdops[37].max[1] = 2.000000f;
	kdops[37].max[2] = 23.590000f;
	kdops[37].max[3] = 50.950001f;
	kdops[37].max[4] = 51.950001f;
	kdops[37].max[5] = -18.840000f;
	kdops[37].max[6] = 25.340000f;
	kdops[37].max[7] = 29.360001f;
	kdops[37].max[8] = 73.540001f;
	kdops[38].min[0] = 66.389999f;
	kdops[38].min[1] = -1.000000f;
	kdops[38].min[2] = 94.669998f;
	kdops[38].min[3] = 64.639999f;
	kdops[38].min[4] = 65.639999f;
	kdops[38].min[5] = -98.419998f;
	kdops[38].min[6] = 93.919998f;
	kdops[38].min[7] = -31.029999f;
	kdops[38].min[8] = 161.309998f;
	kdops[38].max[0] = 69.389999f;
	kdops[38].max[1] = 2.000000f;
	kdops[38].max[2] = 97.669998f;
	kdops[38].max[3] = 70.139999f;
	kdops[38].max[4] = 71.139999f;
	kdops[38].max[5] = -92.919998f;
	kdops[38].max[6] = 99.419998f;
	kdops[38].max[7] = -25.529999f;
	kdops[38].max[8] = 166.809998f;
	kdops[39].min[0] = 17.780001f;
	kdops[39].min[1] = -1.000000f;
	kdops[39].min[2] = 66.430000f;
	kdops[39].min[3] = 16.030001f;
	kdops[39].min[4] = 17.030001f;
	kdops[39].min[5] = -70.180000f;
	kdops[39].min[6] = 65.680000f;
	kdops[39].min[7] = -51.400002f;
	kdops[39].min[8] = 84.459999f;
	kdops[39].max[0] = 20.780001f;
	kdops[39].max[1] = 2.000000f;
	kdops[39].max[2] = 69.430000f;
	kdops[39].max[3] = 21.530001f;
	kdops[39].max[4] = 22.530001f;
	kdops[39].max[5] = -64.680000f;
	kdops[39].max[6] = 71.180000f;
	kdops[39].max[7] = -45.900002f;
	kdops[39].max[8] = 89.959999f;
	kdops[40].min[0] = 31.430000f;
	kdops[40].min[1] = -1.000000f;
	kdops[40].min[2] = 92.989998f;
	kdops[40].min[3] = 29.680000f;
	kdops[40].min[4] = 30.680000f;
	kdops[40].min[5] = -96.739998f;
	kdops[40].min[6] = 92.239998f;
	kdops[40].min[7] = -64.309998f;
	kdops[40].min[8] = 124.669998f;
	kdops[40].max[0] = 34.430000f;
	kdops[40].max[1] = 2.000000f;
	kdops[40].max[2] = 95.989998f;
	kdops[40].max[3] = 35.180000f;
	kdops[40].max[4] = 36.180000f;
	kdops[40].max[5] = -91.239998f;
	kdops[40].max[6] = 97.739998f;
	kdops[40].max[7] = -58.809998f;
	kdops[40].max[8] = 130.169998f;
	kdops[41].min[0] = 45.189999f;
	kdops[41].min[1] = -1.000000f;
	kdops[41].min[2] = 11.420000f;
	kdops[41].min[3] = 43.439999f;
	kdops[41].min[4] = 44.439999f;
	kdops[41].min[5] = -15.170000f;
	kdops[41].min[6] = 10.670000f;
	kdops[41].min[7] = 31.019997f;
	kdops[41].min[8] = 56.860001f;
	kdops[41].max[0] = 48.189999f;
	kdops[41].max[1] = 2.000000f;
	kdops[41].max[2] = 14.420000f;
	kdops[41].max[3] = 48.939999f;
	kdops[41].max[4] = 49.939999f;
	kdops[41].max[5] = -9.670000f;
	kdops[41].max[6] = 16.170000f;
	kdops[41].max[7] = 36.519997f;
	kdops[41].max[8] = 62.360001f;
	kdops[42].min[0] = 75.360001f;
	kdops[42].min[1] = -1.000000f;
	kdops[42].min[2] = 13.280000f;
	kdops[42].min[3] = 73.610001f;
	kdops[42].min[4] = 74.610001f;
	kdops[42].min[5] = -17.029999f;
	kdops[42].min[6] = 12.530000f;
	kdops[42].min[7] = 59.330002f;
	kdops[42].min[8] = 88.889999f;
	kdops[42].max[0] = 78.360001f;
	kdops[42].max[1] = 2.000000f;
	kdops[42].max[2] = 16.279999f;
	kdops[42].max[3] = 79.110001f;
	kdops[42].max[4] = 80.110001f;
	kdops[42].max[5] = -11.530000f;
	kdops[42].max[6] = 18.029999f;
	kdops[42].max[7] = 64.830002f;
	kdops[42].max[8] = 94.389999f;
	kdops[43].min[0] = 87.980003f;
	kdops[43].min[1] = -1.000000f;
	kdops[43].min[2] = 40.529999f;
	kdops[43].min[3] = 86.230003f;
	kdops[43].min[4] = 87.230003f;
	kdops[43].min[5] = -44.279999f;
	kdops[43].min[6] = 39.779999f;
	kdops[43].min[7] = 44.700005f;
	kdops[43].min[8] = 128.760010f;
	kdops[43].max[0] = 90.980003f;
	kdops[43].max[1] = 2.000000f;
	kdops[43].max[2] = 43.529999f;
	kdops[43].max[3] = 91.730003f;
	kdops[43].max[4] = 92.730003f;
	kdops[43].max[5] = -38.779999f;
	kdops[43].max[6] = 45.279999f;
	kdops[43].max[7] = 50.200005f;
	kdops[43].max[8] = 134.260010f;
	kdops[44].min[0] = 52.389999f;
	kdops[44].min[1] = -1.000000f;
	kdops[44].min[2] = 23.990000f;
	kdops[44].min[3] = 50.639999f;
	kdops[44].min[4] = 51.639999f;
	kdops[44].min[5] = -27.740000f;
	kdops[44].min[6] = 23.240000f;
	kdops[44].min[7] = 25.650000f;
	kdops[44].min[8] = 76.629997f;
	kdops[44].max[0] = 55.389999f;
	kdops[44].max[1] = 2.000000f;
	kdops[44].max[2] = 26.990000f;
	kdops[44].max[3] = 56.139999f;
	kdops[44].max[4] = 57.139999f;
	kdops[44].max[5] = -22.240000f;
	kdops[44].max[6] = 28.740000f;
	kdops[44].max[7] = 31.150000f;
	kdops[44].max[8] = 82.129997f;
	kdops[45].min[0] = 44.820000f;
	kdops[45].min[1] = -1.000000f;
	kdops[45].min[2] = 54.840000f;
	kdops[45].min[3] = 43.070000f;
	kdops[45].min[4] = 44.070000f;
	kdops[45].min[5] = -58.590000f;
	kdops[45].min[6] = 54.090000f;
	kdops[45].min[7] = -12.770000f;
	kdops[45].min[8] = 99.910004f;
	kdops[45].max[0] = 47.820000f;
	kdops[45].max[1] = 2.000000f;
	kdops[45].max[2] = 57.840000f;
	kdops[45].max[3] = 48.570000f;
	kdops[45].max[4] = 49.570000f;
	kdops[45].max[5] = -53.090000f;
	kdops[45].max[6] = 59.590000f;
	kdops[45].max[7] = -7.270000f;
	kdops[45].max[8] = 105.410004f;
	kdops[46].min[0] = 63.209999f;
	kdops[46].min[1] = -1.000000f;
	kdops[46].min[2] = 27.139999f;
	kdops[46].min[3] = 61.459999f;
	kdops[46].min[4] = 62.459999f;
	kdops[46].min[5] = -30.889999f;
	kdops[46].min[6] = 26.389999f;
	kdops[46].min[7] = 33.320000f;
	kdops[46].min[8] = 90.599998f;
	kdops[46].max[0] = 66.209999f;
	kdops[46].max[1] = 2.000000f;
	kdops[46].max[2] = 30.139999f;
	kdops[46].max[3] = 66.959999f;
	kdops[46].max[4] = 67.959999f;
	kdops[46].max[5] = -25.389999f;
	kdops[46].max[6] = 31.889999f;
	kdops[46].max[7] = 38.820000f;
	kdops[46].max[8] = 96.099998f;
	kdops[47].min[0] = 87.440002f;
	kdops[47].min[1] = -1.000000f;
	kdops[47].min[2] = 12.780000f;
	kdops[47].min[3] = 85.690002f;
	kdops[47].min[4] = 86.690002f;
	kdops[47].min[5] = -16.529999f;
	kdops[47].min[6] = 12.030000f;
	kdops[47].min[7] = 71.910004f;
	kdops[47].min[8] = 100.470001f;
	kdops[47].max[0] = 90.440002f;
	kdops[47].max[1] = 2.000000f;
	kdops[47].max[2] = 15.780000f;
	kdops[47].max[3] = 91.190002f;
	kdops[47].max[4] = 92.190002f;
	kdops[47].max[5] = -11.030000f;
	kdops[47].max[6] = 17.529999f;
	kdops[47].max[7] = 77.410004f;
	kdops[47].max[8] = 105.970001f;
	kdops[48].min[0] = 9.100000f;
	kdops[48].min[1] = -1.000000f;
	kdops[48].min[2] = 50.139999f;
	kdops[48].min[3] = 7.350000f;
	kdops[48].min[4] = 8.350000f;
	kdops[48].min[5] = -53.889999f;
	kdops[48].min[6] = 49.389999f;
	kdops[48].min[7] = -43.790001f;
	kdops[48].min[8] = 59.489998f;
	kdops[48].max[0] = 12.100000f;
	kdops[48].max[1] = 2.000000f;
	kdops[48].max[2] = 53.139999f;
	kdops[48].max[3] = 12.850000f;
	kdops[48].max[4] = 13.850000f;
	kdops[48].max[5] = -48.389999f;
	kdops[48].max[6] = 54.889999f;
	kdops[48].max[7] = -38.290001f;
	kdops[48].max[8] = 64.989998f;
	kdops[49].min[0] = 75.480003f;
	kdops[49].min[1] = -1.000000f;
	kdops[49].min[2] = 95.720001f;
	kdops[49].min[3] = 73.730003f;
	kdops[49].min[4] = 74.730003f;
	kdops[49].min[5] = -99.470001f;
	kdops[49].min[6] = 94.970001f;
	kdops[49].min[7] = -22.989998f;
	kdops[49].min[8] = 171.450012f;
	kdops[49].max[0] = 78.480003f;
	kdops[49].max[1] = 2.000000f;
	kdops[49].max[2] = 98.720001f;
	kdops[49].max[3] = 79.230003f;
	kdops[49].max[4] = 80.230003f;
	kdops[49].max[5] = -93.970001f;
	kdops[49].max[6] = 100.470001f;
	kdops[49].max[7] = -17.489998f;
	kdops[49].max[8] = 176.950012f;
	kdops[50].min[0] = 13.700000f;
	kdops[50].min[1] = -1.000000f;
	kdops[50].min[2] = 92.279999f;
	kdops[50].min[3] = 11.950000f;
	kdops[50].min[4] = 12.950000f;
	kdops[50].min[5] = -96.029999f;
	kdops[50].min[6] = 91.529999f;
	kdops[50].min[7] = -81.330002f;
	kdops[50].min[8] = 106.229996f;
	kdops[50].max[0] = 16.700001f;
	kdops[50].max[1] = 2.000000f;
	kdops[50].max[2] = 95.279999f;
	kdops[50].max[3] = 17.450001f;
	kdops[50].max[4] = 18.450001f;
	kdops[50].max[5] = -90.529999f;
	kdops[50].max[6] = 97.029999f;
	kdops[50].max[7] = -75.830002f;
	kdops[50].max[8] = 111.729996f;
	kdops[51].min[0] = 48.880001f;
	kdops[51].min[1] = -1.000000f;
	kdops[51].min[2] = 33.900002f;
	kdops[51].min[3] = 47.130001f;
	kdops[51].min[4] = 48.130001f;
	kdops[51].min[5] = -37.650002f;
	kdops[51].min[6] = 33.150002f;
	kdops[51].min[7] = 12.230000f;
	kdops[51].min[8] = 83.029999f;
	kdops[51].max[0] = 51.880001f;
	kdops[51].max[1] = 2.000000f;
	kdops[51].max[2] = 36.900002f;
	kdops[51].max[3] = 52.630001f;
	kdops[51].max[4] = 53.630001f;
	kdops[51].max[5] = -32.150002f;
	kdops[51].max[6] = 38.650002f;
	kdops[51].max[7] = 17.730000f;
	kdops[51].max[8] = 88.529999f;
	kdops[52].min[0] = 22.959999f;
	kdops[52].min[1] = -1.000000f;
	kdops[52].min[2] = 74.870003f;
	kdops[52].min[3] = 21.209999f;
	kdops[52].min[4] = 22.209999f;
	kdops[52].min[5] = -78.620003f;
	kdops[52].min[6] = 74.120003f;
	kdops[52].min[7] = -54.660004f;
	kdops[52].min[8] = 98.080002f;
	kdops[52].max[0] = 25.959999f;
	kdops[52].max[1] = 2.000000f;
	kdops[52].max[2] = 77.870003f;
	kdops[52].max[3] = 26.709999f;
	kdops[52].max[4] = 27.709999f;
	kdops[52].max[5] = -73.120003f;
	kdops[52].max[6] = 79.620003f;
	kdops[52].max[7] = -49.160004f;
	kdops[52].max[8] = 103.580002f;
	kdops[53].min[0] = 65.720001f;
	kdops[53].min[1] = -1.000000f;
	kdops[53].min[2] = 32.950001f;
	kdops[53].min[3] = 63.970001f;
	kdops[53].min[4] = 64.970001f;
	kdops[53].min[5] = -36.700001f;
	kdops[53].min[6] = 32.200001f;
	kdops[53].min[7] = 30.020000f;
	kdops[53].min[8] = 98.919998f;
	kdops[53].max[0] = 68.720001f;
	kdops[53].max[1] = 2.000000f;
	kdops[53].max[2] = 35.950001f;
	kdops[53].max[3] = 69.470001f;
	kdops[53].max[4] = 70.470001f;
	kdops[53].max[5] = -31.200001f;
	kdops[53].max[6] = 37.700001f;
	kdops[53].max[7] = 35.520000f;
	kdops[53].max[8] = 104.419998f;
	kdops[54].min[0] = 90.059998f;
	kdops[54].min[1] = -1.000000f;
	kdops[54].min[2] = 32.310001f;
	kdops[54].min[3] = 88.309998f;
	kdops[54].min[4] = 89.309998f;
	kdops[54].min[5] = -36.060001f;
	kdops[54].min[6] = 31.560001f;
	kdops[54].min[7] = 54.999996f;
	kdops[54].min[8] = 122.619995f;
	kdops[54].max[0] = 93.059998f;
	kdops[54].max[1] = 2.000000f;
	kdops[54].max[2] = 35.310001f;
	kdops[54].max[3] = 93.809998f;
	kdops[54].max[4] = 94.809998f;
	kdops[54].max[5] = -30.560001f;
	kdops[54].max[6] = 37.060001f;
	kdops[54].max[7] = 60.499996f;
	kdops[54].max[8] = 128.119995f;
	kdops[55].min[0] = 23.549999f;
	kdops[55].min[1] = -1.000000f;
	kdops[55].min[2] = 43.290001f;
	kdops[55].min[3] = 21.799999f;
	kdops[55].min[4] = 22.799999f;
	kdops[55].min[5] = -47.040001f;
	kdops[55].min[6] = 42.540001f;
	kdops[55].min[7] = -22.490002f;
	kdops[55].min[8] = 67.089996f;
	kdops[55].max[0] = 26.549999f;
	kdops[55].max[1] = 2.000000f;
	kdops[55].max[2] = 46.290001f;
	kdops[55].max[3] = 27.299999f;
	kdops[55].max[4] = 28.299999f;
	kdops[55].max[5] = -41.540001f;
	kdops[55].max[6] = 48.040001f;
	kdops[55].max[7] = -16.990002f;
	kdops[55].max[8] = 72.589996f;
	kdops[56].min[0] = 27.170000f;
	kdops[56].min[1] = -1.000000f;
	kdops[56].min[2] = 77.769997f;
	kdops[56].min[3] = 25.420000f;
	kdops[56].min[4] = 26.420000f;
	kdops[56].min[5] = -81.519997f;
	kdops[56].min[6] = 77.019997f;
	kdops[56].min[7] = -53.349998f;
	kdops[56].min[8] = 105.189995f;
	kdops[56].max[0] = 30.170000f;
	kdops[56].max[1] = 2.000000f;
	kdops[56].max[2] = 80.769997f;
	kdops[56].max[3] = 30.920000f;
	kdops[56].max[4] = 31.920000f;
	kdops[56].max[5] = -76.019997f;
	kdops[56].max[6] = 82.519997f;
	kdops[56].max[7] = -47.849998f;
	kdops[56].max[8] = 110.689995f;
	kdops[57].min[0] = 81.110001f;
	kdops[57].min[1] = -1.000000f;
	kdops[57].min[2] = 80.419998f;
	kdops[57].min[3] = 79.360001f;
	kdops[57].min[4] = 80.360001f;
	kdops[57].min[5] = -84.169998f;
	kdops[57].min[6] = 79.669998f;
	kdops[57].min[7] = -2.059998f;
	kdops[57].min[8] = 161.779999f;
	kdops[57].max[0] = 84.110001f;
	kdops[57].max[1] = 2.000000f;
	kdops[57].max[2] = 83.419998f;
	kdops[57].max[3] = 84.860001f;
	kdops[57].max[4] = 85.860001f;
	kdops[57].max[5] = -78.669998f;
	kdops[57].max[6] = 85.169998f;
	kdops[57].max[7] = 3.440002f;
	kdops[57].max[8] = 167.279999f;
	kdops[58].min[0] = 41.040001f;
	kdops[58].min[1] = -1.000000f;
	kdops[58].min[2] = 13.480000f;
	kdops[58].min[3] = 39.290001f;
	kdops[58].min[4] = 40.290001f;
	kdops[58].min[5] = -17.230000f;
	kdops[58].min[6] = 12.730000f;
	kdops[58].min[7] = 24.810001f;
	kdops[58].min[8] = 54.770000f;
	kdops[58].max[0] = 44.040001f;
	kdops[58].max[1] = 2.000000f;
	kdops[58].max[2] = 16.480000f;
	kdops[58].max[3] = 44.790001f;
	kdops[58].max[4] = 45.790001f;
	kdops[58].max[5] = -11.730000f;
	kdops[58].max[6] = 18.230000f;
	kdops[58].max[7] = 30.310001f;
	kdops[58].max[8] = 60.270000f;
	kdops[59].min[0] = 41.860001f;
	kdops[59].min[1] = -1.000000f;
	kdops[59].min[2] = 8.560000f;
	kdops[59].min[3] = 40.110001f;
	kdops[59].min[4] = 41.110001f;
	kdops[59].min[5] = -12.310000f;
	kdops[59].min[6] = 7.810000f;
	kdops[59].min[7] = 30.549999f;
	kdops[59].min[8] = 50.670002f;
	kdops[59].max[0] = 44.860001f;
	kdops[59].max[1] = 2.000000f;
	kdops[59].max[2] = 11.560000f;
	kdops[59].max[3] = 45.610001f;
	kdops[59].max[4] = 46.610001f;
	kdops[59].max[5] = -6.810000f;
	kdops[59].max[6] = 13.310000f;
	kdops[59].max[7] = 36.049999f;
	kdops[59].max[8] = 56.170002f;
	kdops[60].min[0] = 35.459999f;
	kdops[60].min[1] = -1.000000f;
	kdops[60].min[2] = 53.310001f;
	kdops[60].min[3] = 33.709999f;
	kdops[60].min[4] = 34.709999f;
	kdops[60].min[5] = -57.060001f;
	kdops[60].min[6] = 52.560001f;
	kdops[60].min[7] = -20.600002f;
	kdops[60].min[8] = 89.020004f;
	kdops[60].max[0] = 38.459999f;
	kdops[60].max[1] = 2.000000f;
	kdops[60].max[2] = 56.310001f;
	kdops[60].max[3] = 39.209999f;
	kdops[60].max[4] = 40.209999f;
	kdops[60].max[5] = -51.560001f;
	kdops[60].max[6] = 58.060001f;
	kdops[60].max[7] = -15.100002f;
	kdops[60].max[8] = 94.520004f;
	kdops[61].min[0] = 12.430000f;
	kdops[61].min[1] = -1.000000f;
	kdops[61].min[2] = 74.800003f;
	kdops[61].min[3] = 10.680000f;
	kdops[61].min[4] = 11.680000f;
	kdops[61].min[5] = -78.550003f;
	kdops[61].min[6] = 74.050003f;
	kdops[61].min[7] = -65.120003f;
	kdops[61].min[8] = 87.480003f;
	kdops[61].max[0] = 15.430000f;
	kdops[61].max[1] = 2.000000f;
	kdops[61].max[2] = 77.800003f;
	kdops[61].max[3] = 16.180000f;
	kdops[61].max[4] = 17.180000f;
	kdops[61].max[5] = -73.050003f;
	kdops[61].max[6] = 79.550003f;
	kdops[61].max[7] = -59.620003f;
	kdops[61].max[8] = 92.980003f;
	kdops[62].min[0] = 69.129997f;
	kdops[62].min[1] = -1.000000f;
	kdops[62].min[2] = 36.099998f;
	kdops[62].min[3] = 67.379997f;
	kdops[62].min[4] = 68.379997f;
	kdops[62].min[5] = -39.849998f;
	kdops[62].min[6] = 35.349998f;
	kdops[62].min[7] = 30.279999f;
	kdops[62].min[8] = 105.479996f;
	kdops[62].max[0] = 72.129997f;
	kdops[62].max[1] = 2.000000f;
	kdops[62].max[2] = 39.099998f;
	kdops[62].max[3] = 72.879997f;
	kdops[62].max[4] = 73.879997f;
	kdops[62].max[5] = -34.349998f;
	kdops[62].max[6] = 40.849998f;
	kdops[62].max[7] = 35.779999f;
	kdops[62].max[8] = 110.979996f;
	kdops[63].min[0] = 71.839996f;
	kdops[63].min[1] = -1.000000f;
	kdops[63].min[2] = 70.949997f;
	kdops[63].min[3] = 70.089996f;
	kdops[63].min[4] = 71.089996f;
	kdops[63].min[5] = -74.699997f;
	kdops[63].min[6] = 70.199997f;
	kdops[63].min[7] = -1.860001f;
	kdops[63].min[8] = 143.039993f;
	kdops[63].max[0] = 74.839996f;
	kdops[63].max[1] = 2.000000f;
	kdops[63].max[2] = 73.949997f;
	kdops[63].max[3] = 75.589996f;
	kdops[63].max[4] = 76.589996f;
	kdops[63].max[5] = -69.199997f;
	kdops[63].max[6] = 75.699997f;
	kdops[63].max[7] = 3.639999f;
	kdops[63].max[8] = 148.539993f;

	return kdops;
}

std::string getKDOPsAsString(sp_uint length, sp_uint spaceSize = 10000u)
{
	DOP18* kdops = getRandomKDOPs(64, spaceSize);

	std::stringstream out;
	out.precision(4);
	out << "DOP18* kdops = ALLOC_NEW_ARRAY(DOP18, " << length << ");" << END_OF_LINE;
	for (sp_uint i = 0; i < length; i++)
	{
		for (sp_uint j = 0; j < 9; j++)
			out << "kdops[" << i << "].min[" << j << "] = " << std::to_string(kdops[i].min[j]) << "f;" << END_OF_LINE;

		for (sp_uint j = 0; j < 9; j++)
			out << "kdops[" << i << "].max[" << j << "] = " << std::to_string(kdops[i].max[j]) << "f;" << END_OF_LINE;
	}

	ALLOC_RELEASE(kdops);
	return out.str();
}

AABB* get1000AABBs()
{
	AABB* aabbs = ALLOC_ARRAY(AABB, 1000);
	aabbs[0] = AABB({ 272.0f, 544.0f, 360.0f }, { 273.0f, 545.0f, 362.0f });
	aabbs[1] = AABB({ 583.0f, 506.0f, 140.0f }, { 584.0f, 518.0f, 144.0f });
	aabbs[2] = AABB({ 791.0f, 759.0f, 216.0f }, { 799.0f, 775.0f, 227.0f });
	aabbs[3] = AABB({ 732.0f, 540.0f, 922.0f }, { 749.0f, 543.0f, 929.0f });
	aabbs[4] = AABB({ 278.0f, 385.0f, 348.0f }, { 284.0f, 388.0f, 375.0f });
	aabbs[5] = AABB({ 751.0f, 818.0f, 769.0f }, { 766.0f, 846.0f, 773.0f });
	aabbs[6] = AABB({ 993.0f, 141.0f, 369.0f }, { 995.0f, 148.0f, 377.0f });
	aabbs[7] = AABB({ 505.0f, 616.0f, 267.0f }, { 508.0f, 621.0f, 268.0f });
	aabbs[8] = AABB({ 216.0f, 383.0f, 325.0f }, { 237.0f, 389.0f, 328.0f });
	aabbs[9] = AABB({ 864.0f, 963.0f, 631.0f }, { 871.0f, 968.0f, 632.0f });
	aabbs[10] = AABB({ 102.0f, 943.0f, 887.0f }, { 110.0f, 950.0f, 888.0f });
	aabbs[11] = AABB({ 861.0f, 479.0f, 121.0f }, { 879.0f, 483.0f, 139.0f });
	aabbs[12] = AABB({ 433.0f, 287.0f, 517.0f }, { 445.0f, 302.0f, 525.0f });
	aabbs[13] = AABB({ 440.0f, 136.0f, 192.0f }, { 444.0f, 140.0f, 193.0f });
	aabbs[14] = AABB({ 312.0f, 1003.0f, 187.0f }, { 333.0f, 1023.0f, 190.0f });
	aabbs[15] = AABB({ 325.0f, 880.0f, 573.0f }, { 329.0f, 903.0f, 574.0f });
	aabbs[16] = AABB({ 436.0f, 884.0f, 581.0f }, { 439.0f, 894.0f, 585.0f });
	aabbs[17] = AABB({ 740.0f, 621.0f, 134.0f }, { 751.0f, 627.0f, 160.0f });
	aabbs[18] = AABB({ 301.0f, 858.0f, 783.0f }, { 304.0f, 866.0f, 785.0f });
	aabbs[19] = AABB({ 388.0f, 68.0f, 974.0f }, { 405.0f, 70.0f, 990.0f });
	aabbs[20] = AABB({ 395.0f, 937.0f, 257.0f }, { 400.0f, 944.0f, 267.0f });
	aabbs[21] = AABB({ 152.0f, 955.0f, 457.0f }, { 157.0f, 965.0f, 481.0f });
	aabbs[22] = AABB({ 916.0f, 280.0f, 224.0f }, { 917.0f, 286.0f, 233.0f });
	aabbs[23] = AABB({ 686.0f, 623.0f, 372.0f }, { 693.0f, 648.0f, 380.0f });
	aabbs[24] = AABB({ 383.0f, 894.0f, 37.0f }, { 396.0f, 895.0f, 47.0f });
	aabbs[25] = AABB({ 605.0f, 506.0f, 731.0f }, { 611.0f, 531.0f, 735.0f });
	aabbs[26] = AABB({ 520.0f, 539.0f, 798.0f }, { 537.0f, 542.0f, 799.0f });
	aabbs[27] = AABB({ 234.0f, 391.0f, 846.0f }, { 238.0f, 392.0f, 852.0f });
	aabbs[28] = AABB({ 589.0f, 587.0f, 521.0f }, { 601.0f, 592.0f, 543.0f });
	aabbs[29] = AABB({ 335.0f, 501.0f, 687.0f }, { 348.0f, 519.0f, 703.0f });
	aabbs[30] = AABB({ 568.0f, 375.0f, 763.0f }, { 587.0f, 378.0f, 770.0f });
	aabbs[31] = AABB({ 22.0f, 213.0f, 113.0f }, { 28.0f, 217.0f, 117.0f });
	aabbs[32] = AABB({ 232.0f, 172.0f, 459.0f }, { 237.0f, 176.0f, 487.0f });
	aabbs[33] = AABB({ 575.0f, 63.0f, 80.0f }, { 577.0f, 76.0f, 81.0f });
	aabbs[34] = AABB({ 63.0f, 160.0f, 892.0f }, { 72.0f, 161.0f, 900.0f });
	aabbs[35] = AABB({ 682.0f, 661.0f, 174.0f }, { 697.0f, 688.0f, 200.0f });
	aabbs[36] = AABB({ 308.0f, 711.0f, 947.0f }, { 314.0f, 725.0f, 953.0f });
	aabbs[37] = AABB({ 918.0f, 783.0f, 709.0f }, { 926.0f, 794.0f, 717.0f });
	aabbs[38] = AABB({ 181.0f, 963.0f, 668.0f }, { 196.0f, 987.0f, 671.0f });
	aabbs[39] = AABB({ 153.0f, 79.0f, 634.0f }, { 165.0f, 91.0f, 648.0f });
	aabbs[40] = AABB({ 376.0f, 140.0f, 968.0f }, { 387.0f, 145.0f, 972.0f });
	aabbs[41] = AABB({ 358.0f, 839.0f, 683.0f }, { 375.0f, 844.0f, 702.0f });
	aabbs[42] = AABB({ 925.0f, 399.0f, 572.0f }, { 944.0f, 406.0f, 574.0f });
	aabbs[43] = AABB({ 943.0f, 857.0f, 755.0f }, { 968.0f, 867.0f, 764.0f });
	aabbs[44] = AABB({ 860.0f, 334.0f, 614.0f }, { 885.0f, 337.0f, 633.0f });
	aabbs[45] = AABB({ 408.0f, 450.0f, 569.0f }, { 411.0f, 472.0f, 579.0f });
	aabbs[46] = AABB({ 878.0f, 123.0f, 928.0f }, { 890.0f, 132.0f, 935.0f });
	aabbs[47] = AABB({ 816.0f, 817.0f, 30.0f }, { 832.0f, 821.0f, 52.0f });
	aabbs[48] = AABB({ 702.0f, 499.0f, 705.0f }, { 711.0f, 506.0f, 711.0f });
	aabbs[49] = AABB({ 961.0f, 620.0f, 455.0f }, { 970.0f, 621.0f, 462.0f });
	aabbs[50] = AABB({ 926.0f, 514.0f, 858.0f }, { 939.0f, 515.0f, 876.0f });
	aabbs[51] = AABB({ 875.0f, 60.0f, 789.0f }, { 899.0f, 89.0f, 791.0f });
	aabbs[52] = AABB({ 436.0f, 203.0f, 916.0f }, { 445.0f, 204.0f, 918.0f });
	aabbs[53] = AABB({ 997.0f, 490.0f, 281.0f }, { 998.0f, 497.0f, 289.0f });
	aabbs[54] = AABB({ 212.0f, 982.0f, 407.0f }, { 219.0f, 985.0f, 427.0f });
	aabbs[55] = AABB({ 47.0f, 884.0f, 577.0f }, { 74.0f, 896.0f, 584.0f });
	aabbs[56] = AABB({ 666.0f, 689.0f, 582.0f }, { 667.0f, 698.0f, 593.0f });
	aabbs[57] = AABB({ 1001.0f, 152.0f, 316.0f }, { 1009.0f, 158.0f, 328.0f });
	aabbs[58] = AABB({ 683.0f, 773.0f, 459.0f }, { 684.0f, 789.0f, 472.0f });
	aabbs[59] = AABB({ 847.0f, 862.0f, 282.0f }, { 863.0f, 863.0f, 285.0f });
	aabbs[60] = AABB({ 70.0f, 137.0f, 908.0f }, { 82.0f, 151.0f, 919.0f });
	aabbs[61] = AABB({ 100.0f, 215.0f, 525.0f }, { 109.0f, 229.0f, 544.0f });
	aabbs[62] = AABB({ 364.0f, 379.0f, 264.0f }, { 371.0f, 385.0f, 266.0f });
	aabbs[63] = AABB({ 120.0f, 351.0f, 342.0f }, { 128.0f, 355.0f, 348.0f });
	aabbs[64] = AABB({ 156.0f, 378.0f, 867.0f }, { 158.0f, 389.0f, 880.0f });
	aabbs[65] = AABB({ 945.0f, 239.0f, 543.0f }, { 964.0f, 240.0f, 552.0f });
	aabbs[66] = AABB({ 996.0f, 711.0f, 589.0f }, { 1019.0f, 712.0f, 599.0f });
	aabbs[67] = AABB({ 590.0f, 686.0f, 481.0f }, { 596.0f, 688.0f, 488.0f });
	aabbs[68] = AABB({ 494.0f, 583.0f, 54.0f }, { 511.0f, 604.0f, 55.0f });
	aabbs[69] = AABB({ 779.0f, 253.0f, 33.0f }, { 792.0f, 260.0f, 42.0f });
	aabbs[70] = AABB({ 791.0f, 683.0f, 683.0f }, { 802.0f, 688.0f, 699.0f });
	aabbs[71] = AABB({ 428.0f, 902.0f, 204.0f }, { 452.0f, 918.0f, 205.0f });
	aabbs[72] = AABB({ 254.0f, 449.0f, 749.0f }, { 255.0f, 465.0f, 753.0f });
	aabbs[73] = AABB({ 779.0f, 272.0f, 257.0f }, { 787.0f, 273.0f, 271.0f });
	aabbs[74] = AABB({ 806.0f, 452.0f, 572.0f }, { 809.0f, 453.0f, 589.0f });
	aabbs[75] = AABB({ 536.0f, 178.0f, 147.0f }, { 545.0f, 188.0f, 172.0f });
	aabbs[76] = AABB({ 538.0f, 13.0f, 726.0f }, { 549.0f, 17.0f, 748.0f });
	aabbs[77] = AABB({ 396.0f, 941.0f, 405.0f }, { 417.0f, 956.0f, 406.0f });
	aabbs[78] = AABB({ 47.0f, 834.0f, 759.0f }, { 59.0f, 837.0f, 764.0f });
	aabbs[79] = AABB({ 342.0f, 343.0f, 95.0f }, { 345.0f, 368.0f, 100.0f });
	aabbs[80] = AABB({ 322.0f, 323.0f, 427.0f }, { 323.0f, 327.0f, 452.0f });
	aabbs[81] = AABB({ 712.0f, 215.0f, 400.0f }, { 737.0f, 241.0f, 404.0f });
	aabbs[82] = AABB({ 626.0f, 447.0f, 573.0f }, { 637.0f, 461.0f, 585.0f });
	aabbs[83] = AABB({ 922.0f, 265.0f, 794.0f }, { 932.0f, 267.0f, 796.0f });
	aabbs[84] = AABB({ 716.0f, 710.0f, 514.0f }, { 729.0f, 718.0f, 535.0f });
	aabbs[85] = AABB({ 276.0f, 733.0f, 456.0f }, { 281.0f, 750.0f, 464.0f });
	aabbs[86] = AABB({ 987.0f, 504.0f, 434.0f }, { 995.0f, 527.0f, 442.0f });
	aabbs[87] = AABB({ 346.0f, 192.0f, 272.0f }, { 360.0f, 197.0f, 293.0f });
	aabbs[88] = AABB({ 288.0f, 443.0f, 919.0f }, { 294.0f, 454.0f, 932.0f });
	aabbs[89] = AABB({ 470.0f, 1005.0f, 500.0f }, { 471.0f, 1006.0f, 513.0f });
	aabbs[90] = AABB({ 118.0f, 238.0f, 891.0f }, { 125.0f, 242.0f, 911.0f });
	aabbs[91] = AABB({ 655.0f, 676.0f, 93.0f }, { 657.0f, 684.0f, 99.0f });
	aabbs[92] = AABB({ 602.0f, 988.0f, 664.0f }, { 611.0f, 1010.0f, 693.0f });
	aabbs[93] = AABB({ 504.0f, 556.0f, 432.0f }, { 526.0f, 575.0f, 458.0f });
	aabbs[94] = AABB({ 451.0f, 637.0f, 615.0f }, { 458.0f, 653.0f, 631.0f });
	aabbs[95] = AABB({ 802.0f, 650.0f, 571.0f }, { 818.0f, 671.0f, 574.0f });
	aabbs[96] = AABB({ 887.0f, 826.0f, 238.0f }, { 888.0f, 827.0f, 249.0f });
	aabbs[97] = AABB({ 123.0f, 846.0f, 693.0f }, { 140.0f, 850.0f, 702.0f });
	aabbs[98] = AABB({ 68.0f, 492.0f, 231.0f }, { 81.0f, 498.0f, 240.0f });
	aabbs[99] = AABB({ 178.0f, 989.0f, 615.0f }, { 180.0f, 993.0f, 635.0f });
	aabbs[100] = AABB({ 1009.0f, 192.0f, 859.0f }, { 1011.0f, 195.0f, 866.0f });
	aabbs[101] = AABB({ 387.0f, 825.0f, 405.0f }, { 388.0f, 838.0f, 416.0f });
	aabbs[102] = AABB({ 525.0f, 839.0f, 249.0f }, { 554.0f, 845.0f, 264.0f });
	aabbs[103] = AABB({ 826.0f, 134.0f, 722.0f }, { 850.0f, 148.0f, 733.0f });
	aabbs[104] = AABB({ 819.0f, 721.0f, 851.0f }, { 820.0f, 736.0f, 854.0f });
	aabbs[105] = AABB({ 623.0f, 274.0f, 761.0f }, { 625.0f, 286.0f, 767.0f });
	aabbs[106] = AABB({ 910.0f, 976.0f, 811.0f }, { 934.0f, 992.0f, 836.0f });
	aabbs[107] = AABB({ 717.0f, 981.0f, 712.0f }, { 737.0f, 998.0f, 723.0f });
	aabbs[108] = AABB({ 887.0f, 164.0f, 606.0f }, { 911.0f, 175.0f, 623.0f });
	aabbs[109] = AABB({ 530.0f, 212.0f, 633.0f }, { 545.0f, 214.0f, 647.0f });
	aabbs[110] = AABB({ 199.0f, 618.0f, 692.0f }, { 200.0f, 626.0f, 703.0f });
	aabbs[111] = AABB({ 970.0f, 1006.0f, 314.0f }, { 973.0f, 1011.0f, 332.0f });
	aabbs[112] = AABB({ 124.0f, 791.0f, 870.0f }, { 136.0f, 803.0f, 886.0f });
	aabbs[113] = AABB({ 613.0f, 563.0f, 418.0f }, { 618.0f, 570.0f, 423.0f });
	aabbs[114] = AABB({ 400.0f, 555.0f, 772.0f }, { 415.0f, 571.0f, 792.0f });
	aabbs[115] = AABB({ 401.0f, 622.0f, 463.0f }, { 417.0f, 645.0f, 464.0f });
	aabbs[116] = AABB({ 745.0f, 349.0f, 601.0f }, { 746.0f, 352.0f, 622.0f });
	aabbs[117] = AABB({ 221.0f, 439.0f, 223.0f }, { 238.0f, 460.0f, 228.0f });
	aabbs[118] = AABB({ 123.0f, 289.0f, 667.0f }, { 144.0f, 298.0f, 677.0f });
	aabbs[119] = AABB({ 666.0f, 476.0f, 436.0f }, { 667.0f, 480.0f, 455.0f });
	aabbs[120] = AABB({ 553.0f, 780.0f, 322.0f }, { 559.0f, 796.0f, 346.0f });
	aabbs[121] = AABB({ 289.0f, 897.0f, 383.0f }, { 307.0f, 911.0f, 405.0f });
	aabbs[122] = AABB({ 151.0f, 659.0f, 580.0f }, { 154.0f, 671.0f, 604.0f });
	aabbs[123] = AABB({ 780.0f, 437.0f, 719.0f }, { 800.0f, 451.0f, 729.0f });
	aabbs[124] = AABB({ 263.0f, 20.0f, 212.0f }, { 267.0f, 48.0f, 234.0f });
	aabbs[125] = AABB({ 449.0f, 447.0f, 77.0f }, { 462.0f, 463.0f, 81.0f });
	aabbs[126] = AABB({ 40.0f, 664.0f, 58.0f }, { 49.0f, 693.0f, 77.0f });
	aabbs[127] = AABB({ 769.0f, 610.0f, 404.0f }, { 797.0f, 612.0f, 415.0f });
	aabbs[128] = AABB({ 510.0f, 466.0f, 40.0f }, { 518.0f, 489.0f, 41.0f });
	aabbs[129] = AABB({ 681.0f, 785.0f, 502.0f }, { 683.0f, 793.0f, 525.0f });
	aabbs[130] = AABB({ 323.0f, 226.0f, 20.0f }, { 326.0f, 243.0f, 39.0f });
	aabbs[131] = AABB({ 664.0f, 59.0f, 271.0f }, { 666.0f, 66.0f, 286.0f });
	aabbs[132] = AABB({ 495.0f, 528.0f, 499.0f }, { 506.0f, 541.0f, 502.0f });
	aabbs[133] = AABB({ 458.0f, 56.0f, 920.0f }, { 460.0f, 67.0f, 927.0f });
	aabbs[134] = AABB({ 581.0f, 132.0f, 144.0f }, { 606.0f, 152.0f, 153.0f });
	aabbs[135] = AABB({ 247.0f, 148.0f, 184.0f }, { 256.0f, 165.0f, 186.0f });
	aabbs[136] = AABB({ 210.0f, 188.0f, 626.0f }, { 233.0f, 212.0f, 627.0f });
	aabbs[137] = AABB({ 210.0f, 545.0f, 232.0f }, { 221.0f, 547.0f, 249.0f });
	aabbs[138] = AABB({ 739.0f, 763.0f, 21.0f }, { 743.0f, 790.0f, 38.0f });
	aabbs[139] = AABB({ 911.0f, 297.0f, 233.0f }, { 914.0f, 310.0f, 241.0f });
	aabbs[140] = AABB({ 622.0f, 874.0f, 735.0f }, { 637.0f, 895.0f, 748.0f });
	aabbs[141] = AABB({ 535.0f, 90.0f, 923.0f }, { 540.0f, 101.0f, 926.0f });
	aabbs[142] = AABB({ 547.0f, 767.0f, 530.0f }, { 562.0f, 770.0f, 534.0f });
	aabbs[143] = AABB({ 643.0f, 672.0f, 923.0f }, { 658.0f, 691.0f, 924.0f });
	aabbs[144] = AABB({ 719.0f, 876.0f, 901.0f }, { 731.0f, 881.0f, 902.0f });
	aabbs[145] = AABB({ 570.0f, 411.0f, 168.0f }, { 573.0f, 428.0f, 169.0f });
	aabbs[146] = AABB({ 276.0f, 761.0f, 468.0f }, { 281.0f, 784.0f, 479.0f });
	aabbs[147] = AABB({ 106.0f, 668.0f, 228.0f }, { 126.0f, 671.0f, 232.0f });
	aabbs[148] = AABB({ 409.0f, 1016.0f, 527.0f }, { 410.0f, 1019.0f, 536.0f });
	aabbs[149] = AABB({ 396.0f, 530.0f, 648.0f }, { 397.0f, 532.0f, 669.0f });
	aabbs[150] = AABB({ 570.0f, 349.0f, 754.0f }, { 578.0f, 359.0f, 766.0f });
	aabbs[151] = AABB({ 120.0f, 610.0f, 112.0f }, { 126.0f, 611.0f, 117.0f });
	aabbs[152] = AABB({ 198.0f, 172.0f, 667.0f }, { 218.0f, 178.0f, 684.0f });
	aabbs[153] = AABB({ 652.0f, 890.0f, 558.0f }, { 674.0f, 895.0f, 580.0f });
	aabbs[154] = AABB({ 327.0f, 979.0f, 217.0f }, { 338.0f, 992.0f, 235.0f });
	aabbs[155] = AABB({ 413.0f, 17.0f, 512.0f }, { 418.0f, 24.0f, 521.0f });
	aabbs[156] = AABB({ 619.0f, 735.0f, 107.0f }, { 620.0f, 736.0f, 124.0f });
	aabbs[157] = AABB({ 541.0f, 138.0f, 549.0f }, { 563.0f, 141.0f, 552.0f });
	aabbs[158] = AABB({ 357.0f, 30.0f, 208.0f }, { 379.0f, 49.0f, 216.0f });
	aabbs[159] = AABB({ 875.0f, 870.0f, 342.0f }, { 880.0f, 880.0f, 363.0f });
	aabbs[160] = AABB({ 230.0f, 145.0f, 542.0f }, { 233.0f, 155.0f, 543.0f });
	aabbs[161] = AABB({ 329.0f, 209.0f, 51.0f }, { 335.0f, 233.0f, 52.0f });
	aabbs[162] = AABB({ 251.0f, 767.0f, 1001.0f }, { 270.0f, 794.0f, 1006.0f });
	aabbs[163] = AABB({ 671.0f, 215.0f, 610.0f }, { 677.0f, 232.0f, 626.0f });
	aabbs[164] = AABB({ 487.0f, 141.0f, 1018.0f }, { 497.0f, 142.0f, 1019.0f });
	aabbs[165] = AABB({ 713.0f, 823.0f, 236.0f }, { 718.0f, 824.0f, 244.0f });
	aabbs[166] = AABB({ 558.0f, 328.0f, 558.0f }, { 575.0f, 342.0f, 587.0f });
	aabbs[167] = AABB({ 243.0f, 179.0f, 583.0f }, { 260.0f, 199.0f, 584.0f });
	aabbs[168] = AABB({ 609.0f, 905.0f, 58.0f }, { 630.0f, 914.0f, 74.0f });
	aabbs[169] = AABB({ 728.0f, 324.0f, 456.0f }, { 740.0f, 331.0f, 476.0f });
	aabbs[170] = AABB({ 912.0f, 743.0f, 118.0f }, { 930.0f, 747.0f, 130.0f });
	aabbs[171] = AABB({ 572.0f, 466.0f, 774.0f }, { 586.0f, 470.0f, 784.0f });
	aabbs[172] = AABB({ 327.0f, 250.0f, 867.0f }, { 335.0f, 257.0f, 869.0f });
	aabbs[173] = AABB({ 693.0f, 703.0f, 61.0f }, { 704.0f, 719.0f, 63.0f });
	aabbs[174] = AABB({ 378.0f, 155.0f, 448.0f }, { 394.0f, 156.0f, 450.0f });
	aabbs[175] = AABB({ 611.0f, 618.0f, 590.0f }, { 615.0f, 621.0f, 602.0f });
	aabbs[176] = AABB({ 333.0f, 323.0f, 939.0f }, { 354.0f, 336.0f, 962.0f });
	aabbs[177] = AABB({ 984.0f, 345.0f, 91.0f }, { 996.0f, 369.0f, 112.0f });
	aabbs[178] = AABB({ 150.0f, 512.0f, 55.0f }, { 168.0f, 525.0f, 57.0f });
	aabbs[179] = AABB({ 591.0f, 603.0f, 241.0f }, { 599.0f, 604.0f, 247.0f });
	aabbs[180] = AABB({ 952.0f, 124.0f, 247.0f }, { 962.0f, 127.0f, 251.0f });
	aabbs[181] = AABB({ 448.0f, 328.0f, 619.0f }, { 455.0f, 353.0f, 625.0f });
	aabbs[182] = AABB({ 83.0f, 375.0f, 365.0f }, { 99.0f, 396.0f, 389.0f });
	aabbs[183] = AABB({ 200.0f, 136.0f, 223.0f }, { 205.0f, 158.0f, 237.0f });
	aabbs[184] = AABB({ 33.0f, 974.0f, 989.0f }, { 47.0f, 988.0f, 990.0f });
	aabbs[185] = AABB({ 134.0f, 486.0f, 605.0f }, { 138.0f, 495.0f, 616.0f });
	aabbs[186] = AABB({ 315.0f, 269.0f, 328.0f }, { 317.0f, 278.0f, 331.0f });
	aabbs[187] = AABB({ 209.0f, 67.0f, 692.0f }, { 214.0f, 68.0f, 716.0f });
	aabbs[188] = AABB({ 853.0f, 851.0f, 991.0f }, { 861.0f, 858.0f, 1012.0f });
	aabbs[189] = AABB({ 129.0f, 280.0f, 921.0f }, { 134.0f, 294.0f, 926.0f });
	aabbs[190] = AABB({ 484.0f, 569.0f, 774.0f }, { 494.0f, 571.0f, 796.0f });
	aabbs[191] = AABB({ 24.0f, 47.0f, 360.0f }, { 29.0f, 68.0f, 373.0f });
	aabbs[192] = AABB({ 613.0f, 365.0f, 824.0f }, { 621.0f, 377.0f, 827.0f });
	aabbs[193] = AABB({ 16.0f, 525.0f, 171.0f }, { 17.0f, 532.0f, 172.0f });
	aabbs[194] = AABB({ 580.0f, 976.0f, 504.0f }, { 581.0f, 994.0f, 514.0f });
	aabbs[195] = AABB({ 987.0f, 174.0f, 65.0f }, { 1002.0f, 178.0f, 70.0f });
	aabbs[196] = AABB({ 406.0f, 176.0f, 794.0f }, { 424.0f, 192.0f, 801.0f });
	aabbs[197] = AABB({ 249.0f, 60.0f, 978.0f }, { 261.0f, 73.0f, 979.0f });
	aabbs[198] = AABB({ 980.0f, 396.0f, 800.0f }, { 995.0f, 398.0f, 824.0f });
	aabbs[199] = AABB({ 622.0f, 839.0f, 443.0f }, { 636.0f, 841.0f, 465.0f });
	aabbs[200] = AABB({ 112.0f, 834.0f, 776.0f }, { 114.0f, 844.0f, 784.0f });
	aabbs[201] = AABB({ 787.0f, 618.0f, 182.0f }, { 811.0f, 631.0f, 187.0f });
	aabbs[202] = AABB({ 794.0f, 514.0f, 631.0f }, { 795.0f, 519.0f, 656.0f });
	aabbs[203] = AABB({ 379.0f, 679.0f, 170.0f }, { 384.0f, 703.0f, 175.0f });
	aabbs[204] = AABB({ 482.0f, 504.0f, 567.0f }, { 489.0f, 526.0f, 583.0f });
	aabbs[205] = AABB({ 307.0f, 314.0f, 636.0f }, { 308.0f, 328.0f, 654.0f });
	aabbs[206] = AABB({ 220.0f, 563.0f, 968.0f }, { 227.0f, 585.0f, 977.0f });
	aabbs[207] = AABB({ 664.0f, 342.0f, 981.0f }, { 684.0f, 347.0f, 1003.0f });
	aabbs[208] = AABB({ 747.0f, 565.0f, 564.0f }, { 766.0f, 575.0f, 575.0f });
	aabbs[209] = AABB({ 833.0f, 83.0f, 275.0f }, { 841.0f, 100.0f, 277.0f });
	aabbs[210] = AABB({ 742.0f, 138.0f, 111.0f }, { 749.0f, 155.0f, 120.0f });
	aabbs[211] = AABB({ 215.0f, 468.0f, 953.0f }, { 222.0f, 485.0f, 975.0f });
	aabbs[212] = AABB({ 360.0f, 901.0f, 949.0f }, { 373.0f, 908.0f, 957.0f });
	aabbs[213] = AABB({ 330.0f, 237.0f, 686.0f }, { 336.0f, 248.0f, 689.0f });
	aabbs[214] = AABB({ 437.0f, 86.0f, 421.0f }, { 438.0f, 105.0f, 431.0f });
	aabbs[215] = AABB({ 442.0f, 641.0f, 981.0f }, { 444.0f, 664.0f, 997.0f });
	aabbs[216] = AABB({ 33.0f, 617.0f, 962.0f }, { 38.0f, 630.0f, 977.0f });
	aabbs[217] = AABB({ 808.0f, 96.0f, 324.0f }, { 811.0f, 115.0f, 345.0f });
	aabbs[218] = AABB({ 477.0f, 928.0f, 12.0f }, { 499.0f, 940.0f, 34.0f });
	aabbs[219] = AABB({ 357.0f, 472.0f, 632.0f }, { 358.0f, 498.0f, 646.0f });
	aabbs[220] = AABB({ 649.0f, 241.0f, 506.0f }, { 674.0f, 243.0f, 521.0f });
	aabbs[221] = AABB({ 462.0f, 47.0f, 388.0f }, { 470.0f, 52.0f, 396.0f });
	aabbs[222] = AABB({ 51.0f, 864.0f, 38.0f }, { 54.0f, 882.0f, 49.0f });
	aabbs[223] = AABB({ 887.0f, 883.0f, 886.0f }, { 894.0f, 912.0f, 894.0f });
	aabbs[224] = AABB({ 169.0f, 691.0f, 538.0f }, { 180.0f, 693.0f, 539.0f });
	aabbs[225] = AABB({ 733.0f, 488.0f, 232.0f }, { 749.0f, 505.0f, 235.0f });
	aabbs[226] = AABB({ 954.0f, 212.0f, 577.0f }, { 975.0f, 217.0f, 579.0f });
	aabbs[227] = AABB({ 284.0f, 32.0f, 612.0f }, { 297.0f, 38.0f, 631.0f });
	aabbs[228] = AABB({ 168.0f, 926.0f, 288.0f }, { 169.0f, 929.0f, 314.0f });
	aabbs[229] = AABB({ 229.0f, 666.0f, 761.0f }, { 231.0f, 691.0f, 773.0f });
	aabbs[230] = AABB({ 895.0f, 867.0f, 809.0f }, { 909.0f, 873.0f, 824.0f });
	aabbs[231] = AABB({ 381.0f, 557.0f, 343.0f }, { 392.0f, 561.0f, 360.0f });
	aabbs[232] = AABB({ 373.0f, 202.0f, 79.0f }, { 377.0f, 209.0f, 102.0f });
	aabbs[233] = AABB({ 111.0f, 558.0f, 897.0f }, { 133.0f, 562.0f, 925.0f });
	aabbs[234] = AABB({ 653.0f, 648.0f, 507.0f }, { 659.0f, 651.0f, 508.0f });
	aabbs[235] = AABB({ 62.0f, 649.0f, 952.0f }, { 73.0f, 664.0f, 957.0f });
	aabbs[236] = AABB({ 386.0f, 601.0f, 341.0f }, { 387.0f, 604.0f, 350.0f });
	aabbs[237] = AABB({ 681.0f, 355.0f, 169.0f }, { 691.0f, 368.0f, 183.0f });
	aabbs[238] = AABB({ 807.0f, 824.0f, 522.0f }, { 809.0f, 827.0f, 525.0f });
	aabbs[239] = AABB({ 739.0f, 979.0f, 219.0f }, { 742.0f, 981.0f, 225.0f });
	aabbs[240] = AABB({ 774.0f, 512.0f, 1008.0f }, { 779.0f, 521.0f, 1026.0f });
	aabbs[241] = AABB({ 447.0f, 1019.0f, 893.0f }, { 448.0f, 1026.0f, 904.0f });
	aabbs[242] = AABB({ 1013.0f, 537.0f, 455.0f }, { 1019.0f, 539.0f, 456.0f });
	aabbs[243] = AABB({ 30.0f, 423.0f, 718.0f }, { 52.0f, 450.0f, 735.0f });
	aabbs[244] = AABB({ 808.0f, 415.0f, 810.0f }, { 809.0f, 427.0f, 820.0f });
	aabbs[245] = AABB({ 817.0f, 580.0f, 825.0f }, { 832.0f, 606.0f, 829.0f });
	aabbs[246] = AABB({ 639.0f, 575.0f, 837.0f }, { 652.0f, 583.0f, 856.0f });
	aabbs[247] = AABB({ 974.0f, 129.0f, 695.0f }, { 996.0f, 142.0f, 701.0f });
	aabbs[248] = AABB({ 278.0f, 497.0f, 676.0f }, { 282.0f, 520.0f, 688.0f });
	aabbs[249] = AABB({ 118.0f, 672.0f, 305.0f }, { 121.0f, 677.0f, 310.0f });
	aabbs[250] = AABB({ 396.0f, 988.0f, 679.0f }, { 408.0f, 1000.0f, 703.0f });
	aabbs[251] = AABB({ 712.0f, 602.0f, 579.0f }, { 718.0f, 611.0f, 583.0f });
	aabbs[252] = AABB({ 202.0f, 777.0f, 56.0f }, { 207.0f, 801.0f, 77.0f });
	aabbs[253] = AABB({ 406.0f, 77.0f, 686.0f }, { 411.0f, 102.0f, 692.0f });
	aabbs[254] = AABB({ 132.0f, 888.0f, 283.0f }, { 142.0f, 892.0f, 289.0f });
	aabbs[255] = AABB({ 244.0f, 427.0f, 240.0f }, { 246.0f, 432.0f, 251.0f });
	aabbs[256] = AABB({ 187.0f, 435.0f, 782.0f }, { 210.0f, 446.0f, 805.0f });
	aabbs[257] = AABB({ 730.0f, 767.0f, 163.0f }, { 734.0f, 781.0f, 174.0f });
	aabbs[258] = AABB({ 373.0f, 129.0f, 729.0f }, { 397.0f, 152.0f, 738.0f });
	aabbs[259] = AABB({ 39.0f, 647.0f, 396.0f }, { 40.0f, 648.0f, 404.0f });
	aabbs[260] = AABB({ 633.0f, 790.0f, 550.0f }, { 635.0f, 795.0f, 556.0f });
	aabbs[261] = AABB({ 864.0f, 360.0f, 155.0f }, { 865.0f, 368.0f, 162.0f });
	aabbs[262] = AABB({ 795.0f, 210.0f, 133.0f }, { 814.0f, 222.0f, 136.0f });
	aabbs[263] = AABB({ 96.0f, 407.0f, 220.0f }, { 103.0f, 410.0f, 224.0f });
	aabbs[264] = AABB({ 870.0f, 213.0f, 196.0f }, { 888.0f, 227.0f, 200.0f });
	aabbs[265] = AABB({ 236.0f, 515.0f, 68.0f }, { 244.0f, 541.0f, 88.0f });
	aabbs[266] = AABB({ 1013.0f, 908.0f, 791.0f }, { 1017.0f, 917.0f, 806.0f });
	aabbs[267] = AABB({ 643.0f, 1000.0f, 244.0f }, { 650.0f, 1012.0f, 265.0f });
	aabbs[268] = AABB({ 361.0f, 263.0f, 872.0f }, { 367.0f, 267.0f, 889.0f });
	aabbs[269] = AABB({ 264.0f, 840.0f, 933.0f }, { 276.0f, 842.0f, 951.0f });
	aabbs[270] = AABB({ 871.0f, 489.0f, 873.0f }, { 882.0f, 509.0f, 884.0f });
	aabbs[271] = AABB({ 973.0f, 590.0f, 510.0f }, { 975.0f, 595.0f, 519.0f });
	aabbs[272] = AABB({ 530.0f, 190.0f, 562.0f }, { 540.0f, 195.0f, 581.0f });
	aabbs[273] = AABB({ 368.0f, 226.0f, 92.0f }, { 385.0f, 231.0f, 93.0f });
	aabbs[274] = AABB({ 901.0f, 559.0f, 973.0f }, { 912.0f, 575.0f, 978.0f });
	aabbs[275] = AABB({ 376.0f, 603.0f, 609.0f }, { 380.0f, 624.0f, 619.0f });
	aabbs[276] = AABB({ 408.0f, 18.0f, 418.0f }, { 428.0f, 37.0f, 428.0f });
	aabbs[277] = AABB({ 931.0f, 595.0f, 140.0f }, { 940.0f, 599.0f, 158.0f });
	aabbs[278] = AABB({ 199.0f, 76.0f, 125.0f }, { 203.0f, 77.0f, 127.0f });
	aabbs[279] = AABB({ 926.0f, 534.0f, 868.0f }, { 929.0f, 556.0f, 869.0f });
	aabbs[280] = AABB({ 838.0f, 393.0f, 527.0f }, { 853.0f, 397.0f, 537.0f });
	aabbs[281] = AABB({ 494.0f, 452.0f, 156.0f }, { 519.0f, 455.0f, 159.0f });
	aabbs[282] = AABB({ 847.0f, 925.0f, 666.0f }, { 854.0f, 929.0f, 670.0f });
	aabbs[283] = AABB({ 320.0f, 434.0f, 952.0f }, { 346.0f, 447.0f, 971.0f });
	aabbs[284] = AABB({ 499.0f, 475.0f, 580.0f }, { 502.0f, 476.0f, 599.0f });
	aabbs[285] = AABB({ 540.0f, 327.0f, 368.0f }, { 542.0f, 342.0f, 380.0f });
	aabbs[286] = AABB({ 824.0f, 338.0f, 975.0f }, { 854.0f, 348.0f, 977.0f });
	aabbs[287] = AABB({ 768.0f, 927.0f, 584.0f }, { 773.0f, 929.0f, 589.0f });
	aabbs[288] = AABB({ 620.0f, 121.0f, 632.0f }, { 621.0f, 134.0f, 637.0f });
	aabbs[289] = AABB({ 486.0f, 117.0f, 143.0f }, { 506.0f, 129.0f, 146.0f });
	aabbs[290] = AABB({ 548.0f, 117.0f, 780.0f }, { 554.0f, 127.0f, 791.0f });
	aabbs[291] = AABB({ 717.0f, 274.0f, 1003.0f }, { 746.0f, 288.0f, 1017.0f });
	aabbs[292] = AABB({ 702.0f, 795.0f, 850.0f }, { 712.0f, 824.0f, 854.0f });
	aabbs[293] = AABB({ 670.0f, 636.0f, 357.0f }, { 675.0f, 651.0f, 358.0f });
	aabbs[294] = AABB({ 89.0f, 849.0f, 396.0f }, { 106.0f, 877.0f, 417.0f });
	aabbs[295] = AABB({ 164.0f, 959.0f, 146.0f }, { 166.0f, 970.0f, 147.0f });
	aabbs[296] = AABB({ 561.0f, 557.0f, 1001.0f }, { 580.0f, 558.0f, 1015.0f });
	aabbs[297] = AABB({ 171.0f, 684.0f, 438.0f }, { 176.0f, 685.0f, 447.0f });
	aabbs[298] = AABB({ 280.0f, 762.0f, 468.0f }, { 289.0f, 763.0f, 479.0f });
	aabbs[299] = AABB({ 722.0f, 670.0f, 798.0f }, { 740.0f, 674.0f, 820.0f });
	aabbs[300] = AABB({ 785.0f, 787.0f, 427.0f }, { 786.0f, 798.0f, 432.0f });
	aabbs[301] = AABB({ 445.0f, 991.0f, 62.0f }, { 468.0f, 1009.0f, 73.0f });
	aabbs[302] = AABB({ 295.0f, 759.0f, 355.0f }, { 316.0f, 771.0f, 365.0f });
	aabbs[303] = AABB({ 181.0f, 290.0f, 492.0f }, { 183.0f, 297.0f, 515.0f });
	aabbs[304] = AABB({ 218.0f, 677.0f, 443.0f }, { 245.0f, 690.0f, 467.0f });
	aabbs[305] = AABB({ 442.0f, 780.0f, 144.0f }, { 468.0f, 786.0f, 149.0f });
	aabbs[306] = AABB({ 695.0f, 870.0f, 686.0f }, { 703.0f, 888.0f, 690.0f });
	aabbs[307] = AABB({ 408.0f, 802.0f, 204.0f }, { 409.0f, 810.0f, 219.0f });
	aabbs[308] = AABB({ 461.0f, 538.0f, 135.0f }, { 465.0f, 540.0f, 159.0f });
	aabbs[309] = AABB({ 375.0f, 159.0f, 661.0f }, { 383.0f, 188.0f, 680.0f });
	aabbs[310] = AABB({ 877.0f, 62.0f, 151.0f }, { 884.0f, 83.0f, 168.0f });
	aabbs[311] = AABB({ 938.0f, 84.0f, 341.0f }, { 940.0f, 85.0f, 353.0f });
	aabbs[312] = AABB({ 115.0f, 679.0f, 55.0f }, { 135.0f, 681.0f, 69.0f });
	aabbs[313] = AABB({ 563.0f, 445.0f, 979.0f }, { 566.0f, 446.0f, 995.0f });
	aabbs[314] = AABB({ 889.0f, 288.0f, 196.0f }, { 900.0f, 305.0f, 203.0f });
	aabbs[315] = AABB({ 866.0f, 275.0f, 413.0f }, { 875.0f, 286.0f, 426.0f });
	aabbs[316] = AABB({ 136.0f, 816.0f, 872.0f }, { 143.0f, 825.0f, 876.0f });
	aabbs[317] = AABB({ 683.0f, 1002.0f, 744.0f }, { 699.0f, 1016.0f, 749.0f });
	aabbs[318] = AABB({ 296.0f, 605.0f, 563.0f }, { 305.0f, 616.0f, 572.0f });
	aabbs[319] = AABB({ 207.0f, 904.0f, 725.0f }, { 228.0f, 910.0f, 745.0f });
	aabbs[320] = AABB({ 316.0f, 652.0f, 625.0f }, { 341.0f, 653.0f, 629.0f });
	aabbs[321] = AABB({ 811.0f, 991.0f, 283.0f }, { 821.0f, 1005.0f, 290.0f });
	aabbs[322] = AABB({ 171.0f, 496.0f, 292.0f }, { 187.0f, 505.0f, 305.0f });
	aabbs[323] = AABB({ 514.0f, 369.0f, 913.0f }, { 523.0f, 370.0f, 918.0f });
	aabbs[324] = AABB({ 151.0f, 781.0f, 154.0f }, { 154.0f, 803.0f, 157.0f });
	aabbs[325] = AABB({ 690.0f, 355.0f, 152.0f }, { 703.0f, 376.0f, 153.0f });
	aabbs[326] = AABB({ 329.0f, 269.0f, 187.0f }, { 338.0f, 283.0f, 190.0f });
	aabbs[327] = AABB({ 213.0f, 170.0f, 982.0f }, { 214.0f, 195.0f, 994.0f });
	aabbs[328] = AABB({ 165.0f, 306.0f, 1001.0f }, { 173.0f, 312.0f, 1006.0f });
	aabbs[329] = AABB({ 812.0f, 296.0f, 15.0f }, { 822.0f, 300.0f, 27.0f });
	aabbs[330] = AABB({ 324.0f, 317.0f, 159.0f }, { 333.0f, 338.0f, 161.0f });
	aabbs[331] = AABB({ 668.0f, 853.0f, 533.0f }, { 695.0f, 859.0f, 542.0f });
	aabbs[332] = AABB({ 784.0f, 910.0f, 743.0f }, { 791.0f, 920.0f, 750.0f });
	aabbs[333] = AABB({ 186.0f, 924.0f, 873.0f }, { 193.0f, 939.0f, 875.0f });
	aabbs[334] = AABB({ 474.0f, 427.0f, 893.0f }, { 487.0f, 437.0f, 898.0f });
	aabbs[335] = AABB({ 108.0f, 488.0f, 615.0f }, { 109.0f, 494.0f, 627.0f });
	aabbs[336] = AABB({ 619.0f, 104.0f, 258.0f }, { 633.0f, 117.0f, 271.0f });
	aabbs[337] = AABB({ 354.0f, 837.0f, 280.0f }, { 382.0f, 859.0f, 297.0f });
	aabbs[338] = AABB({ 105.0f, 294.0f, 683.0f }, { 117.0f, 298.0f, 710.0f });
	aabbs[339] = AABB({ 317.0f, 767.0f, 683.0f }, { 324.0f, 771.0f, 688.0f });
	aabbs[340] = AABB({ 456.0f, 394.0f, 426.0f }, { 461.0f, 413.0f, 435.0f });
	aabbs[341] = AABB({ 621.0f, 344.0f, 979.0f }, { 626.0f, 353.0f, 989.0f });
	aabbs[342] = AABB({ 431.0f, 178.0f, 316.0f }, { 435.0f, 197.0f, 318.0f });
	aabbs[343] = AABB({ 13.0f, 548.0f, 990.0f }, { 38.0f, 570.0f, 993.0f });
	aabbs[344] = AABB({ 157.0f, 511.0f, 555.0f }, { 167.0f, 519.0f, 566.0f });
	aabbs[345] = AABB({ 857.0f, 110.0f, 577.0f }, { 858.0f, 129.0f, 590.0f });
	aabbs[346] = AABB({ 770.0f, 704.0f, 322.0f }, { 774.0f, 707.0f, 331.0f });
	aabbs[347] = AABB({ 718.0f, 799.0f, 715.0f }, { 723.0f, 807.0f, 717.0f });
	aabbs[348] = AABB({ 159.0f, 318.0f, 505.0f }, { 162.0f, 322.0f, 516.0f });
	aabbs[349] = AABB({ 280.0f, 99.0f, 702.0f }, { 305.0f, 102.0f, 717.0f });
	aabbs[350] = AABB({ 753.0f, 807.0f, 97.0f }, { 762.0f, 813.0f, 112.0f });
	aabbs[351] = AABB({ 265.0f, 520.0f, 631.0f }, { 275.0f, 531.0f, 646.0f });
	aabbs[352] = AABB({ 395.0f, 988.0f, 40.0f }, { 405.0f, 995.0f, 46.0f });
	aabbs[353] = AABB({ 404.0f, 13.0f, 770.0f }, { 411.0f, 23.0f, 788.0f });
	aabbs[354] = AABB({ 752.0f, 164.0f, 230.0f }, { 754.0f, 173.0f, 232.0f });
	aabbs[355] = AABB({ 320.0f, 173.0f, 760.0f }, { 322.0f, 174.0f, 768.0f });
	aabbs[356] = AABB({ 340.0f, 956.0f, 228.0f }, { 345.0f, 957.0f, 232.0f });
	aabbs[357] = AABB({ 666.0f, 741.0f, 292.0f }, { 681.0f, 746.0f, 300.0f });
	aabbs[358] = AABB({ 373.0f, 714.0f, 291.0f }, { 383.0f, 728.0f, 302.0f });
	aabbs[359] = AABB({ 82.0f, 696.0f, 369.0f }, { 88.0f, 709.0f, 379.0f });
	aabbs[360] = AABB({ 498.0f, 262.0f, 413.0f }, { 499.0f, 276.0f, 433.0f });
	aabbs[361] = AABB({ 414.0f, 93.0f, 381.0f }, { 418.0f, 95.0f, 398.0f });
	aabbs[362] = AABB({ 246.0f, 274.0f, 372.0f }, { 247.0f, 285.0f, 382.0f });
	aabbs[363] = AABB({ 411.0f, 251.0f, 437.0f }, { 437.0f, 274.0f, 444.0f });
	aabbs[364] = AABB({ 757.0f, 613.0f, 187.0f }, { 761.0f, 619.0f, 206.0f });
	aabbs[365] = AABB({ 829.0f, 826.0f, 640.0f }, { 841.0f, 848.0f, 644.0f });
	aabbs[366] = AABB({ 264.0f, 299.0f, 204.0f }, { 267.0f, 306.0f, 206.0f });
	aabbs[367] = AABB({ 168.0f, 614.0f, 551.0f }, { 192.0f, 616.0f, 561.0f });
	aabbs[368] = AABB({ 253.0f, 235.0f, 978.0f }, { 272.0f, 249.0f, 986.0f });
	aabbs[369] = AABB({ 779.0f, 251.0f, 391.0f }, { 792.0f, 253.0f, 396.0f });
	aabbs[370] = AABB({ 904.0f, 406.0f, 101.0f }, { 912.0f, 411.0f, 116.0f });
	aabbs[371] = AABB({ 806.0f, 591.0f, 459.0f }, { 812.0f, 595.0f, 466.0f });
	aabbs[372] = AABB({ 803.0f, 30.0f, 410.0f }, { 815.0f, 37.0f, 414.0f });
	aabbs[373] = AABB({ 752.0f, 987.0f, 771.0f }, { 763.0f, 1003.0f, 793.0f });
	aabbs[374] = AABB({ 242.0f, 961.0f, 607.0f }, { 246.0f, 971.0f, 608.0f });
	aabbs[375] = AABB({ 741.0f, 650.0f, 793.0f }, { 754.0f, 651.0f, 800.0f });
	aabbs[376] = AABB({ 234.0f, 131.0f, 881.0f }, { 247.0f, 132.0f, 899.0f });
	aabbs[377] = AABB({ 786.0f, 663.0f, 529.0f }, { 789.0f, 688.0f, 532.0f });
	aabbs[378] = AABB({ 241.0f, 890.0f, 725.0f }, { 257.0f, 894.0f, 728.0f });
	aabbs[379] = AABB({ 912.0f, 225.0f, 991.0f }, { 923.0f, 226.0f, 1005.0f });
	aabbs[380] = AABB({ 270.0f, 692.0f, 73.0f }, { 283.0f, 713.0f, 88.0f });
	aabbs[381] = AABB({ 251.0f, 701.0f, 252.0f }, { 268.0f, 708.0f, 259.0f });
	aabbs[382] = AABB({ 737.0f, 461.0f, 305.0f }, { 740.0f, 478.0f, 332.0f });
	aabbs[383] = AABB({ 832.0f, 741.0f, 667.0f }, { 833.0f, 754.0f, 675.0f });
	aabbs[384] = AABB({ 688.0f, 448.0f, 887.0f }, { 689.0f, 455.0f, 898.0f });
	aabbs[385] = AABB({ 354.0f, 566.0f, 648.0f }, { 361.0f, 578.0f, 656.0f });
	aabbs[386] = AABB({ 560.0f, 923.0f, 804.0f }, { 561.0f, 943.0f, 814.0f });
	aabbs[387] = AABB({ 225.0f, 813.0f, 334.0f }, { 230.0f, 818.0f, 340.0f });
	aabbs[388] = AABB({ 94.0f, 611.0f, 726.0f }, { 109.0f, 624.0f, 731.0f });
	aabbs[389] = AABB({ 541.0f, 842.0f, 247.0f }, { 547.0f, 848.0f, 255.0f });
	aabbs[390] = AABB({ 190.0f, 420.0f, 780.0f }, { 193.0f, 431.0f, 802.0f });
	aabbs[391] = AABB({ 372.0f, 154.0f, 121.0f }, { 389.0f, 170.0f, 123.0f });
	aabbs[392] = AABB({ 152.0f, 139.0f, 492.0f }, { 166.0f, 149.0f, 496.0f });
	aabbs[393] = AABB({ 418.0f, 192.0f, 331.0f }, { 421.0f, 219.0f, 336.0f });
	aabbs[394] = AABB({ 796.0f, 162.0f, 861.0f }, { 801.0f, 167.0f, 876.0f });
	aabbs[395] = AABB({ 547.0f, 246.0f, 853.0f }, { 556.0f, 250.0f, 855.0f });
	aabbs[396] = AABB({ 790.0f, 446.0f, 367.0f }, { 794.0f, 447.0f, 369.0f });
	aabbs[397] = AABB({ 830.0f, 857.0f, 434.0f }, { 838.0f, 874.0f, 449.0f });
	aabbs[398] = AABB({ 624.0f, 255.0f, 803.0f }, { 626.0f, 263.0f, 809.0f });
	aabbs[399] = AABB({ 373.0f, 133.0f, 765.0f }, { 395.0f, 134.0f, 788.0f });
	aabbs[400] = AABB({ 100.0f, 945.0f, 169.0f }, { 116.0f, 951.0f, 183.0f });
	aabbs[401] = AABB({ 87.0f, 317.0f, 726.0f }, { 106.0f, 334.0f, 739.0f });
	aabbs[402] = AABB({ 855.0f, 783.0f, 898.0f }, { 866.0f, 785.0f, 916.0f });
	aabbs[403] = AABB({ 449.0f, 576.0f, 236.0f }, { 460.0f, 584.0f, 246.0f });
	aabbs[404] = AABB({ 750.0f, 298.0f, 675.0f }, { 756.0f, 299.0f, 678.0f });
	aabbs[405] = AABB({ 750.0f, 659.0f, 518.0f }, { 763.0f, 673.0f, 539.0f });
	aabbs[406] = AABB({ 204.0f, 236.0f, 547.0f }, { 211.0f, 252.0f, 573.0f });
	aabbs[407] = AABB({ 696.0f, 646.0f, 341.0f }, { 699.0f, 649.0f, 366.0f });
	aabbs[408] = AABB({ 161.0f, 694.0f, 840.0f }, { 162.0f, 702.0f, 865.0f });
	aabbs[409] = AABB({ 612.0f, 577.0f, 946.0f }, { 624.0f, 601.0f, 958.0f });
	aabbs[410] = AABB({ 630.0f, 538.0f, 211.0f }, { 654.0f, 561.0f, 220.0f });
	aabbs[411] = AABB({ 396.0f, 85.0f, 738.0f }, { 397.0f, 107.0f, 745.0f });
	aabbs[412] = AABB({ 722.0f, 105.0f, 885.0f }, { 725.0f, 115.0f, 890.0f });
	aabbs[413] = AABB({ 841.0f, 170.0f, 812.0f }, { 843.0f, 194.0f, 819.0f });
	aabbs[414] = AABB({ 413.0f, 724.0f, 476.0f }, { 436.0f, 743.0f, 478.0f });
	aabbs[415] = AABB({ 839.0f, 373.0f, 696.0f }, { 853.0f, 384.0f, 697.0f });
	aabbs[416] = AABB({ 176.0f, 278.0f, 328.0f }, { 179.0f, 288.0f, 338.0f });
	aabbs[417] = AABB({ 882.0f, 88.0f, 969.0f }, { 900.0f, 109.0f, 973.0f });
	aabbs[418] = AABB({ 254.0f, 97.0f, 306.0f }, { 278.0f, 114.0f, 315.0f });
	aabbs[419] = AABB({ 899.0f, 941.0f, 458.0f }, { 914.0f, 953.0f, 466.0f });
	aabbs[420] = AABB({ 580.0f, 876.0f, 103.0f }, { 586.0f, 880.0f, 105.0f });
	aabbs[421] = AABB({ 583.0f, 192.0f, 296.0f }, { 588.0f, 195.0f, 312.0f });
	aabbs[422] = AABB({ 536.0f, 231.0f, 272.0f }, { 537.0f, 248.0f, 275.0f });
	aabbs[423] = AABB({ 75.0f, 964.0f, 718.0f }, { 89.0f, 981.0f, 721.0f });
	aabbs[424] = AABB({ 17.0f, 473.0f, 576.0f }, { 22.0f, 478.0f, 602.0f });
	aabbs[425] = AABB({ 664.0f, 180.0f, 728.0f }, { 688.0f, 183.0f, 735.0f });
	aabbs[426] = AABB({ 218.0f, 527.0f, 765.0f }, { 226.0f, 543.0f, 776.0f });
	aabbs[427] = AABB({ 57.0f, 893.0f, 780.0f }, { 67.0f, 914.0f, 794.0f });
	aabbs[428] = AABB({ 654.0f, 903.0f, 485.0f }, { 669.0f, 905.0f, 486.0f });
	aabbs[429] = AABB({ 765.0f, 350.0f, 791.0f }, { 779.0f, 370.0f, 798.0f });
	aabbs[430] = AABB({ 95.0f, 841.0f, 379.0f }, { 115.0f, 846.0f, 399.0f });
	aabbs[431] = AABB({ 926.0f, 820.0f, 278.0f }, { 934.0f, 840.0f, 294.0f });
	aabbs[432] = AABB({ 324.0f, 100.0f, 716.0f }, { 332.0f, 102.0f, 724.0f });
	aabbs[433] = AABB({ 323.0f, 28.0f, 234.0f }, { 328.0f, 36.0f, 235.0f });
	aabbs[434] = AABB({ 389.0f, 653.0f, 520.0f }, { 408.0f, 659.0f, 523.0f });
	aabbs[435] = AABB({ 572.0f, 209.0f, 419.0f }, { 594.0f, 210.0f, 428.0f });
	aabbs[436] = AABB({ 17.0f, 759.0f, 309.0f }, { 35.0f, 760.0f, 321.0f });
	aabbs[437] = AABB({ 908.0f, 420.0f, 470.0f }, { 909.0f, 429.0f, 475.0f });
	aabbs[438] = AABB({ 719.0f, 384.0f, 403.0f }, { 723.0f, 408.0f, 432.0f });
	aabbs[439] = AABB({ 986.0f, 942.0f, 676.0f }, { 991.0f, 956.0f, 679.0f });
	aabbs[440] = AABB({ 537.0f, 320.0f, 815.0f }, { 555.0f, 322.0f, 824.0f });
	aabbs[441] = AABB({ 646.0f, 302.0f, 13.0f }, { 647.0f, 308.0f, 30.0f });
	aabbs[442] = AABB({ 666.0f, 762.0f, 920.0f }, { 677.0f, 768.0f, 934.0f });
	aabbs[443] = AABB({ 890.0f, 231.0f, 434.0f }, { 918.0f, 233.0f, 446.0f });
	aabbs[444] = AABB({ 186.0f, 367.0f, 333.0f }, { 209.0f, 379.0f, 340.0f });
	aabbs[445] = AABB({ 936.0f, 161.0f, 375.0f }, { 939.0f, 170.0f, 385.0f });
	aabbs[446] = AABB({ 445.0f, 445.0f, 384.0f }, { 451.0f, 446.0f, 387.0f });
	aabbs[447] = AABB({ 420.0f, 38.0f, 347.0f }, { 449.0f, 42.0f, 348.0f });
	aabbs[448] = AABB({ 649.0f, 164.0f, 855.0f }, { 671.0f, 173.0f, 862.0f });
	aabbs[449] = AABB({ 473.0f, 738.0f, 108.0f }, { 490.0f, 757.0f, 110.0f });
	aabbs[450] = AABB({ 756.0f, 534.0f, 46.0f }, { 773.0f, 545.0f, 55.0f });
	aabbs[451] = AABB({ 550.0f, 873.0f, 450.0f }, { 551.0f, 880.0f, 462.0f });
	aabbs[452] = AABB({ 395.0f, 450.0f, 824.0f }, { 411.0f, 461.0f, 834.0f });
	aabbs[453] = AABB({ 446.0f, 161.0f, 649.0f }, { 453.0f, 169.0f, 659.0f });
	aabbs[454] = AABB({ 395.0f, 283.0f, 289.0f }, { 400.0f, 294.0f, 300.0f });
	aabbs[455] = AABB({ 747.0f, 707.0f, 544.0f }, { 748.0f, 708.0f, 557.0f });
	aabbs[456] = AABB({ 99.0f, 144.0f, 420.0f }, { 103.0f, 150.0f, 429.0f });
	aabbs[457] = AABB({ 423.0f, 825.0f, 326.0f }, { 446.0f, 827.0f, 331.0f });
	aabbs[458] = AABB({ 833.0f, 253.0f, 454.0f }, { 860.0f, 272.0f, 470.0f });
	aabbs[459] = AABB({ 296.0f, 463.0f, 657.0f }, { 304.0f, 479.0f, 666.0f });
	aabbs[460] = AABB({ 793.0f, 890.0f, 847.0f }, { 794.0f, 906.0f, 868.0f });
	aabbs[461] = AABB({ 576.0f, 186.0f, 268.0f }, { 577.0f, 201.0f, 286.0f });
	aabbs[462] = AABB({ 60.0f, 626.0f, 529.0f }, { 64.0f, 637.0f, 530.0f });
	aabbs[463] = AABB({ 419.0f, 722.0f, 415.0f }, { 431.0f, 732.0f, 420.0f });
	aabbs[464] = AABB({ 789.0f, 270.0f, 315.0f }, { 793.0f, 287.0f, 317.0f });
	aabbs[465] = AABB({ 287.0f, 624.0f, 976.0f }, { 288.0f, 641.0f, 978.0f });
	aabbs[466] = AABB({ 350.0f, 486.0f, 434.0f }, { 375.0f, 492.0f, 438.0f });
	aabbs[467] = AABB({ 853.0f, 619.0f, 797.0f }, { 873.0f, 629.0f, 798.0f });
	aabbs[468] = AABB({ 932.0f, 150.0f, 188.0f }, { 947.0f, 162.0f, 196.0f });
	aabbs[469] = AABB({ 305.0f, 167.0f, 745.0f }, { 314.0f, 168.0f, 755.0f });
	aabbs[470] = AABB({ 769.0f, 104.0f, 574.0f }, { 774.0f, 114.0f, 596.0f });
	aabbs[471] = AABB({ 49.0f, 389.0f, 220.0f }, { 58.0f, 390.0f, 229.0f });
	aabbs[472] = AABB({ 455.0f, 794.0f, 401.0f }, { 460.0f, 806.0f, 409.0f });
	aabbs[473] = AABB({ 838.0f, 330.0f, 334.0f }, { 857.0f, 341.0f, 345.0f });
	aabbs[474] = AABB({ 126.0f, 601.0f, 98.0f }, { 134.0f, 611.0f, 112.0f });
	aabbs[475] = AABB({ 367.0f, 398.0f, 724.0f }, { 371.0f, 414.0f, 749.0f });
	aabbs[476] = AABB({ 36.0f, 218.0f, 281.0f }, { 62.0f, 238.0f, 305.0f });
	aabbs[477] = AABB({ 134.0f, 772.0f, 732.0f }, { 136.0f, 787.0f, 735.0f });
	aabbs[478] = AABB({ 697.0f, 852.0f, 560.0f }, { 707.0f, 866.0f, 569.0f });
	aabbs[479] = AABB({ 798.0f, 407.0f, 642.0f }, { 799.0f, 421.0f, 656.0f });
	aabbs[480] = AABB({ 917.0f, 953.0f, 142.0f }, { 933.0f, 956.0f, 162.0f });
	aabbs[481] = AABB({ 563.0f, 817.0f, 777.0f }, { 570.0f, 824.0f, 792.0f });
	aabbs[482] = AABB({ 400.0f, 53.0f, 794.0f }, { 402.0f, 72.0f, 806.0f });
	aabbs[483] = AABB({ 331.0f, 181.0f, 870.0f }, { 337.0f, 182.0f, 872.0f });
	aabbs[484] = AABB({ 140.0f, 484.0f, 575.0f }, { 162.0f, 487.0f, 577.0f });
	aabbs[485] = AABB({ 378.0f, 853.0f, 843.0f }, { 382.0f, 874.0f, 851.0f });
	aabbs[486] = AABB({ 762.0f, 616.0f, 885.0f }, { 763.0f, 629.0f, 906.0f });
	aabbs[487] = AABB({ 650.0f, 267.0f, 70.0f }, { 664.0f, 277.0f, 77.0f });
	aabbs[488] = AABB({ 825.0f, 315.0f, 274.0f }, { 830.0f, 338.0f, 282.0f });
	aabbs[489] = AABB({ 932.0f, 95.0f, 594.0f }, { 946.0f, 121.0f, 595.0f });
	aabbs[490] = AABB({ 695.0f, 961.0f, 246.0f }, { 703.0f, 977.0f, 258.0f });
	aabbs[491] = AABB({ 197.0f, 157.0f, 111.0f }, { 215.0f, 163.0f, 131.0f });
	aabbs[492] = AABB({ 724.0f, 314.0f, 767.0f }, { 738.0f, 318.0f, 768.0f });
	aabbs[493] = AABB({ 639.0f, 733.0f, 845.0f }, { 640.0f, 736.0f, 849.0f });
	aabbs[494] = AABB({ 65.0f, 919.0f, 305.0f }, { 76.0f, 939.0f, 310.0f });
	aabbs[495] = AABB({ 661.0f, 818.0f, 518.0f }, { 666.0f, 846.0f, 542.0f });
	aabbs[496] = AABB({ 490.0f, 437.0f, 650.0f }, { 497.0f, 443.0f, 678.0f });
	aabbs[497] = AABB({ 642.0f, 853.0f, 107.0f }, { 647.0f, 864.0f, 121.0f });
	aabbs[498] = AABB({ 177.0f, 375.0f, 200.0f }, { 187.0f, 384.0f, 207.0f });
	aabbs[499] = AABB({ 37.0f, 582.0f, 423.0f }, { 60.0f, 593.0f, 425.0f });
	aabbs[500] = AABB({ 542.0f, 235.0f, 629.0f }, { 550.0f, 255.0f, 630.0f });
	aabbs[501] = AABB({ 201.0f, 855.0f, 388.0f }, { 207.0f, 859.0f, 400.0f });
	aabbs[502] = AABB({ 377.0f, 219.0f, 664.0f }, { 390.0f, 224.0f, 669.0f });
	aabbs[503] = AABB({ 419.0f, 730.0f, 875.0f }, { 446.0f, 733.0f, 883.0f });
	aabbs[504] = AABB({ 160.0f, 961.0f, 240.0f }, { 168.0f, 968.0f, 260.0f });
	aabbs[505] = AABB({ 943.0f, 496.0f, 214.0f }, { 949.0f, 501.0f, 217.0f });
	aabbs[506] = AABB({ 194.0f, 157.0f, 816.0f }, { 206.0f, 171.0f, 821.0f });
	aabbs[507] = AABB({ 839.0f, 178.0f, 642.0f }, { 861.0f, 192.0f, 669.0f });
	aabbs[508] = AABB({ 993.0f, 935.0f, 815.0f }, { 1013.0f, 936.0f, 831.0f });
	aabbs[509] = AABB({ 319.0f, 207.0f, 487.0f }, { 334.0f, 212.0f, 495.0f });
	aabbs[510] = AABB({ 952.0f, 867.0f, 844.0f }, { 963.0f, 871.0f, 870.0f });
	aabbs[511] = AABB({ 1012.0f, 382.0f, 986.0f }, { 1016.0f, 399.0f, 996.0f });
	aabbs[512] = AABB({ 218.0f, 711.0f, 569.0f }, { 219.0f, 724.0f, 585.0f });
	aabbs[513] = AABB({ 589.0f, 332.0f, 904.0f }, { 595.0f, 337.0f, 917.0f });
	aabbs[514] = AABB({ 881.0f, 514.0f, 309.0f }, { 906.0f, 529.0f, 310.0f });
	aabbs[515] = AABB({ 203.0f, 850.0f, 372.0f }, { 214.0f, 865.0f, 398.0f });
	aabbs[516] = AABB({ 722.0f, 163.0f, 51.0f }, { 728.0f, 170.0f, 53.0f });
	aabbs[517] = AABB({ 237.0f, 519.0f, 11.0f }, { 265.0f, 541.0f, 32.0f });
	aabbs[518] = AABB({ 991.0f, 702.0f, 96.0f }, { 998.0f, 705.0f, 103.0f });
	aabbs[519] = AABB({ 640.0f, 236.0f, 400.0f }, { 656.0f, 237.0f, 422.0f });
	aabbs[520] = AABB({ 426.0f, 488.0f, 248.0f }, { 433.0f, 492.0f, 264.0f });
	aabbs[521] = AABB({ 602.0f, 884.0f, 856.0f }, { 622.0f, 904.0f, 865.0f });
	aabbs[522] = AABB({ 621.0f, 156.0f, 437.0f }, { 633.0f, 162.0f, 453.0f });
	aabbs[523] = AABB({ 513.0f, 562.0f, 434.0f }, { 527.0f, 571.0f, 442.0f });
	aabbs[524] = AABB({ 454.0f, 720.0f, 491.0f }, { 470.0f, 740.0f, 498.0f });
	aabbs[525] = AABB({ 193.0f, 144.0f, 485.0f }, { 213.0f, 156.0f, 502.0f });
	aabbs[526] = AABB({ 830.0f, 722.0f, 682.0f }, { 844.0f, 723.0f, 687.0f });
	aabbs[527] = AABB({ 595.0f, 114.0f, 658.0f }, { 600.0f, 134.0f, 672.0f });
	aabbs[528] = AABB({ 745.0f, 251.0f, 646.0f }, { 759.0f, 259.0f, 657.0f });
	aabbs[529] = AABB({ 197.0f, 331.0f, 290.0f }, { 199.0f, 346.0f, 301.0f });
	aabbs[530] = AABB({ 410.0f, 653.0f, 414.0f }, { 423.0f, 655.0f, 417.0f });
	aabbs[531] = AABB({ 506.0f, 168.0f, 593.0f }, { 510.0f, 190.0f, 596.0f });
	aabbs[532] = AABB({ 765.0f, 98.0f, 648.0f }, { 775.0f, 102.0f, 655.0f });
	aabbs[533] = AABB({ 450.0f, 817.0f, 398.0f }, { 458.0f, 825.0f, 408.0f });
	aabbs[534] = AABB({ 407.0f, 231.0f, 827.0f }, { 408.0f, 254.0f, 829.0f });
	aabbs[535] = AABB({ 147.0f, 434.0f, 477.0f }, { 169.0f, 444.0f, 487.0f });
	aabbs[536] = AABB({ 892.0f, 232.0f, 746.0f }, { 912.0f, 234.0f, 755.0f });
	aabbs[537] = AABB({ 790.0f, 402.0f, 692.0f }, { 800.0f, 406.0f, 717.0f });
	aabbs[538] = AABB({ 129.0f, 142.0f, 604.0f }, { 135.0f, 143.0f, 608.0f });
	aabbs[539] = AABB({ 252.0f, 918.0f, 673.0f }, { 263.0f, 941.0f, 676.0f });
	aabbs[540] = AABB({ 406.0f, 473.0f, 402.0f }, { 416.0f, 486.0f, 422.0f });
	aabbs[541] = AABB({ 923.0f, 978.0f, 831.0f }, { 940.0f, 979.0f, 850.0f });
	aabbs[542] = AABB({ 863.0f, 919.0f, 313.0f }, { 878.0f, 921.0f, 324.0f });
	aabbs[543] = AABB({ 751.0f, 569.0f, 185.0f }, { 759.0f, 574.0f, 195.0f });
	aabbs[544] = AABB({ 934.0f, 344.0f, 855.0f }, { 939.0f, 361.0f, 862.0f });
	aabbs[545] = AABB({ 642.0f, 326.0f, 728.0f }, { 643.0f, 338.0f, 729.0f });
	aabbs[546] = AABB({ 961.0f, 853.0f, 352.0f }, { 979.0f, 855.0f, 364.0f });
	aabbs[547] = AABB({ 287.0f, 200.0f, 453.0f }, { 304.0f, 209.0f, 454.0f });
	aabbs[548] = AABB({ 526.0f, 618.0f, 904.0f }, { 531.0f, 629.0f, 918.0f });
	aabbs[549] = AABB({ 775.0f, 526.0f, 305.0f }, { 777.0f, 532.0f, 310.0f });
	aabbs[550] = AABB({ 902.0f, 289.0f, 106.0f }, { 911.0f, 303.0f, 111.0f });
	aabbs[551] = AABB({ 581.0f, 952.0f, 625.0f }, { 592.0f, 970.0f, 637.0f });
	aabbs[552] = AABB({ 27.0f, 963.0f, 16.0f }, { 30.0f, 990.0f, 29.0f });
	aabbs[553] = AABB({ 640.0f, 782.0f, 470.0f }, { 651.0f, 796.0f, 476.0f });
	aabbs[554] = AABB({ 209.0f, 981.0f, 988.0f }, { 210.0f, 983.0f, 994.0f });
	aabbs[555] = AABB({ 618.0f, 286.0f, 592.0f }, { 634.0f, 289.0f, 593.0f });
	aabbs[556] = AABB({ 290.0f, 969.0f, 281.0f }, { 295.0f, 971.0f, 289.0f });
	aabbs[557] = AABB({ 37.0f, 140.0f, 460.0f }, { 43.0f, 149.0f, 471.0f });
	aabbs[558] = AABB({ 493.0f, 549.0f, 819.0f }, { 522.0f, 558.0f, 831.0f });
	aabbs[559] = AABB({ 129.0f, 378.0f, 777.0f }, { 136.0f, 393.0f, 799.0f });
	aabbs[560] = AABB({ 368.0f, 721.0f, 661.0f }, { 369.0f, 736.0f, 680.0f });
	aabbs[561] = AABB({ 1003.0f, 42.0f, 816.0f }, { 1025.0f, 61.0f, 837.0f });
	aabbs[562] = AABB({ 401.0f, 300.0f, 739.0f }, { 402.0f, 304.0f, 746.0f });
	aabbs[563] = AABB({ 574.0f, 208.0f, 407.0f }, { 593.0f, 209.0f, 436.0f });
	aabbs[564] = AABB({ 586.0f, 608.0f, 449.0f }, { 594.0f, 622.0f, 450.0f });
	aabbs[565] = AABB({ 392.0f, 101.0f, 6.0f }, { 412.0f, 119.0f, 20.0f });
	aabbs[566] = AABB({ 516.0f, 270.0f, 896.0f }, { 530.0f, 280.0f, 900.0f });
	aabbs[567] = AABB({ 930.0f, 24.0f, 729.0f }, { 951.0f, 39.0f, 747.0f });
	aabbs[568] = AABB({ 646.0f, 145.0f, 708.0f }, { 650.0f, 148.0f, 722.0f });
	aabbs[569] = AABB({ 816.0f, 620.0f, 988.0f }, { 835.0f, 633.0f, 1003.0f });
	aabbs[570] = AABB({ 257.0f, 828.0f, 112.0f }, { 281.0f, 843.0f, 115.0f });
	aabbs[571] = AABB({ 36.0f, 739.0f, 90.0f }, { 48.0f, 745.0f, 113.0f });
	aabbs[572] = AABB({ 27.0f, 265.0f, 831.0f }, { 48.0f, 274.0f, 832.0f });
	aabbs[573] = AABB({ 571.0f, 976.0f, 186.0f }, { 581.0f, 992.0f, 210.0f });
	aabbs[574] = AABB({ 71.0f, 362.0f, 642.0f }, { 87.0f, 373.0f, 643.0f });
	aabbs[575] = AABB({ 844.0f, 556.0f, 804.0f }, { 847.0f, 562.0f, 825.0f });
	aabbs[576] = AABB({ 460.0f, 295.0f, 344.0f }, { 472.0f, 299.0f, 364.0f });
	aabbs[577] = AABB({ 781.0f, 844.0f, 774.0f }, { 782.0f, 861.0f, 797.0f });
	aabbs[578] = AABB({ 42.0f, 212.0f, 162.0f }, { 49.0f, 220.0f, 172.0f });
	aabbs[579] = AABB({ 340.0f, 384.0f, 181.0f }, { 341.0f, 385.0f, 186.0f });
	aabbs[580] = AABB({ 609.0f, 617.0f, 807.0f }, { 623.0f, 625.0f, 811.0f });
	aabbs[581] = AABB({ 342.0f, 82.0f, 590.0f }, { 363.0f, 85.0f, 597.0f });
	aabbs[582] = AABB({ 741.0f, 714.0f, 779.0f }, { 761.0f, 727.0f, 783.0f });
	aabbs[583] = AABB({ 105.0f, 47.0f, 390.0f }, { 126.0f, 73.0f, 406.0f });
	aabbs[584] = AABB({ 492.0f, 898.0f, 508.0f }, { 495.0f, 906.0f, 528.0f });
	aabbs[585] = AABB({ 497.0f, 578.0f, 553.0f }, { 505.0f, 593.0f, 554.0f });
	aabbs[586] = AABB({ 815.0f, 116.0f, 158.0f }, { 822.0f, 126.0f, 178.0f });
	aabbs[587] = AABB({ 5.0f, 259.0f, 12.0f }, { 29.0f, 269.0f, 34.0f });
	aabbs[588] = AABB({ 676.0f, 690.0f, 740.0f }, { 678.0f, 693.0f, 766.0f });
	aabbs[589] = AABB({ 617.0f, 661.0f, 14.0f }, { 633.0f, 667.0f, 24.0f });
	aabbs[590] = AABB({ 621.0f, 670.0f, 846.0f }, { 627.0f, 671.0f, 851.0f });
	aabbs[591] = AABB({ 147.0f, 890.0f, 942.0f }, { 148.0f, 904.0f, 960.0f });
	aabbs[592] = AABB({ 289.0f, 28.0f, 766.0f }, { 304.0f, 37.0f, 780.0f });
	aabbs[593] = AABB({ 740.0f, 906.0f, 926.0f }, { 745.0f, 907.0f, 947.0f });
	aabbs[594] = AABB({ 443.0f, 438.0f, 535.0f }, { 453.0f, 450.0f, 555.0f });
	aabbs[595] = AABB({ 520.0f, 945.0f, 770.0f }, { 530.0f, 963.0f, 776.0f });
	aabbs[596] = AABB({ 46.0f, 107.0f, 705.0f }, { 49.0f, 109.0f, 713.0f });
	aabbs[597] = AABB({ 707.0f, 770.0f, 450.0f }, { 712.0f, 772.0f, 453.0f });
	aabbs[598] = AABB({ 232.0f, 756.0f, 93.0f }, { 236.0f, 763.0f, 116.0f });
	aabbs[599] = AABB({ 526.0f, 993.0f, 371.0f }, { 538.0f, 1004.0f, 391.0f });
	aabbs[600] = AABB({ 453.0f, 500.0f, 343.0f }, { 464.0f, 508.0f, 357.0f });
	aabbs[601] = AABB({ 856.0f, 131.0f, 460.0f }, { 870.0f, 147.0f, 465.0f });
	aabbs[602] = AABB({ 968.0f, 103.0f, 827.0f }, { 972.0f, 120.0f, 849.0f });
	aabbs[603] = AABB({ 827.0f, 444.0f, 936.0f }, { 828.0f, 452.0f, 950.0f });
	aabbs[604] = AABB({ 188.0f, 527.0f, 73.0f }, { 203.0f, 544.0f, 88.0f });
	aabbs[605] = AABB({ 917.0f, 452.0f, 880.0f }, { 932.0f, 453.0f, 897.0f });
	aabbs[606] = AABB({ 511.0f, 504.0f, 1024.0f }, { 531.0f, 521.0f, 1025.0f });
	aabbs[607] = AABB({ 148.0f, 296.0f, 670.0f }, { 151.0f, 304.0f, 699.0f });
	aabbs[608] = AABB({ 515.0f, 75.0f, 748.0f }, { 520.0f, 76.0f, 772.0f });
	aabbs[609] = AABB({ 605.0f, 539.0f, 615.0f }, { 610.0f, 553.0f, 619.0f });
	aabbs[610] = AABB({ 699.0f, 759.0f, 364.0f }, { 700.0f, 761.0f, 380.0f });
	aabbs[611] = AABB({ 834.0f, 393.0f, 43.0f }, { 850.0f, 399.0f, 44.0f });
	aabbs[612] = AABB({ 166.0f, 293.0f, 125.0f }, { 192.0f, 308.0f, 128.0f });
	aabbs[613] = AABB({ 295.0f, 166.0f, 513.0f }, { 313.0f, 168.0f, 529.0f });
	aabbs[614] = AABB({ 970.0f, 28.0f, 446.0f }, { 977.0f, 37.0f, 451.0f });
	aabbs[615] = AABB({ 397.0f, 602.0f, 339.0f }, { 398.0f, 611.0f, 357.0f });
	aabbs[616] = AABB({ 791.0f, 132.0f, 250.0f }, { 793.0f, 138.0f, 271.0f });
	aabbs[617] = AABB({ 543.0f, 1014.0f, 105.0f }, { 555.0f, 1016.0f, 125.0f });
	aabbs[618] = AABB({ 415.0f, 258.0f, 985.0f }, { 440.0f, 262.0f, 993.0f });
	aabbs[619] = AABB({ 824.0f, 180.0f, 377.0f }, { 847.0f, 188.0f, 381.0f });
	aabbs[620] = AABB({ 259.0f, 784.0f, 166.0f }, { 265.0f, 799.0f, 182.0f });
	aabbs[621] = AABB({ 1007.0f, 997.0f, 1006.0f }, { 1012.0f, 1002.0f, 1024.0f });
	aabbs[622] = AABB({ 625.0f, 53.0f, 643.0f }, { 632.0f, 61.0f, 658.0f });
	aabbs[623] = AABB({ 309.0f, 215.0f, 1012.0f }, { 315.0f, 235.0f, 1013.0f });
	aabbs[624] = AABB({ 403.0f, 706.0f, 297.0f }, { 422.0f, 719.0f, 324.0f });
	aabbs[625] = AABB({ 958.0f, 119.0f, 120.0f }, { 970.0f, 133.0f, 131.0f });
	aabbs[626] = AABB({ 553.0f, 374.0f, 318.0f }, { 566.0f, 387.0f, 347.0f });
	aabbs[627] = AABB({ 222.0f, 529.0f, 638.0f }, { 228.0f, 553.0f, 651.0f });
	aabbs[628] = AABB({ 469.0f, 120.0f, 442.0f }, { 482.0f, 137.0f, 456.0f });
	aabbs[629] = AABB({ 660.0f, 751.0f, 80.0f }, { 663.0f, 773.0f, 81.0f });
	aabbs[630] = AABB({ 686.0f, 639.0f, 138.0f }, { 691.0f, 663.0f, 139.0f });
	aabbs[631] = AABB({ 440.0f, 791.0f, 70.0f }, { 441.0f, 800.0f, 80.0f });
	aabbs[632] = AABB({ 560.0f, 60.0f, 200.0f }, { 561.0f, 79.0f, 215.0f });
	aabbs[633] = AABB({ 333.0f, 283.0f, 37.0f }, { 341.0f, 289.0f, 38.0f });
	aabbs[634] = AABB({ 419.0f, 769.0f, 687.0f }, { 428.0f, 782.0f, 689.0f });
	aabbs[635] = AABB({ 877.0f, 779.0f, 992.0f }, { 879.0f, 783.0f, 994.0f });
	aabbs[636] = AABB({ 175.0f, 788.0f, 823.0f }, { 199.0f, 797.0f, 828.0f });
	aabbs[637] = AABB({ 580.0f, 669.0f, 493.0f }, { 582.0f, 677.0f, 496.0f });
	aabbs[638] = AABB({ 553.0f, 870.0f, 801.0f }, { 570.0f, 875.0f, 820.0f });
	aabbs[639] = AABB({ 414.0f, 143.0f, 322.0f }, { 416.0f, 156.0f, 326.0f });
	aabbs[640] = AABB({ 948.0f, 771.0f, 128.0f }, { 967.0f, 772.0f, 132.0f });
	aabbs[641] = AABB({ 523.0f, 17.0f, 670.0f }, { 543.0f, 29.0f, 691.0f });
	aabbs[642] = AABB({ 292.0f, 142.0f, 49.0f }, { 305.0f, 151.0f, 75.0f });
	aabbs[643] = AABB({ 105.0f, 497.0f, 130.0f }, { 107.0f, 510.0f, 157.0f });
	aabbs[644] = AABB({ 440.0f, 871.0f, 267.0f }, { 452.0f, 874.0f, 291.0f });
	aabbs[645] = AABB({ 313.0f, 380.0f, 367.0f }, { 314.0f, 383.0f, 370.0f });
	aabbs[646] = AABB({ 316.0f, 702.0f, 328.0f }, { 317.0f, 708.0f, 350.0f });
	aabbs[647] = AABB({ 756.0f, 380.0f, 191.0f }, { 766.0f, 405.0f, 193.0f });
	aabbs[648] = AABB({ 62.0f, 930.0f, 287.0f }, { 74.0f, 932.0f, 289.0f });
	aabbs[649] = AABB({ 272.0f, 567.0f, 201.0f }, { 280.0f, 577.0f, 203.0f });
	aabbs[650] = AABB({ 21.0f, 447.0f, 647.0f }, { 22.0f, 453.0f, 648.0f });
	aabbs[651] = AABB({ 181.0f, 331.0f, 351.0f }, { 192.0f, 353.0f, 352.0f });
	aabbs[652] = AABB({ 554.0f, 573.0f, 341.0f }, { 580.0f, 583.0f, 358.0f });
	aabbs[653] = AABB({ 383.0f, 722.0f, 696.0f }, { 386.0f, 725.0f, 704.0f });
	aabbs[654] = AABB({ 107.0f, 831.0f, 502.0f }, { 110.0f, 852.0f, 520.0f });
	aabbs[655] = AABB({ 971.0f, 978.0f, 995.0f }, { 972.0f, 984.0f, 998.0f });
	aabbs[656] = AABB({ 970.0f, 259.0f, 218.0f }, { 977.0f, 284.0f, 224.0f });
	aabbs[657] = AABB({ 773.0f, 415.0f, 293.0f }, { 787.0f, 439.0f, 297.0f });
	aabbs[658] = AABB({ 957.0f, 521.0f, 109.0f }, { 965.0f, 542.0f, 134.0f });
	aabbs[659] = AABB({ 680.0f, 855.0f, 839.0f }, { 704.0f, 861.0f, 842.0f });
	aabbs[660] = AABB({ 344.0f, 928.0f, 879.0f }, { 352.0f, 941.0f, 887.0f });
	aabbs[661] = AABB({ 316.0f, 685.0f, 786.0f }, { 339.0f, 692.0f, 787.0f });
	aabbs[662] = AABB({ 598.0f, 158.0f, 513.0f }, { 617.0f, 160.0f, 519.0f });
	aabbs[663] = AABB({ 401.0f, 137.0f, 369.0f }, { 402.0f, 141.0f, 374.0f });
	aabbs[664] = AABB({ 770.0f, 840.0f, 152.0f }, { 790.0f, 845.0f, 153.0f });
	aabbs[665] = AABB({ 522.0f, 88.0f, 395.0f }, { 524.0f, 101.0f, 399.0f });
	aabbs[666] = AABB({ 356.0f, 474.0f, 304.0f }, { 364.0f, 478.0f, 318.0f });
	aabbs[667] = AABB({ 243.0f, 486.0f, 749.0f }, { 261.0f, 499.0f, 755.0f });
	aabbs[668] = AABB({ 466.0f, 681.0f, 610.0f }, { 483.0f, 702.0f, 616.0f });
	aabbs[669] = AABB({ 423.0f, 777.0f, 528.0f }, { 435.0f, 781.0f, 533.0f });
	aabbs[670] = AABB({ 940.0f, 962.0f, 531.0f }, { 943.0f, 974.0f, 551.0f });
	aabbs[671] = AABB({ 104.0f, 34.0f, 173.0f }, { 105.0f, 62.0f, 189.0f });
	aabbs[672] = AABB({ 923.0f, 83.0f, 799.0f }, { 924.0f, 93.0f, 815.0f });
	aabbs[673] = AABB({ 141.0f, 988.0f, 702.0f }, { 153.0f, 1004.0f, 720.0f });
	aabbs[674] = AABB({ 59.0f, 306.0f, 356.0f }, { 67.0f, 307.0f, 372.0f });
	aabbs[675] = AABB({ 533.0f, 718.0f, 700.0f }, { 555.0f, 721.0f, 707.0f });
	aabbs[676] = AABB({ 1004.0f, 259.0f, 527.0f }, { 1013.0f, 275.0f, 535.0f });
	aabbs[677] = AABB({ 993.0f, 174.0f, 338.0f }, { 996.0f, 185.0f, 339.0f });
	aabbs[678] = AABB({ 812.0f, 488.0f, 719.0f }, { 816.0f, 489.0f, 735.0f });
	aabbs[679] = AABB({ 690.0f, 464.0f, 914.0f }, { 692.0f, 482.0f, 916.0f });
	aabbs[680] = AABB({ 759.0f, 459.0f, 722.0f }, { 778.0f, 470.0f, 733.0f });
	aabbs[681] = AABB({ 504.0f, 406.0f, 186.0f }, { 523.0f, 417.0f, 188.0f });
	aabbs[682] = AABB({ 722.0f, 714.0f, 492.0f }, { 734.0f, 724.0f, 506.0f });
	aabbs[683] = AABB({ 955.0f, 374.0f, 334.0f }, { 974.0f, 399.0f, 341.0f });
	aabbs[684] = AABB({ 649.0f, 934.0f, 709.0f }, { 676.0f, 935.0f, 714.0f });
	aabbs[685] = AABB({ 346.0f, 26.0f, 31.0f }, { 357.0f, 34.0f, 33.0f });
	aabbs[686] = AABB({ 571.0f, 203.0f, 311.0f }, { 583.0f, 224.0f, 315.0f });
	aabbs[687] = AABB({ 23.0f, 274.0f, 37.0f }, { 35.0f, 283.0f, 64.0f });
	aabbs[688] = AABB({ 366.0f, 277.0f, 526.0f }, { 380.0f, 286.0f, 527.0f });
	aabbs[689] = AABB({ 729.0f, 982.0f, 951.0f }, { 750.0f, 991.0f, 975.0f });
	aabbs[690] = AABB({ 321.0f, 969.0f, 701.0f }, { 322.0f, 970.0f, 719.0f });
	aabbs[691] = AABB({ 897.0f, 281.0f, 909.0f }, { 912.0f, 292.0f, 925.0f });
	aabbs[692] = AABB({ 892.0f, 948.0f, 53.0f }, { 898.0f, 962.0f, 56.0f });
	aabbs[693] = AABB({ 233.0f, 360.0f, 363.0f }, { 248.0f, 370.0f, 368.0f });
	aabbs[694] = AABB({ 571.0f, 532.0f, 455.0f }, { 583.0f, 550.0f, 463.0f });
	aabbs[695] = AABB({ 299.0f, 201.0f, 598.0f }, { 307.0f, 202.0f, 609.0f });
	aabbs[696] = AABB({ 87.0f, 541.0f, 807.0f }, { 88.0f, 553.0f, 825.0f });
	aabbs[697] = AABB({ 720.0f, 791.0f, 292.0f }, { 734.0f, 796.0f, 296.0f });
	aabbs[698] = AABB({ 780.0f, 137.0f, 27.0f }, { 800.0f, 138.0f, 30.0f });
	aabbs[699] = AABB({ 456.0f, 169.0f, 74.0f }, { 470.0f, 192.0f, 87.0f });
	aabbs[700] = AABB({ 190.0f, 711.0f, 871.0f }, { 210.0f, 726.0f, 892.0f });
	aabbs[701] = AABB({ 964.0f, 325.0f, 429.0f }, { 970.0f, 335.0f, 430.0f });
	aabbs[702] = AABB({ 743.0f, 1000.0f, 11.0f }, { 749.0f, 1014.0f, 35.0f });
	aabbs[703] = AABB({ 931.0f, 527.0f, 924.0f }, { 955.0f, 534.0f, 932.0f });
	aabbs[704] = AABB({ 445.0f, 413.0f, 604.0f }, { 459.0f, 417.0f, 618.0f });
	aabbs[705] = AABB({ 1014.0f, 456.0f, 378.0f }, { 1017.0f, 457.0f, 392.0f });
	aabbs[706] = AABB({ 524.0f, 238.0f, 365.0f }, { 542.0f, 249.0f, 367.0f });
	aabbs[707] = AABB({ 909.0f, 241.0f, 534.0f }, { 924.0f, 246.0f, 560.0f });
	aabbs[708] = AABB({ 194.0f, 582.0f, 603.0f }, { 205.0f, 595.0f, 616.0f });
	aabbs[709] = AABB({ 740.0f, 415.0f, 897.0f }, { 751.0f, 418.0f, 903.0f });
	aabbs[710] = AABB({ 619.0f, 637.0f, 648.0f }, { 624.0f, 652.0f, 661.0f });
	aabbs[711] = AABB({ 885.0f, 479.0f, 95.0f }, { 900.0f, 494.0f, 107.0f });
	aabbs[712] = AABB({ 348.0f, 412.0f, 673.0f }, { 365.0f, 422.0f, 678.0f });
	aabbs[713] = AABB({ 492.0f, 188.0f, 323.0f }, { 508.0f, 201.0f, 326.0f });
	aabbs[714] = AABB({ 328.0f, 106.0f, 199.0f }, { 331.0f, 121.0f, 201.0f });
	aabbs[715] = AABB({ 386.0f, 44.0f, 442.0f }, { 412.0f, 58.0f, 452.0f });
	aabbs[716] = AABB({ 63.0f, 383.0f, 899.0f }, { 71.0f, 399.0f, 902.0f });
	aabbs[717] = AABB({ 643.0f, 728.0f, 197.0f }, { 661.0f, 745.0f, 205.0f });
	aabbs[718] = AABB({ 846.0f, 971.0f, 736.0f }, { 876.0f, 974.0f, 743.0f });
	aabbs[719] = AABB({ 902.0f, 1010.0f, 924.0f }, { 918.0f, 1011.0f, 931.0f });
	aabbs[720] = AABB({ 821.0f, 682.0f, 453.0f }, { 823.0f, 702.0f, 458.0f });
	aabbs[721] = AABB({ 102.0f, 437.0f, 541.0f }, { 125.0f, 438.0f, 547.0f });
	aabbs[722] = AABB({ 655.0f, 164.0f, 836.0f }, { 670.0f, 165.0f, 845.0f });
	aabbs[723] = AABB({ 680.0f, 97.0f, 550.0f }, { 684.0f, 118.0f, 551.0f });
	aabbs[724] = AABB({ 608.0f, 872.0f, 450.0f }, { 630.0f, 881.0f, 454.0f });
	aabbs[725] = AABB({ 924.0f, 369.0f, 323.0f }, { 934.0f, 374.0f, 345.0f });
	aabbs[726] = AABB({ 830.0f, 855.0f, 678.0f }, { 844.0f, 868.0f, 692.0f });
	aabbs[727] = AABB({ 913.0f, 162.0f, 140.0f }, { 918.0f, 173.0f, 144.0f });
	aabbs[728] = AABB({ 946.0f, 670.0f, 842.0f }, { 964.0f, 674.0f, 847.0f });
	aabbs[729] = AABB({ 323.0f, 234.0f, 824.0f }, { 326.0f, 239.0f, 835.0f });
	aabbs[730] = AABB({ 298.0f, 101.0f, 111.0f }, { 301.0f, 102.0f, 123.0f });
	aabbs[731] = AABB({ 570.0f, 275.0f, 536.0f }, { 576.0f, 283.0f, 556.0f });
	aabbs[732] = AABB({ 346.0f, 983.0f, 626.0f }, { 350.0f, 993.0f, 628.0f });
	aabbs[733] = AABB({ 994.0f, 344.0f, 465.0f }, { 1018.0f, 345.0f, 467.0f });
	aabbs[734] = AABB({ 419.0f, 639.0f, 870.0f }, { 420.0f, 643.0f, 877.0f });
	aabbs[735] = AABB({ 831.0f, 859.0f, 324.0f }, { 851.0f, 860.0f, 352.0f });
	aabbs[736] = AABB({ 546.0f, 836.0f, 343.0f }, { 559.0f, 841.0f, 348.0f });
	aabbs[737] = AABB({ 948.0f, 818.0f, 851.0f }, { 960.0f, 819.0f, 853.0f });
	aabbs[738] = AABB({ 684.0f, 973.0f, 192.0f }, { 702.0f, 974.0f, 200.0f });
	aabbs[739] = AABB({ 736.0f, 499.0f, 251.0f }, { 739.0f, 511.0f, 254.0f });
	aabbs[740] = AABB({ 316.0f, 615.0f, 301.0f }, { 328.0f, 631.0f, 306.0f });
	aabbs[741] = AABB({ 321.0f, 155.0f, 251.0f }, { 349.0f, 183.0f, 261.0f });
	aabbs[742] = AABB({ 920.0f, 495.0f, 766.0f }, { 934.0f, 497.0f, 774.0f });
	aabbs[743] = AABB({ 341.0f, 233.0f, 347.0f }, { 349.0f, 236.0f, 368.0f });
	aabbs[744] = AABB({ 971.0f, 492.0f, 464.0f }, { 974.0f, 494.0f, 474.0f });
	aabbs[745] = AABB({ 644.0f, 56.0f, 871.0f }, { 648.0f, 66.0f, 881.0f });
	aabbs[746] = AABB({ 982.0f, 656.0f, 785.0f }, { 984.0f, 657.0f, 791.0f });
	aabbs[747] = AABB({ 486.0f, 779.0f, 102.0f }, { 488.0f, 785.0f, 116.0f });
	aabbs[748] = AABB({ 906.0f, 963.0f, 862.0f }, { 913.0f, 975.0f, 871.0f });
	aabbs[749] = AABB({ 1007.0f, 952.0f, 424.0f }, { 1012.0f, 978.0f, 435.0f });
	aabbs[750] = AABB({ 665.0f, 621.0f, 866.0f }, { 690.0f, 640.0f, 874.0f });
	aabbs[751] = AABB({ 119.0f, 159.0f, 731.0f }, { 127.0f, 178.0f, 737.0f });
	aabbs[752] = AABB({ 654.0f, 36.0f, 808.0f }, { 658.0f, 48.0f, 821.0f });
	aabbs[753] = AABB({ 535.0f, 587.0f, 476.0f }, { 557.0f, 605.0f, 506.0f });
	aabbs[754] = AABB({ 498.0f, 776.0f, 509.0f }, { 514.0f, 783.0f, 519.0f });
	aabbs[755] = AABB({ 121.0f, 40.0f, 142.0f }, { 122.0f, 41.0f, 147.0f });
	aabbs[756] = AABB({ 981.0f, 715.0f, 753.0f }, { 994.0f, 723.0f, 766.0f });
	aabbs[757] = AABB({ 810.0f, 16.0f, 719.0f }, { 832.0f, 24.0f, 747.0f });
	aabbs[758] = AABB({ 23.0f, 716.0f, 100.0f }, { 48.0f, 720.0f, 113.0f });
	aabbs[759] = AABB({ 1010.0f, 437.0f, 698.0f }, { 1016.0f, 447.0f, 719.0f });
	aabbs[760] = AABB({ 745.0f, 683.0f, 745.0f }, { 746.0f, 691.0f, 763.0f });
	aabbs[761] = AABB({ 679.0f, 310.0f, 282.0f }, { 685.0f, 316.0f, 295.0f });
	aabbs[762] = AABB({ 569.0f, 625.0f, 481.0f }, { 573.0f, 637.0f, 484.0f });
	aabbs[763] = AABB({ 786.0f, 51.0f, 758.0f }, { 792.0f, 65.0f, 770.0f });
	aabbs[764] = AABB({ 464.0f, 782.0f, 229.0f }, { 465.0f, 797.0f, 241.0f });
	aabbs[765] = AABB({ 1001.0f, 1008.0f, 701.0f }, { 1002.0f, 1022.0f, 704.0f });
	aabbs[766] = AABB({ 482.0f, 987.0f, 707.0f }, { 486.0f, 990.0f, 721.0f });
	aabbs[767] = AABB({ 130.0f, 92.0f, 449.0f }, { 131.0f, 100.0f, 453.0f });
	aabbs[768] = AABB({ 973.0f, 706.0f, 221.0f }, { 992.0f, 720.0f, 249.0f });
	aabbs[769] = AABB({ 799.0f, 666.0f, 956.0f }, { 816.0f, 667.0f, 975.0f });
	aabbs[770] = AABB({ 500.0f, 489.0f, 795.0f }, { 516.0f, 505.0f, 796.0f });
	aabbs[771] = AABB({ 243.0f, 763.0f, 864.0f }, { 261.0f, 784.0f, 867.0f });
	aabbs[772] = AABB({ 794.0f, 880.0f, 744.0f }, { 798.0f, 898.0f, 757.0f });
	aabbs[773] = AABB({ 415.0f, 193.0f, 947.0f }, { 421.0f, 214.0f, 966.0f });
	aabbs[774] = AABB({ 877.0f, 311.0f, 851.0f }, { 882.0f, 315.0f, 878.0f });
	aabbs[775] = AABB({ 954.0f, 609.0f, 842.0f }, { 967.0f, 612.0f, 853.0f });
	aabbs[776] = AABB({ 945.0f, 1004.0f, 818.0f }, { 948.0f, 1005.0f, 826.0f });
	aabbs[777] = AABB({ 107.0f, 858.0f, 732.0f }, { 109.0f, 859.0f, 740.0f });
	aabbs[778] = AABB({ 899.0f, 489.0f, 205.0f }, { 903.0f, 490.0f, 225.0f });
	aabbs[779] = AABB({ 181.0f, 770.0f, 473.0f }, { 189.0f, 787.0f, 475.0f });
	aabbs[780] = AABB({ 417.0f, 657.0f, 657.0f }, { 426.0f, 658.0f, 671.0f });
	aabbs[781] = AABB({ 15.0f, 19.0f, 906.0f }, { 16.0f, 20.0f, 932.0f });
	aabbs[782] = AABB({ 386.0f, 664.0f, 126.0f }, { 404.0f, 681.0f, 132.0f });
	aabbs[783] = AABB({ 211.0f, 89.0f, 987.0f }, { 220.0f, 95.0f, 1014.0f });
	aabbs[784] = AABB({ 133.0f, 59.0f, 293.0f }, { 147.0f, 68.0f, 317.0f });
	aabbs[785] = AABB({ 418.0f, 471.0f, 27.0f }, { 441.0f, 489.0f, 30.0f });
	aabbs[786] = AABB({ 636.0f, 201.0f, 71.0f }, { 643.0f, 210.0f, 83.0f });
	aabbs[787] = AABB({ 437.0f, 296.0f, 15.0f }, { 439.0f, 302.0f, 27.0f });
	aabbs[788] = AABB({ 772.0f, 477.0f, 460.0f }, { 790.0f, 500.0f, 483.0f });
	aabbs[789] = AABB({ 109.0f, 170.0f, 508.0f }, { 127.0f, 178.0f, 513.0f });
	aabbs[790] = AABB({ 807.0f, 306.0f, 924.0f }, { 832.0f, 311.0f, 940.0f });
	aabbs[791] = AABB({ 276.0f, 938.0f, 635.0f }, { 283.0f, 960.0f, 636.0f });
	aabbs[792] = AABB({ 38.0f, 318.0f, 916.0f }, { 43.0f, 320.0f, 921.0f });
	aabbs[793] = AABB({ 900.0f, 692.0f, 337.0f }, { 901.0f, 708.0f, 349.0f });
	aabbs[794] = AABB({ 793.0f, 330.0f, 846.0f }, { 810.0f, 337.0f, 847.0f });
	aabbs[795] = AABB({ 638.0f, 263.0f, 903.0f }, { 647.0f, 275.0f, 908.0f });
	aabbs[796] = AABB({ 628.0f, 812.0f, 977.0f }, { 641.0f, 814.0f, 978.0f });
	aabbs[797] = AABB({ 93.0f, 629.0f, 940.0f }, { 98.0f, 641.0f, 966.0f });
	aabbs[798] = AABB({ 216.0f, 806.0f, 875.0f }, { 220.0f, 826.0f, 895.0f });
	aabbs[799] = AABB({ 928.0f, 679.0f, 988.0f }, { 951.0f, 684.0f, 1003.0f });
	aabbs[800] = AABB({ 461.0f, 792.0f, 330.0f }, { 471.0f, 795.0f, 331.0f });
	aabbs[801] = AABB({ 673.0f, 408.0f, 712.0f }, { 690.0f, 412.0f, 733.0f });
	aabbs[802] = AABB({ 287.0f, 709.0f, 376.0f }, { 294.0f, 710.0f, 389.0f });
	aabbs[803] = AABB({ 845.0f, 975.0f, 780.0f }, { 859.0f, 980.0f, 782.0f });
	aabbs[804] = AABB({ 248.0f, 656.0f, 485.0f }, { 268.0f, 659.0f, 490.0f });
	aabbs[805] = AABB({ 315.0f, 442.0f, 385.0f }, { 326.0f, 452.0f, 387.0f });
	aabbs[806] = AABB({ 67.0f, 433.0f, 966.0f }, { 79.0f, 435.0f, 976.0f });
	aabbs[807] = AABB({ 419.0f, 539.0f, 222.0f }, { 420.0f, 540.0f, 237.0f });
	aabbs[808] = AABB({ 32.0f, 430.0f, 144.0f }, { 38.0f, 448.0f, 153.0f });
	aabbs[809] = AABB({ 249.0f, 469.0f, 318.0f }, { 258.0f, 485.0f, 325.0f });
	aabbs[810] = AABB({ 358.0f, 992.0f, 283.0f }, { 368.0f, 1014.0f, 284.0f });
	aabbs[811] = AABB({ 1005.0f, 658.0f, 72.0f }, { 1020.0f, 671.0f, 87.0f });
	aabbs[812] = AABB({ 878.0f, 427.0f, 331.0f }, { 904.0f, 431.0f, 333.0f });
	aabbs[813] = AABB({ 459.0f, 82.0f, 513.0f }, { 474.0f, 92.0f, 522.0f });
	aabbs[814] = AABB({ 162.0f, 392.0f, 186.0f }, { 171.0f, 393.0f, 187.0f });
	aabbs[815] = AABB({ 179.0f, 153.0f, 379.0f }, { 183.0f, 164.0f, 385.0f });
	aabbs[816] = AABB({ 953.0f, 897.0f, 595.0f }, { 964.0f, 908.0f, 622.0f });
	aabbs[817] = AABB({ 144.0f, 624.0f, 294.0f }, { 159.0f, 632.0f, 316.0f });
	aabbs[818] = AABB({ 160.0f, 964.0f, 125.0f }, { 172.0f, 966.0f, 140.0f });
	aabbs[819] = AABB({ 118.0f, 123.0f, 612.0f }, { 123.0f, 133.0f, 631.0f });
	aabbs[820] = AABB({ 272.0f, 698.0f, 284.0f }, { 273.0f, 700.0f, 286.0f });
	aabbs[821] = AABB({ 897.0f, 805.0f, 101.0f }, { 907.0f, 814.0f, 114.0f });
	aabbs[822] = AABB({ 973.0f, 475.0f, 788.0f }, { 983.0f, 479.0f, 789.0f });
	aabbs[823] = AABB({ 599.0f, 625.0f, 863.0f }, { 600.0f, 635.0f, 878.0f });
	aabbs[824] = AABB({ 59.0f, 165.0f, 950.0f }, { 84.0f, 170.0f, 972.0f });
	aabbs[825] = AABB({ 723.0f, 936.0f, 482.0f }, { 743.0f, 958.0f, 484.0f });
	aabbs[826] = AABB({ 400.0f, 525.0f, 602.0f }, { 409.0f, 544.0f, 622.0f });
	aabbs[827] = AABB({ 733.0f, 211.0f, 849.0f }, { 734.0f, 232.0f, 850.0f });
	aabbs[828] = AABB({ 339.0f, 149.0f, 92.0f }, { 349.0f, 155.0f, 108.0f });
	aabbs[829] = AABB({ 456.0f, 641.0f, 940.0f }, { 457.0f, 644.0f, 941.0f });
	aabbs[830] = AABB({ 64.0f, 585.0f, 852.0f }, { 88.0f, 596.0f, 854.0f });
	aabbs[831] = AABB({ 880.0f, 781.0f, 249.0f }, { 897.0f, 794.0f, 262.0f });
	aabbs[832] = AABB({ 479.0f, 393.0f, 886.0f }, { 485.0f, 400.0f, 897.0f });
	aabbs[833] = AABB({ 940.0f, 910.0f, 983.0f }, { 947.0f, 938.0f, 997.0f });
	aabbs[834] = AABB({ 484.0f, 994.0f, 628.0f }, { 505.0f, 999.0f, 638.0f });
	aabbs[835] = AABB({ 774.0f, 357.0f, 343.0f }, { 776.0f, 378.0f, 369.0f });
	aabbs[836] = AABB({ 868.0f, 724.0f, 362.0f }, { 894.0f, 726.0f, 367.0f });
	aabbs[837] = AABB({ 127.0f, 126.0f, 401.0f }, { 130.0f, 127.0f, 416.0f });
	aabbs[838] = AABB({ 492.0f, 328.0f, 317.0f }, { 502.0f, 356.0f, 336.0f });
	aabbs[839] = AABB({ 509.0f, 48.0f, 903.0f }, { 514.0f, 50.0f, 911.0f });
	aabbs[840] = AABB({ 320.0f, 190.0f, 526.0f }, { 327.0f, 200.0f, 534.0f });
	aabbs[841] = AABB({ 353.0f, 451.0f, 246.0f }, { 354.0f, 468.0f, 249.0f });
	aabbs[842] = AABB({ 937.0f, 360.0f, 895.0f }, { 939.0f, 378.0f, 899.0f });
	aabbs[843] = AABB({ 944.0f, 496.0f, 145.0f }, { 969.0f, 501.0f, 152.0f });
	aabbs[844] = AABB({ 578.0f, 821.0f, 522.0f }, { 584.0f, 832.0f, 528.0f });
	aabbs[845] = AABB({ 164.0f, 414.0f, 976.0f }, { 168.0f, 417.0f, 984.0f });
	aabbs[846] = AABB({ 348.0f, 778.0f, 555.0f }, { 350.0f, 791.0f, 565.0f });
	aabbs[847] = AABB({ 732.0f, 800.0f, 618.0f }, { 733.0f, 801.0f, 633.0f });
	aabbs[848] = AABB({ 486.0f, 622.0f, 332.0f }, { 499.0f, 633.0f, 346.0f });
	aabbs[849] = AABB({ 737.0f, 127.0f, 979.0f }, { 745.0f, 152.0f, 986.0f });
	aabbs[850] = AABB({ 128.0f, 454.0f, 714.0f }, { 145.0f, 476.0f, 733.0f });
	aabbs[851] = AABB({ 640.0f, 255.0f, 991.0f }, { 654.0f, 269.0f, 1007.0f });
	aabbs[852] = AABB({ 420.0f, 968.0f, 1001.0f }, { 435.0f, 975.0f, 1024.0f });
	aabbs[853] = AABB({ 213.0f, 246.0f, 82.0f }, { 228.0f, 256.0f, 84.0f });
	aabbs[854] = AABB({ 856.0f, 728.0f, 688.0f }, { 876.0f, 754.0f, 694.0f });
	aabbs[855] = AABB({ 506.0f, 417.0f, 752.0f }, { 532.0f, 427.0f, 754.0f });
	aabbs[856] = AABB({ 845.0f, 251.0f, 684.0f }, { 846.0f, 260.0f, 710.0f });
	aabbs[857] = AABB({ 552.0f, 677.0f, 157.0f }, { 556.0f, 692.0f, 162.0f });
	aabbs[858] = AABB({ 579.0f, 211.0f, 906.0f }, { 597.0f, 228.0f, 916.0f });
	aabbs[859] = AABB({ 286.0f, 846.0f, 224.0f }, { 310.0f, 857.0f, 228.0f });
	aabbs[860] = AABB({ 986.0f, 237.0f, 542.0f }, { 993.0f, 239.0f, 545.0f });
	aabbs[861] = AABB({ 486.0f, 674.0f, 254.0f }, { 509.0f, 685.0f, 259.0f });
	aabbs[862] = AABB({ 798.0f, 401.0f, 645.0f }, { 813.0f, 413.0f, 666.0f });
	aabbs[863] = AABB({ 853.0f, 482.0f, 610.0f }, { 866.0f, 505.0f, 622.0f });
	aabbs[864] = AABB({ 197.0f, 917.0f, 35.0f }, { 203.0f, 927.0f, 50.0f });
	aabbs[865] = AABB({ 510.0f, 999.0f, 163.0f }, { 518.0f, 1026.0f, 185.0f });
	aabbs[866] = AABB({ 786.0f, 384.0f, 635.0f }, { 794.0f, 402.0f, 643.0f });
	aabbs[867] = AABB({ 140.0f, 73.0f, 698.0f }, { 149.0f, 79.0f, 718.0f });
	aabbs[868] = AABB({ 515.0f, 129.0f, 764.0f }, { 531.0f, 132.0f, 779.0f });
	aabbs[869] = AABB({ 104.0f, 803.0f, 97.0f }, { 108.0f, 828.0f, 106.0f });
	aabbs[870] = AABB({ 691.0f, 626.0f, 207.0f }, { 710.0f, 634.0f, 216.0f });
	aabbs[871] = AABB({ 175.0f, 620.0f, 300.0f }, { 188.0f, 636.0f, 310.0f });
	aabbs[872] = AABB({ 788.0f, 588.0f, 248.0f }, { 795.0f, 596.0f, 251.0f });
	aabbs[873] = AABB({ 553.0f, 691.0f, 690.0f }, { 572.0f, 703.0f, 691.0f });
	aabbs[874] = AABB({ 538.0f, 833.0f, 395.0f }, { 540.0f, 839.0f, 415.0f });
	aabbs[875] = AABB({ 852.0f, 737.0f, 918.0f }, { 861.0f, 752.0f, 935.0f });
	aabbs[876] = AABB({ 238.0f, 661.0f, 230.0f }, { 241.0f, 678.0f, 231.0f });
	aabbs[877] = AABB({ 207.0f, 796.0f, 847.0f }, { 215.0f, 811.0f, 867.0f });
	aabbs[878] = AABB({ 226.0f, 465.0f, 464.0f }, { 232.0f, 475.0f, 493.0f });
	aabbs[879] = AABB({ 634.0f, 533.0f, 551.0f }, { 647.0f, 554.0f, 565.0f });
	aabbs[880] = AABB({ 304.0f, 989.0f, 308.0f }, { 320.0f, 990.0f, 311.0f });
	aabbs[881] = AABB({ 100.0f, 743.0f, 639.0f }, { 101.0f, 744.0f, 655.0f });
	aabbs[882] = AABB({ 914.0f, 414.0f, 785.0f }, { 936.0f, 415.0f, 794.0f });
	aabbs[883] = AABB({ 757.0f, 737.0f, 229.0f }, { 762.0f, 741.0f, 231.0f });
	aabbs[884] = AABB({ 679.0f, 569.0f, 269.0f }, { 691.0f, 572.0f, 284.0f });
	aabbs[885] = AABB({ 723.0f, 444.0f, 129.0f }, { 727.0f, 445.0f, 133.0f });
	aabbs[886] = AABB({ 355.0f, 251.0f, 859.0f }, { 358.0f, 258.0f, 876.0f });
	aabbs[887] = AABB({ 825.0f, 170.0f, 308.0f }, { 835.0f, 174.0f, 319.0f });
	aabbs[888] = AABB({ 699.0f, 138.0f, 802.0f }, { 707.0f, 143.0f, 811.0f });
	aabbs[889] = AABB({ 249.0f, 847.0f, 790.0f }, { 252.0f, 848.0f, 806.0f });
	aabbs[890] = AABB({ 793.0f, 472.0f, 256.0f }, { 814.0f, 483.0f, 269.0f });
	aabbs[891] = AABB({ 558.0f, 821.0f, 819.0f }, { 560.0f, 826.0f, 822.0f });
	aabbs[892] = AABB({ 310.0f, 888.0f, 851.0f }, { 335.0f, 893.0f, 866.0f });
	aabbs[893] = AABB({ 511.0f, 308.0f, 215.0f }, { 528.0f, 311.0f, 220.0f });
	aabbs[894] = AABB({ 90.0f, 362.0f, 507.0f }, { 98.0f, 368.0f, 510.0f });
	aabbs[895] = AABB({ 959.0f, 407.0f, 777.0f }, { 962.0f, 409.0f, 799.0f });
	aabbs[896] = AABB({ 717.0f, 396.0f, 470.0f }, { 728.0f, 399.0f, 472.0f });
	aabbs[897] = AABB({ 74.0f, 534.0f, 979.0f }, { 83.0f, 545.0f, 1003.0f });
	aabbs[898] = AABB({ 109.0f, 672.0f, 456.0f }, { 123.0f, 674.0f, 461.0f });
	aabbs[899] = AABB({ 782.0f, 797.0f, 510.0f }, { 800.0f, 808.0f, 523.0f });
	aabbs[900] = AABB({ 464.0f, 834.0f, 245.0f }, { 487.0f, 846.0f, 257.0f });
	aabbs[901] = AABB({ 637.0f, 298.0f, 240.0f }, { 645.0f, 314.0f, 245.0f });
	aabbs[902] = AABB({ 630.0f, 605.0f, 27.0f }, { 634.0f, 608.0f, 29.0f });
	aabbs[903] = AABB({ 272.0f, 1005.0f, 430.0f }, { 295.0f, 1019.0f, 436.0f });
	aabbs[904] = AABB({ 482.0f, 716.0f, 427.0f }, { 495.0f, 726.0f, 430.0f });
	aabbs[905] = AABB({ 334.0f, 863.0f, 288.0f }, { 335.0f, 888.0f, 291.0f });
	aabbs[906] = AABB({ 549.0f, 544.0f, 399.0f }, { 558.0f, 548.0f, 400.0f });
	aabbs[907] = AABB({ 674.0f, 225.0f, 760.0f }, { 689.0f, 247.0f, 761.0f });
	aabbs[908] = AABB({ 920.0f, 890.0f, 332.0f }, { 930.0f, 897.0f, 337.0f });
	aabbs[909] = AABB({ 170.0f, 1009.0f, 394.0f }, { 192.0f, 1013.0f, 395.0f });
	aabbs[910] = AABB({ 993.0f, 171.0f, 505.0f }, { 994.0f, 189.0f, 531.0f });
	aabbs[911] = AABB({ 266.0f, 855.0f, 807.0f }, { 276.0f, 860.0f, 819.0f });
	aabbs[912] = AABB({ 110.0f, 303.0f, 718.0f }, { 121.0f, 305.0f, 729.0f });
	aabbs[913] = AABB({ 82.0f, 119.0f, 158.0f }, { 98.0f, 121.0f, 161.0f });
	aabbs[914] = AABB({ 331.0f, 729.0f, 991.0f }, { 336.0f, 733.0f, 1003.0f });
	aabbs[915] = AABB({ 380.0f, 310.0f, 233.0f }, { 385.0f, 320.0f, 251.0f });
	aabbs[916] = AABB({ 553.0f, 607.0f, 272.0f }, { 563.0f, 608.0f, 279.0f });
	aabbs[917] = AABB({ 811.0f, 147.0f, 151.0f }, { 820.0f, 163.0f, 155.0f });
	aabbs[918] = AABB({ 123.0f, 87.0f, 316.0f }, { 130.0f, 100.0f, 322.0f });
	aabbs[919] = AABB({ 194.0f, 366.0f, 329.0f }, { 200.0f, 376.0f, 333.0f });
	aabbs[920] = AABB({ 526.0f, 800.0f, 599.0f }, { 527.0f, 815.0f, 600.0f });
	aabbs[921] = AABB({ 837.0f, 436.0f, 644.0f }, { 863.0f, 448.0f, 655.0f });
	aabbs[922] = AABB({ 280.0f, 157.0f, 198.0f }, { 304.0f, 163.0f, 217.0f });
	aabbs[923] = AABB({ 817.0f, 513.0f, 508.0f }, { 818.0f, 542.0f, 527.0f });
	aabbs[924] = AABB({ 731.0f, 954.0f, 92.0f }, { 744.0f, 977.0f, 98.0f });
	aabbs[925] = AABB({ 793.0f, 430.0f, 391.0f }, { 814.0f, 455.0f, 410.0f });
	aabbs[926] = AABB({ 99.0f, 576.0f, 63.0f }, { 105.0f, 589.0f, 64.0f });
	aabbs[927] = AABB({ 467.0f, 423.0f, 100.0f }, { 475.0f, 430.0f, 112.0f });
	aabbs[928] = AABB({ 711.0f, 472.0f, 924.0f }, { 712.0f, 478.0f, 925.0f });
	aabbs[929] = AABB({ 508.0f, 659.0f, 946.0f }, { 515.0f, 660.0f, 953.0f });
	aabbs[930] = AABB({ 346.0f, 912.0f, 478.0f }, { 353.0f, 921.0f, 485.0f });
	aabbs[931] = AABB({ 358.0f, 705.0f, 461.0f }, { 368.0f, 706.0f, 472.0f });
	aabbs[932] = AABB({ 185.0f, 564.0f, 1001.0f }, { 206.0f, 575.0f, 1009.0f });
	aabbs[933] = AABB({ 934.0f, 96.0f, 718.0f }, { 949.0f, 113.0f, 719.0f });
	aabbs[934] = AABB({ 434.0f, 784.0f, 572.0f }, { 441.0f, 801.0f, 582.0f });
	aabbs[935] = AABB({ 471.0f, 956.0f, 125.0f }, { 479.0f, 971.0f, 126.0f });
	aabbs[936] = AABB({ 184.0f, 69.0f, 562.0f }, { 212.0f, 72.0f, 582.0f });
	aabbs[937] = AABB({ 546.0f, 939.0f, 487.0f }, { 550.0f, 948.0f, 515.0f });
	aabbs[938] = AABB({ 907.0f, 194.0f, 382.0f }, { 913.0f, 200.0f, 397.0f });
	aabbs[939] = AABB({ 366.0f, 444.0f, 860.0f }, { 372.0f, 461.0f, 881.0f });
	aabbs[940] = AABB({ 682.0f, 251.0f, 63.0f }, { 693.0f, 254.0f, 69.0f });
	aabbs[941] = AABB({ 105.0f, 44.0f, 268.0f }, { 109.0f, 45.0f, 275.0f });
	aabbs[942] = AABB({ 419.0f, 614.0f, 442.0f }, { 433.0f, 618.0f, 460.0f });
	aabbs[943] = AABB({ 783.0f, 839.0f, 477.0f }, { 799.0f, 847.0f, 484.0f });
	aabbs[944] = AABB({ 597.0f, 528.0f, 687.0f }, { 602.0f, 533.0f, 705.0f });
	aabbs[945] = AABB({ 141.0f, 825.0f, 829.0f }, { 144.0f, 838.0f, 832.0f });
	aabbs[946] = AABB({ 698.0f, 373.0f, 750.0f }, { 714.0f, 376.0f, 753.0f });
	aabbs[947] = AABB({ 52.0f, 237.0f, 52.0f }, { 53.0f, 240.0f, 54.0f });
	aabbs[948] = AABB({ 987.0f, 341.0f, 172.0f }, { 994.0f, 364.0f, 175.0f });
	aabbs[949] = AABB({ 172.0f, 689.0f, 421.0f }, { 174.0f, 704.0f, 422.0f });
	aabbs[950] = AABB({ 580.0f, 851.0f, 598.0f }, { 587.0f, 862.0f, 619.0f });
	aabbs[951] = AABB({ 196.0f, 737.0f, 173.0f }, { 203.0f, 747.0f, 194.0f });
	aabbs[952] = AABB({ 270.0f, 623.0f, 868.0f }, { 284.0f, 648.0f, 870.0f });
	aabbs[953] = AABB({ 134.0f, 597.0f, 327.0f }, { 154.0f, 607.0f, 330.0f });
	aabbs[954] = AABB({ 364.0f, 389.0f, 171.0f }, { 387.0f, 391.0f, 175.0f });
	aabbs[955] = AABB({ 1003.0f, 354.0f, 553.0f }, { 1020.0f, 371.0f, 579.0f });
	aabbs[956] = AABB({ 520.0f, 739.0f, 565.0f }, { 523.0f, 756.0f, 592.0f });
	aabbs[957] = AABB({ 851.0f, 242.0f, 634.0f }, { 863.0f, 247.0f, 639.0f });
	aabbs[958] = AABB({ 202.0f, 923.0f, 687.0f }, { 213.0f, 938.0f, 707.0f });
	aabbs[959] = AABB({ 916.0f, 981.0f, 957.0f }, { 918.0f, 983.0f, 970.0f });
	aabbs[960] = AABB({ 390.0f, 848.0f, 597.0f }, { 411.0f, 859.0f, 603.0f });
	aabbs[961] = AABB({ 234.0f, 850.0f, 242.0f }, { 245.0f, 863.0f, 243.0f });
	aabbs[962] = AABB({ 106.0f, 212.0f, 614.0f }, { 119.0f, 218.0f, 619.0f });
	aabbs[963] = AABB({ 122.0f, 798.0f, 154.0f }, { 131.0f, 802.0f, 168.0f });
	aabbs[964] = AABB({ 813.0f, 864.0f, 584.0f }, { 834.0f, 867.0f, 597.0f });
	aabbs[965] = AABB({ 286.0f, 429.0f, 239.0f }, { 294.0f, 432.0f, 257.0f });
	aabbs[966] = AABB({ 671.0f, 654.0f, 783.0f }, { 679.0f, 655.0f, 802.0f });
	aabbs[967] = AABB({ 970.0f, 262.0f, 391.0f }, { 986.0f, 265.0f, 397.0f });
	aabbs[968] = AABB({ 46.0f, 105.0f, 312.0f }, { 48.0f, 108.0f, 321.0f });
	aabbs[969] = AABB({ 145.0f, 93.0f, 902.0f }, { 169.0f, 102.0f, 910.0f });
	aabbs[970] = AABB({ 335.0f, 70.0f, 861.0f }, { 338.0f, 78.0f, 862.0f });
	aabbs[971] = AABB({ 697.0f, 234.0f, 59.0f }, { 703.0f, 238.0f, 67.0f });
	aabbs[972] = AABB({ 264.0f, 655.0f, 634.0f }, { 284.0f, 660.0f, 650.0f });
	aabbs[973] = AABB({ 518.0f, 454.0f, 704.0f }, { 529.0f, 455.0f, 708.0f });
	aabbs[974] = AABB({ 745.0f, 128.0f, 458.0f }, { 767.0f, 135.0f, 472.0f });
	aabbs[975] = AABB({ 750.0f, 134.0f, 852.0f }, { 759.0f, 135.0f, 853.0f });
	aabbs[976] = AABB({ 313.0f, 944.0f, 407.0f }, { 322.0f, 951.0f, 419.0f });
	aabbs[977] = AABB({ 954.0f, 770.0f, 899.0f }, { 959.0f, 789.0f, 921.0f });
	aabbs[978] = AABB({ 638.0f, 878.0f, 468.0f }, { 658.0f, 900.0f, 487.0f });
	aabbs[979] = AABB({ 18.0f, 542.0f, 777.0f }, { 38.0f, 553.0f, 781.0f });
	aabbs[980] = AABB({ 63.0f, 723.0f, 830.0f }, { 70.0f, 734.0f, 831.0f });
	aabbs[981] = AABB({ 104.0f, 238.0f, 777.0f }, { 133.0f, 254.0f, 797.0f });
	aabbs[982] = AABB({ 626.0f, 883.0f, 535.0f }, { 637.0f, 899.0f, 540.0f });
	aabbs[983] = AABB({ 666.0f, 791.0f, 869.0f }, { 668.0f, 805.0f, 881.0f });
	aabbs[984] = AABB({ 81.0f, 745.0f, 260.0f }, { 84.0f, 759.0f, 261.0f });
	aabbs[985] = AABB({ 129.0f, 273.0f, 768.0f }, { 154.0f, 294.0f, 777.0f });
	aabbs[986] = AABB({ 452.0f, 197.0f, 556.0f }, { 481.0f, 201.0f, 572.0f });
	aabbs[987] = AABB({ 969.0f, 263.0f, 686.0f }, { 974.0f, 269.0f, 703.0f });
	aabbs[988] = AABB({ 954.0f, 765.0f, 633.0f }, { 958.0f, 780.0f, 634.0f });
	aabbs[989] = AABB({ 156.0f, 750.0f, 188.0f }, { 157.0f, 758.0f, 190.0f });
	aabbs[990] = AABB({ 637.0f, 1003.0f, 661.0f }, { 638.0f, 1024.0f, 662.0f });
	aabbs[991] = AABB({ 309.0f, 269.0f, 1002.0f }, { 332.0f, 270.0f, 1008.0f });
	aabbs[992] = AABB({ 915.0f, 980.0f, 596.0f }, { 919.0f, 997.0f, 597.0f });
	aabbs[993] = AABB({ 485.0f, 499.0f, 795.0f }, { 486.0f, 520.0f, 799.0f });
	aabbs[994] = AABB({ 710.0f, 937.0f, 614.0f }, { 731.0f, 942.0f, 622.0f });
	aabbs[995] = AABB({ 887.0f, 897.0f, 1003.0f }, { 900.0f, 919.0f, 1018.0f });
	aabbs[996] = AABB({ 670.0f, 845.0f, 21.0f }, { 688.0f, 859.0f, 27.0f });
	aabbs[997] = AABB({ 492.0f, 129.0f, 685.0f }, { 506.0f, 149.0f, 686.0f });
	aabbs[998] = AABB({ 39.0f, 1014.0f, 756.0f }, { 53.0f, 1023.0f, 761.0f });
	aabbs[999] = AABB({ 6.0f, 468.0f, 983.0f }, { 18.0f, 478.0f, 984.0f });

	return aabbs;
}

namespace NAMESPACE_PHYSICS_TEST
{
	sp_int comparatorXAxisForQuickSortKDOP(const void* a, const void* b)
	{
		DOP18* obj1 = (DOP18*)a;
		DOP18* obj2 = (DOP18*)b;

		if (obj1->min[0] < obj2->min[0])
			return -1;
		else
			if (obj1->min[0] > obj2->min[0])
				return 1;

		return 0;
	}

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(findCollisions_WithAABB);
		SP_TEST_METHOD_DEF(findCollisions_WithKDOPs);

#ifdef OPENCL_ENABLED
		SP_TEST_METHOD_DEF(axisIdForAABB);
		SP_TEST_METHOD_DEF(buildInputElements1);
		SP_TEST_METHOD_DEF(buildInputElements2);
		SP_TEST_METHOD_DEF(buildInputElements3);
		SP_TEST_METHOD_DEF(buildInputElements4);
		SP_TEST_METHOD_DEF(buildInputElements5);
		SP_TEST_METHOD_DEF(buildInputElements6);
		SP_TEST_METHOD_DEF(findCollisionsGPU_WithAABB);
		SP_TEST_METHOD_DEF(findCollisionsGPU_WithKDOPs);
#endif

	};

	SP_TEST_METHOD(CLASS_NAME, axisIdForAABB)
	{
		SpPhysicSettings::instance()->boundingVolumeType(BoundingVolumeType::AABB);
		SweepAndPrune sap;

		Vec3 axis = Vec3(0.1f, 0.8f, 0.1f);
		normalize(axis);
		sap.updateAxis(axis);
		Assert::AreEqual(1, (sp_int)sap.axis, L"wrong value", LINE_INFO());

		axis = Vec3(0.1f, -0.8f, 0.1f);
		normalize(axis);
		sap.updateAxis(axis);
		Assert::AreEqual(1, (sp_int)sap.axis, L"wrong value", LINE_INFO());

		axis = Vec3(0.8f, 0.1f, 0.1f);
		normalize(axis);
		sap.updateAxis(axis);
		Assert::AreEqual(0, (sp_int)sap.axis, L"wrong value", LINE_INFO());

		axis = Vec3(-0.8f, 0.1f, 0.1f);
		normalize(axis);
		sap.updateAxis(axis);
		Assert::AreEqual(0, (sp_int)sap.axis, L"wrong value", LINE_INFO());

		axis = Vec3(0.1f, 0.1f, 0.9f);
		normalize(axis);
		sap.updateAxis(axis);
		Assert::AreEqual(2, (sp_int)sap.axis, L"wrong value", LINE_INFO());

		axis = Vec3(0.1f, 0.1f, -0.9f);
		normalize(axis);
		sap.updateAxis(axis);
		Assert::AreEqual(2, (sp_int)sap.axis, L"wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, findCollisions_WithAABB)
	{
		sp_uint count = 1000;
		AABB* aabbs = get1000AABBs();

		std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

		SweepAndPruneResult result = SweepAndPrune::findCollisions(aabbs, count);

		std::chrono::high_resolution_clock::time_point currentTime2 = std::chrono::high_resolution_clock::now();
		std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime2 - currentTime);

		Assert::AreEqual(9u, result.length, L"wrong value", LINE_INFO());

		ALLOC_RELEASE(aabbs);
	}

	SP_TEST_METHOD(CLASS_NAME, findCollisions_WithKDOPs)
	{
		for (sp_uint iterations = 0; iterations < 10; iterations++)
		{
			sp_uint length = 50;
			DOP18* kdops = getRandomKDOPs(length);

			AlgorithmSorting::quickSortNative(kdops, length, DOP18_SIZE, comparatorXAxisForQuickSortKDOP);

			std::vector<std::pair<sp_uint, sp_uint>> pairs;
			sp_uint expectedLength = ZERO_UINT;
			for (sp_uint i = 0; i < length; i++)
			{
				DOP18 currentKDOP = kdops[i];

				for (sp_uint j = i + 1; j < length; j++)
					if (currentKDOP.collisionStatus(kdops[j]) == CollisionStatus::INSIDE)
					{
						expectedLength++;
						std::pair<sp_uint, sp_uint> p(i, j);
						pairs.push_back(p);
					}
			}


			SweepAndPruneResult resultCpu;
			resultCpu.indexes = ALLOC_ARRAY(sp_uint, multiplyBy4(length));

			sp_uint* indexes = ALLOC_ARRAY(sp_uint, length);
			for (sp_uint i = ZERO_UINT; i < length; i++)
				indexes[i] = i;

			PerformanceCounter counter; counter.start();

			SweepAndPrune::findCollisions(kdops, indexes, length, &resultCpu);

			sp_longlong elapsedTime = counter.diff();

			Assert::AreEqual(expectedLength, resultCpu.length, L"wrong value", LINE_INFO());

			ALLOC_RELEASE(kdops);
		}
	}

#ifdef OPENCL_ENABLED

	SP_TEST_METHOD(CLASS_NAME, buildInputElements1)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		sp_uint inputLength = 5u;
		DOP18 dops[5u];

		for (sp_uint i = 0; i < inputLength; i++)
			dops[i].translate(Vec3(i * 10.0f, 0.0f, 0.0f));

		const sp_size groupLength = gpu->getGroupLength(inputLength, inputLength);
		const sp_size globalWorkSize[3] = { inputLength, 0, 0 };
		const sp_size localWorkSize[3] = { groupLength, 0, 0 };

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << inputLength;
		buildOptions << " -DINPUT_STRIDE=" << DOP18_STRIDER;
		buildOptions << " -DINPUT_OFFSET=0";

		GpuIndexes* commandIndexes = ALLOC_NEW(GpuIndexes)();
		commandIndexes->init(gpu, buildOptions.str().c_str());
		commandIndexes->setParametersCreateIndexes(inputLength);

		sp_char filename[512];
		currentDirectory(filename, 512);
		directoryAddPath(filename, std::strlen(filename), SP_DIRECTORY_OPENCL_SOURCE, SP_DIRECTORY_OPENCL_SOURCE_LENGTH);
		directoryAddPath(filename, std::strlen(filename), "SweepAndPrune.cl", 16);

		SP_FILE file;
		file.open(filename, std::ios::in);
		const sp_size fileSize = file.length();
		sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
		file.read(source, fileSize);
		file.close();

		cl_program program; 
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, buildOptions.str().c_str(), &program);

		ALLOC_RELEASE(source);

		sp_int boundingVolumeType = (sp_int)BoundingVolumeType::DOP18;

		cl_mem indexesGpu = commandIndexes->execute(ZERO_UINT, NULL, NULL);
		cl_mem elementsGPU = gpu->createBuffer(inputLength * sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		sp_uint indexesCPU[5];
		gpu->commandManager->readBuffer(indexesGpu, inputLength * sizeof(sp_float), indexesCPU);

		sp_size axis = DOP18_AXIS_X;

		cl_event evt;
		GpuCommand* commandBuildElements = gpu->commandManager->createCommand()
			->setInputParameter(dops, inputLength * sizeof(DOP18), CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR)
			->setInputParameter(&boundingVolumeType, sizeof(sp_int), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR)
			->setInputParameter(indexesGpu, sizeof(sp_uint) * inputLength)
			->setInputParameter(&inputLength, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR)
			->setInputParameter(elementsGPU, inputLength * sizeof(sp_float))
			->buildFromProgram(program, "buildInputElements")
			->execute(1, globalWorkSize, localWorkSize, &axis, ZERO_UINT, NULL, &evt);

		sp_float elements[5];
		gpu->commandManager->readBuffer(elementsGPU, inputLength * sizeof(sp_float), elements, ONE_UINT, &evt);
		gpu->releaseEvent(evt);

		for (sp_uint i = 0; i < inputLength; i++)
			Assert::AreEqual((sp_float)(i * 10u) - 0.5f, elements[i], L"Wrong value.", LINE_INFO());

		sp_mem_delete(commandBuildElements, GpuCommand);
	}

	SP_TEST_METHOD(CLASS_NAME, buildInputElements2)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		sp_uint inputLength = 5u;
		DOP18 dops[5u];

		for (sp_uint i = 0; i < inputLength; i++)
			dops[i].translate(Vec3(0.0f, i * 10.0f, 0.0f));

		const sp_size groupLength = gpu->getGroupLength(inputLength, inputLength);
		const sp_size globalWorkSize[3] = { inputLength, 0, 0 };
		const sp_size localWorkSize[3] = { groupLength, 0, 0 };

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << inputLength;
		buildOptions << " -DINPUT_STRIDE=" << DOP18_STRIDER;
		buildOptions << " -DINPUT_OFFSET=0";

		GpuIndexes* commandIndexes = ALLOC_NEW(GpuIndexes)();
		commandIndexes->init(gpu, buildOptions.str().c_str());
		commandIndexes->setParametersCreateIndexes(inputLength);

		sp_char filename[512];
		currentDirectory(filename, 512);
		directoryAddPath(filename, std::strlen(filename), SP_DIRECTORY_OPENCL_SOURCE, SP_DIRECTORY_OPENCL_SOURCE_LENGTH);
		directoryAddPath(filename, std::strlen(filename), "SweepAndPrune.cl", 16);

		SP_FILE file;
		file.open(filename, std::ios::in);
		const sp_size fileSize = file.length();
		sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
		file.read(source, fileSize);
		file.close();

		cl_program program;
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, buildOptions.str().c_str(), &program);

		ALLOC_RELEASE(source);

		sp_int boundingVolumeType = (sp_int)BoundingVolumeType::DOP18;

		cl_mem indexesGpu = commandIndexes->execute(ZERO_UINT, NULL, NULL);
		cl_mem elementsGPU = gpu->createBuffer(inputLength * sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		sp_uint indexesCPU[5];
		gpu->commandManager->readBuffer(indexesGpu, inputLength * sizeof(sp_float), indexesCPU);

		sp_size axis = DOP18_AXIS_Y;

		cl_event evt;
		GpuCommand* commandBuildElements = gpu->commandManager->createCommand()
			->setInputParameter(dops, inputLength * sizeof(DOP18), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR)
			->setInputParameter(&boundingVolumeType, sizeof(sp_int), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR)
			->setInputParameter(indexesGpu, sizeof(sp_uint) * inputLength)
			->setInputParameter(&inputLength, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR)
			->setInputParameter(elementsGPU, inputLength * sizeof(sp_float))
			->buildFromProgram(program, "buildInputElements")
			->execute(1, globalWorkSize, localWorkSize, &axis, ZERO_UINT, NULL, &evt);

		sp_float elements[5];
		gpu->commandManager->readBuffer(elementsGPU, inputLength * sizeof(sp_float), elements, ONE_UINT, &evt);
		gpu->releaseEvent(evt);

		for (sp_uint i = 0; i < inputLength; i++)
			Assert::AreEqual((sp_float)(i * 10u) - 0.5f, elements[i], L"Wrong value.", LINE_INFO());

		sp_mem_delete(commandBuildElements, GpuCommand);
	}

	SP_TEST_METHOD(CLASS_NAME, buildInputElements3)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		sp_uint inputLength = 5u;
		AABB aabbs[5u];

		for (sp_uint i = 0; i < inputLength; i++)
			aabbs[i].translate(Vec3(0.0f, i * 10.0f, 0.0f));

		const sp_size groupLength = gpu->getGroupLength(inputLength, inputLength);
		const sp_size globalWorkSize[3] = { inputLength, 0, 0 };
		const sp_size localWorkSize[3] = { groupLength, 0, 0 };

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << inputLength;
		buildOptions << " -DINPUT_STRIDE=" << DOP18_STRIDER;
		buildOptions << " -DINPUT_OFFSET=0";

		GpuIndexes* commandIndexes = ALLOC_NEW(GpuIndexes)();
		commandIndexes->init(gpu, buildOptions.str().c_str());
		commandIndexes->setParametersCreateIndexes(inputLength);

		sp_char filename[512];
		currentDirectory(filename, 512);
		directoryAddPath(filename, std::strlen(filename), SP_DIRECTORY_OPENCL_SOURCE, SP_DIRECTORY_OPENCL_SOURCE_LENGTH);
		directoryAddPath(filename, std::strlen(filename), "SweepAndPrune.cl", 16);

		SP_FILE file;
		file.open(filename, std::ios::in);
		const sp_size fileSize = file.length();
		sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
		file.read(source, fileSize);
		file.close();

		cl_program program; 
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, buildOptions.str().c_str(), &program);

		ALLOC_RELEASE(source);

		sp_int boundingVolumeType = (sp_int)BoundingVolumeType::AABB;

		cl_mem indexesGpu = commandIndexes->execute(ZERO_UINT, NULL, NULL);
		cl_mem elementsGPU = gpu->createBuffer(inputLength * sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		sp_uint indexesCPU[5];
		gpu->commandManager->readBuffer(indexesGpu, inputLength * sizeof(sp_float), indexesCPU);

		sp_size axis = DOP18_AXIS_Y;

		cl_event evt;
		GpuCommand* commandBuildElements = gpu->commandManager->createCommand()
			->setInputParameter(aabbs, inputLength * sizeof(AABB), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR)
			->setInputParameter(&boundingVolumeType, sizeof(sp_int), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR)
			->setInputParameter(indexesGpu, sizeof(sp_uint) * inputLength)
			->setInputParameter(&inputLength, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR)
			->setInputParameter(elementsGPU, inputLength * sizeof(sp_float))
			->buildFromProgram(program, "buildInputElements")
			->execute(1, globalWorkSize, localWorkSize, &axis, ZERO_UINT, NULL, &evt);

		sp_float elements[5];
		gpu->commandManager->readBuffer(elementsGPU, inputLength * sizeof(sp_float), elements, ONE_UINT, &evt);
		gpu->releaseEvent(evt);

		for (sp_uint i = 0; i < inputLength; i++)
			Assert::AreEqual((sp_float)(i * 10u) - 0.5f, elements[i], L"Wrong value.", LINE_INFO());

		sp_mem_delete(commandBuildElements, GpuCommand);
	}

	SP_TEST_METHOD(CLASS_NAME, buildInputElements4)
	{
		SpPhysicSettings::instance()->boundingVolumeType(BoundingVolumeType::AABB);
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		sp_uint inputLength = 5u;
		AABB aabbs[5u];

		for (sp_uint i = 0; i < inputLength; i++)
			aabbs[i].translate(Vec3(0.0f, i * 10.0f, 0.0f));

		const sp_size groupLength = gpu->getGroupLength(inputLength, inputLength);
		const sp_size globalWorkSize[3] = { inputLength, 0, 0 };
		const sp_size localWorkSize[3] = { groupLength, 0, 0 };

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << inputLength;
		buildOptions << " -DINPUT_STRIDE=" << DOP18_STRIDER;
		buildOptions << " -DINPUT_OFFSET=0";
#ifdef ENV_64BITS
		buildOptions << " -DENV_64BITS";
#endif

		GpuIndexes* commandIndexes = ALLOC_NEW(GpuIndexes)();
		commandIndexes->init(gpu, buildOptions.str().c_str());
		commandIndexes->setParametersCreateIndexes(inputLength);

		sp_char filename[512];
		currentDirectory(filename, 512);
		directoryAddPath(filename, std::strlen(filename), SP_DIRECTORY_OPENCL_SOURCE, SP_DIRECTORY_OPENCL_SOURCE_LENGTH);
		directoryAddPath(filename, std::strlen(filename), "SweepAndPrune.cl", 16);

		SP_FILE file;
		file.open(filename, std::ios::in);
		const sp_size fileSize = file.length();
		sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
		file.read(source, fileSize);
		file.close();

		cl_program program; 
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, buildOptions.str().c_str(), &program);

		ALLOC_RELEASE(source);

		sp_int boundingVolumeType = (sp_int)BoundingVolumeType::AABB;

		cl_mem indexesGpu = commandIndexes->execute(ZERO_UINT, NULL, NULL);
		cl_mem elementsGPU = gpu->createBuffer(inputLength * sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		sp_uint indexesCPU[5];
		gpu->commandManager->readBuffer(indexesGpu, inputLength * sizeof(sp_float), indexesCPU);

		SweepAndPrune sap;
		sap.updateAxis(DOP18_NORMALS[DOP18_PLANES_UP_DEPTH_INDEX]);
		sp_size axis = sap.axis; 

		cl_event evt;
		GpuCommand* commandBuildElements = gpu->commandManager->createCommand()
			->setInputParameter(aabbs, inputLength * sizeof(AABB), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR)
			->setInputParameter(&boundingVolumeType, sizeof(sp_int), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR)
			->setInputParameter(indexesGpu, sizeof(sp_uint) * inputLength)
			->setInputParameter(&inputLength, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR)
			->setInputParameter(elementsGPU, inputLength * sizeof(sp_float))
			->buildFromProgram(program, "buildInputElements")
			->execute(1, globalWorkSize, localWorkSize, &axis, ZERO_UINT, NULL, &evt);

		sp_float elements[5];
		gpu->commandManager->readBuffer(elementsGPU, inputLength * sizeof(sp_float), elements, ONE_UINT, &evt);
		gpu->releaseEvent(evt);

		for (sp_uint i = 0; i < inputLength; i++)
			Assert::AreEqual((sp_float)(i * 10u) - 0.5f, elements[i], L"Wrong value.", LINE_INFO());

		sp_mem_delete(commandBuildElements, GpuCommand);
	}

	SP_TEST_METHOD(CLASS_NAME, buildInputElements5)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		sp_uint inputLength = 5u;
		Sphere spheres[5u];

		for (sp_uint i = 0; i < inputLength; i++)
			spheres[i].translate(Vec3((i * 10.0f) - 5.0f, 0.0f, 0.0f));

		const sp_size groupLength = gpu->getGroupLength(inputLength, inputLength);
		const sp_size globalWorkSize[3] = { inputLength, 0, 0 };
		const sp_size localWorkSize[3] = { groupLength, 0, 0 };

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << inputLength;
		buildOptions << " -DINPUT_STRIDE=" << DOP18_STRIDER;
		buildOptions << " -DINPUT_OFFSET=0";
#ifdef ENV_64BITS
		buildOptions << " -DENV_64BITS";
#endif

		GpuIndexes* commandIndexes = ALLOC_NEW(GpuIndexes)();
		commandIndexes->init(gpu, buildOptions.str().c_str());
		commandIndexes->setParametersCreateIndexes(inputLength);

		sp_char filename[512];
		currentDirectory(filename, 512);
		directoryAddPath(filename, std::strlen(filename), SP_DIRECTORY_OPENCL_SOURCE, SP_DIRECTORY_OPENCL_SOURCE_LENGTH);
		directoryAddPath(filename, std::strlen(filename), "SweepAndPrune.cl", 16);

		SP_FILE file;
		file.open(filename, std::ios::in);
		const sp_size fileSize = file.length();
		sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
		file.read(source, fileSize);
		file.close();

		cl_program program; 
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, buildOptions.str().c_str(), &program);

		ALLOC_RELEASE(source);

		sp_int boundingVolumeType = (sp_int)BoundingVolumeType::Sphere;

		cl_mem indexesGpu = commandIndexes->execute(ZERO_UINT, NULL, NULL);
		cl_mem elementsGPU = gpu->createBuffer(inputLength * sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		sp_uint indexesCPU[5];
		gpu->commandManager->readBuffer(indexesGpu, inputLength * sizeof(sp_float), indexesCPU);

		sp_size axis = DOP18_AXIS_X;

		cl_event evt;
		GpuCommand* commandBuildElements = gpu->commandManager->createCommand()
			->setInputParameter(spheres, inputLength * sizeof(Sphere), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR)
			->setInputParameter(&boundingVolumeType, sizeof(sp_int), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR)
			->setInputParameter(indexesGpu, sizeof(sp_uint) * inputLength)
			->setInputParameter(&inputLength, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR)
			->setInputParameter(elementsGPU, inputLength * sizeof(sp_float))
			->buildFromProgram(program, "buildInputElements")
			->execute(1, globalWorkSize, localWorkSize, &axis, ZERO_UINT, NULL, &evt);

		sp_float elements[5];
		gpu->commandManager->readBuffer(elementsGPU, inputLength * sizeof(sp_float), elements, ONE_UINT, &evt);
		gpu->releaseEvent(evt);

		for (sp_uint i = 0; i < inputLength; i++)
			Assert::AreEqual(((sp_float)(i * 10u)) - 5.0f - spheres[i].ray, elements[i], L"Wrong value.", LINE_INFO());

		sp_mem_delete(commandBuildElements, GpuCommand);
	}

	SP_TEST_METHOD(CLASS_NAME, buildInputElements6)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		sp_uint inputLength = 5u;
		Sphere spheres[5u];

		for (sp_uint i = 0; i < inputLength; i++)
			spheres[i].translate(Vec3((i * 10.0f) - 5.0f, 0.0f, 0.0f));

		const sp_size groupLength = gpu->getGroupLength(inputLength, inputLength);
		const sp_size globalWorkSize[3] = { inputLength, 0, 0 };
		const sp_size localWorkSize[3] = { groupLength, 0, 0 };

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << inputLength;
		buildOptions << " -DINPUT_STRIDE=" << DOP18_STRIDER;
		buildOptions << " -DINPUT_OFFSET=0";
#ifdef ENV_64BITS
		buildOptions << " -DENV_64BITS";
#endif

		GpuIndexes* commandIndexes = ALLOC_NEW(GpuIndexes)();
		commandIndexes->init(gpu, buildOptions.str().c_str());
		commandIndexes->setParametersCreateIndexes(inputLength);

		sp_char filename[512];
		currentDirectory(filename, 512);
		directoryAddPath(filename, std::strlen(filename), SP_DIRECTORY_OPENCL_SOURCE, SP_DIRECTORY_OPENCL_SOURCE_LENGTH);
		directoryAddPath(filename, std::strlen(filename), "SweepAndPrune.cl", 16);

		SP_FILE file;
		file.open(filename, std::ios::in);
		const sp_size fileSize = file.length();
		sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
		file.read(source, fileSize);
		file.close();

		cl_program program; 
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, buildOptions.str().c_str(), &program);

		ALLOC_RELEASE(source);

		sp_int boundingVolumeType = (sp_int)BoundingVolumeType::Sphere;

		cl_mem indexesGpu = commandIndexes->execute(ZERO_UINT, NULL, NULL);
		cl_mem elementsGPU = gpu->createBuffer(inputLength * sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		sp_uint indexesCPU[5];
		gpu->commandManager->readBuffer(indexesGpu, inputLength * sizeof(sp_float), indexesCPU);

		sp_size axis = DOP18_AXIS_UP_RIGHT;

		cl_event evt;
		GpuCommand* commandBuildElements = gpu->commandManager->createCommand()
			->setInputParameter(spheres, inputLength * sizeof(Sphere), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR)
			->setInputParameter(&boundingVolumeType, sizeof(sp_int), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR)
			->setInputParameter(indexesGpu, sizeof(sp_uint) * inputLength)
			->setInputParameter(&inputLength, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR)
			->setInputParameter(elementsGPU, inputLength * sizeof(sp_float))
			->buildFromProgram(program, "buildInputElements")
			->execute(1, globalWorkSize, localWorkSize, &axis, ZERO_UINT, NULL, &evt);

		sp_float elements[5];
		gpu->commandManager->readBuffer(elementsGPU, inputLength * sizeof(sp_float), elements, ONE_UINT, &evt);
		gpu->releaseEvent(evt);

		sp_uint index = ZERO_UINT;
		for (sp_float i = -5.53606796f; i < inputLength; i += 7.07213593f)
		{
			Assert::AreEqual(i, elements[index], L"Wrong value.", LINE_INFO());
			index++;
		}

		sp_mem_delete(commandBuildElements, GpuCommand);
	}

	SP_TEST_METHOD(CLASS_NAME, findCollisionsGPU_WithAABB)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		const size_t count = 20;
		//const sp_uint count = (sp_uint)std::pow(2.0, 17.0);
		AABB* aabbs1 = getRandomAABBs(count, 100);
		AABB* aabbs2 = ALLOC_COPY(aabbs1, AABB, count);
		cl_mem aabbsGpu = gpu->createBuffer(aabbs2, sizeof(AABB) * count, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem outputGpu = gpu->createBuffer(sizeof(sp_uint) * count * 20, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		cl_mem outputLengthGpu = gpu->createBuffer(sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		SpRigidBody3D* rigidBodies = ALLOC_NEW_ARRAY(SpRigidBody3D, count);
		for (sp_uint i = 0; i < count; i++)
			rigidBodies[i].mass(8.0f);
		cl_mem rigidBodiesGpu = gpu->createBuffer(rigidBodies, sizeof(SpRigidBody3D) * count, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, true);

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << count;
		
		SweepAndPrune* sap = ALLOC_NEW(SweepAndPrune)();
		sap->init(gpu, buildOptions.str().c_str());
		sap->setParameters(aabbsGpu, count, BoundingVolumeType::AABB, AABB_STRIDER,
			rigidBodiesGpu, sizeof(SpRigidBody3D), outputLengthGpu, outputGpu);

		PerformanceCounter counter; counter.start();
		SweepAndPruneResult result1 = SweepAndPrune::findCollisions(aabbs1, count);
		sp_longlong currentTime1 = counter.diff();
		
		counter.start();
		sap->execute(ZERO_UINT, NULL, NULL);
		sp_longlong currentTime2 = counter.diff();

		cl_event evt;
		sp_uint collisionsLength = sap->fetchCollisionLength(ZERO_UINT, NULL, &evt);
		gpu->releaseEvent(evt);

		//sp_uint* indexes = (sp_uint*) ALLOC_SIZE(sizeof(sp_uint) * count * 2);
		sp_uint* indexes = (sp_uint*) ALLOC_SIZE(640);
		sap->fetchCollisionIndexes(indexes, ZERO_UINT, NULL, &evt);
		gpu->releaseEvent(evt);

		if (result1.length != collisionsLength)
			int a = 1;

		Assert::AreEqual(result1.length, collisionsLength, L"wrong value", LINE_INFO());

		gpu->releaseBuffer(aabbsGpu);
		gpu->releaseBuffer(outputGpu);
		gpu->releaseBuffer(outputLengthGpu);		
		ALLOC_RELEASE(indexes);
		ALLOC_DELETE(sap, SweepAndPrune);
	}

	SP_TEST_METHOD(CLASS_NAME, findCollisionsGPU_WithKDOPs)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();
		PerformanceCounter performanceCounter;
		SweepAndPrune sap;

		const sp_uint minXAxis = DOP18_AXIS_X;
		const sp_uint maxXAxis = minXAxis + DOP18_ORIENTATIONS;

		//const sp_uint length = (sp_uint)std::pow(2.0, 17.0);
		//DOP18* kdops1 = getRandomKDOPs(length, 100000u);

		const sp_uint length = 64u;
		DOP18* kdops1 = get64KDOPs();

		DOP18* kdops2 = ALLOC_COPY(kdops1, DOP18, length);
		cl_mem inputGpu = gpu->createBuffer(kdops2, DOP18_SIZE * length, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		
		SpRigidBody3D* rigidBodies = ALLOC_NEW_ARRAY(SpRigidBody3D, length);
		for (sp_uint i = 0; i < length; i++)
			rigidBodies[i].mass(8.0f);

		const sp_size axis = ZERO_UINT;
		cl_mem rigidBodiesGpu = gpu->createBuffer(rigidBodies, sizeof(SpRigidBody3D) * length, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR , true);
		cl_mem outputGpu = gpu->createBuffer(sizeof(sp_uint) * length * SP_SAP_MAX_COLLISION_PER_OBJECT, CL_MEM_READ_ONLY | CL_MEM_ALLOC_HOST_PTR);
		cl_mem outputLengthGpu = gpu->createBuffer(sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_ALLOC_HOST_PTR);

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << length
					<< " -DINPUT_STRIDE=" << DOP18_STRIDER
					<< " -DINPUT_OFFSET=" << axis;

		sap.init(gpu, buildOptions.str().c_str());
		sap.setParameters(inputGpu, length, BoundingVolumeType::DOP18, DOP18_STRIDER, rigidBodiesGpu,
			sizeof(SpRigidBody3D), outputLengthGpu, outputGpu);

		performanceCounter.start();

		SweepAndPruneResult expected;
		expected.indexes = ALLOC_ARRAY(sp_uint, multiplyBy2(length) * SP_SAP_MAX_COLLISION_PER_OBJECT);

		sp_uint* indexes = ALLOC_ARRAY(sp_uint, length);
		for (sp_uint i = ZERO_UINT; i < length; i++)
			indexes[i] = i;

		SweepAndPrune::findCollisions(kdops1, indexes, length, &expected);

		sp_longlong cpuPerformance = performanceCounter.diff();

		cl_event evt;
		performanceCounter.start();
		sap.execute(ZERO_UINT, NULL, &evt);
		sp_longlong gpuPerformance = performanceCounter.diff();
		
		sp_uint collisionsLength = sap.fetchCollisionLength(ZERO_UINT, &evt, NULL);

		Assert::AreEqual(expected.length, collisionsLength, L"wrong value", LINE_INFO());

		gpu->releaseBuffer(inputGpu);
		sap.dispose();
		ALLOC_RELEASE(kdops1);
	}
#endif // OPENCL_ENABLED

}

#undef CLASS_NAME
