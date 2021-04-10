#include "SweepAndPrune.h"
#include "SpPhysicSimulator.h"
#include "SpWorld.h"

namespace NAMESPACE_PHYSICS
{
	DOP18* globalKDOPs;

	sp_int comparatorXAxisForQuickSort(const void* a, const void* b)
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
	
	sp_int comparatorXAxisForQuickSortKDOP(const void* index1, const void* index2)
	{
		DOP18* obj1 = &globalKDOPs[*(sp_uint*)index1];
		DOP18* obj2 = &globalKDOPs[*(sp_uint*)index2];

		if (obj1->min[0] > obj2->min[0])
			return -1;
		else
			if (obj1->min[0] < obj2->min[0])
				return 1;

		return 0;
	}

	sp_uint axisToSweep;
	static sp_int comparatorForQuickSortKDOP(const void* a, const void* b)
	{
		DOP18* obj1 = (DOP18*)a;
		DOP18* obj2 = (DOP18*)b;

		if (obj1->min[axisToSweep] < obj2->min[axisToSweep])
			return -1;
		else
			if (obj1->min[axisToSweep] > obj2->min[axisToSweep])
				return 1;

		return 0;
	}

	template<typename T>
	void erase_element(std::vector<T>& vector, sp_uint index)
	{
		std::swap(vector[index], vector.back());

		vector.pop_back();

		if (vector.size() != index)
			std::swap(vector[index], vector.back());
	}

	template<typename T>
	void erase_element(T* arrayOfT, sp_uint count, sp_uint index)
	{
		std::memcpy(arrayOfT + index, arrayOfT + index + 1, (count - index - 1) * sizeof(T));
	}

	SweepAndPruneResult SweepAndPrune::findCollisions(AABB* aabbs, sp_uint count)
	{
		sp_uint* indexes = ALLOC_ARRAY(sp_uint, count*2);
		sp_uint aabbIndex = 0;

		sp_uint* activeListIndex = ALLOC_ARRAY(sp_uint, count);
		sp_uint activeListIndexCount = 0;
		sp_uint activeListAABBIndex = 0;
		
		AlgorithmSorting::quickSortNative(aabbs, count, sizeof(AABB), comparatorXAxisForQuickSort);
		
		for (sp_uint i = 0; i < count; ++i)
		{
			for (sp_uint j = activeListIndexCount; j > 0; --j)
			{
				activeListAABBIndex = activeListIndex[j - 1];

				if (aabbs[activeListAABBIndex].maxPoint.x < aabbs[i].minPoint.x)
				{
					erase_element(activeListIndex, activeListIndexCount, j - 1); //remove from active list
					--activeListIndexCount;
				}
				else
				{
					//check collision AABB x AABB
					if (  (aabbs[i].maxPoint.x >= aabbs[activeListAABBIndex].minPoint.x && aabbs[i].minPoint.x <= aabbs[activeListAABBIndex].maxPoint.x)
						&&(aabbs[i].maxPoint.y >= aabbs[activeListAABBIndex].minPoint.y && aabbs[i].minPoint.y <= aabbs[activeListAABBIndex].maxPoint.y)
						&&(aabbs[i].maxPoint.z >= aabbs[activeListAABBIndex].minPoint.z && aabbs[i].minPoint.z <= aabbs[activeListAABBIndex].maxPoint.z))
					{
						indexes[aabbIndex++] = i;
						indexes[aabbIndex++] = activeListAABBIndex;
					}
				}
			}

			activeListIndex[activeListIndexCount++] = i;
		}

		ALLOC_RELEASE(activeListIndex);
		return SweepAndPruneResult(indexes, aabbIndex >> 1);
	}

	void SweepAndPrune::findCollisions(DOP18* kdops, sp_uint* indexes, sp_uint length, SweepAndPruneResult* resultCpu)
	{
		sp_uint* activeListIndex = ALLOC_ARRAY(sp_uint, length);
		sp_uint activeListIndexCount = ZERO_UINT;
		sp_uint kdopIndexJ = ZERO_UINT;

		globalKDOPs = kdops;
		
		AlgorithmSorting::quickSortNative(indexes, length, SIZEOF_UINT, comparatorXAxisForQuickSortKDOP);

		for (sp_uint i = 0; i < length; ++i)
		{
			sp_uint kdopIndexI = indexes[i];

			for (sp_uint j = activeListIndexCount; j > 0; --j)
			{
				kdopIndexJ = indexes[activeListIndex[j - 1]];

				if (kdops[kdopIndexJ].max[0] < kdops[kdopIndexI].min[0])
				{
					erase_element(activeListIndex, activeListIndexCount, j - 1); //remove from active list
					--activeListIndexCount;
				}
				else
				{
					if (   (kdops[kdopIndexI].max[0] >= kdops[kdopIndexJ].min[0] && kdops[kdopIndexI].min[0] <= kdops[kdopIndexJ].max[0])
						&& (kdops[kdopIndexI].max[1] >= kdops[kdopIndexJ].min[1] && kdops[kdopIndexI].min[1] <= kdops[kdopIndexJ].max[1])
						&& (kdops[kdopIndexI].max[2] >= kdops[kdopIndexJ].min[2] && kdops[kdopIndexI].min[2] <= kdops[kdopIndexJ].max[2])
						&& (kdops[kdopIndexI].max[3] >= kdops[kdopIndexJ].min[3] && kdops[kdopIndexI].min[3] <= kdops[kdopIndexJ].max[3])
						&& (kdops[kdopIndexI].max[4] >= kdops[kdopIndexJ].min[4] && kdops[kdopIndexI].min[4] <= kdops[kdopIndexJ].max[4])
						&& (kdops[kdopIndexI].max[5] >= kdops[kdopIndexJ].min[5] && kdops[kdopIndexI].min[5] <= kdops[kdopIndexJ].max[5])
						&& (kdops[kdopIndexI].max[6] >= kdops[kdopIndexJ].min[6] && kdops[kdopIndexI].min[6] <= kdops[kdopIndexJ].max[6])
						&& (kdops[kdopIndexI].max[7] >= kdops[kdopIndexJ].min[7] && kdops[kdopIndexI].min[7] <= kdops[kdopIndexJ].max[7])
						&& (kdops[kdopIndexI].max[8] >= kdops[kdopIndexJ].min[8] && kdops[kdopIndexI].min[8] <= kdops[kdopIndexJ].max[8])
						)
					{
						resultCpu->indexes[resultCpu->length++] = kdopIndexI;
						resultCpu->indexes[resultCpu->length++] = kdopIndexJ;
					}
				}
			}

			activeListIndex[activeListIndexCount++] = i;
		}

		ALLOC_RELEASE(activeListIndex);
		resultCpu->length = divideBy2(resultCpu->length);
	}

	sp_bool SweepAndPrune::pca(Vec3& output, const sp_uint maxIterations) const
	{
		SpWorld* world = SpWorldManagerInstance->current();

		Mat matrix(world->objectsLength(), 3u);
		sp_float* values = matrix;

		for (sp_uint i = 0; i < world->objectsLength(); i++)
			std::memcpy(&values[i * 3u], world->transforms(i)->position, sizeof(Vec3));

		sp_uint iterations;
		Mat u(matrix.rows(), matrix.rows()), s(matrix.columns(), matrix.columns()), v(matrix.columns(), matrix.columns());

		if (!matrix.svd(u, s, v, iterations, maxIterations))
			return false;

		sp_uint eigenVectorIndex = ZERO_UINT;
		sp_float maxEigenValue = s.get(0u, 0u);

		sp_float eigenValue = s.get(1u, 1u);
		if (eigenValue > maxEigenValue)
		{
			eigenVectorIndex = ONE_UINT;
			maxEigenValue = eigenValue;
		}

		eigenValue = s.get(2u, 2u);
		if (eigenValue > maxEigenValue)
		{
			eigenVectorIndex = TWO_UINT;
			maxEigenValue = eigenValue;
		}

		v.column(eigenVectorIndex, output);

		return true;
	}

#ifdef OPENCL_ENABLED
	
	SweepAndPrune* SweepAndPrune::init(GpuDevice* gpu, const char* buildOptions)
	{
		this->gpu = gpu;

		if (sapProgram != NULL)
			return this;

		radixSorting = sp_mem_new(GpuRadixSorting)();
		radixSorting->init(gpu, nullptr);

		SpDirectory* filename = SpDirectory::currentDirectory()
			->add(SP_DIRECTORY_OPENCL_SOURCE)
			->add("SweepAndPrune.cl");

		SP_FILE file;
		file.open(filename->name()->data(), std::ios::in);
		sp_mem_delete(filename, SpDirectory);
		const sp_size fileSize = file.length();
		sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
		file.read(source, fileSize);
		file.close();

		sp_uint sapIndex = gpu->commandManager->cacheProgram(source, SIZEOF_CHAR * fileSize, buildOptions);
		sapProgram = gpu->commandManager->cachedPrograms[sapIndex];

		ALLOC_RELEASE(source);
		return this;
	}

	void SweepAndPrune::initIndexes(sp_uint inputLength)
	{
		indexesLengthGPU = gpu->createBuffer(&inputLength, SIZEOF_UINT, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);

		GpuIndexes* createIndexes = sp_mem_new(GpuIndexes)();
		createIndexes->init(gpu, nullptr);
		createIndexes->setParametersCreateIndexes(inputLength);
		indexesGPU = createIndexes->execute();
		sp_mem_delete(createIndexes, GpuIndexes);
	}

	void SweepAndPrune::setParameters(cl_mem boundingVolumesGpu, sp_uint boundingVolumesLength, BoundingVolumeType boundingVolumeType, sp_uint strider, cl_mem rigidBodies, const sp_uint rigidBodySize, cl_mem outputIndexLength, cl_mem outputIndex, const sp_char* commandName)
	{
		globalWorkSize[0] = boundingVolumesLength;
		localWorkSize[0] = 1u;

		const sp_uint collisionsSize = boundingVolumesLength * 2u * SP_SAP_MAX_COLLISION_PER_OBJECT * SIZEOF_UINT;

		initIndexes(boundingVolumesLength);

		elementsGPU = gpu->createBuffer(sizeof(sp_float) * boundingVolumesLength, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		commandBuildElements = gpu->commandManager->createCommand()
			->setInputParameter(boundingVolumesGpu, boundingVolumesLength * strider * SIZEOF_FLOAT)
			->setInputParameter(&boundingVolumeType, sizeof(sp_int), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR)
			->setInputParameter(indexesGPU, boundingVolumesLength * SIZEOF_UINT)
			->setInputParameter(indexesLengthGPU, SIZEOF_UINT, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR)
			->setInputParameter(elementsGPU, sizeof(sp_float) * boundingVolumesLength)
			->buildFromProgram(sapProgram, "buildInputElements");

		radixSorting->setParameters(elementsGPU, boundingVolumesLength, indexesGPU, indexesLengthGPU, ONE_UINT);

		commandSaPCollisions = gpu->commandManager->createCommand()
			->setInputParameter(boundingVolumesGpu, boundingVolumesLength * strider * SIZEOF_FLOAT)
			->setInputParameter(rigidBodies, boundingVolumesLength * rigidBodySize)
			->setInputParameter(indexesLengthGPU, SIZEOF_UINT)
			->setInputParameter(indexesGPU, boundingVolumesLength * SIZEOF_UINT)
			->setInputParameter(outputIndexLength, SIZEOF_UINT)
			->setInputParameter(outputIndex, collisionsSize)
			->buildFromProgram(sapProgram, commandName);
	}

	cl_mem SweepAndPrune::execute(sp_uint previousEventsLength, cl_event* previousEvents)
	{
		commandBuildElements->execute(ONE_UINT, globalWorkSize, localWorkSize, &axis, previousEvents, previousEventsLength);

		/*
		AABB aabbs[512];
		gpu->commandManager->readBuffer(commandBuildElements->getInputParameter(0), sizeof(AABB) * 512, aabbs);

		sp_float elements[512];
		gpu->commandManager->readBuffer(commandBuildElements->getInputParameter(4), sizeof(sp_float) * 512, elements);
		*/

		/*
		sp_uint* indexes = ALLOC_ARRAY(sp_uint, len);
		gpu->commandManager->readBuffer(indexesGPU, sizeof(sp_uint) * len, indexes, ONE_UINT, &commandBuildElements->lastEvent);

		sp_float* elements = ALLOC_ARRAY(sp_float, len);
		gpu->commandManager->readBuffer(elementsGPU, sizeof(sp_float) * len, elements, ONE_UINT, &commandBuildElements->lastEvent);

		sp_log_info1s("BEGIN INDEXES"); sp_log_newline();
		for (sp_uint i = 0; i < len; i++)
		{
			sp_log_info1u(indexes[i]); 
			sp_log_info1s(" -> ");
			sp_log_info1f(elements[indexes[i]]);

			sp_log_newline();
		}
		sp_log_info1s("END INDEXES"); sp_log_newline();
		*/

		cl_mem sortedIndexes = radixSorting->execute(ONE_UINT, &commandBuildElements->lastEvent);

		/* check if sorting is OK		
		sp_uint* sorted = ALLOC_ARRAY(sp_uint, len);
		lastEvent = gpu->commandManager->readBuffer(sortedIndexes, 4 * len, sorted, ONE_UINT, &radixSorting->lastEvent);

		sp_log_info1s("SORTED INDEXES INDEXES"); sp_log_newline();
		for (sp_uint i = 0; i < len; i++)
		{
			sp_log_info1u(sorted[i]);
			sp_log_info1s(" -> ");
			sp_log_info1f(elements[sorted[i]]); 

			sp_log_newline();
		}
		sp_log_info1s("END SORTED INDEXES"); sp_log_newline();
		*/

		sp_uint zeroValue = ZERO_UINT;

		commandSaPCollisions
			->updateInputParameter(3, sortedIndexes)
			->updateInputParameterValue(4, &zeroValue)
			->execute(1, globalWorkSize, localWorkSize, &axis, &radixSorting->lastEvent, ONE_UINT);

		lastEvent = commandSaPCollisions->lastEvent;

		return commandSaPCollisions->getInputParameter(4u);
	}

	sp_uint SweepAndPrune::fetchCollisionLength()
	{
		sp_uint value;
		commandSaPCollisions->fetchInOutParameter<sp_uint>(4, &value);
		
		return divideBy2(value);
	}

	void SweepAndPrune::fetchCollisionIndexes(sp_uint* output) const
	{
		commandSaPCollisions->fetchInOutParameter<sp_uint>(5, output);
	}

#endif // OPENCL_ENALBED

}