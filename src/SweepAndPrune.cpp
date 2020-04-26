#include "SweepAndPrune.h"

namespace NAMESPACE_PHYSICS
{
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

	template<typename T>
	void erase_element(std::vector<T>& vector, sp_uint index)
	{
		std::swap(vector[index], vector.back());

		vector.pop_back();

		if (vector.size() != index)
			std::swap(vector[index], vector.back());
	}

	template<typename T>
	void erase_element(T* array, sp_uint count, sp_uint index)
	{
		std::memmove(array + index, array + index + 1, (count - index - 1) * sizeof(T));
	}

	SweepAndPruneResultCpu SweepAndPrune::findCollisions(AABB* aabbs, sp_uint count)
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
		return SweepAndPruneResultCpu(indexes, aabbIndex >> 1);
	}

#ifdef OPENCL_ENABLED
	static cl_program sapProgram = NULL;

	SweepAndPrune* SweepAndPrune::init(GpuDevice* gpu, const char* buildOptions)
	{
		if (sapProgram != NULL)
			return this;

		this->gpu = gpu;

		radixSorting = ALLOC_NEW(GpuRadixSorting)();
		radixSorting->init(gpu, buildOptions);


		SP_FILE file;
		file.open("SweepAndPrune.cl", std::ios::in);
		const sp_size fileSize = file.length();
		sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
		file.read(source, fileSize);
		file.close();

		sp_uint sapIndex = gpu->commandManager->cacheProgram(source, sizeof(char) * fileSize, buildOptions);

		ALLOC_RELEASE(source);

		sapProgram = gpu->commandManager->cachedPrograms[sapIndex];
		return this;
	}

	sp_float* values;

	void SweepAndPrune::setParameters(sp_float* input, sp_uint inputLength, sp_uint strider, sp_uint offset, sp_uint minPointIndex, sp_uint maxPointIndex)
	{
		values = input;
		globalWorkSize[0] = gpu->maxWorkGroupSize;
		localWorkSize[0] = nextPowOf2(inputLength) / gpu->maxWorkGroupSize;

		radixSorting->setParameters(input, inputLength, strider, offset);

		sp_uint globalIndex = 0;

		sp_uint outputSize = inputLength * 2 * SIZEOF_UINT;
		output = gpu->createBuffer(outputSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		commandSaP = gpu->commandManager->createCommand()
			->setInputParameter(radixSorting->inputGpu, inputLength * strider * SIZEOF_FLOAT)
			->setInputParameter(radixSorting->indexesLengthGpu, SIZEOF_UINT)
			->setInputParameter(radixSorting->output, inputLength * SIZEOF_UINT)
			->setInputParameter(&globalIndex, SIZEOF_UINT, CL_MEM_READ_WRITE)
			->setInputParameter(output, outputSize)
			->buildFromProgram(sapProgram, "sweepAndPrune");
	}

	cl_mem SweepAndPrune::execute()
	{
		radixSorting->execute();

		commandSaP
			->updateInputParameter(2, radixSorting->output)
			->execute(1, globalWorkSize, localWorkSize, 0, &radixSorting->lastEvent, 1u);
		lastEvent = commandSaP->lastEvent;

		return output;
	}

	sp_uint SweepAndPrune::fetchCollisionLength()
	{
		return divideBy2(*commandSaP->fetchInOutParameter<sp_uint>(3));
	}
#endif // OPENCL_ENALBED

	SweepAndPrune::~SweepAndPrune()
	{
	#if OPENCL_ENABLED
		ALLOC_DELETE(commandSaP, GpuCommand);
		ALLOC_DELETE(radixSorting, GpuRadixSorting);
	#endif
	}
}