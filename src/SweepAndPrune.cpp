#include "SweepAndPrune.h"

namespace NAMESPACE_PHYSICS
{
	DOP18* tempKDOPs;

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
		DOP18* obj1 = &tempKDOPs[*(sp_uint*)index1];
		DOP18* obj2 = &tempKDOPs[*(sp_uint*)index2];

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

		tempKDOPs = kdops;

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

#ifdef OPENCL_ENABLED
	static cl_program sapProgram = NULL;

	SweepAndPrune* SweepAndPrune::init(GpuDevice* gpu, const char* buildOptions)
	{
		if (sapProgram != NULL)
			return this;

		this->gpu = gpu;

		radixSorting = sp_mem_new(GpuRadixSorting)();
		radixSorting->init(gpu, buildOptions);
		
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

	void SweepAndPrune::setParameters(cl_mem inputGpu, sp_uint inputLength, sp_uint strider, sp_uint offset, sp_size axisLength)
	{
		globalWorkSize[0] = inputLength;
		localWorkSize[0] = 1u;

		const sp_uint collisionsSize = inputLength * 20u * SIZEOF_UINT;
		collisionsLength = gpu->createBuffer(SIZEOF_UINT, CL_MEM_READ_WRITE);
		collisions = gpu->createBuffer(collisionsSize, CL_MEM_READ_WRITE);

		initIndexes(inputLength);

		radixSorting->setParameters(inputGpu, inputLength, indexesGPU, indexesLengthGPU, strider, offset);

		commandSaPCollisions = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputLength * strider * SIZEOF_FLOAT)
			->setInputParameter(indexesLengthGPU, SIZEOF_UINT)
			->setInputParameter(radixSorting->output, inputLength * SIZEOF_UINT)
			->setInputParameter(collisionsLength, SIZEOF_UINT)
			->setInputParameter(collisions, collisionsSize)
			->buildFromProgram(sapProgram, "sweepAndPruneSingleAxis");
	}

	cl_mem SweepAndPrune::execute(sp_uint previousEventsLength, cl_event* previousEvents)
	{
		const sp_uint zeroValue = ZERO_UINT;

		radixSorting->execute(previousEventsLength, previousEvents);
		lastEvent = radixSorting->lastEvent;

		/*
		const sp_uint len = 512u;
		sp_uint* buffer = ALLOC_ARRAY(sp_uint, len);
		lastEvent = gpu->commandManager->readBuffer(radixSorting->output, 4 * len, buffer, ONE_UINT, &lastEvent);
		std::cout << "BEGIN SORT" << END_OF_LINE;
		for (size_t i = 0; i < len; i++)
			std::cout << aabbs[buffer[i]].minPoint.x << END_OF_LINE;
		std::cout << "END SORT" << END_OF_LINE;
		ALLOC_RELEASE(buffer);
		*/

		commandSaPCollisions
			->updateInputParameter(2, radixSorting->output)
			->updateInputParameterValue(3, &zeroValue)
			->execute(1, globalWorkSize, localWorkSize, 0, &lastEvent, ONE_UINT);

		lastEvent = commandSaPCollisions->lastEvent;

		return collisions;
	}

	/*
	cl_mem SweepAndPrune::execute()
	{
		DOP18* kdops = (DOP18*)values;
		sp_uint zeroValue = ZERO_UINT;

		std::vector<sp_uint> keptIndexes;
		for (sp_uint i = 0; i < inputLength; i++)
			keptIndexes.push_back(i);

		std::stringstream out;

		sp_size axis = 0;
		for (; axis < axisLength - 1; axis++)
		{	
			sp_uint length;
			gpu->commandManager->executeReadBuffer(indexesLengthGPU, SIZEOF_UINT, &length, true);
			sp_uint* indexes = ALLOC_ARRAY(sp_uint, length);
			gpu->commandManager->executeReadBuffer(indexesGPU, length * SIZEOF_UINT, indexes, true);

			radixSorting->updateIndexes(indexesGPU, indexesLengthGPU);
			radixSorting->execute();

			sp_uint* sortedIndexes = ALLOC_ARRAY(sp_uint, length);
			gpu->commandManager->executeReadBuffer(radixSorting->output, length * SIZEOF_UINT, sortedIndexes, true);

			// TODO: REMOVE - CHECK if the sorting is OK
			for (sp_uint i = 1; i < length; i++)
			{
				sp_uint previousIndex = sortedIndexes[i - 1];
				sp_uint currentIndex = sortedIndexes[i];

				sp_float previous = ((sp_float*)&kdops[previousIndex])[DOP18_OFFSET + axis];
				sp_float current = ((sp_float*)&kdops[currentIndex])[DOP18_OFFSET + axis];

				if (previous > current)
					break; // error
			}

			lastEvent = gpu->commandManager->updateBuffer(newIndexesLengthGPU, SIZEOF_UINT, &zeroValue, ONE_UINT, &radixSorting->lastEvent);
			sp_uint temp;
			gpu->commandManager->executeReadBuffer(newIndexesLengthGPU, SIZEOF_UINT, &temp, true);
			gpu->commandManager->executeReadBuffer(indexesLengthGPU, SIZEOF_UINT, &length, true);

			commandSaPStep
				->updateInputParameter(1, indexesLengthGPU)
				->updateInputParameter(2, radixSorting->output)
				->updateInputParameterValue(3, &zeroValue)
				->execute(1, globalWorkSize, localWorkSize, &axis, &lastEvent, ONE_UINT);

			sp_float* p1 = commandSaPStep->fetchInOutParameter<sp_float>(0);
			sp_uint* p2 = commandSaPStep->fetchInOutParameter<sp_uint>(1);
			sp_uint* p3 = commandSaPStep->fetchInOutParameter<sp_uint>(2);
			sp_uint* p4 = commandSaPStep->fetchInOutParameter<sp_uint>(3);
			sp_uint* p5 = commandSaPStep->fetchInOutParameter<sp_uint>(4);

			sp_uint newLengthCpu;
			gpu->commandManager->executeReadBuffer(newIndexesLengthGPU, SIZEOF_UINT, &newLengthCpu, true);
			sp_uint* newIndexesCpu = ALLOC_ARRAY(sp_uint, length);
			gpu->commandManager->executeReadBuffer(newIndexesGPU, length * SIZEOF_UINT, newIndexesCpu, true);
			

			// TODO: REMOVE - CHECK sap step is OK
			for (sp_uint i = 0; i < length; i++)
			{
				sp_bool foundCollision = false;
				sp_uint index = sortedIndexes[i];

				auto item = std::find(keptIndexes.begin(), keptIndexes.end(), index);
				if (item == keptIndexes.end()) // se index not kept,  ignore
					continue;

				for (sp_uint j = i+1; j < length; j++)
				{
					sp_uint nextIndex = sortedIndexes[j];

					auto itemJ = std::find(keptIndexes.begin(), keptIndexes.end(), nextIndex);
					if (itemJ == keptIndexes.end()) // se index not kept, ignore
						continue;

					if (kdops[index].max[axis] >= kdops[nextIndex].min[axis] && kdops[index].min[axis] <= kdops[nextIndex].max[axis])
					{
						foundCollision = true;
						break;
					}
				}

				if (!foundCollision) // if not found collision, remove index
					keptIndexes.erase(item);
			}

			out.clear();
			std::vector<sp_uint> itensFound;
			for (sp_uint i = 0; i < keptIndexes.size(); i++)
			{
				sp_bool foundInList = false;

				for (sp_uint j = 0; j < newLengthCpu; j++)
				{
					if (newIndexesCpu[j] == values[keptIndexes.at(i)])
					{
						itensFound.push_back(j);
						foundInList = true;
						break;
					}
				}

				if (! foundInList)
					out << keptIndexes.at(i) << ',' << END_OF_LINE;
			}

			std::string str = out.str();

			sp_assert(keptIndexes.size() == newLengthCpu, "error");

			radixSorting->lastEvent = lastEvent = commandSaPStep->lastEvent;

			gpu->commandManager->copyBuffer(newIndexesLengthGPU, indexesLengthGPU, SIZEOF_UINT, 0, 0, 1u, &lastEvent);
			lastEvent = gpu->commandManager->copyBuffer(newIndexesGPU, indexesGPU, inputLength * SIZEOF_UINT, 0, 0, 1u, &lastEvent);
			
			ALLOC_RELEASE(indexes);
		}


		sp_uint length;
		gpu->commandManager->executeReadBuffer(indexesLengthGPU, SIZEOF_UINT, &length, true);
		sp_uint* indexes = ALLOC_ARRAY(sp_uint, length);
		gpu->commandManager->executeReadBuffer(indexesGPU, length * SIZEOF_UINT, indexes, true);


		commandSaPCollisions  // last axis and get collisions pair!
			->execute(1, globalWorkSize, localWorkSize, &axis, &lastEvent , ONE_UINT);
		lastEvent = commandSaPCollisions->lastEvent;

		return collisions;
	}
	*/

	sp_uint SweepAndPrune::fetchCollisionLength()
	{
		return divideBy2((*commandSaPCollisions->fetchInOutParameter<sp_uint>(3)));
	}

#endif // OPENCL_ENALBED

}