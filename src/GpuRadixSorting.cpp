#ifdef OPENCL_ENABLED

#include "GpuRadixSorting.h"

#define BUCKET_LENGTH 10u

namespace NAMESPACE_PHYSICS
{
	GpuRadixSorting* GpuRadixSorting::init(GpuDevice* gpu, const sp_char* buildOptions)
	{
		if (this->gpu != NULL)
			return this;

		this->gpu = gpu;

		commandCreateIndexes = ALLOC_NEW(GpuIndexes)();
		commandCreateIndexes->init(gpu, buildOptions);

		IFileManager* fileManager = Factory::getFileManagerInstance();

		std::string sourceRadixSort = fileManager->readTextFile("RadixSorting.cl");
		radixSortProgramIndex = gpu->commandManager->cacheProgram(sourceRadixSort.c_str(), SIZEOF_CHAR * sourceRadixSort.length(), buildOptions);

		program = gpu->commandManager->cachedPrograms[radixSortProgramIndex];

		findMinMax = ALLOC_NEW(GpuFindMinMax)();
		findMinMax->init(gpu, buildOptions);

		delete fileManager;
		return this;
	}

	GpuRadixSorting* GpuRadixSorting::setParameters(sp_float* input, sp_uint inputLength, sp_uint strider, sp_uint offset)
	{
		sp_uint offsetPrefixScanCpu = BUCKET_LENGTH;
		sp_bool useExpoent = false;
		sp_uint digitIndex = 0;
		const sp_uint inputSize = inputLength * strider * SIZEOF_FLOAT;
		
		findMinMax->setParameters(input, inputLength, strider, offset);
		findMinMax->execute();

		commandCreateIndexes->setParametersCreateIndexes(inputLength);
		indexesGpu = commandCreateIndexes->execute();

		inputGpu = gpu->createBuffer(input, inputSize, CL_MEM_READ_ONLY);
		indexesLengthGpu = gpu->createBuffer(&inputLength, SIZEOF_UINT, CL_MEM_READ_ONLY);
		offsetGpu = gpu->createBuffer(&offset, SIZEOF_UINT, CL_MEM_READ_ONLY);
		offsetPrefixScanGpu = gpu->createBuffer(&offsetPrefixScanCpu, SIZEOF_UINT, CL_MEM_READ_WRITE);
		digitIndexGpu = gpu->createBuffer(&digitIndex, SIZEOF_UINT, CL_MEM_READ_WRITE);
		useExpoentGpu = gpu->createBuffer(&useExpoent, SIZEOF_UINT, CL_MEM_READ_WRITE);
		outputIndexes = gpu->createBuffer(inputLength * SIZEOF_UINT, CL_MEM_READ_WRITE);
		
		gpu->commandManager->executeReadBuffer(findMinMax->output, SIZEOF_FLOAT * 2, minMaxValues, true);
		
		sp_size* gridConfig = gpu->getGridConfigForOneDimension(inputLength);

		globalWorkSize[0] = gridConfig[0];
		localWorkSize[0] = gridConfig[1];

		ALLOC_RELEASE(gridConfig);

		threadsLength = std::min(inputLength, threadsLength);

		const sp_uint offsetTableSize = SIZEOF_UINT * 10 * threadsLength;
		offsetTable1 = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE);
		offsetTable2 = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE);
		offsetTableResult = offsetTable2;

		commandCount = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(indexesGpu, SIZEOF_UINT * inputLength)
			->setInputParameter(indexesLengthGpu, SIZEOF_UINT)
			->setInputParameter(digitIndexGpu, SIZEOF_UINT)
			->setInputParameter(useExpoentGpu, SIZEOF_BOOL)
			->setInputParameter(findMinMax->output, SIZEOF_FLOAT * 2)
			->setInputParameter(offsetTable1, offsetTableSize)
			->buildFromProgram(program, "count");

		commandPrefixScan = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable1, offsetTableSize)  //use buffer hosted GPU
			->setInputParameter(offsetTable2, offsetTableSize)
			->setInputParameter(offsetPrefixScanGpu, SIZEOF_UINT)
			->buildFromProgram(program, "prefixScan");

		commandPrefixScanSwaped = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable2, offsetTableSize)  //use buffer hosted GPU
			->setInputParameter(offsetTable1, offsetTableSize)
			->setInputParameter(offsetPrefixScanGpu, SIZEOF_UINT)
			->buildFromProgram(program, "prefixScan");

		//commandUpdatePrefixScan = gpu->commandManager->createCommand()
		//	->setInputParameter(offsetPrefixScanGpu, SIZEOF_UINT)
		//	->buildFromProgram(program, "updateNextPrefixScan");

		commandReorder = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(indexesLengthGpu, SIZEOF_UINT)
			->setInputParameter(digitIndexGpu, SIZEOF_UINT)
			->setInputParameter(useExpoentGpu, SIZEOF_BOOL)
			->setInputParameter(offsetTableResult, offsetTableSize)
			->setInputParameter(findMinMax->output, SIZEOF_FLOAT * 2)
			->setInputParameter(outputIndexes, SIZEOF_UINT * inputLength)
			->setInputParameter(indexesGpu, SIZEOF_UINT * inputLength)
			->setInputParameter(offsetPrefixScanGpu, SIZEOF_UINT)
			->buildFromProgram(program, "reorder");

		return this;
	}

	cl_mem GpuRadixSorting::execute()
	{
		sp_bool useExpoent = false;
		sp_uint digitIndex = 0;

		sp_bool offsetChanged = false;
		sp_bool indexesChanged = false;
		sp_bool prefixScanSwaped = true;

		sp_float minValue = -std::min(0.0f, minMaxValues[0]);
		
		commandCount->execute(1, globalWorkSize, localWorkSize);
		commandPrefixScan->execute(1, globalWorkSize, localWorkSize);

		const sp_uint maxIteration = (sp_uint)std::ceil(std::log(multiplyBy10(threadsLength)));

		do  // for each digit in one element
		{
			offsetChanged = false;

			for (sp_uint i = 0; i < maxIteration; i++)
			{
				//commandUpdatePrefixScan
				//	->execute(1, globalWorkSizeOneThread, localWorkSizeOneThread);

				offsetPrefixScanCpu = multiplyBy2(offsetPrefixScanCpu);
				
				if (prefixScanSwaped)
					commandPrefixScanSwaped->execute(1, globalWorkSize, localWorkSize);
				else
					commandPrefixScan->execute(1, globalWorkSize, localWorkSize);
				
				offsetChanged = !offsetChanged;
				prefixScanSwaped = !prefixScanSwaped;
			}
			offsetPrefixScanCpu = BUCKET_LENGTH;

			offsetTableResult = offsetChanged ? offsetTable1 : offsetTable2;

			commandReorder
				->updateInputParameter(4, offsetTableResult)
				->swapInputParameter(6, 7)
				->execute(1, globalWorkSize, localWorkSize);
			indexesChanged = !indexesChanged;

			if (++digitIndex > maxDigits)  // check the algorithm reach the result
			{
				if (useExpoent)
					break;

				useExpoent = true;
				digitIndex = 0;
				maxDigits = digitCount((sp_int)(minMaxValues[1] + minValue));  // MAX_DIGITS_EXPOENT - 1;
			}

			commandCount
				->updateInputParameter(1, indexesChanged ? outputIndexes : indexesGpu)
				->execute(1, globalWorkSize, localWorkSize);

			commandPrefixScan
				->execute(1, globalWorkSize, localWorkSize);

			prefixScanSwaped = true;

		} while (true);

		return outputIndexes;
	}

	GpuRadixSorting::~GpuRadixSorting()
	{
		gpu->releaseBuffer(9, inputGpu, indexesGpu, indexesLengthGpu, offsetGpu,
							offsetPrefixScanGpu, digitIndexGpu, useExpoentGpu,
							offsetTable1, offsetTable2);

		ALLOC_DELETE(commandCreateIndexes, GpuIndexes);
		ALLOC_DELETE(findMinMax, GpuFindMinMax);
		//ALLOC_DELETE(commandUpdatePrefixScan, GpuCommand);
		ALLOC_DELETE(commandCount, GpuCommand);
		ALLOC_DELETE(commandPrefixScan, GpuCommand);
		ALLOC_DELETE(commandPrefixScanSwaped, GpuCommand);
		ALLOC_DELETE(commandReorder, GpuCommand);
	}
}

#undef BUCKET_LENGTH

#endif