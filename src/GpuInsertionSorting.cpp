#ifdef OPENCL_ENABLED

#include "GpuInsertionSorting.h"

GpuInsertionSorting* GpuInsertionSorting::init(GpuDevice* gpu, const sp_char* buildOptions)
{
	if (this->gpu != NULL)
		return this;

	this->gpu = gpu;

	GpuCommands::init(gpu, buildOptions);

	IFileManager* fileManager = Factory::getFileManagerInstance();

	std::string sourceInsertionSort = fileManager->readTextFile("InsertionSorting.cl");
	insertionSortProgramIndex = (cl_uint) gpu->commandManager->cacheProgram(sourceInsertionSort.c_str(), SIZEOF_CHAR * sourceInsertionSort.length(), buildOptions);

	delete fileManager;
	return this;
}

GpuInsertionSorting* GpuInsertionSorting::setParameters(sp_float* input, sp_uint inputLength, sp_uint strider, sp_uint offset)
{
	const sp_uint inputSize = inputLength * strider * SIZEOF_FLOAT;

	inputGpu = gpu->createBuffer(input, inputSize, CL_MEM_READ_ONLY);
	indexesGpu = GpuCommands::creteIndexes(gpu, inputLength);
	inputLengthGpu = gpu->createBuffer(&inputLength, SIZEOF_UINT, CL_MEM_READ_ONLY);

	sortingCommand = gpu->commandManager
		->createCommand()
		->setInputParameter(inputGpu, inputSize)
		->setInputParameter(indexesGpu, inputLength * SIZEOF_UINT)
		->setInputParameter(inputLengthGpu, SIZEOF_UINT)
		->buildFromProgram(gpu->commandManager->cachedPrograms[insertionSortProgramIndex], "sort");

	const sp_uint elementsLengthAsPowOf2 = nextPowOf2(inputLength); //required for OpenCL
	const sp_uint elementsPerWorkItem = std::max( (sp_uint)(elementsLengthAsPowOf2 / gpu->maxWorkGroupSize) , (sp_uint)1u );

	globalWorkSize[0] = elementsLengthAsPowOf2 / elementsPerWorkItem;
	globalWorkSize[1] = 0;
	globalWorkSize[2] = 0;
	localWorkSize[0] = elementsPerWorkItem;
	localWorkSize[1] = 0;
	localWorkSize[2] = 0;

	return this;
}

cl_mem GpuInsertionSorting::execute()
{
	sortingCommand->execute(1, globalWorkSize, localWorkSize);

	return indexesGpu;
}

GpuInsertionSorting::~GpuInsertionSorting()
{
	gpu->releaseBuffer(3, inputGpu, indexesGpu, inputLengthGpu);

	sortingCommand->~GpuCommand();
}

#endif // OPENCL_ENABLED