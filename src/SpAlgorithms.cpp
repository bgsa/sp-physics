#include "SpAlgorithms.h"

namespace NAMESPACE_PHYSICS
{

	void jacobi(sp_float* A, sp_float* b, sp_uint rowLength, sp_uint columnLength, sp_float* approximation, sp_float* output, const sp_float _epsilon)
	{
		sp_bool doNewIteration = true;
		while (doNewIteration)
		{
			doNewIteration = false;
	
			for (sp_uint i = 0; i < rowLength; i++)
			{
				sp_float* row = &A[i * rowLength];
				sp_float xi = b[i];

				for (sp_uint j = 0; j < columnLength; j++)
					if (i != j)
						xi -= row[j] * approximation[j];
				
				output[i] = xi / row[i];
				doNewIteration = doNewIteration || fabsf(approximation[i] - output[i]) > _epsilon;
			}

			if (doNewIteration)
				std::memcpy(approximation, output, columnLength * SIZEOF_FLOAT);
		}
	}

	void gaussSeidel(sp_float* A, sp_float* b, sp_uint rowLength, sp_uint columnLength, sp_float* approximation, sp_float* output, const sp_float _epsilon)
	{
		sp_bool doNewIteration = true;
		while (doNewIteration)
		{
			doNewIteration = false;

			for (sp_uint i = 0; i < rowLength; i++)
			{
				sp_float* row = &A[i * rowLength];
				sp_float xi = b[i];

				for (sp_uint j = 0; j < i; j++)
					xi -= row[j] * output[j];

				for (sp_uint j = i + ONE_UINT; j < columnLength; j++)
					xi -= row[j] * approximation[j];

				output[i] = xi / row[i];
				doNewIteration = doNewIteration || fabsf(approximation[i] - output[i]) > _epsilon;
			}

			if (doNewIteration)
				std::memcpy(approximation, output, columnLength * SIZEOF_FLOAT);
		}
	}

	void sor(sp_float* A, sp_float* b, sp_uint rowLength, sp_uint columnLength, sp_float relaxation, sp_float* approximation, sp_float* output, const sp_float _epsilon)
	{
		sp_bool doNewIteration = true;
		while (doNewIteration)
		{
			doNewIteration = false;

			for (sp_uint i = 0; i < rowLength; i++)
			{
				sp_float* row = &A[i * rowLength];
				sp_float xi = b[i];

				for (sp_uint j = 0; j < i; j++)
					xi -= row[j] * output[j];

				for (sp_uint j = i + ONE_UINT; j < columnLength; j++)
					xi -= row[j] * approximation[j];

				output[i] = (ONE_FLOAT - relaxation) * approximation[i] + xi * (relaxation / row[i]);
				doNewIteration = doNewIteration || fabsf(approximation[i] - output[i]) > _epsilon;
			}

			if (doNewIteration)
				std::memcpy(approximation, output, columnLength * SIZEOF_FLOAT);
		}
	}

}