#ifndef SWEEP_AND_PRUNE_HEADER
#define SWEEP_AND_PRUNE_HEADER

#include "SpectrumPhysics.h"
#include "AABB.h"
#include "DOP18.h"
#include "AlgorithmSorting.h"

#if OPENCL_ENABLED
	#include "GpuComposedCommand.h"
	#include "GpuContext.h"
	#include "GpuRadixSorting.h"
	#include <FileSystem.h>
#endif

namespace NAMESPACE_PHYSICS
{
#define SP_SAP_MAX_COLLISION_PER_OBJECT (32)

	class SweepAndPruneResult
	{
	public:
		sp_uint* indexes;
		sp_uint length;

		SweepAndPruneResult()
		{
			this->indexes = nullptr;
			this->length = ZERO_UINT;
		}

		SweepAndPruneResult(sp_uint* indexes, sp_uint count)
		{
			this->indexes = indexes;
			this->length = count;
		}

		~SweepAndPruneResult()
		{
			if (indexes != nullptr)
			{
				sp_mem_release(indexes);
				indexes = nullptr;
			}
		}
	};
	
	class SweepAndPrune
		: public GpuComposedCommand
	{
	private:
		
#ifdef OPENCL_ENABLED

		GpuDevice* gpu;
		cl_program sapProgram;

		GpuRadixSorting* radixSorting;
		GpuCommand* commandSaPCollisions;
		GpuCommand* commandBuildElements;

		cl_mem indexesLengthGPU;
		cl_mem indexesGPU;

		cl_mem elementsGPU;

		sp_size globalWorkSize[3] = { 0, 0, 0 };
		sp_size localWorkSize[3] = { 0, 0, 0 };

		sp_size axis;

		void initIndexes(sp_uint inputLength);

		/// <summary>
		/// Given an axis, get the axis ID based on k-DOP axis (9 axis availables)
		/// </summary>
		API_INTERFACE inline sp_uint axisIdForDOP18(const Vec3& axisVector) const
		{
			sp_float value = axisVector.dot(DOP18_NORMALS[DOP18_PLANES_LEFT_INDEX]);
			sp_uint id = DOP18_PLANES_LEFT_INDEX;

			for (sp_uint i = DOP18_PLANES_LEFT_INDEX + 1u; i <= DOP18_PLANES_LEFT_FRONT_INDEX; i++)
			{
				Vec3 v;
				NAMESPACE_PHYSICS::normalize(DOP18_NORMALS[i], v);

				const sp_float newValue = axisVector.dot(v);

				if (newValue > value)
				{
					value = newValue;
					id = divideBy2(i);
				}
			}

			return id;
		}

		/// <summary>
		/// Given an axis, get the axis ID based on AABB axis (3 axis availables)
		/// </summary>
		API_INTERFACE inline sp_uint axisIdForAABB(const Vec3& axisVector) const
		{
			sp_float value = axisVector.dot(AABB_NORMALS[AABB_PLANES_LEFT_INDEX]);
			sp_uint id = AABB_PLANES_LEFT_INDEX;

			for (sp_uint i = AABB_PLANES_LEFT_INDEX + 1u; i <= AABB_PLANES_DEPTH_INDEX; i++)
			{
				Vec3 v;
				NAMESPACE_PHYSICS::normalize(AABB_NORMALS[i], v);

				const sp_float newValue = axisVector.dot(v);

				if (newValue > value)
				{
					value = newValue;
					id = divideBy2(i);
				}
			}

			sp_assert(id == AABB_AXIS_X || id == AABB_AXIS_Y || id == AABB_AXIS_Z, "ApplicationException");
			return id;
		}

#endif // OPENCL_ENABLED

	public:

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline SweepAndPrune()
		{
			axis = ZERO_SIZE;

#ifdef OPENCL_ENABLED
			gpu = nullptr;
			sapProgram = nullptr;

			radixSorting = nullptr;
			commandSaPCollisions = nullptr;
			commandBuildElements = nullptr;

			indexesLengthGPU = nullptr;
			indexesGPU = nullptr;

			elementsGPU = nullptr;
#endif // OPENCL_ENABLED
		}

		///<summary>
		/// Find the collisions using Sweep and Prune method
		/// Returns the pair indexes
		///</summary>
		API_INTERFACE static SweepAndPruneResult findCollisions(AABB* aabbs, sp_uint length);

		///<summary>
		/// Find the collisions using Sweep and Prune method
		/// Returns the pair indexes and length in the last parameter
		///</summary>
		API_INTERFACE static void findCollisions(DOP18* kdops, sp_uint* indexes, sp_uint length, SweepAndPruneResult* resultCpu);

		/// <summary>
		/// Get the best axis to sweep and prune
		/// </summary>
		/// <param name="output">Axis</param>
		/// <returns>output axis parameter; True if the method converged orelse False</returns>
		API_INTERFACE sp_bool pca(Vec3& output, const sp_uint maxIterations = SP_UINT_MAX) const;

		API_INTERFACE sp_uint updateAxis(const Vec3& axisVector)
		{
			switch (SpPhysicSettings::instance()->boundingVolumeType())
			{
			case BoundingVolumeType::DOP18:
			case BoundingVolumeType::Sphere:
				axis = axisIdForDOP18(axisVector);
				break;
			case BoundingVolumeType::AABB:
				axis = axisIdForAABB(axisVector);
				break;

			default:
				sp_assert(false, "InvalidArgumentException");
				break;
			}

			return axis;
		}

#ifdef OPENCL_ENABLED

		///<summary>
		/// Init Sweep And Prune Algorithm on GPU
		///</summary>
		API_INTERFACE SweepAndPrune* init(GpuDevice* gpu, const char* buildOptions = NULL) override;

		///<summary>
		/// Set the parameters for running SweepAndPrune Command
		///</summary>
		API_INTERFACE void setParameters(cl_mem boundingVolumesGpu, sp_uint boundingVolumesLength, BoundingVolumeType boundingVolumeType, sp_uint strider, cl_mem rigidBodies, const sp_uint rigidBodySize, cl_mem outputIndexLength, cl_mem outputIndex);

		///<summary>
		/// Find the collisions using Sweep and Prune method in GPU
		/// Returns the pair indexes
		///</summary>
		API_INTERFACE cl_mem execute(sp_uint previousEventsLength, cl_event* previousEvents, cl_event* evt) override;

		///<summary>
		/// Get the length of collisions pairs detected
		///</summary>
		API_INTERFACE sp_uint fetchCollisionLength(const sp_uint previousEventsLength, cl_event* previousEvents, cl_event* evt);

		/// <summary>
		/// Get the colision index pairs
		/// </summary>
		API_INTERFACE void fetchCollisionIndexes(sp_uint* output, const sp_uint previousEventsLength, cl_event* previousEvents, cl_event* evt) const;

		///<summary>
		/// Update rigid bodies data on GPU
		///</summary>
		API_INTERFACE inline void updatePhysicProperties(void* rigidBodies3D)
		{
			cl_event evt;
			commandSaPCollisions->updateInputParameterValue(2u, rigidBodies3D, ZERO_UINT, NULL, &evt);
			gpu->releaseEvent(evt);
		}

		///<summary>
		/// Update bouding volume parameter
		///</summary>
		API_INTERFACE inline void updateBoundingVolumes(void* boudingVolumes)
		{
			cl_event evt;
			commandSaPCollisions->updateInputParameterValue(0u, boudingVolumes, ZERO_UINT, NULL, &evt);
			gpu->releaseEvent(evt);
		}

#endif // OPENCL_ENABLED

		/// <summary>
		/// Release allocated resources from this object
		/// </summary>
		API_INTERFACE void dispose() override
		{
#if OPENCL_ENABLED
			if (radixSorting != nullptr)
			{
				sp_mem_delete(radixSorting, GpuRadixSorting);
				radixSorting = nullptr;
			}

			if (commandSaPCollisions != nullptr)
			{
				sp_mem_delete(commandSaPCollisions, GpuCommand);
				commandSaPCollisions = nullptr;
			}

			if (commandBuildElements != nullptr)
			{
				sp_mem_delete(commandBuildElements, GpuCommand);
				commandBuildElements = nullptr;
			}

			if (indexesGPU != nullptr)
			{
				gpu->releaseBuffer(indexesGPU);
				indexesGPU = nullptr;
			}
			
			if (indexesLengthGPU != nullptr)
			{
				gpu->releaseBuffer(indexesLengthGPU);
				indexesLengthGPU = nullptr;
			}

			if (elementsGPU != nullptr)
			{
				gpu->releaseBuffer(elementsGPU);
				elementsGPU = nullptr;
			}

			if (sapProgram != nullptr)
			{
				HANDLE_OPENCL_ERROR(clReleaseProgram(sapProgram));
				sapProgram = nullptr;
			}
#endif // OPENCL_ENABLED
		}

		/// <summary>
		/// Description of the object
		/// </summary>
		API_INTERFACE const sp_char* toString() override
		{
			return "Sweep And Prune";
		}

		~SweepAndPrune() { dispose(); }

	};

}

#endif // SWEEP_AND_PRUNE_HEADER