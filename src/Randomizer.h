#ifndef RANDOMIZER_HEADER
#define RANDOMIZER_HEADER

#include "SpectrumPhysics.h"
#include <random>

namespace NAMESPACE_PHYSICS
{
	static std::default_random_engine randomGenerator{ std::random_device{}() };

	class Randomizer
	{
	private:
		std::uniform_int_distribution<sp_int> distribution;
		std::uniform_real_distribution<sp_float> distributionFloat;

	public:

		API_INTERFACE Randomizer(sp_int from, sp_int to)
		{
			distribution = std::uniform_int_distribution<sp_int>(from, to);
		}

		API_INTERFACE Randomizer(sp_float from, sp_float to)
		{
			distributionFloat = std::uniform_real_distribution<sp_float>(from, to);
		}

		API_INTERFACE inline sp_int randInt()
		{
			return distribution(randomGenerator);
		}

		API_INTERFACE inline sp_float randFloat()
		{
			return distributionFloat(randomGenerator);
		}
	};
}

#endif // RANDOMIZER_HEADER