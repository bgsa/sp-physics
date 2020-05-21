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

	public:

		Randomizer(sp_int from, sp_int to)
		{
			distribution = std::uniform_int_distribution<sp_int>(from, to);
		}

		API_INTERFACE inline sp_int rand()
		{
			return distribution(randomGenerator);
		}
	};
}

#endif // RANDOMIZER_HEADER