#include "SpectrumPhysicsTest.h"
#include <Vec3.h>
#include <Timer.h>

#define CLASS_NAME Vec3TestPerformance

namespace NAMESPACE_PHYSICS_TEST
{


	SP_TEST_CLASS(CLASS_NAME)
	{
	private:
		Timer timer;

	public:
		SP_TEST_METHOD_DEF(swap);
	};

	SP_TEST_METHOD(CLASS_NAME, swap)
	{
		Vec3 vec1(1.0f, 2.0f, 3.0f);
		Vec3 vec2(4.0f, 5.0f, 5.0f);

		const sp_uint iterations = 100000u;

		timer.start();
		timer.update();

		for (sp_uint i = 0; i < iterations; i++)
			NAMESPACE_PHYSICS::swap(vec1, vec2);

		const sp_float elapsedTime1 = timer.elapsedTime();

		timer.update();

		for (sp_uint i = 0; i < iterations; i++)
			std::swap(vec1, vec2);

		const sp_float elapsedTime2 = timer.elapsedTime();

		timer.update();

		for (sp_uint i = 0; i < iterations; i++)
		{
			Vec3 temp;
			std::memcpy(temp, vec1, sizeof(Vec3));
			std::memcpy(vec1, vec2, sizeof(Vec3));
			std::memcpy(vec2, temp, sizeof(Vec3));
		}

		const sp_float elapsedTime3 = timer.elapsedTime();

		SpLogger::instance()->debug("Performance std::swap( Vec3 , Vec3 ): ", elapsedTime1);
		SpLogger::instance()->debug("Compare: ", elapsedTime2);
		SpLogger::instance()->debug("Compare: ", elapsedTime3);
	}

}

#undef CLASS_NAME