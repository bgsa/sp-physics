#include "SpectrumPhysicsTest.h"
#include <ParticleSystem.h>

#define CLASS_NAME ParticleSystemTest

namespace NAMESPACE_PHYSICS_TEST
{

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		SP_TEST_METHOD_DEF(ParticleSystem_constructor_empty);
		SP_TEST_METHOD_DEF(ParticleSystem_addForce);
		//SP_TEST_METHOD_DEF(ParticleSystem_update);
	};

	SP_TEST_METHOD(CLASS_NAME, ParticleSystem_constructor_empty)
	{
		ParticleSystem particleSystem = ParticleSystem(1);

		Assert::AreEqual(size_t(1), particleSystem.particlesCount, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, ParticleSystem_addForce)
	{
		ParticleSystem particleSystem = ParticleSystem(1);
		Vec3 force = { 1.0f, 2.0f, 3.0f };

		particleSystem.addForce(force);

		Assert::AreEqual(force, particleSystem.particles[0].force, L"Wrong value.", LINE_INFO());
	}

	/*
	SP_TEST_METHOD(CLASS_NAME, ParticleSystem_update)
	{
		ParticleSystem particleSystem = ParticleSystem(1);

		particleSystem.addConstantForce({1.0f, 2.0f, 3.0f});
		particleSystem.addForce({ 1.0f, 2.0f, 3.0f });

		particleSystem.update(33);

		Assert::AreEqual(Vec3(1.6f, 3.2f, 4.8f), particleSystem.particles[0].acceleration, L"Wrong value.", LINE_INFO());
	}
	*/

}

#undef CLASS_NAME