#include "TestHeader.h"
#include <CollisionResponse.h>

#define CLASS_NAME CollisionResponseTest

namespace SP_PHYSICS_TEST_NAMESPACE
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(CollisionResponse_handle_spheres1);

		SP_TEST_METHOD_DEF(CollisionResponse_handle_spheres2);

		SP_TEST_METHOD_DEF(CollisionResponse_handle_spheres3);

		SP_TEST_METHOD_DEF(CollisionResponse_handle_spheres4);

		SP_TEST_METHOD_DEF(CollisionResponse_separate_spheres);
		
	};

	SP_TEST_METHOD(CLASS_NAME, CollisionResponse_handle_spheres1)
	{
		Sphere sphere1 = Sphere(Vec3f(-0.5f, 0.0f, 0.0f), 1.0f);
		Sphere sphere2 = Sphere(Vec3f(0.5f, 0.0f, 0.0f), 1.0f);

		sphere1.particleSystem->particles[0].previousPosition = Vec3f(-1.0f, 0.0f, 0.0f);
		sphere2.particleSystem->particles[0].previousPosition = Vec3f(-1.0f, 0.0f, 0.0f);

		CollisionResponse* response = CollisionResponse::handle(&sphere1, &sphere2);

		Assert::IsNull(response, L"wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, CollisionResponse_handle_spheres2)
	{
		Sphere sphere1 = Sphere(Vec3f(-1.0f, 0.0f, 0.0f), 1.0f);
		Sphere sphere2 = Sphere(Vec3f(1.0f, 0.0f, 0.0f), 1.0f);

		sphere1.particleSystem->particles[0].previousPosition = Vec3f(-2.0f, 0.0f, 0.0f);
		sphere1.particleSystem->particles[0].velocity = Vec3f(20.0f, 0.0f, 0.0f);
		sphere1.particleSystem->particles[0].inverseMass = 0.8f;

		sphere2.particleSystem->particles[0].previousPosition = Vec3f(1.0f, 0.5f, 0.0f);
		sphere2.particleSystem->particles[0].velocity = Vec3f(10.0f, 0.0f, 0.0f);
		sphere2.particleSystem->particles[0].inverseMass = 0.8f;

		CollisionResponse* response = CollisionResponse::handle(&sphere1, &sphere2);

		Assert::AreEqual(Vec3f(0), response->contactPoint, L"wrong value", LINE_INFO());

		ALLOC_RELEASE(response);
	}

	SP_TEST_METHOD(CLASS_NAME, CollisionResponse_handle_spheres3)
	{
		Sphere sphere1 = Sphere(Vec3f(-7.5f, 9.0f, 0.0f), 1.0f);
		Sphere sphere2 = Sphere(Vec3f(-6.5f, 8.0f, 0.0f), 1.0f);

		//ball
		sphere1.particleSystem->particles[0].previousPosition = Vec3f(-8.5f, 10.0f, 0.0f);
		sphere1.particleSystem->particles[0].velocity = Vec3f(40.0f, 40.0f, 0.0f);
		sphere1.particleSystem->particles[0].inverseMass = 1.0f / 0.15f;
		sphere1.particleSystem->particles[0].coeficientOfRestitution = 0.46f;

		//bat
		sphere2.particleSystem->particles[0].previousPosition = Vec3f(-5.5f, 7.0f, 0.0f);
		sphere2.particleSystem->particles[0].velocity = Vec3f(-31.0f, 31.0f, 0.0f);
		sphere2.particleSystem->particles[0].inverseMass = 1.0f / 1.02f;
		sphere2.particleSystem->particles[0].coeficientOfRestitution = 0.90f;

		CollisionResponse* response = CollisionResponse::handle(&sphere1, &sphere2);

		Assert::AreEqual(Vec3f(-7.0f, 8.5f, 0.0f), response->contactPoint, L"wrong value", LINE_INFO());

		ALLOC_RELEASE(response);
	}

	SP_TEST_METHOD(CLASS_NAME, CollisionResponse_handle_spheres4)
	{
		Sphere sphere1 = Sphere(Vec3f(-7.5f, 9.0f, 0.0f), 1.0f);
		Sphere sphere2 = Sphere(Vec3f(-6.5f, 8.0f, 0.0f), 1.0f);

		//ball
		sphere1.particleSystem->particles[0].previousPosition = Vec3f(-8.5f, 10.0f, 0.0f);
		sphere1.particleSystem->particles[0].velocity = Vec3f(40.0f, 40.0f, 0.0f);
		sphere1.particleSystem->particles[0].inverseMass = 1.0f / 0.15f;
		sphere1.particleSystem->particles[0].coeficientOfRestitution = 0.46f;

		//bat
		sphere2.particleSystem->particles[0].previousPosition = Vec3f(-5.5f, 7.0f, 0.0f);
		sphere2.particleSystem->particles[0].velocity = Vec3f(-31.0f, 31.0f, 0.0f);
		sphere2.particleSystem->particles[0].inverseMass = 1.0f / 1.02f;
		sphere2.particleSystem->particles[0].coeficientOfRestitution = 0.90f;

		CollisionResponse* response = CollisionResponse::handle(&sphere1, &sphere2);

		Assert::AreEqual(Vec3f(-7.0f, 8.5f, 0.0f), response->contactPoint, L"wrong value", LINE_INFO());

		ALLOC_RELEASE(response);
	}

	SP_TEST_METHOD(CLASS_NAME, CollisionResponse_separate_spheres)
	{
		Sphere sphere1 = Sphere(Vec3f(10.0f, 0.0f, 0.0f), 5.0f);
		sphere1.particleSystem->particles[0].previousPosition = Vec3f(9.0f, 0.0f, 0.0f);

		Sphere sphere2 = Sphere(Vec3f(20.0f, 0.0f, 0.0f), 8.0f);
		sphere2.particleSystem->particles[0].previousPosition = Vec3f(21.0f, 0.0f, 0.0f);

		CollisionResponse::separeteSpheres(&sphere1, &sphere2);

		Assert::AreEqual(Vec3f(8.5f, 0.0f, 0.0f), sphere1.center, L"wrong value", LINE_INFO());
		Assert::AreEqual(Vec3f(21.5f, 0.0f, 0.0f), sphere2.center, L"wrong value", LINE_INFO());

		Assert::AreEqual(sphere1.center, sphere1.particleSystem->particles[0].position, L"wrong value", LINE_INFO());
		Assert::AreEqual(sphere2.center, sphere2.particleSystem->particles[0].position, L"wrong value", LINE_INFO());
	}

}

#undef CLASS_NAME