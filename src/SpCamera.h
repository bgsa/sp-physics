#ifndef SP_CAMERA_HEADER
#define SP_CAMERA_HEADER

#include "SpectrumPhysics.h"
#include "SpViewportData.h"

#define SP_CAMERA_MIN_FIELD_OF_VIEW     1.0f
#define SP_CAMERA_DEFAULT_FIELD_OF_VIEW 60.0f
#define SP_CAMERA_MAX_FIELD_OF_VIEW     90.0f

namespace NAMESPACE_PHYSICS
{
	class SpCamera
	{
	protected:
		Mat4 projectionMatrix;
		Mat4 viewMatrix;

		Vec3 position;
		Vec3 target;
		Vec3 _up;
		Vec3 _forward;
		Vec3 _right;
		
		sp_float _fieldOfView;
		sp_float aspectRatio;
		sp_float nearFrustum;
		sp_float farFrustum ;

		Vec4 nearUpperLeft, nearLowerLeft, nearUpperRight, nearLowerRight;
		Vec4 farUpperLeft, farLowerLeft, farUpperRright, farLowerRight;

		sp_float _invertY;
		sp_float _velocity;

		void updateViewMatrix();

	public:
		
		API_INTERFACE void init();
		API_INTERFACE void init(const Vec3& position, const Vec3& target, const Vec3& up = Vec3(ZERO_FLOAT, ONE_FLOAT, ZERO_FLOAT), const sp_bool invertY = false);
		API_INTERFACE void initProjectionPerspective(const Vec3& position, const Vec3& target, sp_float aspectRatio);

		API_INTERFACE inline sp_float fieldOfView() const
		{
			return _fieldOfView;
		}

		API_INTERFACE inline void fieldOfView(const sp_float newFieldOfView)
		{
			sp_assert(_fieldOfView >= SP_CAMERA_MIN_FIELD_OF_VIEW && _fieldOfView <= SP_CAMERA_MAX_FIELD_OF_VIEW, "InvalidArgumentException");
			_fieldOfView = newFieldOfView;
		}

		API_INTERFACE inline sp_float velocity() const
		{
			return _velocity;
		}
		API_INTERFACE inline void velocity(const sp_float newVelocity)
		{
			_velocity = newVelocity;
		}

		API_INTERFACE inline sp_bool isYAxisInverted() const
		{
			return _invertY == -ONE_FLOAT;
		}

		API_INTERFACE virtual void zoom(sp_float scale) {};
		API_INTERFACE virtual void moveForward(sp_float distance) {};
		API_INTERFACE virtual void moveBackward(sp_float distance) {};
		API_INTERFACE virtual void moveLeft(sp_float distance) {};
		API_INTERFACE virtual void moveRight(sp_float distance) {};
		API_INTERFACE virtual void lookAtHorizontal(sp_float angleInRadians) {};
		API_INTERFACE virtual void lookAtVertical(sp_float angleInRadians) {};
		API_INTERFACE virtual void rotateX(sp_float angle) {};
		API_INTERFACE virtual void rotateY(sp_float angle) {};
		API_INTERFACE virtual void rotateZ(sp_float angle) {};

		API_INTERFACE inline Mat4& getProjectionMatrix() noexcept
		{
			return projectionMatrix;
		}

		API_INTERFACE inline Mat4& getViewMatrix() noexcept
		{
			return viewMatrix;
		}

		API_INTERFACE void getHUDProjectionMatrix(sp_float width, sp_float height, Mat4& output) const;

		API_INTERFACE void setProjectionPerspective(sp_float aspect, sp_float near, sp_float far);
		API_INTERFACE void setProjectionOrthographic(sp_float xMin, sp_float xMax, sp_float yMin, sp_float yMax, sp_float zMin, sp_float zMax);

		API_INTERFACE void updateProjectionPerspectiveAspect(sp_float aspectRatio);

		API_INTERFACE Vec3 getFromWorldToScreen(const Vec3& vertex, const Mat4& modelViewMatrix, const SpViewportData* viewport);
	};
}

#endif // SP_CAMERA_HEADER