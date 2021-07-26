#ifndef SP_CAMERA_HEADER
#define SP_CAMERA_HEADER

#include "SpectrumPhysics.h"
#include "SpViewportData.h"
#include "SpPhysicSettings.h"
#include "Ray.h"
#include "SpSize.h"

#define SP_CAMERA_MIN_FIELD_OF_VIEW     (0.01745329f) /* one degree */
#define SP_CAMERA_DEFAULT_FIELD_OF_VIEW (1.04719755f) /* 60 degrees */
#define SP_CAMERA_MAX_FIELD_OF_VIEW     (1.57079632f) /* 90 degrees */

#define SP_CAMERA_PROJECTION_TYPE_PERSPECTIVE  (0)
#define SP_CAMERA_PROJECTION_TYPE_ORTHOGRAPHIC (1)

namespace NAMESPACE_PHYSICS
{
	class SpCamera
	{
	private:
		Vec3 _position;
		Vec3 _target;
		Vec3 _up;
		Vec3 _forward;
		Vec3 _right;
		Vec3 _direction;
		
		sp_float _fieldOfView;
		sp_float _aspectRatio;
		sp_float _invertY;
		sp_int _projectionType;
		sp_float _nearFrustum;
		sp_float _farFrustum;
		Vec4 nearUpperLeft, nearLowerLeft, nearUpperRight, nearLowerRight;
		Vec4 farUpperLeft, farLowerLeft, farUpperRright, farLowerRight;
		sp_float xMin, xMax, yMin, yMax, zMin, zMax;

		Mat4 _projectionMatrix;
		Mat4 _viewMatrix;

		sp_bool _isDirty;
		sp_float _velocity;

		inline void updateViewMatrix()
		{
			normalize(_position - _target, _forward);   //zAxis

			cross(_forward, Vec3Up, _right);
			normalize(_right);     //xAxis

			cross(_right, _forward, _up);      //yAxis

			_direction = _forward * SpPhysicSettings::instance()->windingOrder();

			_viewMatrix = {
				_right.x, _up.x, _forward.x, ZERO_FLOAT,
				_right.y, _up.y, _forward.y, ZERO_FLOAT,
				_right.z, _up.z, _forward.z, ZERO_FLOAT,
				ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT, ONE_FLOAT
			};

			Mat4 translation;
			createTranslate(-_position.x, -_position.y, -_position.z, translation);

			Mat4 temp;
			_viewMatrix.multiply(translation, temp);

			std::memcpy(_viewMatrix, temp, sizeof(Mat4));
		}

	public:
		
		API_INTERFACE inline void init()
		{
			init(Vec3(0.0f, 5.0f, 5.0f), Vec3Zeros);
		}

		API_INTERFACE inline void init(const Vec3& position, const Vec3& target, const Vec3& up = Vec3Up, const sp_float aspectRatio = 1.0f, const sp_bool invertY = false)
		{
			_position = position;
			_target = target;
			_up = up;

			_invertY = invertY ? -ONE_FLOAT : ONE_FLOAT;
			_velocity = 0.2f;

			_fieldOfView = SP_CAMERA_DEFAULT_FIELD_OF_VIEW;
			_nearFrustum = ONE_FLOAT;
			_farFrustum = 1000.0f;

			_projectionType = SP_CAMERA_PROJECTION_TYPE_PERSPECTIVE;
			perspectiveProjection(1.0f, _nearFrustum, _farFrustum);
			updateViewMatrix();

			_isDirty = false;
		}

		API_INTERFACE inline void initProjectionPerspective(const Vec3& position, const Vec3& target, const sp_float aspectRatio)
		{
			init(position, target, Vec3Up, aspectRatio);
		}

		API_INTERFACE inline Vec3 position() const
		{
			return _position;
		}

		API_INTERFACE inline void position(const Vec3& newPosition)
		{
			_position = newPosition;
			_isDirty = true;
		}

		API_INTERFACE inline Vec3 target() const
		{
			return _target;
		}

		API_INTERFACE inline void target(const Vec3& newTarget)
		{
			_target = newTarget;
			_isDirty = true;
		}

		API_INTERFACE inline Vec3 up() const
		{
			return _up;
		}

		API_INTERFACE inline void up(const Vec3& newUp)
		{
			_up = newUp;
			_isDirty = true;
		}

		API_INTERFACE inline Vec3 right() const
		{
			return _right;
		}

		API_INTERFACE inline Vec3 forward() const
		{
			return _forward;
		}

		API_INTERFACE inline Vec3 direction() const
		{
			return _direction;
		}

		API_INTERFACE inline sp_float fieldOfView() const
		{
			return _fieldOfView;
		}

		API_INTERFACE inline void fieldOfView(const sp_float newFieldOfView)
		{
			sp_assert(_fieldOfView >= SP_CAMERA_MIN_FIELD_OF_VIEW && _fieldOfView <= SP_CAMERA_MAX_FIELD_OF_VIEW, "InvalidArgumentException");
			_fieldOfView = newFieldOfView;
			_isDirty = true;
		}

		API_INTERFACE inline sp_float aspectRatio() const
		{
			return _aspectRatio;
		}

		API_INTERFACE inline void aspectRatio(const sp_float newAspectRatio)
		{
			_aspectRatio = newAspectRatio;
			_isDirty = true;
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

		API_INTERFACE inline sp_float Y() const
		{
			return _invertY;
		}

		API_INTERFACE inline sp_float nearFrustum() const
		{
			return _nearFrustum;
		}

		API_INTERFACE inline sp_float farFrustum() const
		{
			return _farFrustum;
		}

		API_INTERFACE inline Mat4 projectionMatrix() const noexcept
		{
			return _projectionMatrix;
		}

		API_INTERFACE inline Mat4 viewMatrix() const noexcept
		{
			return _viewMatrix;
		}

		API_INTERFACE inline void hudProjectionMatrix(const sp_float width, const sp_float height, Mat4& output) const
		{
			createOrthographicMatrix(0.0f, width, 0.0f, height, -1.0f, 1.0f, output);
		}

		API_INTERFACE inline void perspectiveProjection(const sp_float aspectRatio, const sp_float near, const sp_float far)
		{
			_projectionType = SP_CAMERA_PROJECTION_TYPE_PERSPECTIVE;
			_aspectRatio = aspectRatio;
			_nearFrustum = near;
			_farFrustum = far;

			sp_float xmin, xmax, ymin, ymax;       // Dimensions of near clipping plane
			sp_float xFmin, xFmax, yFmin, yFmax;   // Dimensions of far  clipping plane

			// Do the Math for the near clipping plane
			ymax = near * sp_tan(_fieldOfView * 0.5f); // field-of-view in radians = 0.5f, in degree = PI_DIV_360
			ymin = -ymax;
			xmin = ymin * aspectRatio;
			xmax = -xmin;

			// Construct the projection matrix
			std::memcpy(_projectionMatrix, Mat4Identity, sizeof(Mat4));
			_projectionMatrix.m11 = (TWO_FLOAT * near) / (xmax - xmin);
			_projectionMatrix.m22 = (TWO_FLOAT * near) / (ymax - ymin);
			_projectionMatrix.m31 = (xmax + xmin) / (xmax - xmin);
			_projectionMatrix.m32 = (ymax + ymin) / (ymax - ymin);
			_projectionMatrix.m33 = -((far + near) / (far - near));
			_projectionMatrix.m34 = -ONE_FLOAT;
			_projectionMatrix.m43 = -((TWO_FLOAT * far * near) / (far - near));
			_projectionMatrix.m44 = ZERO_FLOAT;

			// Do the Math for the far clipping plane
			yFmax = far * sp_tan(_fieldOfView * 0.5f); // field-of-view in radians = 0.5f, in degree = PI_DIV_360
			yFmin = -yFmax;
			xFmin = yFmin * aspectRatio;
			xFmax = -xFmin;

			// Fill in values for untransformed Frustum corners
			nearUpperLeft.x = nearLowerLeft.x = xmin;
			nearUpperLeft.y = nearUpperRight.y = ymax;
			nearLowerLeft.y = nearLowerRight.y = ymin;
			nearUpperRight.x = nearLowerRight.x = xmax;
			farUpperLeft.x = farLowerLeft.x = xFmin;
			farUpperLeft.y = farUpperRright.y = yFmax;
			farLowerLeft.y = farLowerRight.y = yFmin;
			farUpperRright.x = farLowerRight.x = xFmax;
			nearUpperLeft.z = nearLowerLeft.z = nearUpperRight.z = nearLowerRight.z = -near;
			farUpperLeft.z = farLowerLeft.z = farUpperRright.z = farLowerRight.z = -far;

			nearUpperLeft.w
				= nearLowerLeft.w
				= nearUpperRight.w
				= nearLowerRight.w
				= farUpperLeft.w
				= farLowerLeft.w
				= farUpperRright.w
				= farLowerRight.w
				= ONE_FLOAT;
		}
		
		API_INTERFACE inline void orthographicProjection(const sp_float xMin, const sp_float xMax, const sp_float yMin, const sp_float yMax, const sp_float zMin, const sp_float zMax)
		{
			this->xMin = xMin;
			this->xMax = xMax;
			this->yMin = yMin;
			this->yMax = yMax;
			this->zMin = zMin;
			this->zMax = zMax;

			_projectionType = SP_CAMERA_PROJECTION_TYPE_ORTHOGRAPHIC;
			createOrthographicMatrix(xMin, xMax, yMin, yMax, zMin, zMax, _projectionMatrix);

			// Fill in values for untransformed Frustum corners	// Near Upper Left
			nearUpperLeft.x = nearLowerLeft.x = farUpperLeft.x = farLowerLeft.x = xMin;
			nearUpperRight.x = nearLowerRight.x = farUpperRright.x = farLowerRight.x = xMax;
			nearLowerLeft.y = nearLowerRight.y = farLowerLeft.y = farLowerRight.y = yMin;
			nearUpperLeft.y = nearUpperRight.y = farUpperLeft.y = farUpperRright.y = yMax;
			nearUpperLeft.z = nearLowerLeft.z = nearUpperRight.z = nearLowerRight.z = zMin;
			farUpperLeft.z = farLowerLeft.z = farUpperRright.z = farLowerRight.z = zMax;

			nearUpperLeft.w
				= nearLowerLeft.w
				= nearUpperRight.w
				= nearLowerRight.w
				= farUpperLeft.w
				= farLowerLeft.w
				= farUpperRright.w
				= farLowerRight.w
				= ONE_FLOAT;
		}

		API_INTERFACE inline void update(const sp_float aspectRatio)
		{
			_aspectRatio = aspectRatio;

			if (_projectionType == SP_CAMERA_PROJECTION_TYPE_PERSPECTIVE)
				perspectiveProjection(aspectRatio, _nearFrustum, _farFrustum);
			else
				orthographicProjection(xMin, xMax, yMin, yMax, zMin, zMax);

			updateViewMatrix();

			_isDirty = false;
		}

		API_INTERFACE inline void raycast(const Vec2& screenCoordinates, const SpSize<sp_float>& viewport, Ray& ray) const
		{
			//const sp_float tangentFoV = sp_abs(sp_tan(_fieldOfView * 0.5f));
	
			const sp_float windingOrder = SpPhysicSettings::instance()->windingOrder();
			const Vec2 screenCenter(viewport.width * 0.5f, viewport.height * 0.5f);

			const sp_float angleIncrementX = (_fieldOfView / viewport.width) * _aspectRatio;
			const sp_float angleIncrementY = (_fieldOfView / viewport.height);

			const sp_float angleX = (screenCoordinates.x - screenCenter.x + 0.5f) * angleIncrementX;
			const sp_float angleY = (screenCoordinates.y - screenCenter.y + 0.5f) * angleIncrementY;

			const Quat xAxis = Quat::createRotate(angleX * windingOrder, up());
			const Quat yAxis = Quat::createRotate(angleY * windingOrder, right());

			Vec3 rayTemp;
			rotate(xAxis, direction(), rayTemp);
			rotate(yAxis, rayTemp, ray.direction);
			normalize(ray.direction);
			ray.point = _position;
		}

		API_INTERFACE inline void fromWorldToScreen(const Vec3& vertex, const Mat4& modelViewMatrix, const SpViewportData* viewport, Vec3& output)
		{
			const sp_float halhWidth = viewport->width * HALF_FLOAT;
			const sp_float halhHeight = viewport->height * HALF_FLOAT;

			Mat4 m1, m2;
			_projectionMatrix.multiply(_viewMatrix, m1);
			m1.multiply(modelViewMatrix, m2);

			Vec4 vertex4D;
			m2.multiply(Vec4(vertex, 1.0f), vertex4D);

			/*
			Vec4 temp1, temp2;
			modelViewMatrix.multiply(Vec4(vertex, ONE_FLOAT), temp1);
			viewMatrix.multiply(temp1, temp2);
			projectionMatrix.multiply(temp2, vertex4D);
			*/

			vertex4D /= vertex4D.w;

			output.x = (vertex4D.x + ONE_FLOAT) * halhWidth;
			output.y = (vertex4D.y + ONE_FLOAT) * halhHeight;
			output.z = vertex4D.w;
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
	};
}

#endif // SP_CAMERA_HEADER