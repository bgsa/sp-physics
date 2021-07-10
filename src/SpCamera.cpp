#include "SpCamera.h"

namespace NAMESPACE_PHYSICS
{

	void SpCamera::init()
	{
		init(Vec3(0.0f, 5.0f, 5.0f), Vec3Zeros);
	}

	void SpCamera::init(const Vec3& position, const Vec3& target, const Vec3& up, const sp_bool invertY)
	{
		_position = position;
		_target = target;
		_up = up;

		_invertY = invertY ? -ONE_FLOAT : ONE_FLOAT;
		_velocity = 0.2f;

		this->_fieldOfView = SP_CAMERA_DEFAULT_FIELD_OF_VIEW;
		_nearFrustum = ONE_FLOAT;
		_farFrustum = 1000.0f;

		updateViewMatrix();
	}

	void SpCamera::updateViewMatrix()
	{
		normalize(_position - _target, _forward);   //zAxis
		
		cross(_forward, _up, _right);
		normalize(_right);     //xAxis

		cross(_right, _forward, _up);      //yAxis

		viewMatrix = {
			_right[0], _up[0], _forward[0], ZERO_FLOAT,
			_right[1], _up[1], _forward[1], ZERO_FLOAT,
			_right[2], _up[2], _forward[2], ZERO_FLOAT,
			ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT, ONE_FLOAT
		};

		Mat4 translation;
		createTranslate(-_position[0], -_position[1], -_position[2], translation);

		Mat4 temp;
		viewMatrix.multiply(translation, temp);

		std::memcpy(viewMatrix, temp, sizeof(Mat4));
	}

	void SpCamera::initProjectionPerspective(const Vec3& position, const Vec3& target, sp_float aspectRatio)
	{
		init(position, target);
		updateProjectionPerspectiveAspect(aspectRatio);
	}

	void SpCamera::updateProjectionPerspectiveAspect(const sp_float aspectRatio)
	{
		_aspectRatio = aspectRatio;
		setProjectionPerspective(aspectRatio, _nearFrustum, _farFrustum);
	}

	void SpCamera::setProjectionPerspective(const sp_float aspectRatio, const sp_float near, const sp_float far)
	{
		_aspectRatio = aspectRatio;
		_nearFrustum = near;
		_farFrustum = far;

		sp_float xmin, xmax, ymin, ymax;       // Dimensions of near clipping plane
		sp_float xFmin, xFmax, yFmin, yFmax;   // Dimensions of far  clipping plane

		// Do the Math for the near clipping plane
		ymax = near * sp_tan(_fieldOfView * PI_DIV_360);
		ymin = -ymax;
		xmin = ymin * aspectRatio;
		xmax = -xmin;

		// Construct the projection matrix
		projectionMatrix = Mat4Identity;
		projectionMatrix.m11 = (TWO_FLOAT * near) / (xmax - xmin);
		projectionMatrix.m22 = (TWO_FLOAT * near) / (ymax - ymin);
		projectionMatrix.m31 = (xmax + xmin) / (xmax - xmin);
		projectionMatrix.m32 = (ymax + ymin) / (ymax - ymin);
		projectionMatrix.m33 = -((far + near) / (far - near));
		projectionMatrix.m34 = -ONE_FLOAT;
		projectionMatrix.m43 = -((TWO_FLOAT * far * near) / (far - near));
		projectionMatrix.m44 = ZERO_FLOAT;

		// Do the Math for the far clipping plane
		yFmax = far * sp_tan(_fieldOfView * PI_DIV_360);
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

	void SpCamera::setProjectionOrthographic(const sp_float xMin, const sp_float xMax, const sp_float yMin, const sp_float yMax, const sp_float zMin, const sp_float zMax)
	{
		createOrthographicMatrix(xMin, xMax, yMin, yMax, zMin, zMax, projectionMatrix);

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

	void SpCamera::getHUDProjectionMatrix(const sp_float width, const sp_float height, Mat4& output) const
	{
		createOrthographicMatrix(ZERO_FLOAT, width, ZERO_FLOAT, height, -ONE_FLOAT, ONE_FLOAT, output);
	}

	Vec3 SpCamera::getFromWorldToScreen(const Vec3& vertex, const Mat4& modelViewMatrix, const SpViewportData* viewport)
	{
		sp_float halhWidth = viewport->width * HALF_FLOAT;
		sp_float halhHeight = viewport->height * HALF_FLOAT;

		Vec4 vertex4D;

		Mat4 m1, m2;
		projectionMatrix.multiply(viewMatrix, m1);
		m1.multiply(modelViewMatrix, m2);
		m2.multiply(Vec4(vertex, 1.0f), vertex4D);

		/*
		Vec4 temp1, temp2;
		modelViewMatrix.multiply(Vec4(vertex, ONE_FLOAT), temp1);
		viewMatrix.multiply(temp1, temp2);
		projectionMatrix.multiply(temp2, vertex4D);
		*/

		vertex4D /= vertex4D.w;

		Vec3 vertexOnDeviceSpace = vertex4D.toVec3();

		return Vec3(
			(vertexOnDeviceSpace[0] + ONE_FLOAT) * halhWidth,
			(vertexOnDeviceSpace[1] + ONE_FLOAT) * halhHeight,
			vertex4D[3]
		);
	}

}