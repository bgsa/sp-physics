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

		this->_invertY = invertY ? -ONE_FLOAT : ONE_FLOAT;
		this->_velocity = 0.2f;

		this->_fieldOfView = SP_CAMERA_DEFAULT_FIELD_OF_VIEW;
		this->nearFrustum = ONE_FLOAT;
		this->farFrustum = 1000.0f;

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

	void SpCamera::updateProjectionPerspectiveAspect(sp_float aspectRatio)
	{
		this->aspectRatio = aspectRatio;
		setProjectionPerspective(aspectRatio, nearFrustum, farFrustum);
	}

	void SpCamera::setProjectionPerspective(sp_float aspectRatio, sp_float near, sp_float far)
	{
		this->aspectRatio = aspectRatio;
		this->nearFrustum = near;
		this->farFrustum = far;

		sp_float xmin, xmax, ymin, ymax;       // Dimensions of near clipping plane
		sp_float xFmin, xFmax, yFmin, yFmax;   // Dimensions of far  clipping plane

		// Do the Math for the near clipping plane
		ymax = near * tanf(sp_float(_fieldOfView * PI_DIV_360));
		ymin = -ymax;
		xmin = ymin * aspectRatio;
		xmax = -xmin;

		// Construct the projection matrix
		projectionMatrix = Mat4Identity;
		projectionMatrix[0] = (TWO_FLOAT * near) / (xmax - xmin);
		projectionMatrix[5] = (TWO_FLOAT * near) / (ymax - ymin);
		projectionMatrix[8] = (xmax + xmin) / (xmax - xmin);
		projectionMatrix[9] = (ymax + ymin) / (ymax - ymin);
		projectionMatrix[10] = -((far + near) / (far - near));
		projectionMatrix[11] = -ONE_FLOAT;
		projectionMatrix[14] = -((TWO_FLOAT * far * near) / (far - near));
		projectionMatrix[15] = ZERO_FLOAT;

		// Do the Math for the far clipping plane
		yFmax = far * tanf((sp_float)(_fieldOfView * PI_DIV_360));
		yFmin = -yFmax;
		xFmin = yFmin * aspectRatio;
		xFmax = -xFmin;

		// Fill in values for untransformed Frustum corners
		// Near Upper Left
		nearUpperLeft[0] = xmin;
		nearUpperLeft[1] = ymax;
		nearUpperLeft[2] = -near;
		nearUpperLeft[3] = ONE_FLOAT;

		// Near Lower Left
		nearLowerLeft[0] = xmin;
		nearLowerLeft[1] = ymin;
		nearLowerLeft[2] = -near;
		nearLowerLeft[3] = ONE_FLOAT;

		// Near Upper Right
		nearUpperRight[0] = xmax;
		nearUpperRight[1] = ymax;
		nearUpperRight[2] = -near;
		nearUpperRight[3] = ONE_FLOAT;

		// Near Lower Right
		nearLowerRight[0] = xmax;
		nearLowerRight[1] = ymin;
		nearLowerRight[2] = -near;
		nearLowerRight[3] = ONE_FLOAT;

		// Far Upper Left
		farUpperLeft[0] = xFmin;
		farUpperLeft[1] = yFmax;
		farUpperLeft[2] = -far;
		farUpperLeft[3] = ONE_FLOAT;

		// Far Lower Left
		farLowerLeft[0] = xFmin;
		farLowerLeft[1] = yFmin;
		farLowerLeft[2] = -far;
		farLowerLeft[3] = ONE_FLOAT;

		// Far Upper Right
		farUpperRright[0] = xFmax;
		farUpperRright[1] = yFmax;
		farUpperRright[2] = -far;
		farUpperRright[3] = ONE_FLOAT;

		// Far Lower Right
		farLowerRight[0] = xFmax;
		farLowerRight[1] = yFmin;
		farLowerRight[2] = -far;
		farLowerRight[3] = ONE_FLOAT;
	}

	void SpCamera::setProjectionOrthographic(sp_float xMin, sp_float xMax, sp_float yMin, sp_float yMax, sp_float zMin, sp_float zMax)
	{
		createOrthographicMatrix(xMin, xMax, yMin, yMax, zMin, zMax, projectionMatrix);

		// Fill in values for untransformed Frustum corners	// Near Upper Left
		nearUpperLeft[0] = xMin;
		nearUpperLeft[1] = yMax;
		nearUpperLeft[2] = zMin;
		nearUpperLeft[3] = ONE_FLOAT;

		// Near Lower Left
		nearLowerLeft[0] = xMin;
		nearLowerLeft[1] = yMin;
		nearLowerLeft[2] = zMin;
		nearLowerLeft[3] = ONE_FLOAT;

		// Near Upper Right
		nearUpperRight[0] = xMax;
		nearUpperRight[1] = yMax;
		nearUpperRight[2] = zMin;
		nearUpperRight[3] = ONE_FLOAT;

		// Near Lower Right
		nearLowerRight[0] = xMax;
		nearLowerRight[1] = yMin;
		nearLowerRight[2] = zMin;
		nearLowerRight[3] = ONE_FLOAT;

		// Far Upper Left
		farUpperLeft[0] = xMin;
		farUpperLeft[1] = yMax;
		farUpperLeft[2] = zMax;
		farUpperLeft[3] = ONE_FLOAT;

		// Far Lower Left
		farLowerLeft[0] = xMin;
		farLowerLeft[1] = yMin;
		farLowerLeft[2] = zMax;
		farLowerLeft[3] = ONE_FLOAT;

		// Far Upper Right
		farUpperRright[0] = xMax;
		farUpperRright[1] = yMax;
		farUpperRright[2] = zMax;
		farUpperRright[3] = ONE_FLOAT;

		// Far Lower Right
		farLowerRight[0] = xMax;
		farLowerRight[1] = yMin;
		farLowerRight[2] = zMax;
		farLowerRight[3] = ONE_FLOAT;
	}

	void SpCamera::getHUDProjectionMatrix(sp_float width, sp_float height, Mat4& output) const
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