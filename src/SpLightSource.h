#ifndef SP_LIGHT_SOURCE_HEADER
#define SP_LIGHT_SOURCE_HEADER

#include "SpectrumPhysics.h"
#include "SpColorRGB.h"

namespace NAMESPACE_PHYSICS 
{

#define SP_LIGHT_SOURCE_TYPE_AMBIENT  (1)
#define SP_LIGHT_SOURCE_TYPE_DIFFUSE  (2)
#define SP_LIGHT_SOURCE_TYPE_SPECULAR (3)

#define SP_LIGHT_SOURCE_SPOTLIGHT_CONE     (1)
#define SP_LIGHT_SOURCE_SPOTLIGHT_CILINDER (2)

	class SpLightSource
	{
	private:
		sp_int _type;
		sp_int _enabled; /* light is turned on or off */
		sp_int _static; /* light position static or dynamic */
		sp_float _factor; /* intensity of light (distance reached) */
		Vec3 _position;
		sp_float _spotlightAngle;
		Vec3 _direction;
		sp_int _spotlightType;
		SpColorRGB _color;
		sp_int _dummy; /* only for gpu memory alignment */

	public:

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline SpLightSource()
		{
			_type = SP_LIGHT_SOURCE_TYPE_DIFFUSE;
			_enabled = true;
			_static = true;
			_spotlightType = SP_LIGHT_SOURCE_SPOTLIGHT_CONE;
			_factor = 100.0f;
		}

		/// <summary>
		/// Type of the light
		/// </summary>
		/// <returns>SP_LIGHT_SOURCE_TYPE_*</returns>
		API_INTERFACE inline sp_int type() const
		{
			return _type;
		}

		/// <summary>
		/// Set the type of the light
		/// </summary>
		/// <param name="newType">SP_LIGHT_SOURCE_TYPE_*</param>
		/// <returns>void</returns>
		API_INTERFACE inline void type(const sp_int newType)
		{
			_type = newType;
		}

		/// <summary>
		/// Check the light is enabled
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_bool isEnabled() const
		{
			return _enabled;
		}

		/// <summary>
		/// Set the enabling of the light
		/// </summary>
		/// <param name="enabled"></param>
		/// <returns>void</returns>
		API_INTERFACE inline void lightSwitch(const sp_bool enabled)
		{
			_enabled = enabled;
		}

		/// <summary>
		/// Check the light is static
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_bool isStatic() const
		{
			return _static;
		}

		/// <summary>
		/// Set the light is static or not
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline void staticLight(const sp_bool isStaticLight)
		{
			_static = isStaticLight;
		}

		/// <summary>
		/// Factor of the light
		/// </summary>
		/// <returns>Color</returns>
		API_INTERFACE inline sp_float factor() const
		{
			return _factor;
		}

		/// <summary>
		/// Set the factor of the light
		/// </summary>
		/// <param name="newFactor"></param>
		/// <returns>void</returns>
		API_INTERFACE inline void factor(const sp_float newFactor)
		{
			_factor = newFactor;
		}

		/// <summary>
		/// Spotlight angle of the light
		/// </summary>
		/// <returns>Color</returns>
		API_INTERFACE inline sp_float spotlightAngle() const
		{
			return _spotlightAngle;
		}

		/// <summary>
		/// Set the spotlight angle of the light
		/// </summary>
		/// <param name="newAngle">Angle in radians</param>
		/// <returns>void</returns>
		API_INTERFACE inline void spotlightAngle(const sp_float newAngle)
		{
			_spotlightAngle = newAngle;
		}

		/// <summary>
		/// Get the spotlight type (SP_LIGHT_SOURCE_SPORTLIGHT_*)
		/// </summary>
		/// <returns>SP_LIGHT_SOURCE_SPORTLIGHT_*</returns>
		API_INTERFACE inline sp_int spotlightType() const
		{
			return _spotlightType;
		}

		/// <summary>
		/// Set the spotlight type
		/// </summary>
		/// <param name="newType">SP_LIGHT_SOURCE_SPORTLIGHT_*</param>
		/// <returns>void</returns>
		API_INTERFACE inline void spotlightAngle(const sp_int newType)
		{
			_spotlightType = newType;
		}

		/// <summary>
		/// Color of the light
		/// </summary>
		/// <returns>Color</returns>
		API_INTERFACE inline SpColorRGB color() const
		{
			return _color;
		}

		/// <summary>
		/// Set the color of the light
		/// </summary>
		/// <param name="newColor">New color</param>
		/// <returns>void</returns>
		API_INTERFACE inline void color(const SpColorRGB& newColor)
		{
			_color = newColor;
		}

		/// <summary>
		/// Position of the light
		/// </summary>
		/// <returns>Position</returns>
		API_INTERFACE inline Vec3 position() const
		{
			return _position;
		}

		/// <summary>
		/// Set the position of the light
		/// </summary>
		/// <param name="newColor">New color</param>
		/// <returns>void</returns>
		API_INTERFACE inline void position(const Vec3& newPosition)
		{
			_position = newPosition;
		}

		/// <summary>
		/// Direction of the light
		/// </summary>
		/// <returns>Direction</returns>
		API_INTERFACE inline Vec3 direction() const
		{
			return _direction;
		}

		/// <summary>
		/// Set the direction of the light
		/// </summary>
		/// <param name="newDirection">New direction</param>
		/// <returns>void</returns>
		API_INTERFACE inline void direction(const Vec3& newDirection)
		{
			_direction = newDirection;
		}

	};

}

#endif // SP_LIGHT_SOURCE_HEADER