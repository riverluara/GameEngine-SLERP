#pragma once


#include <math.h>

#define MIN(x,y) ( ( (x) < (y) ) ? (x) : (y) )
#define MAX(x,y) ( ( (x) > (y) ) ? (x) : (y) )
#define In
#define Out
typedef float f32;
typedef double f64;
namespace Math
{
	////////////////////////////////////////////////////////////////////////////////////////////////
	/// <summary>
	///   Provides some functionality for dealing with angles.
	/// </summary>
	////////////////////////////////////////////////////////////////////////////////////////////////

	//////////
	
	struct Quaternion
	{
		//
		// Common constants.
		//
		static const Quaternion Zero;
		
		/// <summary>
		///   Functionality for adding two quaternions.
		/// </summary>
		/// <remarks>Inlined for performance.</remarks>
		/// <param name="a">The quaternion to add to this quaternion.</param>
		/// <returns>The new quaternion created by this operation.</returns>
		Quaternion operator+(const Quaternion& a) const
		{
			Quaternion r = { x + a.x, y + a.y, z + a.z, w + a.w };
			return r;
		}

		/// <summary>
		///   Functionality for subtracting two quaternions.
		/// </summary>
		/// <remarks>Inlined for performance.</remarks>
		/// <param name="a">The quaternion to subtract from this quaternion.</param>
		/// <returns>The new quaternion created by this operation.</returns>
		Quaternion operator-(const Quaternion& a) const
		{
			Quaternion r = { x - a.x, y - a.y, z - a.z, w - a.w };
			return r;
		}

		/// <summary>
		///   Functionality for multiplying two quaternions.
		/// </summary>
		/// <remarks>Inlined for performance.</remarks>
		/// <param name="a">The quaternion to multiply to this quaterion.</param>
		/// <returns>The new quaternion.</returns>
		Quaternion operator*(Quaternion& a) const
		{
			Quaternion r;
			r.x = w * a.x + x * a.w + y * a.z - z * a.y;
			r.y = w * a.y + y * a.w + z * a.x - x * a.z;
			r.z = w * a.z + z * a.w + x * a.y - y * a.x;
			r.w = w * a.w - x * a.x - y * a.y - z * a.z;
			return r;
		}

		Quaternion operator*(f64& a) const
		{
			Quaternion r;
			r.x = a * x;
			r.y = a * y;
			r.z = a * z;
			r.w = a * w;
			return r;
		}

		/// <summary>
		///   Functionality for multiplying two quaternions with assignment.
		/// </summary>
		/// <remarks>Inlined for performance.</remarks>
		/// <param name="a">The quaternion to multiply to this quaterion.</param>
		/// <returns>A reference to this quaternion.</returns>
		Quaternion& operator*=(Quaternion& a)
		{
			Quaternion r;
			r.x = w * a.x + x * a.w + y * a.z - z * a.y;
			r.y = w * a.y + y * a.w + z * a.x - x * a.z;
			r.z = w * a.z + z * a.w + x * a.y - y * a.x;
			r.w = w * a.w - x * a.x - y * a.y - z * a.z;
			*this = r;
			return *this;
		}

		/// <summary>
		///   Sets a quaternion using rotation axis and angle.
		/// </summary>
		/// <param name="Axis">The normalized vector axis."</param>
		/// <param name="Angle">The angle of the vector axis."</param>
		/// <returns>A reference to this quaternion.</returns>
		
		const Quaternion& Normalize(void);
		/// <summary>
		///   Sets a quaternion using Euler angles.
		/// </summary>
		/// <param name="Angles">The Euler angles for the quaternion."</param>
		/// <returns>A reference to this quaternion.</returns>
		
		/// <summary>
		///   Sets a quaternion using Euler angles.
		/// </summary>
		/// <param name="RotationX">The rotation around X in radians."</param>
		/// <param name="RotationY">The rotation around Y in radians."</param>
		/// <param name="RotationZ">The rotation around Z in radians."</param>
		/// <returns>A reference to this quaternion.</returns>
	

		/// <summary>
		///   Calculates the magnitude of this quaternion.
		/// </summary>
		/// <remarks>Inlined for performance.</remarks>
		/// <returns>The magnitude.</returns>
		f32 Magnitude(void)
		{
			return sqrtf(x*x + y * y + z * z + w * w);
		}

		/// <summary>
		///   Calculates the conjugate or inverse of a quaternion.
		/// </summary>
		/// <remarks>Inlined for performance.</remarks>
		void Conjugate(void)
		{
			x = -x;
			y = -y;
			z = -z;
			w = -w;
		}

		/// <summary>
		///   Rotates the given vector by this quaternion.
		/// </summary>
		/// <param name="a ">The vector to be rotated by this this quaternion.</param>
		/// <remarks>Inlined for performance.</remarks>
		

		f32             x;
		f32             y;
		f32             z;
		f32             w;
	};
	inline f32 Dot(const Quaternion& a, const Quaternion& b)
	{
		return b.x * a.x + b.y * a.y + b.z * a.z + b.w * a.w;
	}


	////////////////////////////////////////////////////////////////////////////////////////////////
	/// <summary>
	///   Row major implementation of a 4x4 matrix.
	/// </summary>
	////////////////////////////////////////////////////////////////////////////////////////////////

	
}
