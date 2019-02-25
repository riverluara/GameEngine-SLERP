#pragma once
#include <intrin.h>

__m128 __forceinline _mm_add3_ps_composite(const __m128& a, const __m128& b, const __m128& c)
{
	return _mm_add_ps(_mm_add_ps(a, b), c);
}
/// <summary>
///   Convenience function for adding 4 __m128 types.
/// </summary>
/// <param name="a">The first variable to add.</param>
/// <param name="b">The second variable to add.</param>
/// <param name="c">The third variable to add.</param>
/// <param name="d">The fourth variable to add.</param>
/// <returns>The four way add result.</returns>
__m128 __forceinline _mm_add4_ps_composite(const __m128& a, const __m128& b,
	const __m128& c, const __m128& d)
{
	return _mm_add_ps(_mm_add_ps(a, b), _mm_add_ps(c, d));
}


/// <summary>
///   Divides 2 __m128 types and zeroes out indeterminate values caused by divide-by-zero.
/// </summary>
/// <remarks>This function is used when masking divide-by-zero.</remarks>
/// <param name="a">The dividend.</param>
/// <param name="b">The divisor.</param>
/// <returns>The quotient.</returns>
__m128 __forceinline _mm_div_ps_composite(const __m128 & a, const __m128& b)
{
	__m128 xMask = _mm_cmpneq_ps(b, _mm_setzero_ps());
	__m128 t = _mm_div_ps(a, b);
	t = _mm_and_ps(t, xMask);
	return t;
}

namespace MathX {
	
	class XVector
	{
		friend class XMatrix4x4;


	public:

		XVector(void) {}

		/// <summary>
		///   Operator new override for allocation of SSE aligned memory.
		/// </summary>
		/// <param name="Size">The size to allocate.</param>
		/// <returns>A pointer to the allocated memory.</returns>
		void* operator new(size_t Size)
		{
			return _aligned_malloc(Size, sizeof(__m128));
		}

		/// <summary>
		///   Operator delete override for deallocation of SSE aligned memory.
		/// </summary>
		/// <param name="p">The pointer to deallocate.</param>
		void operator delete(void* p)
		{
			_aligned_free(p);
		}

		/// <summary>
		///   Provides a const cast to a __m128.
		/// </summary>
		/// <returns>This vector as a __m128.</returns>
		operator const __m128(void) const
		{
			return m_Vector;
		}

		/// <summary>
		///   Returns the x value of the vector.
		/// </summary>
		/// <returns>The x value of the vector.</returns>
		f32 GetX(void)
		{
			f32 x;
			_mm_store_ss(&x, m_Vector);
			return x;
		}

		/// <summary>
		///   Returns the y value of the vector.
		/// </summary>
		/// <returns>The y value of the vector.</returns>
		f32 GetY(void)
		{
			f32 y;
			_mm_store_ss(&y, _mm_shuffle_ps(m_Vector, m_Vector, 1));
			return y;
		}

		/// <summary>
		///   Returns the z value of the vector.
		/// </summary>
		/// <returns>The z value of the vector.</returns>
		f32 GetZ(void)
		{
			f32 z;
			_mm_store_ss(&z, _mm_movehl_ps(m_Vector, m_Vector));
			return z;
		}

		/// <summary>
		///   Returns the y value of the vector.
		/// </summary>
		/// <returns>The w value of the vector.</returns>
		f32 GetW(void)
		{
			f32 w;
			_mm_store_ss(&w, _mm_shuffle_ps(m_Vector, m_Vector, 3));
			return w;
		}


	protected:

		static const __m128 sm_kxUnitX;
		static const __m128 sm_kxUnitY;
		static const __m128 sm_kxUnitZ;
		static const __m128 sm_kxUnitW;
		static const __m128 sm_kxNegateX;
		static const __m128 sm_kxNegateY;
		static const __m128 sm_kxNegateZ;
		static const __m128 sm_kxNegateW;
		static const __m128 sm_kxNegateXYZ;
		static const __m128 sm_kxNegateXYZW;
		static const __m128 sm_kxOne;
		static const __m128 sm_kxTwo;
		static const __m128 sm_kxTranslationMask;

		__m128          m_Vector;
	};

	class XScalar : public XVector
	{
	public:

		
		/// <summary>
		///   Constructor.
		/// </summary>
		/// <param name="x">A broadcasted scalar value.</param>
		XScalar(__m128 x)
		{
		/*	ASSERT((x.m128_f32[0] == x.m128_f32[1]) &&
				(x.m128_f32[0] == x.m128_f32[2]) &&
				(x.m128_f32[0] == x.m128_f32[3]));*/
			m_Vector = x;
		}

		/// <summary>
		///   Provides a cast to an f32.
		/// </summary>
		/// <returns>Ths XScalar as an f32.</returns>
		operator f32(void) const
		{
			f32 s;
			_mm_store_ss(&s, m_Vector);
			return s;
		}
	};
	class XQuaternion : public XVector
	{
	public:

		/// <summary>
		///   Constructor.
		/// </summary>
		XQuaternion(void) {}

		/// <summary>
		///   Constructor.
		/// </summary>
		/// <param name="q">A quaternion in array form.</param>
		XQuaternion(f32* q)
		{
			m_Vector = _mm_loadu_ps(q);
		}

		/// <summary>
		///   Constructor.
		/// </summary>
		/// <param name="x">A quaternion in __m128 form.</param>
		XQuaternion(__m128 x)
		{
			m_Vector = x;
		}

		/// <summary>
		///   Constructor.
		/// </summary>
		/// <param name="x">The x value.</param>
		/// <param name="y">The y value.</param>
		/// <param name="z">The z value.</param>
		/// <param name="w">The w value.</param>
		XQuaternion(f32 x, f32 y, f32 z, f32 w);

		

		/// <summary>
		///   Functionality for multiplying two quaternions.
		/// </summary>
		/// <param name="a">The quaternion to multiply to this quaterion.</param>
		/// <returns>The new quaternion.</returns>
		XQuaternion operator*(XQuaternion& a) const;
		XQuaternion operator+(const XQuaternion& a) const;
		XQuaternion operator-(const XQuaternion& a) const;
		XQuaternion operator*(f64& a) const;
		
		void Normalize(void);
		/// <summary>
		///   Functionality for multiplying two quaternions with assignment.
		/// </summary>
		/// <param name="a">The quaternion to multiply to this quaterion.</param>
		/// <returns>A reference to this quaternion.</returns>
		XQuaternion& operator*=(XQuaternion& a)
		{
			*this = *this * a;
			return *this;
		}

		/// <summary>
		///   Calculates the magnitude of this quaternion.
		/// </summary>
		/// <returns>The magnitude.</returns>
		f32 Magnitude(void) const
		{
			__m128 t;
			__m128 sq = _mm_mul_ps(m_Vector, m_Vector);
			t = _mm_add_ps(sq, _mm_shuffle_ps(sq, sq, _MM_SHUFFLE(3, 1, 0, 2)));
			t = _mm_add_ps(t, _mm_shuffle_ps(sq, sq, _MM_SHUFFLE(3, 0, 2, 1)));
			
			return _mm_sqrt_ps(t).m128_f32[0];
		}

		/// <summary>
		///   Calculates the conjugate or inverse of a quaternion.
		/// </summary>
		void Conjugate(void)
		{
			m_Vector = _mm_xor_ps(m_Vector, sm_kxNegateXYZW);
			//return *this;
		}

		
	


	public:

		static const XQuaternion Zero;
	};
	inline f32 Dot(const XQuaternion& a, const XQuaternion& b)
	{
	/*	const int mask = 0xFF;
		__m128 result = _mm_dp_ps(a, b, mask);*/
		return _mm_dp_ps(a, b, 0xFF).m128_f32[0];
		
	}
}

