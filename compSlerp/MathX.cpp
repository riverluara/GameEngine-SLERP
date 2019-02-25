#include "pch.h"
#include "DataTypes.h"
#include "MathX.h"

using namespace MathX;

_MM_ALIGN16 static const u32 kaTranslationMask[4] = { 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0 };
_MM_ALIGN16 static const f32 kaIdentityMatrix[16] =
{
	1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f
};

const __m128 XVector::sm_kxUnitX = { 1.0f, 0.0f, 0.0f, 0.0f };
const __m128 XVector::sm_kxUnitY = { 0.0f, 1.0f, 0.0f, 0.0f };
const __m128 XVector::sm_kxUnitZ = { 0.0f, 0.0f, 1.0f, 0.0f };
const __m128 XVector::sm_kxUnitW = { 0.0f, 0.0f, 0.0f, 1.0f };
const __m128 XVector::sm_kxNegateX = { -0.0f, 0.0f, 0.0f, 0.0f };
const __m128 XVector::sm_kxNegateY = { 0.0f, -0.0f, 0.0f, 0.0f };
const __m128 XVector::sm_kxNegateZ = { 0.0f, 0.0f, -0.0f, 0.0f };
const __m128 XVector::sm_kxNegateW = { 0.0f, 0.0f, 0.0f, -0.0f };
const __m128 XVector::sm_kxNegateXYZ = { -0.0f, -0.0f, -0.0f, 0.0f };
const __m128 XVector::sm_kxNegateXYZW = { -0.0f, -0.0f, -0.0f, -0.0f };
const __m128 XVector::sm_kxOne = { 1.0f, 1.0f, 1.0f, 1.0f };
const __m128 XVector::sm_kxTwo = { 2.0f, 2.0f, 2.0f, 2.0f };
const __m128 XVector::sm_kxTranslationMask = *((const __m128*)kaTranslationMask);

const XQuaternion XQuaternion::Zero(0.0f, 0.0f, 0.0f, 1.0f);

XQuaternion::XQuaternion(
	f32 x,
	f32 y,
	f32 z,
	f32 w
)
{
	__m128 xx = _mm_load_ss(&x);
	__m128 xy = _mm_load_ss(&y);
	__m128 xz = _mm_load_ss(&z);
	__m128 xw = _mm_load_ss(&w);

	m_Vector = _mm_movelh_ps(_mm_unpacklo_ps(xx, xy), _mm_unpacklo_ps(xz, xw));
}

XQuaternion XQuaternion::operator*(
	XQuaternion& a
	) const
{
	__m128 t0 = _mm_mul_ps(_mm_shuffle_ps(m_Vector, m_Vector, _MM_SHUFFLE(3, 3, 3, 3)),
		a.m_Vector);
	__m128 t1 = _mm_mul_ps(m_Vector,
		_mm_shuffle_ps(a.m_Vector, a.m_Vector, _MM_SHUFFLE(0, 3, 3, 3)));
	__m128 t2 = _mm_mul_ps(_mm_shuffle_ps(m_Vector, m_Vector, _MM_SHUFFLE(1, 0, 2, 1)),
		_mm_shuffle_ps(a.m_Vector, a.m_Vector, _MM_SHUFFLE(1, 1, 0, 2)));
	__m128 t3 = _mm_mul_ps(_mm_shuffle_ps(m_Vector, m_Vector, _MM_SHUFFLE(2, 1, 0, 2)),
		_mm_shuffle_ps(a.m_Vector, a.m_Vector, _MM_SHUFFLE(2, 0, 2, 1)));

	t1 = _mm_xor_ps(t1, sm_kxNegateW);
	t2 = _mm_xor_ps(t2, sm_kxNegateW);

	return XQuaternion(_mm_add_ps(_mm_add_ps(t0, t1), _mm_sub_ps(t2, t3)));
}

XQuaternion XQuaternion::operator+(
	const XQuaternion& a 
	) const
{
	const __m128 result = _mm_add_ps(m_Vector, a);
	

	return XQuaternion(result);
}

XQuaternion XQuaternion::operator-(
	const XQuaternion& a
	) const
{
	
	const __m128 result = _mm_sub_ps(m_Vector, a);
	return XQuaternion(result);
}


XQuaternion XQuaternion::operator*(
	f64& a
	) const
{
	__m128 scalar = _mm_set1_ps(a);

	__m128 result = _mm_mul_ps(m_Vector, scalar);
	return XQuaternion(result);
}
void XQuaternion:: Normalize(void) {

	/*__m128 t;
	__m128 sq = _mm_mul_ps(m_Vector, m_Vector);
	t = _mm_add_ps(sq, _mm_shuffle_ps(sq, sq, _MM_SHUFFLE(3, 1, 0, 2)));
	t = _mm_add_ps(t, _mm_shuffle_ps(sq, sq, _MM_SHUFFLE(3, 0, 2, 1)));*/
	
	//m_Vector = _mm_div_ps(m_Vector, _mm_sqrt_ps(t));
	m_Vector = _mm_mul_ps(m_Vector, _mm_rsqrt_ps(_mm_dp_ps(m_Vector, m_Vector, 0x7F)));
	
		
	
	//return *this;

}
