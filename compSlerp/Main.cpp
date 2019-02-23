// CompSLERP.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include "Math.h"
#include "MathX.h"
#include "time.h"


Math::Quaternion slerp1(Math::Quaternion v0, Math::Quaternion v1, double t);
MathX::XQuaternion slerp2(MathX::XQuaternion v0, MathX::XQuaternion v1, double t);

int main()
{
	Math::Quaternion v0, v1;
	MathX::XQuaternion v2(500, 500, 500, 500), v3(300, 300, 300, 300);
	v0.x = 500;
	v0.y = 500;
	v0.z = 500;
	v0.w = 500;

	v1.x = 300;
	v1.y = 300;
	v1.z = 300;
	v1.w = 300;
	int length = 1000000;
	clock_t start1, end1, start2, end2;
	Math::Quaternion result1;
	MathX::XQuaternion result2;
	start1 = clock();
	for (int i = 0; i < length; i++)
	{
		
		
		result1 = slerp1(v0, v1, 1);
		
	}
	end1 = clock();
	
	start2 = clock();
	for (int i = 0; i < length; i++)
	{
		
		 result2 = slerp2(v2, v3, 1);
		
	}
	end2 = clock();

	

	std::cout << "SLOW ";
	std::cout << (double)(end1 - start1)/CLOCKS_PER_SEC << std::endl;
	std::cout << "FAST ";
	std::cout << (double)(end2 - start2) / CLOCKS_PER_SEC << std::endl;

}

Math::Quaternion slerp1(Math::Quaternion v0, Math::Quaternion v1, double t) {
	// Only unit quaternions are valid rotations.
	// Normalize to avoid undefined behavior.
	v0.Normalize();
	v1.Normalize();

	// Compute the cosine of the angle between the two vectors.
	double dot = Math::Dot(v0, v1);

	// If the dot product is negative, slerp won't take
	// the shorter path. Note that v1 and -v1 are equivalent when
	// the negation is applied to all four components. Fix by 
	// reversing one quaternion.
	if (dot < 0.0f) {
		v1.Conjugate();
		dot = -dot;
	}

	const double DOT_THRESHOLD = 0.9995;
	if (dot > DOT_THRESHOLD) {
		// If the inputs are too close for comfort, linearly interpolate
		// and normalize the result.

		Math::Quaternion result = v0 + (v1 - v0) * t;
		result.Normalize();
		return result;
	}

	// Since dot is in range [0, DOT_THRESHOLD], acos is safe
	double theta_0 = acos(dot);        // theta_0 = angle between input vectors
	double theta = theta_0 * t;          // theta = angle between v0 and result
	double sin_theta = sin(theta);     // compute this value only once
	double sin_theta_0 = sin(theta_0); // compute this value only once

	double s0 = cos(theta) - dot * sin_theta / sin_theta_0;  // == sin(theta_0 - theta) / sin(theta_0)
	double s1 = sin_theta / sin_theta_0;

	return (v0 * s0) + (v1 * s1);
}

MathX::XQuaternion slerp2(MathX::XQuaternion v0, MathX::XQuaternion v1, double t) {
	// Only unit quaternions are valid rotations.
	// Normalize to avoid undefined behavior.
	v0.Normalize();
	v1.Normalize();

	// Compute the cosine of the angle between the two vectors.
	double dot = MathX::Dot(v0, v1);

	// If the dot product is negative, slerp won't take
	// the shorter path. Note that v1 and -v1 are equivalent when
	// the negation is applied to all four components. Fix by 
	// reversing one quaternion.
	if (dot < 0.0f) {
		v1.Conjugate();
		dot = -dot;
	}

	const double DOT_THRESHOLD = 0.9995;
	if (dot > DOT_THRESHOLD) {
		// If the inputs are too close for comfort, linearly interpolate
		// and normalize the result.

		MathX::XQuaternion result = v0 + (v1 - v0) * t;
		result.Normalize();
		return result;
	}

	// Since dot is in range [0, DOT_THRESHOLD], acos is safe
	double theta_0 = acos(dot);        // theta_0 = angle between input vectors
	double theta = theta_0 * t;          // theta = angle between v0 and result
	double sin_theta = sin(theta);     // compute this value only once
	double sin_theta_0 = sin(theta_0); // compute this value only once

	double s0 = cos(theta) - dot * sin_theta / sin_theta_0;  // == sin(theta_0 - theta) / sin(theta_0)
	double s1 = sin_theta / sin_theta_0;

	return (v0 * s0) + (v1 * s1);
}


// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
