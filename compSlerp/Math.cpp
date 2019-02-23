﻿// Copyright � 2008 Intel Corporation
// All Rights Reserved
// 
// The sample source code contained or described herein and all documents
// related to the sample source code ("Material") are owned by Intel Corporation
// or its suppliers or licensors. Title to the Material remains with Intel Corporation
// or its suppliers and licensors. The Material may contain trade secrets and
// proprietary and confidential information of Intel Corporation and its suppliers
// and licensors, and is protected by worldwide copyright and trade secret laws and
// treaty provisions. The sample source code is provided AS IS, with no warranties
// of any kind, express or implied. Except as expressly permitted by the Software
// license, neither Intel Corporation nor its suppliers assumes any responsibility
// or liability for any errors or inaccuracies that may appear herein.



#include "pch.h"
#include "DataTypes.h"
#include "Math.h"

using namespace Math;


const Quaternion Quaternion::Zero = { 0.0f, 0.0f, 0.0f, 1.0f };




const Quaternion& Quaternion::Normalize(void)
{
	f32 Mag = Magnitude();
	if (Mag != 0.0f) {

		f32 Inv = 1.0f / Mag;
		x *= Inv;
		y *= Inv;
		z *= Inv;
		w *= Inv;
	}
	return *this;

	
}


