#pragma once

#include "Math.h"

class Matrix3
{
public:
	union
	{
		struct
		{
			real m00, m01, m02;
			real m10, m11, m12;
			real m20, m21, m22;
		};

		real data[9];
	};
};

