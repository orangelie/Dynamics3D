#pragma once

#include "Math.h"

class Matrix4
{
public:
	union
	{
		struct
		{
			real m00, m01, m02, m03;
			real m10, m11, m12, m13;
			real m20, m21, m22, m23;
		};

		real data[12];
	};
};

