#pragma once

#include <cmath>
#include "AudioLib/MathDefs.h"
#include "FastSin.h"

namespace CloudSeed
{
	class FastSin
	{
	private:
		static const int DataSize = 32768;
		static DSY_SDRAM_BSS float data[DataSize];

	public:
		static void ZeroBuffer(float* buffer, int len);
		static void Init()
		{
			for (int i = 0; i < DataSize; i++)
			{
				data[i] = std::sin(2 * M_PI * i / (float)DataSize);
			}
		}

		static float Get(float index)
		{
			return data[(int)(index * 32767.99999)];
		}
	};
}

