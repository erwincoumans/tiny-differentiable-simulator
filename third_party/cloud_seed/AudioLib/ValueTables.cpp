
#include <cmath>
#include "ValueTables.h"
#include <assert.h>

namespace AudioLib
{
	float DSY_SDRAM_BSS ValueTables::Sqrt[TableSize];
	float DSY_SDRAM_BSS ValueTables::Sqrt3[TableSize];
	float DSY_SDRAM_BSS ValueTables::Pow1_5[TableSize];
	float DSY_SDRAM_BSS ValueTables::Pow2[TableSize];
	float DSY_SDRAM_BSS ValueTables::Pow3[TableSize];
	float DSY_SDRAM_BSS ValueTables::Pow4[TableSize];
	float DSY_SDRAM_BSS ValueTables::x2Pow3[TableSize];

	// octave response. value float every step (2,3,4,5 or 6 steps)
	float DSY_SDRAM_BSS ValueTables::Response2Oct[TableSize];
	float DSY_SDRAM_BSS ValueTables::Response3Oct[TableSize];
	float DSY_SDRAM_BSS ValueTables::Response4Oct[TableSize];
	float DSY_SDRAM_BSS ValueTables::Response5Oct[TableSize];
	float DSY_SDRAM_BSS ValueTables::Response6Oct[TableSize];

	// decade response, value multiplies by 10 every step
	float DSY_SDRAM_BSS ValueTables::Response2Dec[TableSize];
	float DSY_SDRAM_BSS ValueTables::Response3Dec[TableSize];
	float DSY_SDRAM_BSS ValueTables::Response4Dec[TableSize];

	void ValueTables::Init()
	{
		// Initialize tables

		for (int i = 0; i <= 4000; i++)
		{
			float x = i / 4000.0;

			Sqrt[i] = std::sqrt(x);
			Sqrt3[i] = std::pow(x, 1.0 / 3.0);
			Pow1_5[i] = std::pow(x, 1.5);
			Pow2[i] = std::pow(x, 2.0);
			Pow3[i] = std::pow(x, 3.0);
			Pow4[i] = std::pow(x, 4.0);

			x2Pow3[i] = std::pow(2 * x, 3.0);
			Response2Oct[i] = (std::pow(4, x) - 1.0) / 4.0 + 0.25;
			Response3Oct[i] = (std::pow(8, x) - 1.0) / 8.0 + 0.125;
			Response4Oct[i] = (std::pow(16, x) - 1.0) / 16.0 + 0.125 / 2.0;
			Response5Oct[i] = (std::pow(32, x) - 1.0) / 32.0 + 0.125 / 4.0;
			Response6Oct[i] = (std::pow(64, x) - 1.0) / 64.0 + 0.125 / 8.0;

			Response2Dec[i] = std::pow(100, x) / 100.0;
			Response3Dec[i] = std::pow(1000, x) / 1000.0;
			Response4Dec[i] = std::pow(10000, x) / 10000.0;
		}

		for (int i = 1; i <= 4000; i++)
		{
			Response2Oct[i] = (Response2Oct[i] - Response2Oct[0]) / (1 - Response2Oct[0]);
			Response3Oct[i] = (Response3Oct[i] - Response3Oct[0]) / (1 - Response3Oct[0]);
			Response4Oct[i] = (Response4Oct[i] - Response4Oct[0]) / (1 - Response4Oct[0]);
			Response5Oct[i] = (Response5Oct[i] - Response5Oct[0]) / (1 - Response5Oct[0]);
			Response6Oct[i] = (Response6Oct[i] - Response6Oct[0]) / (1 - Response6Oct[0]);
			Response2Dec[i] = (Response2Dec[i] - Response2Dec[0]) / (1 - Response2Dec[0]);
			Response3Dec[i] = (Response3Dec[i] - Response3Dec[0]) / (1 - Response3Dec[0]);
			Response4Dec[i] = (Response4Dec[i] - Response4Dec[0]) / (1 - Response4Dec[0]);
		}

		Response2Oct[0] = 0;
		Response3Oct[0] = 0;
		Response4Oct[0] = 0;
		Response5Oct[0] = 0;
		Response6Oct[0] = 0;
		Response2Dec[0] = 0;
		Response3Dec[0] = 0;
		Response4Dec[0] = 0;
	}

	float ValueTables::Get(float index, float* table)
	{
		if (table == nullptr)
			return index;

		int idx = (int)(index * 4000.999);

		assert(idx >= 0);
		assert(idx < AudioLib::ValueTables::TableSize);
		

		return table[idx];
	}
}
