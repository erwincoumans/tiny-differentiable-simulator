#ifndef AUDIOLIB_VALUETABLES
#define AUDIOLIB_VALUETABLES

#include "MathDefs.h"


namespace AudioLib
{
	class ValueTables
	{
	public:
		static const int TableSize = 4001;

		static DSY_SDRAM_BSS float Sqrt[TableSize];
		static DSY_SDRAM_BSS float Sqrt3[TableSize];
		static DSY_SDRAM_BSS float Pow1_5[TableSize];
		static DSY_SDRAM_BSS float Pow2[TableSize];
		static DSY_SDRAM_BSS float Pow3[TableSize];
		static DSY_SDRAM_BSS float Pow4[TableSize];
		static DSY_SDRAM_BSS float x2Pow3[TableSize];

		// octave response. value float every step (2,3,4,5 or 6 steps)
		static DSY_SDRAM_BSS float Response2Oct[TableSize];
		static DSY_SDRAM_BSS float Response3Oct[TableSize];
		static DSY_SDRAM_BSS float Response4Oct[TableSize];
		static DSY_SDRAM_BSS float Response5Oct[TableSize];
		static DSY_SDRAM_BSS float Response6Oct[TableSize];

		// decade response, value multiplies by 10 every step
		static DSY_SDRAM_BSS float Response2Dec[TableSize];
		static DSY_SDRAM_BSS float Response3Dec[TableSize];
		static DSY_SDRAM_BSS float Response4Dec[TableSize];

		static void Init();
		static float Get(float index, float* table);
	};
}

#endif
