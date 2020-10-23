#ifndef MODULATEDDELAY
#define MODULATEDDELAY

#include "ModulatedDelay.h"
#include "Utils.h"
#include "FastSin.h"

extern void* custom_pool_allocate(size_t size);

namespace CloudSeed
{
	class ModulatedDelay
	{
	private:

		const int ModulationUpdateRate = 8;

		float* delayBuffer;
		float* output;
		int bufferSize;
		int writeIndex;
		int readIndexA;
		int readIndexB;
		int samplesProcessed;
		int delayBufferSizeSamples;

		float modPhase;
		float gainA;
		float gainB;

	public:
		int SampleDelay;
		
		float ModAmount;
		float ModRate;

		ModulatedDelay(int bufferSize, int delayBufferSizeSamples, int sampleDelay)
		{
			this->delayBufferSizeSamples = delayBufferSizeSamples;
			this->bufferSize = bufferSize;
			this->delayBuffer = new (custom_pool_allocate(sizeof(float) * delayBufferSizeSamples)) float[delayBufferSizeSamples];
			this->output = new (custom_pool_allocate(sizeof(float) * bufferSize)) float[bufferSize];
			this->SampleDelay = sampleDelay;
			writeIndex = 0;
			modPhase = 0.01 + 0.98 * (std::rand() / (float)RAND_MAX);
			ModRate = 0.0;
			ModAmount = 0.0;
			Update();
		}

		~ModulatedDelay()
		{
			delete delayBuffer;
			delete output;
		}

		float* GetOutput()
		{
			return output;
		}

		void Process(float* input, int sampleCount)
		{
			for (int i = 0; i < sampleCount; i++)
			{
				if (samplesProcessed == ModulationUpdateRate)
					Update();

				delayBuffer[writeIndex] = input[i];
				output[i] = delayBuffer[readIndexA] * gainA + delayBuffer[readIndexB] * gainB;

				writeIndex++;
				readIndexA++;
				readIndexB++;
				if (writeIndex >= delayBufferSizeSamples) writeIndex -= delayBufferSizeSamples;
				if (readIndexA >= delayBufferSizeSamples) readIndexA -= delayBufferSizeSamples;
				if (readIndexB >= delayBufferSizeSamples) readIndexB -= delayBufferSizeSamples;
				samplesProcessed++;
			}
		}

		void ClearBuffers()
		{
			Utils::ZeroBuffer(delayBuffer, delayBufferSizeSamples);
			Utils::ZeroBuffer(output, bufferSize);
		}


	private:
		void Update()
		{
			modPhase += ModRate * ModulationUpdateRate;
			if (modPhase > 1)
				modPhase = std::fmod(modPhase, 1.0);

			auto mod = FastSin::Get(modPhase);
			auto totalDelay = SampleDelay + ModAmount * mod;

			auto delayA = (int)totalDelay;
			auto delayB = (int)totalDelay + 1;

			auto partial = totalDelay - delayA;

			gainA = 1 - partial;
			gainB = partial;

			readIndexA = writeIndex - delayA;
			readIndexB = writeIndex - delayB;
			if (readIndexA < 0) readIndexA += delayBufferSizeSamples;
			if (readIndexB < 0) readIndexB += delayBufferSizeSamples;

			samplesProcessed = 0;
		}
	};
}

#endif