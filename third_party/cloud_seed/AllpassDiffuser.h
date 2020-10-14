
#pragma once

#include <vector>
#include "ModulatedAllpass.h"
#include "AudioLib/ShaRandom.h"

using namespace std;

namespace CloudSeed
{
	class AllpassDiffuser
	{
	public:
		static const int MaxStageCount = 2;

	private:
		int samplerate;

		vector<ModulatedAllpass*> filters;
		int delay;
		float modRate;
		vector<float> seedValues;
		int seed;
		float crossSeed;
		
	public:
		int Stages;

		AllpassDiffuser(int samplerate, int delayBufferLengthMillis)
		{
			auto delayBufferSize = samplerate * ((float)delayBufferLengthMillis / 1000.0);
			for (int i = 0; i < MaxStageCount; i++)
			{
				filters.push_back(new ModulatedAllpass((int)delayBufferSize, 100));
			}

			crossSeed = 0.0;
			seed = 23456;
			UpdateSeeds();
			Stages = 1;

			SetSamplerate(samplerate);
		}

		~AllpassDiffuser()
		{
			for (auto filter : filters)
				delete filter;
		}

		int GetSamplerate()
		{
			return samplerate;
		}

		void SetSamplerate(int samplerate)
		{
			this->samplerate = samplerate;
			SetModRate(modRate);
		}

		void SetSeed(int seed)
		{
			this->seed = seed;
			UpdateSeeds();
		}

		void SetCrossSeed(float crossSeed)
		{
			this->crossSeed = crossSeed;
			UpdateSeeds();
		}


		bool GetModulationEnabled()
		{
			return filters[0]->ModulationEnabled;
		}

		void SetModulationEnabled(bool value)
		{
			for (auto filter : filters)
				filter->ModulationEnabled = value;
		}

		void SetInterpolationEnabled(bool enabled)
		{
			for (auto filter : filters)
				filter->InterpolationEnabled = enabled;
		}

		float* GetOutput()
		{
			return filters[Stages - 1]->GetOutput();
		}

		
		void SetDelay(int delaySamples)
		{
			delay = delaySamples;
			Update();
		}

		void SetFeedback(float feedback)
		{
			for (auto filter : filters)
				filter->Feedback = feedback;
		}

		void SetModAmount(float amount)
		{
			for (int i = 0; i < filters.size(); i++)
			{
				filters[i]->ModAmount = amount * (0.85 + 0.3 * seedValues[MaxStageCount + i]);
			}
		}

		void SetModRate(float rate)
		{
			modRate = rate;

			for (size_t i = 0; i < filters.size(); i++)
				filters[i]->ModRate = rate * (0.85 + 0.3 * seedValues[MaxStageCount * 2 + i]) / samplerate;
		}

		void Process(float* input, int sampleCount)
		{
			ModulatedAllpass** filterPtr = &filters[0];

			filterPtr[0]->Process(input, sampleCount);

			for (int i = 1; i < Stages; i++)
			{
				filterPtr[i]->Process(filterPtr[i - 1]->GetOutput(), sampleCount);
			}
		}

		void ClearBuffers()
		{
			for (size_t i = 0; i < filters.size(); i++)
				filters[i]->ClearBuffers();
		}

	private:
		void Update()
		{
			for (size_t i = 0; i < filters.size(); i++)
			{
				auto r = seedValues[i];
				auto d = std::pow(10, r) * 0.1; // 0.1 ... 1.0
				filters[i]->SampleDelay = (int)(delay * d);
			}
		}

		void UpdateSeeds()
		{
			this->seedValues = AudioLib::ShaRandom::Generate(seed, MaxStageCount * 3, crossSeed);
			Update();
		}

	};
}
