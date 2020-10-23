
#include "Default.h"
#include "ReverbController.h"
#include "FastSin.h"
#include "AudioLib/ValueTables.h"
#include <assert.h>

using namespace CloudSeed;

bool isInitialized = false;

#include <vector>

std::vector<char> custom_pool;

size_t pool_index = 0;
int allocation_count = 0;

void* custom_pool_allocate(size_t size)
{
	printf("================\n");
	printf("allocation %d\n", allocation_count++);
	printf("current usage: %d\n", pool_index);
	printf("allocation: %d\n", size);

	if (pool_index + size >= custom_pool.size())
	{
		printf("Running out of memory!\n");
	}
	assert(pool_index + size < custom_pool.size());
	void* ptr = &custom_pool[pool_index];
	pool_index += size;
	return ptr;
}

extern "C"
{
	ReverbController* Create(int samplerate)
	{
		if (!isInitialized)
		{
			AudioLib::ValueTables::Init();
			FastSin::Init();
			isInitialized = true;
		}

		return new ReverbController(samplerate);
	}

	void Delete(ReverbController* item)
	{
		delete item;
	}

	int GetSamplerate(ReverbController* item)
	{
		return item->GetSamplerate();
	}

	void SetSamplerate(ReverbController* item, int samplerate)
	{
		return item->SetSamplerate(samplerate);
	}

	int GetParameterCount(ReverbController* item)
	{
		return item->GetParameterCount();
	}

	float* GetAllParameters(ReverbController* item)
	{
		return item->GetAllParameters();
	}

	float GetScaledParameter(ReverbController* item, Parameter param)
	{
		return item->GetScaledParameter(param);
	}

	void SetParameter(ReverbController* item, Parameter param, float value)
	{
		item->SetParameter(param, value);
	}

	void ClearBuffers(ReverbController* item)
	{
		item->ClearBuffers();
	}

	void Process(ReverbController* item, float* input, float* output, int bufferSize)
	{
		item->Process(input, output, bufferSize);
	}
}
