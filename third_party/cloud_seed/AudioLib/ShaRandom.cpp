
#include <climits>
#include "ShaRandom.h"
#include "../Utils/Sha256.h"

namespace AudioLib
{
	using namespace std;

	vector<float> ShaRandom::Generate(long long seed, int count)
	{
		vector<unsigned char> byteList;
		auto iterations = count * sizeof(unsigned int) / (256 / 8) + 1;
		auto byteArr = (unsigned char*)&seed;
		vector<unsigned char> bytes(byteArr, byteArr + 8);

		for (size_t i = 0; i < iterations; i++)
		{
			bytes = sha256(&bytes[0], 8);
			for (auto b : bytes)
				byteList.push_back(b);
		}

		auto intArray = (unsigned int*)(&byteList[0]);
		vector<float> output;

		for (int i = 0; i < count; i++)
		{
			unsigned int val = intArray[i];
			float doubleVal = val / (float)UINT_MAX;
			output.push_back(doubleVal);
		}

		return output;
	}

	vector<float> ShaRandom::Generate(long long seed, int count, float crossSeed)
	{
		auto seedA = seed;
		auto seedB = ~seed;
		auto seriesA = Generate(seedA, count);
		auto seriesB = Generate(seedB, count);

		vector<float> output;
		for (int i = 0; i < count; i++)
			output.push_back(seriesA[i] * (1 - crossSeed) + seriesB[i] * crossSeed);

		return output;
	}
}