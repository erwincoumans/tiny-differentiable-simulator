#ifndef SHARANDOM
#define SHARANDOM

#include <vector>

namespace AudioLib
{
	class ShaRandom
	{
	public:
		static std::vector<float> Generate(long long seed, int count);
		static std::vector<float> Generate(long long seed, int count, float crossSeed);
	};
}

#endif
