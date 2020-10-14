
#ifndef DEFAULT_INCLUDE
#define DEFAULT_INCLUDE

typedef unsigned char byte;
typedef unsigned int uint;
#ifdef _WIN32
typedef unsigned long long ulong;
#endif


#ifdef DEBUG
	#define __inline_always inline
#else
	#define __inline_always __forceinline
#endif

#endif
