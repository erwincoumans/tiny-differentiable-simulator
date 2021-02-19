#include "printf_console.h"
#include <stdio.h>

#include <rbdl/Logging.h>


void logError(const char* msg, const char* arg0, const char* arg1, const char* arg2)
{
	LOG << msg << " " << arg0 << " " << arg1 << " " << arg2 << std::endl;
}
	
void logDebug(const char* msg, float v0, float v1)
{
	LOG << msg << " " << v0 << " " << v1 << std::endl;
};
void logDebug(const char* msg, const char* msg1, const char* arg1)
{
	LOG << msg << " " << msg1 << " " << arg1 << std::endl;
}

void logInform(const char* msg, const char* arg0)
{
	LOG << msg << " " << arg0 << std::endl;
}
void logWarn(const char* msg,int id, const char* arg0)
{
	LOG << msg << " " << id << " " << arg0 << std::endl;
}
