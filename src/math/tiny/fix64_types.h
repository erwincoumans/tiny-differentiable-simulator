/*
 * Copyright 2020 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _FIX64TYPES_H
#define _FIX64TYPES_H

#ifdef __GNUC__
#include <stdint.h>
namespace TINY
{
	typedef int32_t smInt32_t;
	typedef int64_t smInt64_t;
	typedef uint32_t smUint32_t;
	typedef uint64_t smUint64_t;
};
#elif defined(_MSC_VER)
namespace TINY
{
	typedef __int32 smInt32_t;
	typedef __int64 smInt64_t;
	typedef unsigned __int32 smUint32_t;
	typedef unsigned __int64 smUint64_t;
};
#else
namespace TINY
{
	typedef int smInt32_t;
	typedef long long int smInt64_t;
	typedef unsigned int smUint32_t;
	typedef unsigned long long int smUint64_t;
};
#endif

#endif
