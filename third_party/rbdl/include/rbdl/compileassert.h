/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_COMPILE_ASSERT_H
#define RBDL_COMPILE_ASSERT_H

/* 
 * This is a simple compile time assertion tool taken from:
 *   http://blogs.msdn.com/b/abhinaba/archive/2008/10/27/c-c-compile-time-asserts.aspx
 * written by Abhinaba Basu!
 *
 * Thanks!
 */

#ifdef __cplusplus

#define JOIN( X, Y ) JOIN2(X,Y)
#define JOIN2( X, Y ) X##Y

namespace custom_static_assert
{
template <bool> struct STATIC_ASSERT_FAILURE;
template <> struct STATIC_ASSERT_FAILURE<true> { enum { value = 1 }; };

template<int x> struct custom_static_assert_test{};
}

#define COMPILE_ASSERT(x) \
  typedef ::custom_static_assert::custom_static_assert_test<\
sizeof(::custom_static_assert::STATIC_ASSERT_FAILURE< (bool)( x ) >)>\
JOIN(_custom_static_assert_typedef, __LINE__)

#else // __cplusplus

#define COMPILE_ASSERT(x) extern int __dummy[(int)x]

#endif // __cplusplus

#define VERIFY_EXPLICIT_CAST(from, to) COMPILE_ASSERT(sizeof(from) == sizeof(to)) 

// RBDL_COMPILE_ASSERT_H_
#endif
