#ifndef _ASSERTION_H
#define _ASSERTION_H

//#ifdef STDLIB_TRACKING_ASSERT
#ifdef _MSC_BUILD // Microsoft Visual Studio Compiler
#include <assert.h>
#endif

#ifdef KATANA_TRACKING_ASSERT
void func_assert(int x, const char* file, int line);
#define assert(x...) func_assert(x, __FILE__, __LINE__)
#endif

#ifdef __arm__ // arm-none-eabi-gcc

#else // not arm

extern void __assert (const char *msg, const char *file, int line);
#ifdef __GNUC__ // GCC compiler (Linux)
#include <stdio.h>
// #define assert(x) ((void)0)
#define assert(EX) (void)((EX) || (__assert (#EX, __FILE__, __LINE__),0))
#endif

#endif

//if assertions disabled, remove any assert statements
#if !defined(_MSC_BUILD) && !defined(KATANA_TRACKING_ASSERT) //&& !defined(__GNUC__)
#define assert(x) ((void)0)
#endif

#endif // _ASSERTION_H
