/*------------------------------------------------------------------------------
 * MDK Middleware - Component ::USB:Device
 * Copyright (c) 2004-2017 ARM Germany GmbH. All rights reserved.
 *------------------------------------------------------------------------------
 * Name:    usb_compiler.h
 * Purpose: USB Library Compiler specific definitions
 *----------------------------------------------------------------------------*/

#ifndef __USB_COMPILER_H__
#define __USB_COMPILER_H__

#include "cmsis_compiler.h"

// Supported compilers are ARM Compiler 4,5 and 6, and GNU Compiler
#if (defined ( __CC_ARM ) || defined ( __ARMCC_VERSION ) || defined ( __GNUC__ ))

  // Generic macros same for all supported compilers
  #if (defined ( __ARMCC_VERSION ) && ( __ARMCC_VERSION >= 6010050 ))
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wpacked"
  #elif  defined ( __GNUC__ )
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wpacked"
    #pragma GCC diagnostic ignored "-Wattributes"
  #endif
  #ifndef   __UNALIGNED_UINT16_WRITE
    __PACKED_STRUCT T_UINT16_WRITE { uint16_t v; };
    #define __UNALIGNED_UINT16_WRITE(addr, val)  (void)((((struct T_UINT16_WRITE *)(void*)(addr))->v) = (val))
  #endif
  #ifndef   __UNALIGNED_UINT16_READ
    __PACKED_STRUCT T_UINT16_READ  { uint16_t v; };
    #define __UNALIGNED_UINT16_READ(addr)        (((const struct T_UINT16_READ  *)(const void*)(addr))->v)
  #endif
  #ifndef   __UNALIGNED_UINT32_WRITE
    __PACKED_STRUCT T_UINT32_WRITE { uint32_t v; };
    #define __UNALIGNED_UINT32_WRITE(addr, val)  (void)((((struct T_UINT32_WRITE *)(void*)(addr))->v) = (val))
  #endif
  #ifndef   __UNALIGNED_UINT32_READ
    __PACKED_STRUCT T_UINT32_READ  { uint32_t v; };
    #define __UNALIGNED_UINT32_READ(addr)        (((const struct T_UINT32_READ  *)(const void*)(addr))->v)
  #endif
  #if   defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic pop
  #elif defined ( __GNUC__ )
    #pragma GCC diagnostic pop
  #endif

  #ifndef   __SECTION
    #define __SECTION__(x)              __attribute__((section(#x)))
    #define __SECTION(x)                __SECTION__(x)
  #endif

  // Compiler specific macros
  #if   defined ( __CC_ARM )                                              /* ARM Compiler 4/5 */
    #ifndef   __AT_ADDR
      #define __AT_ADDR(x)                __attribute__((at(x)))
    #endif
  #elif defined ( __ARMCC_VERSION ) && ( __ARMCC_VERSION >= 6010050 )     /* ARM Compiler 6 */
    #ifndef   __AT_ADDR
      #define __AT_ADDR__(x)              __attribute__((section(".ARM.__AT_"#x)))
      #define __AT_ADDR(x)                __AT_ADDR__(x)
    #endif
  #elif defined ( __GNUC__ )                                              /* GNU Compiler */
    #warning Position memory containing __AT_ADDR macro at absolute address!
    #ifndef   __AT_ADDR
      #define __AT_ADDR(x)
    #endif
  #endif

#else
  #error Unsupported compiler!
#endif

#endif  // __USB_COMPILER_H__
