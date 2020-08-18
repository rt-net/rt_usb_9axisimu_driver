
/*
 * visibility_control.h
 *
 * License: BSD-3-Clause
 *
 * Copyright (c) 2015-2020 RT Corporation <support@rt-net.jp>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of RT Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RT_USB_9AXISIMU_DRIVER__VISIBILITY_CONTROL_H_
#define RT_USB_9AXISIMU_DRIVER__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RT_USB_9AXISIMU_DRIVER_EXPORT __attribute__ ((dllexport))
    #define RT_USB_9AXISIMU_DRIVER_IMPORT __attribute__ ((dllimport))
  #else
    #define RT_USB_9AXISIMU_DRIVER_EXPORT __declspec(dllexport)
    #define RT_USB_9AXISIMU_DRIVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef RT_USB_9AXISIMU_DRIVER_BUILDING_DLL
    #define RT_USB_9AXISIMU_DRIVER_PUBLIC RT_USB_9AXISIMU_DRIVER_EXPORT
  #else
    #define RT_USB_9AXISIMU_DRIVER_PUBLIC RT_USB_9AXISIMU_DRIVER_IMPORT
  #endif
  #define RT_USB_9AXISIMU_DRIVER_PUBLIC_TYPE RT_USB_9AXISIMU_DRIVER_PUBLIC
  #define RT_USB_9AXISIMU_DRIVER_LOCAL
#else
  #define RT_USB_9AXISIMU_DRIVER_EXPORT __attribute__ ((visibility("default")))
  #define RT_USB_9AXISIMU_DRIVER_IMPORT
  #if __GNUC__ >= 4
    #define RT_USB_9AXISIMU_DRIVER_PUBLIC __attribute__ ((visibility("default")))
    #define RT_USB_9AXISIMU_DRIVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RT_USB_9AXISIMU_DRIVER_PUBLIC
    #define RT_USB_9AXISIMU_DRIVER_LOCAL
  #endif
  #define RT_USB_9AXISIMU_DRIVER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // RT_USB_9AXISIMU_DRIVER__VISIBILITY_CONTROL_H_
