/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once

#ifdef BUILD_STATIC_LIBRARY
#define CDL_EXPORT
#define CDL_NO_EXPORT
#else
#define CDL_EXPORT __attribute__((visibility("default")))
#endif

#ifndef CDL_NO_EXPORT
#define CDL_NO_EXPORT __attribute__((visibility("hidden")))
#endif

#ifndef CDL_ALWAYS_INLINE
#define CDL_ALWAYS_INLINE inline __attribute__((always_inline))
#endif

#ifndef CDL_DEPRECATED
#define CDL_DEPRECATED __attribute__((__deprecated__))
#endif

#ifndef CDL_DEPRECATED_NO_EXPORT
#define CDL_DEPRECATED_NO_EXPORT CDL_NO_EXPORT CDL_DEPRECATED
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
#define CDL_NO_DEPRECATED
#endif
