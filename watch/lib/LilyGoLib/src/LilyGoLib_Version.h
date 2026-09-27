/**
 * @file      LilyGoLib_Version.h
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2026  ShenZhen XinYuan Electronic Technology Co., Ltd
 * @date      2026-06-13
 *
 */

#pragma once

/** Major version number (X.x.x) */
#define LILYGOLIB_VERSION_MAJOR   0
/** Minor version number (x.X.x) */
#define LILYGOLIB_VERSION_MINOR   2
/** Patch version number (x.x.X) */
#define LILYGOLIB_VERSION_PATCH   0

/**
 * Macro to convert LILYGOLIB version number into an integer
 *
 * To be used in comparisons, such as LILYGOLIB_VERSION >= LILYGOLIB_VERSION_VAL(2, 0, 0)
 */
#define LILYGOLIB_VERSION_VAL(major, minor, patch) ((major << 16) | (minor << 8) | (patch))

/**
 * Current LILYGOLIB version, as an integer
 *
 * To be used in comparisons, such as LILYGOLIB_VERSION >= LILYGOLIB_VERSION_VAL(2, 0, 0)
 */
#define LILYGOLIB_VERSION  LILYGOLIB_VERSION_VAL(LILYGOLIB_VERSION_MAJOR, \
                                             LILYGOLIB_VERSION_MINOR, \
                                             LILYGOLIB_VERSION_PATCH)

#define LILYGOLIB_VERSION_STR   "v0.2.0"
