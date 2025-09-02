// Copyright (c) 2025, Koło Naukowe Robotyków
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/*
 * Authors: Jonasz Mularski (https://github.com/JonaszM)
 */

#ifndef DIAGNOSTIC_BROADCASTER__VISIBILITY_CONTROL_H_
#define DIAGNOSTIC_BROADCASTER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DIAGNOSTIC__BROADCASTER__VISIBILITY_EXPORT __attribute__((dllexport))
#define DIAGNOSTIC_BROADCASTER__VISIBILITY_IMPORT __attribute__((dllimport))
#else
#define DIAGNOSTIC__BROADCASTER__VISIBILITY_EXPORT __declspec(dllexport)
#define DIAGNOSTIC_BROADCASTER__VISIBILITY_IMPORT __declspec(dllimport)
#endif
#ifdef DIAGNOSTIC_BROADCASTER__VISIBILITY_BUILDING_DLL
#define DIAGNOSTIC_BROADCASTER__VISIBILITY_PUBLIC DIAGNOSTIC__BROADCASTER_INTERFACE__VISIBILITY_EXPORT
#else
#define DIAGNOSTIC_BROADCASTER__VISIBILITY_PUBLIC DIAGNOSTIC_BROADCASTER_INTERFACE__VISIBILITY_IMPORT
#endif
#define DIAGNOSTIC_BROADCASTER__VISIBILITY_PUBLIC_TYPE DIAGNOSTIC_BROADCASTER_INTERFACE__VISIBILITY_PUBLIC
#define DIAGNOSTIC_BROADCASTER__VISIBILITY_LOCAL
#else
#define DIAGNOSTIC__BROADCASTER__VISIBILITY_EXPORT __attribute__((visibility("default")))
#define DIAGNOSTIC_BROADCASTER_INTERFACE__VISIBILITY_IMPORT
#if __GNUC__ >= 4
#define DIAGNOSTIC_BROADCASTER__VISIBILITY_PUBLIC __attribute__((visibility("default")))
#define DIAGNOSTIC_BROADCASTER__VISIBILITY_LOCAL __attribute__((visibility("hidden")))
#else
#define DIAGNOSTIC_BROADCASTER__VISIBILITY_PUBLIC
#define DIAGNOSTIC_BROADCASTER__VISIBILITY_LOCAL
#endif
#define DIAGNOSTIC_BROADCASTER__VISIBILITY_PUBLIC_TYPE
#endif

#endif  // DIAGNOSTIC_BROADCASTER__VISIBILITY_CONTROL_H_