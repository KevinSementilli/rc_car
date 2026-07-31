#ifndef REDCAT__VISIBILITY_CONTROL_H_
#define REDCAT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define REDCAT_EXPORT __attribute__((dllexport))
#define REDCAT_IMPORT __attribute__((dllimport))
#else
#define REDCAT_EXPORT __declspec(dllexport)
#define REDCAT_IMPORT __declspec(dllimport)
#endif
#ifdef REDCAT_BUILDING_DLL
#define REDCAT_PUBLIC REDCAT_EXPORT
#else
#define REDCAT_PUBLIC REDCAT_IMPORT
#endif
#define REDCAT_PUBLIC_TYPE REDCAT_PUBLIC
#define REDCAT_LOCAL
#else
#define REDCAT_EXPORT __attribute__((visibility("default")))
#define REDCAT_IMPORT
#if __GNUC__ >= 4
#define REDCAT_PUBLIC __attribute__((visibility("default")))
#define REDCAT_LOCAL __attribute__((visibility("hidden")))
#else
#define REDCAT_PUBLIC
#define REDCAT_LOCAL
#endif
#define REDCAT_PUBLIC_TYPE
#endif

#endif  // REDCAT__VISIBILITY_CONTROL_H_