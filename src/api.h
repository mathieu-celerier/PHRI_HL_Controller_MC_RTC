#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define PHRI_HLController_DLLIMPORT __declspec(dllimport)
#  define PHRI_HLController_DLLEXPORT __declspec(dllexport)
#  define PHRI_HLController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define PHRI_HLController_DLLIMPORT __attribute__((visibility("default")))
#    define PHRI_HLController_DLLEXPORT __attribute__((visibility("default")))
#    define PHRI_HLController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define PHRI_HLController_DLLIMPORT
#    define PHRI_HLController_DLLEXPORT
#    define PHRI_HLController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef PHRI_HLController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define PHRI_HLController_DLLAPI
#  define PHRI_HLController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef PHRI_HLController_EXPORTS
#    define PHRI_HLController_DLLAPI PHRI_HLController_DLLEXPORT
#  else
#    define PHRI_HLController_DLLAPI PHRI_HLController_DLLIMPORT
#  endif // PHRI_HLController_EXPORTS
#  define PHRI_HLController_LOCAL PHRI_HLController_DLLLOCAL
#endif // PHRI_HLController_STATIC