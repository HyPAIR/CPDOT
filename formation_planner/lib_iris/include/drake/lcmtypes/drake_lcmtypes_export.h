
#ifndef DRAKE_LCMTYPES_EXPORT_H
#define DRAKE_LCMTYPES_EXPORT_H

#ifdef DRAKE_LCMTYPES_STATIC_DEFINE
#  define DRAKE_LCMTYPES_EXPORT
#  define DRAKE_LCMTYPES_NO_EXPORT
#else
#  ifndef DRAKE_LCMTYPES_EXPORT
#    ifdef drake_lcmtypes_EXPORTS
        /* We are building this library */
#      define DRAKE_LCMTYPES_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define DRAKE_LCMTYPES_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef DRAKE_LCMTYPES_NO_EXPORT
#    define DRAKE_LCMTYPES_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef DRAKE_LCMTYPES_DEPRECATED
#  define DRAKE_LCMTYPES_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef DRAKE_LCMTYPES_DEPRECATED_EXPORT
#  define DRAKE_LCMTYPES_DEPRECATED_EXPORT DRAKE_LCMTYPES_EXPORT DRAKE_LCMTYPES_DEPRECATED
#endif

#ifndef DRAKE_LCMTYPES_DEPRECATED_NO_EXPORT
#  define DRAKE_LCMTYPES_DEPRECATED_NO_EXPORT DRAKE_LCMTYPES_NO_EXPORT DRAKE_LCMTYPES_DEPRECATED
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define DRAKE_LCMTYPES_NO_DEPRECATED
#endif

#endif
