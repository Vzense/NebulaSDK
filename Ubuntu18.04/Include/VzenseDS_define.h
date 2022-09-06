#ifndef VZENSEDS_DEFINE_H
#define VZENSEDS_DEFINE_H

#include "VzenseDS_enums.h"
#include "VzenseDS_types.h"

#ifdef PS_EXPORT_ON
    #ifdef _WIN32
        #define VZENSE_API_EXPORT __declspec(dllexport)
    #else
        #define VZENSE_API_EXPORT __attribute__((visibility("default")))
    #endif
#else
    #ifdef _WIN32
        #define VZENSE_API_EXPORT __declspec(dllimport)
    #else
        #define VZENSE_API_EXPORT __attribute__((visibility("default")))
    #endif
#endif

#ifdef __cplusplus
#define VZENSE_C_API_EXPORT extern "C" VZENSE_API_EXPORT
#else
#define VZENSE_C_API_EXPORT VZENSE_API_EXPORT
#define bool uint8_t
#endif

#endif /* VZENSEDS_DEFINE_H */
