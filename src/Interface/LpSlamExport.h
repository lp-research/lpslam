
#ifndef LPSLAM_EXPORT_H
#define LPSLAM_EXPORT_H

#ifdef  STATIC_DEFINE
        #define LPSLAM_EXPORT
        #define LPSLAM_NO_EXPORT
#else
        #ifndef LPSLAM_EXPORT
                #ifdef LPSLAM_EXPORTS
                /* We are building this library */
                        #ifdef _MSC_VER
                                #define LPSLAM_EXPORT __declspec(dllexport)
                        #else
                                #define LPSLAM_EXPORT __attribute__((visibility("default")))
                        #endif
                #else
                /* We are using this library */
                        #ifdef _MSC_VER
                                #define LPSLAM_EXPORT __declspec(dllimport)
                        #else
                                #define LPSLAM_EXPORT __attribute__((visibility("default")))
                        #endif
                #endif
        #endif
#endif

#endif /* LPSLAM_EXPORT_H */
