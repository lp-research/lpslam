cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# for VC 2017's alignment problems.
# https://gamedev.amazon.com/forums/questions/71621/vs2017-158-stdaligned-storage-error.html
add_definitions(-D_ENABLE_EXTENDED_ALIGNED_STORAGE)

if (UNIX)
   if (LPSLAM_BUILD_OPENVSLAM)
        # for some weird reason, the target has the wrong binary path
        # and not lib/liblpslam.so
        set_target_properties(openvslam PROPERTIES
                        OUTPUT_NAME openvslam
                        LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
   endif()
endif()

find_package(OpenMP)

if (UNIX)
        set(lpslam-compile-options -Wall -Wextra -fopenmp -pedantic)
else()
        set(lpslam-compile-options )
endif()

set(lpslam_src
    Sources/OpenCVCameraSource.cpp
    Sources/FileImageSource.cpp
    Sources/ImageSourceBase.cpp
    Processor/CameraCalibrationProcessor.cpp
    Manager/SlamManager.cpp
    Manager/CameraRegistry.cpp
    Manager/RecordEngine.cpp
    Manager/ReplayEngine.cpp
    Serialize/SlamSerialize.pb.cc
    InterfaceImpl/LpSlamManager.cpp
)

set(lpslam_test_src
        test/TransformationsTest.cpp
        test/SlamManagerTest.cpp
        test/InterfaceTest.cpp
        test/LockedAccessTest.cpp
        test/gtests.cpp)

if (LPSLAM_BUILD_OPTIMIZER)
    list(APPEND lpslam_test_src test/CoordinateSystemAlignmentTest.cpp)
endif()

set(lpslam_libs
        OpenMP::OpenMP_CXX
        CONAN_PKG::tbb
        opencv_core
        opencv_highgui
        opencv_features2d
        opencv_calib3d
        CONAN_PKG::boost
        CONAN_PKG::nlohmann_json
        CONAN_PKG::spdlog
        CONAN_PKG::protobuf)

# in newer cmake versions, the conan package for eigen gets a different name
if(${CMAKE_VERSION} VERSION_GREATER "3.12")
    list(APPEND lpslam_libs CONAN_PKG::eigen)
else()
    list(APPEND lpslam_libs CONAN_PKG::Eigen3)
endif()

if(LPSLAM_BUILD_OPENVSLAM)
  list(APPEND lpslam_libs
        openvslam)
  list(APPEND lpslam_src
        Trackers/OpenVSLAMTrackerBase.cpp
        Trackers/OpenVSLAMTracker.cpp
        Trackers/OpenVSLAMStereoTracker.cpp)
  list(APPEND lpslam_definitions_private
      LPSLAM_BUILD_OPENVSLAM)
endif()

if(LPSLAM_BUILD_OPENVSLAM_PANGOLIN)
  list(APPEND lpslam_libs
          pangolin_viewer)
  list(APPEND lpslam_definitions_private
      LPSLAM_BUILD_OPENVSLAM_PANGOLIN)
endif()

if(LPSLAM_BUILD_WEBOTS)
  list(APPEND lpslam_libs
        webots_controller)
  list(APPEND lpslam_src
          Sources/WebotsSource.cpp)
  list(APPEND lpslam_definitions_private
      LPSLAM_BUILD_WEBOTS)
endif()

if (LPSLAM_BUILD_ZEDCAM)
  list(APPEND lpslam_libs
        zed_open_capture)
  list(APPEND lpslam_src
          Sources/ZedOpenCaptureCameraSource.cpp)
  list(APPEND lpslam_definitions_private
      LPSLAM_BUILD_ZEDCAM)
endif()

if (LPSLAM_BUILD_ZEDSDK)
  list(APPEND lpslam_libs
        zed_open_capture)
  list(APPEND lpslam_src
          Sources/ZedSdkSource.cpp)
  list(APPEND lpslam_definitions_private
      LPSLAM_BUILD_ZEDSDK)
endif()

if (BUILD_PROFILER)
   list(APPEND lpslam_libs profiler)
endif()

set (lpslam_libs_executable)

# older GCC versions need to link the filesystem library explicitly
# when using std::filesystem
if ((CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0) OR (CMAKE_CXX_COMPILER_ID STREQUAL "Clang"))
  list(APPEND lpslam_libs_executable
          stdc++fs)
endif()

if (CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
        # ARMv7 does not support some atomics we use, so we need to
        # link libatomic explicitly
        list(APPEND lpslam_libs_executable
                atomic)
endif()

set(lpslam-test_optional_libs
)

if (UNIX)
    list(APPEND lpslam-test_optional_libs
        pthread)

endif()

if (LPSLAM_BUILD_PROFILE)
# this links google perftools
# install with:
# apt-get install google-perftools
# apt-get install libgoogle-perftools-dev
    list(APPEND lpslam_libs
        profiler)
endif()

if (LPSLAM_BUILD_ZEDSDK)
    list(APPEND lpslam_libs
        sl_zed)
endif()

add_library(lpslam SHARED
        ${lpslam_src})

target_link_libraries(lpslam
        ${lpslam_libs}
        )
target_include_directories(lpslam PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR})

target_include_directories(lpslam INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/Interface)

target_compile_definitions(lpslam PUBLIC
        LPSLAM_EXPORTS
        ${lpslam_definitions_private})

# zed sdk
if (LPSLAM_BUILD_ZEDSDK)
target_include_directories(lpslam PUBLIC
        /usr/local/cuda/include/
        /usr/local/zed/include/)
target_link_directories(lpslam PUBLIC
        /usr/local/zed/lib/)
#target_link_libraries(lpslam PUBLIC
#        sl_zed)
endif()

# when linking g2o statically
if (UNIX)
    target_compile_options(lpslam PRIVATE "-fopenmp")

    if (CMAKE_SYSTEM_PROCESSOR MATCHES "armv")
        # aarch64 uses mfpu=neon by default and gcc does not support
        # this option any more
        message("Enabling neon instructions for ARM CPU")
        target_compile_options(lpslam PUBLIC "-mfpu=neon")
    endif()
endif()

set_target_properties(lpslam PROPERTIES CXX_VISIBILITY_PRESET hidden)
set_target_properties(lpslam PROPERTIES VISIBILITY_INLINES_HIDDEN ON)

add_executable(lpslam-standalone
        tools/lpslam-standalone.cpp)

target_link_libraries(lpslam-standalone
        lpslam
        ${lpslam_libs_executable}
        CONAN_PKG::boost
)

target_compile_options(lpslam-standalone PRIVATE
        ${lpslam-compile-options})

## Build Lpslam test
if (LPSLAM_BUILD_TESTS)
    add_executable(lpslam-test
            ${lpslam_src}
            ${lpslam_test_src})

    target_link_libraries(lpslam-test
            CONAN_PKG::gtest
            ${lpslam_libs}
            ${lpslam-test_optional_libs}
            ${lpslam_libs_executable}
            )

    target_include_directories(lpslam-test PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR})

    target_compile_definitions(lpslam-test PRIVATE
        ${lpslam_definitions_private}
        ${lpslam-test_definitions_private})

     target_compile_options(lpslam-test PRIVATE
        ${lpslam-compile-options})

endif()

install(
  TARGETS lpslam
  EXPORT export_lpslam
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
