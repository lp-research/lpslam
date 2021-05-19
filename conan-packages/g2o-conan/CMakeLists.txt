cmake_minimum_required(VERSION 2.8)
project(conanWrapper)

# this will make sure all required libraries setup by conan will be found
include(conanbuildinfo.cmake)
conan_basic_setup()

add_subdirectory("source_subfolder")
