from conans import ConanFile, CMake, tools

import os

# needs OpenGL lib to install with
# apt install -y libglew-dev

class PangolinConan(ConanFile):
    name = "pangolin"
    version = "0.6.0"
    license = "<Put the package license here>"
    author = "<Put your name here> <And your email here>"
    url = "<Package recipe repository url here, for issues about the package>"
    description = "<Description of Pangolin here>"
    topics = ("<Put some tag here>", "<here>", "<and here>")
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False]}
    default_options = {"shared": False}
    generators = ["cmake", "cmake_find_package"]
    exports_sources = ["CMakeLists.txt"]
    _source_subfolder = "source_subfolder"

    def requirements(self):
        self.requires.add('eigen/3.3.9')

    def source(self):
        self.run("git clone https://github.com/stevenlovegrove/Pangolin.git " + self._source_subfolder)
        self.run("cd " + self._source_subfolder + " && git checkout v0.6")
        self._patch()

    def _patch(self):
        # Visual Studio 2019 does not like some templated contexpr
        tools.replace_in_file(os.path.join(self.source_folder, self._source_subfolder, "include", "mpark", "variant.hpp"),
            '#define MPARK_CPP14_CONSTEXPR',
            '//#define MPARK_CPP14_CONSTEXPR')

    def _configure_cmake(self):
        cmake = CMake(self)
        if self.settings.os == "Linux":
            cmake.definitions["CMAKE_CXX_FLAGS"] = "-fPIC"
        if self.settings.os == "Windows":
            cmake.definitions["MSVC_USE_STATIC_CRT"] = False

        cmake.definitions["BUILD_SHARED_LIBS"] = self.options.shared
        cmake.definitions["BUILD_TESTS"] = False
        cmake.definitions["BUILD_TOOLS"] = False
        cmake.definitions["BUILD_EXAMPLES"] = False
        cmake.definitions["BUILD_PANGOLIN_PYTHON"] = False

        cmake.configure()
        return cmake

    def build(self):
        cmake = self._configure_cmake()
        #cmake.configure(source_folder="Pangolin")
        cmake.build()

    def package(self):
        cmake = self._configure_cmake()
        cmake.install()
        self.copy("LICENCE", src=self._source_subfolder, dst="licenses")

    def package_info(self):
        self.cpp_info.libs = ["pangolin"]
