from conans import ConanFile, CMake, tools
import os

class Dbow2Conan(ConanFile):
    name = "dbow2"
    # dbow2 repository does not provide releases, so pick 1.0 for this commit
    version = "1.0.0"
    license = "<Put the package license here>"
    author = "<Put your name here> <And your email here>"
    url = "<Package recipe repository url here, for issues about the package>"
    description = "<Description of Dbow2 here>"
    topics = ("<Put some tag here>", "<here>", "<and here>")
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False], "avx": [True, False]}
    default_options = {"shared": True, "avx": True}
    generators = ["cmake", "cmake_find_package"]
    exports_sources = ["CMakeLists.txt"]
    _source_subfolder = "source_subfolder"

    def requirements(self):
        self.requires("eigen/3.3.9")
        ## otherwise there will be trouble linking all the sub-libraries of OpenCV
        # only need to enable this when you want to test-compile the package
        #self.options["opencv"].shared = True
        ## makes trouble during linking, we don't need it anyways
        #self.options["opencv"].tiff = False
        #self.options["opencv"].openexr = False

    def source(self):
        self.run("git clone https://github.com/shinsumicco/DBoW2.git " + self._source_subfolder)
        self.run("cd " + self._source_subfolder + " && git checkout e8cc74d24705d0a")
        self._patch()

    def _configure_cmake(self):
        cmake = CMake(self)

        cmake.msbuild_verbosity = "normal"
        cmake.definitions["BUILD_UTILS"] = False
        cmake.definitions["BUILD_SHARED_LIBS"] = self.options.shared

        if self.settings.os == "Windows":
            # csparse_EXPORTS to export csparse symbols, build with AVX support
            # fp:fast flag is not used by sub-packages if not added here
            cxx_flags = "/fp:fast"
            if self.options.avx:
                cxx_flags = cxx_flags + " /arch:AVX"
            cmake.definitions["CMAKE_CXX_FLAGS"] = cxx_flags
            cmake.definitions["CMAKE_C_FLAGS"] = cxx_flags

        cmake.configure()
        return cmake

    def build(self):
        cmake = self._configure_cmake()
        cmake.build()

    def _patch(self):
        ## ensure we use position independant code so we can statically link into shared library
        tools.replace_in_file(os.path.join(self.source_folder, self._source_subfolder, "CMakeLists.txt"),
            '-Wall -Wextra -pedantic',
            '''-Wall -Wextra -pedantic -fPIC''')

    def package(self):
        cmake = self._configure_cmake()
        cmake.install()
        self.copy("LICENSE.txt", src=self._source_subfolder, dst="licenses")

    def package_info(self):
        self.cpp_info.libs = ["dbow2"]

