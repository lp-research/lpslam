from conans import ConanFile, CMake, tools
import os

class G2oConan(ConanFile):
    name = "g2o"
    version = "691dc51ac7c7"
    license = "LGPL3"
    author = "<Put your name here> <And your email here>"
    url = "<Package recipe repository url here, for issues about the package>"
    description = "<Description of G2o here>"
    topics = ("<Put some tag here>", "<here>", "<and here>")
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False], "avx": [True, False]}
    default_options = {"shared": True, "avx": True}
    generators = ["cmake", "cmake_find_package"]
    exports_sources = ["CMakeLists.txt"]
    release_tag = "master"
    _source_subfolder = "source_subfolder"

    def requirements(self):
        self.requires.add('eigen/3.3.9')

    def source(self):
        self.run("git clone --branch {} https://github.com/RainerKuemmerle/g2o.git {}".format(self.release_tag, self._source_subfolder))
        # don't use a newer version yet, because newer relesae don't come with their own
        # csparse
        self.run("cd " + self._source_subfolder + " && git checkout 691dc51ac7c7")
        self._patch()

    def _patch(self):
        # eigen includes will be available, trust me :)
        # g2o's eigen submodule checks for EIGEN3_VERSION, which won't get
        # set by the default find_package of conan eigen due to capilatization
        # note: post 2020.04 releases, the replace needs to be changed to
        # 'find_package(Eigen3 3.3 REQUIRED)'
        tools.replace_in_file(os.path.join(self.source_folder, self._source_subfolder, "CMakeLists.txt"),
            'find_package(Eigen3 3.3 REQUIRED)',
            '''find_package(Eigen3 REQUIRED)
                set(EIGEN3_VERSION ${Eigen3_VERSION})
            ''')
        pass

    def _configure_cmake(self):
        cmake = CMake(self)
        cmake.msbuild_verbosity = "normal"
        if self.settings.os == "Linux":
            cmake.definitions["CMAKE_CXX_FLAGS"] = "-std=c++11 -fPIC"
        if self.settings.os == "Windows":
            # the file os_specifc.h will make trouble if _WINDOWS is not defined
            # and try to include some Linux headers

            # csparse_EXPORTS to export csparse symbols, build with AVX support
            # fp:fast flag is not used by sub-packages if not added here
            cxx_flags = "/D_WINDOWS /Dcsparse_EXPORTS /fp:fast"
            if self.options.avx:
                cxx_flags = cxx_flags + " /arch:AVX"
            cmake.definitions["CMAKE_CXX_FLAGS"] = cxx_flags

            # csparse uses the C compiler
            cmake.definitions["CMAKE_C_FLAGS"] = cxx_flags
        
        # conan cmake does not set this variable, even if we
        # set the correct arch in the conan profile
        # this variable is used by g2o to determine whether its
        # using sse
        if self.settings.arch == "armv7":
            cmake.definitions["DO_SSE_AUTODETECT"] = False
            cmake.definitions["DISABLE_SSE2"] = True
            cmake.definitions["DISABLE_SSE3"] = True
            cmake.definitions["DISABLE_SSE4_1"] = True
            cmake.definitions["DISABLE_SSE4_2"] = True
            cmake.definitions["DISABLE_SSE4_A"] = True

        cmake.definitions["BUILD_SHARED_LIBS"] = False
        cmake.definitions["BUILD_UNITTESTS"] = False
        cmake.definitions["G2O_BUILD_EXAMPLES"] = False

        # can lead to crashes in g2o JacobianWorkbench
        # if set to true
        # https://github.com/raulmur/ORB_SLAM2/issues/341
        # and hosting app does not honor this too
        cmake.definitions["BUILD_WITH_MARCH_NATIVE"] = False
        cmake.definitions["G2O_USE_CHOLMOD"] = False
        # if not installed on the system, it will be buid from
        # the g2o source externals folder
        cmake.definitions["G2O_USE_CSPARSE"] = True
        # to export the symbols of csparse
        cmake.definitions["G2O_LGPL_SHARED_LIBS"] = True

        cmake.definitions["G2O_USE_OPENGL"] = False
        # disabled threading because its experimental
        # and spams processing with many threads on Windows
        cmake.definitions["G2O_USE_OPENMP"] = False
        cmake.definitions["G2O_FAST_MATH"] = True

        # optimization flag /Ox is used by default in the compile

        cmake.configure()
        return cmake

    def build(self):
        cmake = self._configure_cmake()        
        cmake.build()

    def package(self):
        # like here 
        # https://github.com/conan-io/conan-center-index/blob/50a5d7a22bc039963388ed749831eadb89f09d82/recipes/tgbot/all/conanfile.py
        cmake = self._configure_cmake()
        cmake.install()
        # g2o has its licence information only in the readme file
        self.copy("README.md", src=self._source_subfolder, dst="licenses")
        # copy cs.h and cs_api.h so it can be found duringe the host application build
        self.copy("*.h", src=os.path.join(self._source_subfolder, "EXTERNAL", "csparse"),
            dst="include")

    def package_info(self):
        g2o_libs = ["core",
            "stuff",
            "types_sba",
            "types_sim3",
            "solver_dense",
            "solver_eigen",
            "solver_csparse",
            "csparse_extension"]

        for lib_name in g2o_libs:
            self.cpp_info.components[lib_name].target = "g2o::" + lib_name
            self.cpp_info.components[lib_name].lib = lib_name
            # this is the actual so/a to link
            self.cpp_info.components[lib_name].libs = ["g2o_" + lib_name]
            self.cpp_info.components[lib_name].requires = ["eigen::eigen"]

