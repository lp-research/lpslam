
# to build
#conan create --build=missing dbow2-conan/conanfile.py
#conan create --build=missing g2o-conan/conanfile.py
#conan create --build=missing pangolin-conan/conanfile.py

# just put in local cache, don't build
conan export dbow2-conan/conanfile.py
conan export g2o-conan/conanfile.py
conan export pangolin-conan/conanfile.py
