cmake_minimum_required(VERSION 3.28)
project(genetic_optimisation)

set(CMAKE_CXX_STANDARD 20)

include_directories(
        /opt/homebrew/opt/python@3.11/Frameworks/Python.framework/Versions/3.11/include/python3.11
        /opt/homebrew/lib/python3.11/site-packages/pybind11/include
        /System/Library/Frameworks/CoreFoundation.framework
        /unsupported
        /usr/local/include)
link_directories(
        /opt/homebrew/opt/python@3.11/Frameworks/Python.framework/Versions/3.11/lib/python3.11/config-3.11-darwin
        /usr/local/lib)

add_compile_options(
        -fcolor-diagnostics
        -fansi-escape-codes
        -g
        -Wsign-compare
        -Wunreachable-code
        -DNDEBUG
        -O3
        -Wall
)

add_executable(genetic_optimisation
        main.cpp
        include/genetic_optimisation.h
        include/operationalPoint.h
        include/propulsion_system.h
        src/opt_propulsion_system.cpp
        include/nlopt/nlopt.h
)

target_link_libraries(genetic_optimisation
        dl
        python3.11
        nlopt
)

add_subdirectory(test)