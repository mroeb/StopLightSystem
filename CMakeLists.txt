cmake_minimum_required(VERSION 4.0)
project(StopLightSystem)

set(CMAKE_CXX_STANDARD 23)

add_executable(StopLightSystem main.cpp
        src/core/StopLight.cpp
        src/core/StopLight.hpp
        src/core/CrossOver.cpp
        src/core/CrossOver.hpp
        src/core/Pilo.hpp)
