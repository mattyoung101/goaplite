cmake_minimum_required(VERSION 3.15)
project(goap C)

set(CMAKE_C_STANDARD 11)

# enable ASan and stack protection
set (SAFETY_FLAGS "-fstack-protector-strong -fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer -static-libsan")
# these flags tune Omicam builds specifically for the platform it was compiled on, and enable SSE3 instructions
# note: using -flto to enable link time optimisations breaks using "-Og" and "-Ofast", but should be used in future
set (PERFORMANCE_FLAGS "-march=native -mtune=native -msse -msse3")
# (we generate debug info in all modes since it adds no performance penalty and is good for debug)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wno-unused-parameter -g3 -pthread")
set(CMAKE_CXX_FLAGS  "${CMAKE_CsXX_FLAGS} -Wall -Wextra -Wno-unused-parameter -g3 -pthread")

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS} -O0 ${SAFETY_FLAGS}") # safety features and debug optimisation
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 ${SAFETY_FLAGS}")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS} -O3 ${PERFORMANCE_FLAGS}") # full optimisation and no safety features
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_C_FLAGS} -O3 ${PERFORMANCE_FLAGS}")

add_executable(goap main.c goap.c goap.h map.c cJSON.c log/log.c avl.c)