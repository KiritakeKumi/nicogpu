# rtl/gpu/salu/vec/CMakeLists.txt -- builds the randomised-operand vector exporter
# for the SALU/branch/misc unit (dump_salu_rand.cpp).  Like sim/valu it reuses the
# model's own CMake via add_subdirectory (ISA table generated from)
# includes read-only; no model file is modified.
#   cmake -S rtl/gpu/salu/vec -B build/salu_rand -G Ninja -DCMAKE_BUILD_TYPE=Release && cmake --build build/salu_rand
cmake_minimum_required(VERSION 3.20)
project(nicogpu_salu_rand LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(NICOGPU_MODEL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../model" CACHE PATH "nicogpu model directory")
set(BUILD_TESTING OFF)
add_subdirectory("${NICOGPU_MODEL_DIR}" "${CMAKE_BINARY_DIR}/model")

if(MSVC)
  set(RAND_WARNINGS /W4 /permissive- /utf-8 /wd4324 /wd4127)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
else()
  set(RAND_WARNINGS -Wall -Wextra -Wno-unused-parameter)
endif()

add_executable(dump_salu_rand dump_salu_rand.cpp)
target_link_libraries(dump_salu_rand PRIVATE nicogpu_model nicogpu_isa nicogpu_softfloat)
target_include_directories(dump_salu_rand PRIVATE "${NICOGPU_MODEL_DIR}/tests")
target_compile_options(dump_salu_rand PRIVATE ${RAND_WARNINGS})
