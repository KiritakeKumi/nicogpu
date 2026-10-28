# rtl/gpu/valu/vec/CMakeLists.txt -- model-side random-operand exporter for the f32 VALU RTL:
#   dump_valu_fp   randomised-operand arch_insn vectors (same model path as the stock exporter)
# Like tools/vecexport/dump it reuses the model's CMake via add_subdirectory and includes model
# sources/tests read-only; no model file is modified.
#   cmake -S rtl/gpu/valu/vec -B build/valu_fp_dump -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release
#   cmake --build build/valu_fp_dump
cmake_minimum_required(VERSION 3.20)
project(nicogpu_valu_fp_dump LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(NICOGPU_MODEL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../model" CACHE PATH "nicogpu model directory")
set(BUILD_TESTING OFF)
add_subdirectory("${NICOGPU_MODEL_DIR}" "${CMAKE_BINARY_DIR}/model")

if(MSVC)
  set(FP_WARNINGS /W4 /permissive- /utf-8 /wd4324 /wd4127 /wd4100)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
else()
  set(FP_WARNINGS -Wall -Wextra -Wno-unused-parameter)
endif()

add_executable(dump_valu_fp dump_valu_fp.cpp)
target_link_libraries(dump_valu_fp PRIVATE nicogpu_model nicogpu_isa nicogpu_softfloat)
target_include_directories(dump_valu_fp PRIVATE "${NICOGPU_MODEL_DIR}/tests")
target_compile_options(dump_valu_fp PRIVATE ${FP_WARNINGS})
