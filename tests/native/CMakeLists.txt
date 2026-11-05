cmake_minimum_required(VERSION 3.24)
project(SoftBodyFragmentTests LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_MSVC_RUNTIME_LIBRARY MultiThreaded)
get_filename_component(root "../.." ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(VOLT_GMOD_SDK "" CACHE PATH "gmod-project-sdk/gmod/src")
set(VOLT_BACKEND_BUILD "" CACHE PATH "Configured gmod-beta x64 backend build directory")
add_executable(softbody_fragments softbody_fragments.cpp)
target_include_directories(softbody_fragments PRIVATE "${root}/public"
  "${VOLT_BACKEND_BUILD}/sdk_compat" "${root}/vphysics_jolt/compat"
  "${VOLT_GMOD_SDK}/public" "${VOLT_GMOD_SDK}/public/tier0" "${VOLT_GMOD_SDK}/public/tier1")
target_compile_definitions(softbody_fragments PRIVATE NOMINMAX WIN32 _WINDOWS COMPILER_MSVC WIN64 _WIN64
  COMPILER_MSVC64 PLATFORM_64BITS=1 GAME_SDK2013=1 GAME_GMOD=1 GMOD_SDK_PUBLIC=0 GMOD_SDK_X86_64_ABI=1 RAD_TELEMETRY_DISABLED=1)
target_compile_options(softbody_fragments PRIVATE "/FI${root}/vphysics_jolt/compat/sdk_prefix.h")
target_link_libraries(softbody_fragments PRIVATE
  "${VOLT_BACKEND_BUILD}/source_sdk/lib/gmod-beta/windows-x64/Release/tier1.lib"
  "${VOLT_BACKEND_BUILD}/source_sdk/lib/gmod-beta/windows-x64/Release/mathlib.lib"
  "${VOLT_GMOD_SDK}/lib/beta/x64/tier0.lib" "${VOLT_GMOD_SDK}/lib/beta/x64/vstdlib.lib")
