cmake_minimum_required(VERSION 3.24)
project(VoltV1GMod LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Regenerate the test manifest even when every optional probe is disabled.
enable_testing()
# Match the GMod SDK static libraries and Source's per-module memoverride.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
if(NOT MSVC OR NOT (CMAKE_SIZEOF_VOID_P EQUAL 4 OR CMAKE_SIZEOF_VOID_P EQUAL 8))
  message(FATAL_ERROR "This build requires MSVC Win32 or x64")
endif()

set(VOLT_ENGINE_BRANCH "gmod" CACHE STRING "SDK ABI: gmod or sdk2013")
set_property(CACHE VOLT_ENGINE_BRANCH PROPERTY STRINGS gmod sdk2013)
if(NOT VOLT_ENGINE_BRANCH MATCHES "^(gmod|sdk2013)$")
  message(FATAL_ERROR "Unsupported SDK ABI: ${VOLT_ENGINE_BRANCH}")
endif()
set(VOLT_GMOD_SDK "" CACHE PATH "Source/GMod SDK src directory")
if(NOT EXISTS "${VOLT_GMOD_SDK}/public/vphysics_interface.h")
  message(FATAL_ERROR "Set VOLT_GMOD_SDK to the complete Source/GMod SDK src directory")
endif()
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(volt_sdk_libdir "${VOLT_GMOD_SDK}/lib/public/x64")
else()
  set(volt_sdk_libdir "${VOLT_GMOD_SDK}/lib/public")
endif()
# SDK 2013 predates C++20's rewritten equality candidates. Keep the SDK intact
# and disambiguate its symbol comparison in a build-local header overlay.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${VOLT_GMOD_SDK}/public/tier1/utlsymbol.h")
file(READ "${VOLT_GMOD_SDK}/public/tier1/utlsymbol.h" volt_utlsymbol)
string(REPLACE "return Find( pStr ) != UTL_INVAL_SYMBOL;"
  "return static_cast<UtlSymId_t>( Find( pStr ) ) != UTL_INVAL_SYMBOL;"
  volt_utlsymbol "${volt_utlsymbol}")
file(CONFIGURE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/sdk_compat/tier1/utlsymbol.h"
  CONTENT "${volt_utlsymbol}" @ONLY)
get_filename_component(VOLT_ROOT ".." ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
# Forced into every SDK-facing translation unit, including optional probes.
set(VOLT_SDK_PREFIX "${VOLT_ROOT}/vphysics_jolt/compat/sdk_prefix.h")
include("${VOLT_ROOT}/cmake/gmod_convar.cmake")
include("${VOLT_ROOT}/cmake/VJoltSDK.cmake")
include("${VOLT_ROOT}/cmake/GModSDK.cmake")
# gmod-project-sdk ships no x86-64 beta tier1, mathlib or tier2. Its CMake
# project builds them, with the profile's definitions, from gmod/src.
if(VOLT_ENGINE_BRANCH STREQUAL "gmod" AND CMAKE_SIZEOF_VOID_P EQUAL 8
    AND EXISTS "${VOLT_GMOD_SDK}/public/gmod_sdk_branch.h")
  set(SOURCE_SDK_PROFILE gmod-beta CACHE STRING "GMod interface family: gmod-beta (Steam x86-64 branch) or gmod-public")
endif()
if(SOURCE_SDK_PROFILE STREQUAL "gmod-beta")
  get_filename_component(volt_project_sdk "${VOLT_GMOD_SDK}/../.." ABSOLUTE)
  if(NOT EXISTS "${volt_project_sdk}/cmake/SDKLibraries.cmake")
    message(FATAL_ERROR "gmod-beta needs VOLT_GMOD_SDK to be gmod/src of a complete gmod-project-sdk checkout")
  endif()
  add_subdirectory("${volt_project_sdk}" source_sdk EXCLUDE_FROM_ALL)
  set(VOLT_GMOD_SDK_LIB_DIR "${VOLT_GMOD_SDK}/lib/beta/x64")
  set(volt_sdk_module_libraries SourceSDK::tier1)
  set(volt_sdk_backend_libraries SourceSDK::mathlib SourceSDK::tier2_interfaces)
else()
  vjolt_gmod_sdk_library_dir(VOLT_GMOD_SDK_LIB_DIR "${VOLT_GMOD_SDK}")
  set(volt_sdk_module_libraries "${VOLT_GMOD_SDK_LIB_DIR}/tier0.lib"
    "${VOLT_GMOD_SDK_LIB_DIR}/tier1.lib" "${VOLT_GMOD_SDK_LIB_DIR}/vstdlib.lib")
  set(volt_sdk_backend_libraries "${VOLT_GMOD_SDK_LIB_DIR}/mathlib.lib" "${VOLT_GMOD_SDK_LIB_DIR}/tier2.lib")
endif()
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(VOLT_GMOD_PLATFORM_DEFINITIONS WIN64 _WIN64 COMPILER_MSVC64 PLATFORM_64BITS=1)
else()
  set(VOLT_GMOD_PLATFORM_DEFINITIONS COMPILER_MSVC32 PLATFORM_64BITS=0)
endif()
set(VOLT_CPU_VARIANT "avx2" CACHE STRING "Backend instruction set: avx2, sse42 or sse2")
set_property(CACHE VOLT_CPU_VARIANT PROPERTY STRINGS avx2 sse42 sse2)
if(NOT VOLT_CPU_VARIANT MATCHES "^(avx2|sse42|sse2)$")
  message(FATAL_ERROR "Unsupported VOLT_CPU_VARIANT: ${VOLT_CPU_VARIANT}")
endif()
# Each variant has a separate build tree: Jolt and its adapter must use the
# same feature definitions, configured below for each CPU instruction set.
foreach(feature IN ITEMS SSE4_1 SSE4_2 AVX AVX2 AVX512 LZCNT TZCNT F16C FMADD)
  set(USE_${feature} OFF CACHE BOOL "" FORCE)
endforeach()
if(VOLT_CPU_VARIANT STREQUAL "avx2")
  foreach(feature IN ITEMS SSE4_1 SSE4_2 AVX AVX2 LZCNT TZCNT F16C FMADD)
    set(USE_${feature} ON CACHE BOOL "" FORCE)
  endforeach()
elseif(VOLT_CPU_VARIANT STREQUAL "sse42")
  set(USE_SSE4_1 ON CACHE BOOL "" FORCE)
  set(USE_SSE4_2 ON CACHE BOOL "" FORCE)
endif()
set(USE_STATIC_MSVC_RUNTIME_LIBRARY ON CACHE BOOL "" FORCE)
set(INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "" FORCE)
set(ENABLE_ALL_WARNINGS OFF CACHE BOOL "" FORCE)
set(PROFILER_IN_DEBUG_AND_RELEASE OFF CACHE BOOL "" FORCE)
set(DEBUG_RENDERER_IN_DEBUG_AND_RELEASE OFF CACHE BOOL "" FORCE)
set(GENERATE_DEBUG_SYMBOLS ON CACHE BOOL "" FORCE)
option(VOLT_JOLT_CHECKED_BUILD "Enable fatal Jolt assertions for ABI regression runs" OFF)
# Keep the two cache flags coupled in both directions. Reconfiguring a checked
# build for play must not silently retain its expensive body-access assertions.
set(USE_ASSERTS ${VOLT_JOLT_CHECKED_BUILD} CACHE BOOL "" FORCE)
if(VOLT_JOLT_CHECKED_BUILD)
  # The shipped SDK libraries have a Release CRT/allocator ABI. Keep that ABI
  # while checking Jolt's body access, graph, contact-cache and lifetime rules.
  set(USE_ASSERTS ON CACHE BOOL "" FORCE)
endif()
set(JPH_USE_DX12 OFF CACHE BOOL "" FORCE)
set(JPH_USE_VK OFF CACHE BOOL "" FORCE)
set(ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
if(VOLT_ENGINE_BRANCH STREQUAL "sdk2013")
  set(DISABLE_CUSTOM_ALLOCATOR ON CACHE BOOL "" FORCE)
else()
  set(DISABLE_CUSTOM_ALLOCATOR OFF CACHE BOOL "" FORCE)
endif()
add_subdirectory("${VOLT_ROOT}/joltphysics/src/Build" jolt)
set_property(TARGET Jolt PROPERTY CXX_STANDARD 20)

# Backend sources and Source SDK dependencies.
set(volt_sources
  cbase.cpp vjolt_collide.cpp vjolt_collide_trace.cpp vjolt_constraints.cpp
  vjolt_controller_drag.cpp vjolt_controller_fluid.cpp vjolt_controller_motion.cpp
  vjolt_controller_player.cpp vjolt_controller_vehicle.cpp vjolt_controller_shadow.cpp
  vjolt_environment.cpp
  vjolt_environment_lifecycle.cpp vjolt_friction.cpp
  vjolt_havana_constraints.cpp vjolt_interface.cpp vjolt_keyvalues_schema.cpp
  bridge/vjolt_gravity_bridge.cpp bridge/vjolt_constraints_bridge.cpp
  bridge/vjolt_path_bridge.cpp bridge/vjolt_stats_bridge.cpp
  bridge/vjolt_softbody_bridge.cpp bridge/vjolt_softbody_mesh.cpp bridge/vjolt_softbody_query.cpp
  vjolt_object.cpp vjolt_objectpairhash.cpp
  vjolt_parse.cpp vjolt_querymodel.cpp vjolt_surfaceprops.cpp vjolt_listener_contact.cpp)
list(TRANSFORM volt_sources PREPEND "${VOLT_ROOT}/vphysics_jolt/")
set(volt_backend vphysics_jolt_${VOLT_CPU_VARIANT})
add_library(${volt_backend} SHARED ${volt_sources}
  "${VOLT_GMOD_SDK}/public/collisionutils.cpp"
  "${VOLT_GMOD_SDK}/public/filesystem_helpers.cpp"
  "${VOLT_GMOD_SDK}/public/tier0/memoverride.cpp")
function(volt_configure_module target)
  vjolt_configure_source_sdk(${target} "${VOLT_GMOD_SDK}")
  target_include_directories(${target} PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/gmod_convar_abi"
    "${CMAKE_CURRENT_BINARY_DIR}/sdk_compat"
    "${VOLT_ROOT}/vphysics_jolt" "${VOLT_ROOT}/vphysics_jolt/compat"
    "${VOLT_GMOD_SDK}/public" "${VOLT_GMOD_SDK}/public/tier0"
    "${VOLT_GMOD_SDK}/public/tier1")
  target_compile_definitions(${target} PRIVATE
    NOMINMAX WIN32 _WINDOWS COMPILER_MSVC ${VOLT_GMOD_PLATFORM_DEFINITIONS}
    GAME_SDK2013=1 _USRDLL DLLNAME=${target}
    _DLL_EXT=.dll
    _CRT_SECURE_NO_WARNINGS _CRT_NONSTDC_NO_DEPRECATE)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    target_compile_definitions(${target} PRIVATE WIN64 _WIN64 COMPILER_MSVC64 PLATFORM_64BITS=1)
  else()
    target_compile_definitions(${target} PRIVATE COMPILER_MSVC32)
  endif()
  if(VOLT_ENGINE_BRANCH STREQUAL "gmod")
    target_compile_definitions(${target} PRIVATE GAME_GMOD=1)
  endif()
  target_compile_options(${target} PRIVATE /MP /Zi)
  target_link_options(${target} PRIVATE /DEBUG /OPT:REF /OPT:ICF)
  target_link_libraries(${target} PRIVATE VJolt::SDK shell32 user32 advapi32 gdi32 comdlg32 ole32
    ${volt_sdk_module_libraries})
  set_target_properties(${target} PROPERTIES PREFIX "")
endfunction()
volt_configure_module(${volt_backend})
# Windows.h (from vjolt/cpu_policy.h) must not rename the controllers' GetObject.
target_compile_definitions(${volt_backend} PRIVATE NOGDI)
if(VOLT_ENGINE_BRANCH STREQUAL "gmod" AND CMAKE_SIZEOF_VOID_P EQUAL 8)
  volt_fix_gmod_convar(${volt_backend})
endif()
if(VOLT_JOLT_CHECKED_BUILD)
  target_compile_definitions(${volt_backend} PRIVATE VJOLT_FATAL_JOLT_ASSERTS)
endif()
# SSE2 is the MSVC x64 baseline. Jolt supplies its matching feature macros.
if(VOLT_CPU_VARIANT STREQUAL "avx2")
  target_compile_options(${volt_backend} PRIVATE /arch:AVX2)
endif()
set_source_files_properties(${volt_sources}
  "${VOLT_GMOD_SDK}/public/collisionutils.cpp"
  "${VOLT_GMOD_SDK}/public/filesystem_helpers.cpp"
  PROPERTIES COMPILE_OPTIONS "/FI${VOLT_SDK_PREFIX}")
# memoverride must establish its own CRT include order before any SDK headers.
set_source_files_properties("${VOLT_GMOD_SDK}/public/tier0/memoverride.cpp"
  PROPERTIES COMPILE_OPTIONS "/FI${VOLT_ROOT}/vphysics_jolt/compat/branch_overrides.h")
target_link_libraries(${volt_backend} PRIVATE Jolt ${volt_sdk_backend_libraries})

# The external wrapper is CPU-agnostic and must never link Jolt or inherit
# the selected backend's instruction-set requirements.
add_library(vphysics SHARED "${VOLT_ROOT}/vphysics_wrapper/vphysics_wrapper.cpp"
  "${VOLT_GMOD_SDK}/public/tier0/memoverride.cpp")
volt_configure_module(vphysics)
set_source_files_properties("${VOLT_ROOT}/vphysics_wrapper/vphysics_wrapper.cpp"
  PROPERTIES COMPILE_OPTIONS "/FI${VOLT_SDK_PREFIX}")

# Optional regression executables and CTest registration share the SDK setup above.
# They live outside the public tree and are skipped when absent.
if(EXISTS "${VOLT_ROOT}/tools/build_gmod/probes.cmake")
  include("${VOLT_ROOT}/tools/build_gmod/probes.cmake")
endif()
