cmake_minimum_required(VERSION 3.24)
project(JoltLuaSDK LANGUAGES C CXX)

include("${CMAKE_CURRENT_LIST_DIR}/../cmake/VJoltSDK.cmake")
option(VJOLT_BUILD_LUA_MODULES "Build the native modules behind require('jolt')" ON)
option(VJOLT_BUILD_SDK_TESTS "Build the public SDK loader tests" OFF)
set(VJOLT_LUA_RUNTIME "" CACHE PATH "Matching GMod runtime directory containing lua_shared.dll")
set(vjolt_supported_lua_features gravity constraints softbody)
set(VJOLT_LUA_FEATURES "${vjolt_supported_lua_features}"
  CACHE STRING "Native Lua features: gravity;constraints;softbody")
set(VJOLT_ADDON_SOURCE "${CMAKE_CURRENT_LIST_DIR}/../addons/jolt")
set(VJOLT_ADDON_DESTINATION "addons/jolt")

foreach(feature IN LISTS VJOLT_LUA_FEATURES)
  if(NOT feature IN_LIST vjolt_supported_lua_features)
    message(FATAL_ERROR "Unsupported feature '${feature}'. This repository ships: ${vjolt_supported_lua_features}")
  endif()
endforeach()

if(VJOLT_BUILD_LUA_MODULES)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(vjolt_lua_platform_suffix win64)
  elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(vjolt_lua_platform_suffix win32)
  else()
    message(FATAL_ERROR "Unsupported pointer width: ${CMAKE_SIZEOF_VOID_P}")
  endif()
  foreach(feature IN LISTS VJOLT_LUA_FEATURES)
    add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../gmsv_jolt${feature}" "modules/${feature}")
    install(TARGETS gmsv_jolt${feature}_${vjolt_lua_platform_suffix}
      RUNTIME DESTINATION lua/bin COMPONENT Runtime
      LIBRARY DESTINATION lua/bin COMPONENT Runtime)
  endforeach()
endif()

# Both Windows ABIs ship the same five tools, backed by three native modules.
install(DIRECTORY "${VJOLT_ADDON_SOURCE}/" DESTINATION "${VJOLT_ADDON_DESTINATION}"
  COMPONENT Runtime FILES_MATCHING PATTERN "*.lua" PATTERN "addon.json" PATTERN "*.md" PATTERN "*.properties")
install(FILES INSTALL.md DESTINATION "${VJOLT_ADDON_DESTINATION}/docs" RENAME README.md COMPONENT Runtime)
install(FILES GAMEPLAY.md DESTINATION "${VJOLT_ADDON_DESTINATION}/docs" COMPONENT Runtime)
foreach(feature IN ITEMS gravity constraints softbody)
  string(TOUPPER "${feature}" document)
  install(FILES "${CMAKE_CURRENT_LIST_DIR}/../gmsv_jolt${feature}/README.md"
    DESTINATION "${VJOLT_ADDON_DESTINATION}/docs" RENAME "${document}.md" COMPONENT Runtime)
endforeach()
install(FILES "${CMAKE_CURRENT_LIST_DIR}/../gmsv_joltconstraints/PATH.md"
  DESTINATION "${VJOLT_ADDON_DESTINATION}/docs" COMPONENT Runtime)

# VJolt's public physics contracts are separate from the external Source/GMod SDK.
install(DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/../public/vjolt" DESTINATION include
  COMPONENT SDK FILES_MATCHING PATTERN "*.h")
install(TARGETS vjolt_sdk EXPORT VJoltSDKTargets COMPONENT SDK)
install(EXPORT VJoltSDKTargets FILE VJoltSDKConfig.cmake NAMESPACE VJolt::
  DESTINATION lib/cmake/VJoltSDK COMPONENT SDK)
set(vjolt_sdk_docs share/doc/VJoltSDK)
install(FILES "${CMAKE_CURRENT_LIST_DIR}/../examples/physics/SDK.md"
  DESTINATION "${vjolt_sdk_docs}" RENAME README.md COMPONENT SDK)
install(FILES "${CMAKE_CURRENT_LIST_DIR}/../docs/physics-api.md"
  DESTINATION "${vjolt_sdk_docs}/docs" COMPONENT SDK)
install(DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/../examples/physics"
  DESTINATION "${vjolt_sdk_docs}/examples" COMPONENT SDK PATTERN "SDK.md" EXCLUDE)
install(FILES "${CMAKE_CURRENT_LIST_DIR}/../public/vjolt/README.md"
  "${CMAKE_CURRENT_LIST_DIR}/../public/vjolt/stats.h"
  DESTINATION "${vjolt_sdk_docs}/public/vjolt" COMPONENT SDK)
foreach(feature IN ITEMS gravity constraints softbody)
  foreach(document IN ITEMS README.md README.zh-CN.md PATH.md)
    if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/../gmsv_jolt${feature}/${document}")
      install(FILES "${CMAKE_CURRENT_LIST_DIR}/../gmsv_jolt${feature}/${document}"
        DESTINATION "${vjolt_sdk_docs}/gmsv_jolt${feature}" COMPONENT SDK)
    endif()
  endforeach()
endforeach()

enable_testing()
if(VJOLT_BUILD_SDK_TESTS)
  add_subdirectory(tests)
endif()
