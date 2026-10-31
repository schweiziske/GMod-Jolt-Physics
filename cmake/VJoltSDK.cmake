include_guard(GLOBAL)

# Public contracts only: no backend implementation or transitive Jolt linkage.
add_library(vjolt_sdk INTERFACE)
add_library(VJolt::SDK ALIAS vjolt_sdk)
set_target_properties(vjolt_sdk PROPERTIES EXPORT_NAME SDK)
target_include_directories(vjolt_sdk INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../public>"
  "$<INSTALL_INTERFACE:include>")
target_compile_features(vjolt_sdk INTERFACE cxx_std_17)
