include_guard(GLOBAL)

# The public SDK keeps Win32 libraries in lib/public and Win64 libraries in
# lib/public/x64. Both use the same public header family.
function(vjolt_gmod_sdk_library_dir output sdk_root)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(library_dir "${sdk_root}/lib/public/x64")
  elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(library_dir "${sdk_root}/lib/public")
  else()
    message(FATAL_ERROR "Unsupported pointer width: ${CMAKE_SIZEOF_VOID_P}")
  endif()
  set(${output} "${library_dir}" PARENT_SCOPE)
endfunction()

function(vjolt_configure_source_sdk target sdk_root)
  target_include_directories(${target} PRIVATE "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../public")
  # A configured SDK profile (SourceSDK targets) supplies its own definitions.
  if(EXISTS "${sdk_root}/public/gmod_sdk_branch.h" AND NOT TARGET SourceSDK::headers)
    target_compile_definitions(${target} PRIVATE GMOD_SDK_PUBLIC=1)
    vjolt_gmod_sdk_library_dir(sdk_library_dir "${sdk_root}")
    if(NOT EXISTS "${sdk_library_dir}/tier1.lib")
      message(FATAL_ERROR "Build the public SDK tier1 library before configuring this project")
    endif()
  endif()
endfunction()
