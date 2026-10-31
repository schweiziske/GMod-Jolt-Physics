# The supplied SDK header/static tier1 library contain CS:GO ConVar virtuals
# that GMod Win64 does not expose. GMod calls GetFloat/GetInt at slots 12/13.
# Keep the SDK unchanged: generate an overlay header and compile convar.cpp
# against it in each DLL, preventing extraction of the incompatible tier1 OBJ.
function(volt_fix_gmod_convar target)
  if(EXISTS "${VOLT_GMOD_SDK}/public/gmod_sdk_branch.h")
    # gmod-project-sdk selects the public or x86-64 beta ConVar ABI from its
    # profile and builds a matching tier1. This overlay does not apply.
    return()
  endif()
  set(header "${VOLT_GMOD_SDK}/public/tier1/convar.h")
  set(source "${VOLT_GMOD_SDK}/tier1/convar.cpp")
  if(NOT EXISTS "${source}")
    message(FATAL_ERROR "The GMod ConVar ABI fix requires SDK tier1/convar.cpp")
  endif()
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${header}")
  file(READ "${header}" content)
  foreach(method IN ITEMS GetBaseName GetSplitScreenPlayerSlot)
    if(NOT content MATCHES "virtual[^;\n]*${method}")
      message(FATAL_ERROR "Unexpected SDK ConVar declaration for ${method}; recheck the ABI overlay")
    endif()
    # Preserve methods as nonvirtual so the SDK implementation still compiles,
    # while removing these extra entries from the public ConVar vtable.
    string(REGEX REPLACE "virtual([ \t]+[^;\n]*${method}[^;\n]*;)" "\\1" content "${content}")
  endforeach()
  set(overlay "${CMAKE_CURRENT_BINARY_DIR}/gmod_convar_abi")
  file(MAKE_DIRECTORY "${overlay}/tier1")
  file(CONFIGURE OUTPUT "${overlay}/tier1/convar.h" CONTENT "${content}" @ONLY)
  target_include_directories(${target} BEFORE PRIVATE "${overlay}")
  target_sources(${target} PRIVATE "${source}")
endfunction()
