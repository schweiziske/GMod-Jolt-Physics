#pragma once
// Establish GMod ABI gates before SDK headers and keep SDK min/max macros out
// of the C++ standard library and Jolt headers.
#include "branch_overrides.h"
#include "tier0/platform.h"
#include "tier0/threadtools.h"
#include "tier0/valve_minmax_off.h"
// Load the build-local C++20 compatibility header before SDK headers can
// include a sibling utlsymbol.h directly from their own directory.
#include <tier1/utlsymbol.h>
#include "vjolt/source_abi.h"
