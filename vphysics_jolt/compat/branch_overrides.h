
#pragma once

// The selected SDK records the interface family separately from pointer width.
#if __has_include("gmod_sdk_branch.h")
#include "gmod_sdk_branch.h"
#endif

// public Win64 retains the legacy interfaces; only x86-64 uses the newer ABI.
#if defined( GAME_GMOD )
#if PLATFORM_64BITS && ( !defined( GMOD_SDK_PUBLIC ) || !GMOD_SDK_PUBLIC )
#define GAME_GMOD_64X
#endif
#define override_gmod override
#define override_not_gmod
#else
#define override_gmod
#define override_not_gmod override
#endif

#if defined( GAME_CSGO ) || defined( GAME_VITAMIN ) || defined( GAME_GMOD_64X )
#define GAME_CSGO_OR_NEWER
#define override_csgo override
#define override_not_csgo
#else
#define override_csgo
#define override_not_csgo override
#endif

#if defined( GAME_CSGO ) || defined( GAME_VITAMIN ) || defined( GAME_PORTAL2 ) || defined( GAME_GMOD_64X )
#define GAME_PORTAL2_OR_NEWER
#define override_portal2 override
#define override_not_portal2
#else
#define override_portal2
#define override_not_portal2 override
#endif

#if defined( GAME_CSGO ) || defined( GAME_VITAMIN ) || defined( GAME_PORTAL2 ) || defined( GAME_L4D2 ) || defined( GAME_GMOD_64X )
#define GAME_L4D2_OR_NEWER
#define override_l4d2 override
#define override_not_l4d2
#else
#define override_l4d2
#define override_not_l4d2 override
#endif

#if defined( GAME_CSGO ) || defined( GAME_VITAMIN ) || defined( GAME_PORTAL2 ) || defined( GAME_L4D2 ) || defined( GAME_ASW ) || defined( GAME_GMOD_64X )
#define GAME_ASW_OR_NEWER
#define override_asw override
#define override_not_asw
#else
#define override_asw
#define override_not_asw override
#endif

#if !defined(GAME_CSGO_OR_NEWER) || defined( GAME_GMOD_64X )
#define FastASCIIToUpper( c ) ( ( ( (c) >= 'a' ) && ( (c) <= 'z' ) ) ? ( (c) - 32 ) : (c) )
#define FastASCIIToLower( c ) ( ( ( (c) >= 'A' ) && ( (c) <= 'Z' ) ) ? ( (c) + 32 ) : (c) )
#endif
