#pragma once

#include "cmodel.h"
#include "vcollide.h"

namespace VJoltSDK
{
// Verified against both engine profiles; these are implementation records,
// not a consequence of sizeof(void *). Never mix them between Steam branches.
struct SpatialPartitionABI
{
#if defined(GMOD_SDK_PUBLIC) && GMOD_SDK_PUBLIC
    static constexpr size_t Records = 0x20, Capacity = 0x38, Last = 0x4a;
    static constexpr size_t RecordSize = 72, Maxs = 12, Entity = 24;
    static constexpr size_t Mask = 0x20, Flags = 0x22, Next = 0x40, Previous = 0x42;
    static constexpr size_t ServerTree = 0x140;
    static constexpr size_t InsertSlot = 4, RemoveAllSlot = 5, RemoveMaskSlot = 6;
    static constexpr size_t RemoveAndInsertSlot = 7, MoveSlot = 8, DestroySlot = 3, InsertBoundsSlot = 36;
#else
    static constexpr size_t Records = 0x30, Capacity = 0x58, Last = 0x6a;
    static constexpr size_t RecordSize = 80, Maxs = 16, Entity = 32;
    static constexpr size_t Mask = 0x28, Flags = 0x2a, Next = 0x48, Previous = 0x4a;
    static constexpr size_t ServerTree = 0x250;
    static constexpr size_t InsertSlot = 3, RemoveAllSlot = 4, RemoveMaskSlot = 5;
    static constexpr size_t RemoveAndInsertSlot = 6, MoveSlot = 7, DestroySlot = 2, InsertBoundsSlot = 34;
#endif
};

// public's Ray_t has no transform pointer. Never read its padding as a pointer.
inline const matrix3x4_t *RayWorldAxisTransform( const Ray_t &ray )
{
#if defined( GMOD_SDK_PUBLIC ) && GMOD_SDK_PUBLIC
    return nullptr;
#else
    return ray.m_pWorldAxisTransform;
#endif
}

inline void ClearRayWorldAxisTransform( Ray_t &ray )
{
#if !defined( GMOD_SDK_PUBLIC ) || !GMOD_SDK_PUBLIC
    ray.m_pWorldAxisTransform = nullptr;
#endif
}

#if defined( GMOD_SDK_PUBLIC ) && GMOD_SDK_PUBLIC
// GMod's public SDK exposes the legacy interface family on both Windows
// targets. Its optional x64 extension members do not determine pointer width,
// so validate the actual compiled layout instead.
#if INTPTR_MAX == INT64_MAX
static_assert( offsetof( Ray_t, m_IsRay ) == 64, "public x64 Ray_t flag offset" );
static_assert( sizeof( vcollide_t ) == 24, "public x64 vcollide_t layout" );
#elif INTPTR_MAX == INT32_MAX
static_assert( offsetof( Ray_t, m_IsRay ) == 64, "public Win32 Ray_t flag offset" );
static_assert( sizeof( vcollide_t ) == 12, "public Win32 vcollide_t layout" );
#else
#error Unsupported public SDK pointer width
#endif
#else
static_assert( offsetof( Ray_t, m_IsRay ) == 72, "x86-64 Ray_t flag offset" );
static_assert( sizeof( vcollide_t ) == 32, "x86-64 vcollide_t layout" );
#endif
}
