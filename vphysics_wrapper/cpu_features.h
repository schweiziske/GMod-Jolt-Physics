#pragma once

#include <cstdint>

enum CPULevel_t
{
	CPU_HAS_SSE2,
	CPU_HAS_SSE42,
	CPU_HAS_AVX2,
};

struct VJoltCPUFeatures
{
	uint32_t leaf1ECX = 0;
	uint32_t leaf7EBX = 0;
	uint32_t extendedECX = 0;
	uint64_t xcr0 = 0;
};

// Match the instruction sets enabled by vphysics_jolt/CMakeLists.txt.
inline CPULevel_t VJoltSelectCPULevel( const VJoltCPUFeatures &features )
{
	constexpr uint32_t sse42 = ( 1u << 19 ) | ( 1u << 20 ); // SSE4.1/4.2.
	if ( ( features.leaf1ECX & sse42 ) != sse42 )
		return CPU_HAS_SSE2;

	constexpr uint32_t avx = ( 1u << 12 ) | ( 1u << 26 ) | ( 1u << 27 ) | ( 1u << 28 ) | ( 1u << 29 );
	constexpr uint32_t avx2 = ( 1u << 3 ) | ( 1u << 5 ); // BMI1/TZCNT, AVX2.
	constexpr uint32_t lzcnt = 1u << 5;
	if ( ( features.leaf1ECX & avx ) != avx || ( features.leaf7EBX & avx2 ) != avx2
		|| !( features.extendedECX & lzcnt ) || ( features.xcr0 & 6u ) != 6u )
		return CPU_HAS_SSE42;
	return CPU_HAS_AVX2;
}
