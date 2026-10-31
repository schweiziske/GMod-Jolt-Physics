#pragma once

#include <Jolt/Core/TempAllocator.h>

#include <cstdint>

// Jolt's fixed TempAllocatorImpl aborts the process when its arena is exhausted.
// Keep the fast stack arena for the normal case, but use Jolt's own malloc allocator
// for overflow allocations and retain enough counters to diagnose pressure spikes.
class VJoltTempAllocator final : public JPH::TempAllocator
{
public:
	struct Stats
	{
		uint64_t nFixedCapacityBytes = 0;
		uint64_t nFixedPeakBytes = 0;
		uint64_t nFallbackAttempts = 0;
		uint64_t nFallbackAllocations = 0;
		uint64_t nFallbackTotalBytes = 0;
		uint64_t nFallbackInUseBytes = 0;
		uint64_t nFallbackPeakBytes = 0;
		uint64_t nLargestFallbackAllocation = 0;
		uint64_t nAllocationFailures = 0;
	};

	explicit VJoltTempAllocator( uint32_t nFixedSize )
		: m_FixedAllocator( nFixedSize )
		, m_nFixedCapacityBytes( nFixedSize )
	{
	}

	~VJoltTempAllocator() override
	{
		JPH_ASSERT( m_nFallbackInUseBytes == 0 );
	}

	void *Allocate( uint32_t nSize ) override
	{
		if ( nSize == 0 )
			return nullptr;

		if ( m_FixedAllocator.CanAllocate( nSize ) )
		{
			void *pMemory = m_FixedAllocator.Allocate( nSize );
			UpdatePeak( m_nFixedPeakBytes, m_FixedAllocator.GetUsage() );
			return pMemory;
		}

		++m_nFallbackAttempts;
		void *pMemory = m_FallbackAllocator.Allocate( nSize );
		if ( pMemory == nullptr )
		{
			++m_nAllocationFailures;
			return nullptr;
		}

		++m_nFallbackAllocations;
		m_nFallbackTotalBytes += nSize;
		m_nFallbackInUseBytes += nSize;
		UpdatePeak( m_nFallbackPeakBytes, m_nFallbackInUseBytes );
		UpdatePeak( m_nLargestFallbackAllocation, nSize );
		return pMemory;
	}

	void Free( void *pMemory, uint32_t nSize ) override
	{
		if ( pMemory == nullptr )
		{
			JPH_ASSERT( nSize == 0 );
			return;
		}

		if ( m_FixedAllocator.OwnsMemory( pMemory ) )
		{
			m_FixedAllocator.Free( pMemory, nSize );
			return;
		}

		m_FallbackAllocator.Free( pMemory, nSize );
		JPH_ASSERT( m_nFallbackInUseBytes >= nSize );
		m_nFallbackInUseBytes -= nSize;
	}

	Stats GetStats() const
	{
		return {
			.nFixedCapacityBytes = m_nFixedCapacityBytes,
			.nFixedPeakBytes = m_nFixedPeakBytes,
			.nFallbackAttempts = m_nFallbackAttempts,
			.nFallbackAllocations = m_nFallbackAllocations,
			.nFallbackTotalBytes = m_nFallbackTotalBytes,
			.nFallbackInUseBytes = m_nFallbackInUseBytes,
			.nFallbackPeakBytes = m_nFallbackPeakBytes,
			.nLargestFallbackAllocation = m_nLargestFallbackAllocation,
			.nAllocationFailures = m_nAllocationFailures,
		};
	}

private:
	static void UpdatePeak( uint64_t &nPeak, uint64_t nValue )
	{
		if ( nValue > nPeak )
			nPeak = nValue;
	}

	JPH::TempAllocatorImpl m_FixedAllocator;
	JPH::TempAllocatorMalloc m_FallbackAllocator;
	const uint64_t m_nFixedCapacityBytes;
	uint64_t m_nFixedPeakBytes = 0;
	uint64_t m_nFallbackAttempts = 0;
	uint64_t m_nFallbackAllocations = 0;
	uint64_t m_nFallbackTotalBytes = 0;
	uint64_t m_nFallbackInUseBytes = 0;
	uint64_t m_nFallbackPeakBytes = 0;
	uint64_t m_nLargestFallbackAllocation = 0;
	uint64_t m_nAllocationFailures = 0;
};
