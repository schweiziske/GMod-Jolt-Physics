//=================================================================================================
//
// Thread-safe cache for IPhysicsCollisionSolver::ShouldCollide results.
//
// Each pair is indexed from both body keys so RecheckCollisionFilter can invalidate only the
// affected object's neighbours. A mutation epoch prevents a solver result that was computed while
// an invalidation was in flight from being inserted after that invalidation completed.
//
//=================================================================================================

#pragma once

#include "ankerl/unordered_dense.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <shared_mutex>
#include <vector>

class VJoltShouldCollideCache final
{
public:
	struct Lookup
	{
		std::uint64_t pairKey = 0;
		std::uint64_t mutationEpoch = 0;
		bool found = false;
		bool shouldCollide = false;
	};
	struct Pair
	{
		std::uint32_t bodyKey0, bodyKey1;
	};

	Lookup Find( std::uint32_t bodyKey0, std::uint32_t bodyKey1 ) const
	{
		Lookup result = MakeLookup( bodyKey0, bodyKey1 );
		// Published only with simulation jobs parked. A changed answer disables this immutable
		// view immediately; misses and invalidated views retain the existing locked path.
		if ( m_ReadVersion == m_Version.load( std::memory_order_acquire ) )
		{
			result.mutationEpoch = m_ReadMutationEpoch;
			auto entry = m_ReadEntries.find( result.pairKey );
			if ( entry != m_ReadEntries.end() )
			{
				result.found = true;
				result.shouldCollide = entry->second;
				return result;
			}
			// New pairs may have been inserted since publication. They do not
			// invalidate existing snapshot hits, but misses must consult the live map.
			if ( m_ReadContentVersion == m_ContentVersion.load( std::memory_order_acquire ) )
				return result;
		}

		std::shared_lock< std::shared_mutex > lock( m_Mutex );
		result.mutationEpoch = m_MutationEpoch;
		auto entry = m_Entries.find( result.pairKey );
		if ( entry != m_Entries.end() )
		{
			result.found = true;
			result.shouldCollide = entry->second;
		}
		return result;
	}

	// The caller must have completed or suspended all jobs that can read this cache. Publishing
	// never invokes the game solver and does no copying while the contents are stable.
	void PublishReadSnapshot()
	{
		std::shared_lock< std::shared_mutex > lock( m_Mutex );
		const std::uint64_t version = m_Version.load( std::memory_order_acquire );
		const std::uint64_t contentVersion = m_ContentVersion.load( std::memory_order_acquire );
		if ( m_ReadContentVersion == contentVersion )
			return;
		m_ReadEntries = m_Entries;
		m_ReadMutationEpoch = m_MutationEpoch;
		m_ReadVersion = version;
		m_ReadContentVersion = contentVersion;
	}

	// Previously queried pairs only: taking this list never makes an invalidated
	// answer visible again. The owner may revalidate still-nearby pairs in one gate.
	void TakeInvalidatedPairs( std::vector<Pair> &outPairs )
	{
		std::unique_lock< std::shared_mutex > lock( m_Mutex );
		outPairs.clear();
		outPairs.swap( m_InvalidatedPairs );
	}

	bool StoreIfUnchanged( const Lookup &lookup, bool shouldCollide )
	{
		std::unique_lock< std::shared_mutex > lock( m_Mutex );
		if ( lookup.mutationEpoch != m_MutationEpoch )
			return false;

		auto existing = m_Entries.find( lookup.pairKey );
		if ( existing != m_Entries.end() )
		{
			if ( existing->second != shouldCollide )
			{
				m_Version.fetch_add( 1, std::memory_order_release );
				existing->second = shouldCollide;
				m_ContentVersion.fetch_add( 1, std::memory_order_release );
			}
			return true;
		}

		// Insertion cannot change any answer already present in the read snapshot.
		m_Entries.emplace( lookup.pairKey, shouldCollide );
		m_ContentVersion.fetch_add( 1, std::memory_order_release );
		const std::uint32_t bodyKeyA = GetBodyKeyA( lookup.pairKey );
		const std::uint32_t bodyKeyB = GetBodyKeyB( lookup.pairKey );
		m_ObjectPairs[ bodyKeyA ].push_back( lookup.pairKey );
		if ( bodyKeyA != bodyKeyB )
			m_ObjectPairs[ bodyKeyB ].push_back( lookup.pairKey );
		return true;
	}

	void Invalidate( std::uint32_t bodyKey )
	{
		std::unique_lock< std::shared_mutex > lock( m_Mutex );
		++m_MutationEpoch;
		m_Version.fetch_add( 1, std::memory_order_release );
		m_ContentVersion.fetch_add( 1, std::memory_order_release );

		auto object = m_ObjectPairs.find( bodyKey );
		if ( object == m_ObjectPairs.end() )
			return;

		// Move the list out before touching the neighbours: every pair is also present in the other
		// endpoint's compact vector, and those vectors can be edited without invalidating this walk.
		std::vector< std::uint64_t > pairs = std::move( object->second );
		m_ObjectPairs.erase( bodyKey );
		for ( const std::uint64_t pairKey : pairs )
		{
			auto entry = m_Entries.find( pairKey );
			if ( entry == m_Entries.end() )
				continue;

			const std::uint32_t bodyKeyA = GetBodyKeyA( pairKey );
			const std::uint32_t bodyKeyB = GetBodyKeyB( pairKey );
			const std::uint32_t otherBodyKey = bodyKeyA == bodyKey ? bodyKeyB : bodyKeyA;
			if ( otherBodyKey != bodyKey )
			{
				auto other = m_ObjectPairs.find( otherBodyKey );
				if ( other != m_ObjectPairs.end() )
				{
					auto pair = std::find( other->second.begin(), other->second.end(), pairKey );
					if ( pair != other->second.end() )
					{
						*pair = other->second.back();
						other->second.pop_back();
					}
					if ( other->second.empty() )
						m_ObjectPairs.erase( otherBodyKey );
				}
			}

			m_Entries.erase( entry );
			m_InvalidatedPairs.push_back( { bodyKeyA, bodyKeyB } );
		}

	}

	void Clear()
	{
		std::unique_lock< std::shared_mutex > lock( m_Mutex );
		++m_MutationEpoch;
		m_Version.fetch_add( 1, std::memory_order_release );
		m_ContentVersion.fetch_add( 1, std::memory_order_release );
		for ( const auto &entry : m_Entries )
			m_InvalidatedPairs.push_back( { GetBodyKeyA( entry.first ), GetBodyKeyB( entry.first ) } );
		m_Entries.clear();
		m_ObjectPairs.clear();
	}

	std::size_t EntryCount() const
	{
		std::shared_lock< std::shared_mutex > lock( m_Mutex );
		return m_Entries.size();
	}

	std::size_t PairCountForObject( std::uint32_t bodyKey ) const
	{
		std::shared_lock< std::shared_mutex > lock( m_Mutex );
		auto object = m_ObjectPairs.find( bodyKey );
		return object == m_ObjectPairs.end() ? 0 : object->second.size();
	}

private:
	static Lookup MakeLookup( std::uint32_t bodyKey0, std::uint32_t bodyKey1 )
	{
		Lookup result;
		if ( bodyKey0 > bodyKey1 )
		{
			const std::uint32_t swap = bodyKey0;
			bodyKey0 = bodyKey1;
			bodyKey1 = swap;
		}
		result.pairKey = ( std::uint64_t( bodyKey0 ) << 32 ) | bodyKey1;
		return result;
	}

	static std::uint32_t GetBodyKeyA( std::uint64_t pairKey )
	{
		return static_cast< std::uint32_t >( pairKey >> 32 );
	}

	static std::uint32_t GetBodyKeyB( std::uint64_t pairKey )
	{
		return static_cast< std::uint32_t >( pairKey );
	}

	mutable std::shared_mutex m_Mutex;
	std::atomic< std::uint64_t > m_Version{ 0 };
	std::uint64_t m_ReadVersion = ~std::uint64_t( 0 );
	std::uint64_t m_ReadMutationEpoch = 0;
	std::uint64_t m_ReadContentVersion = ~std::uint64_t( 0 );
	std::atomic< std::uint64_t > m_ContentVersion{ 0 };
	ankerl::unordered_dense::map< std::uint64_t, bool > m_ReadEntries;
	std::uint64_t m_MutationEpoch = 0;
	std::vector<Pair> m_InvalidatedPairs;
	ankerl::unordered_dense::map< std::uint64_t, bool > m_Entries;
	ankerl::unordered_dense::map< std::uint32_t, std::vector< std::uint64_t > > m_ObjectPairs;
};
