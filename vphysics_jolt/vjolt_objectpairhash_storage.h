#pragma once

#include <algorithm>
#include <cstddef>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// Source stores one insertion-ordered neighbour list per object plus one unique unordered pair.
// Keep that contract separate from the VPhysics interface so it can be regression-tested without
// loading the engine DLLs.
class VJoltObjectPairStorage
{
public:
	void AddPair( void *pObject0, void *pObject1 )
	{
		const Pair pair = CreateSortedPair( pObject0, pObject1 );
		if ( !m_Pairs.emplace( pair ).second )
			return;

		m_ObjectLists[ pObject0 ].push_back( pObject1 );
		m_ObjectLists[ pObject1 ].push_back( pObject0 );
	}

	void RemovePair( void *pObject0, void *pObject1 )
	{
		const Pair pair = CreateSortedPair( pObject0, pObject1 );
		if ( m_Pairs.erase( pair ) == 0 )
			return;

		RemoveOneFromObjectList( pObject0, pObject1 );
		RemoveOneFromObjectList( pObject1, pObject0 );
	}

	bool HasPair( void *pObject0, void *pObject1 ) const
	{
		return m_Pairs.find( CreateSortedPair( pObject0, pObject1 ) ) != m_Pairs.end();
	}

	void RemoveAllPairsForObject( void *pObject )
	{
		auto object = m_ObjectLists.find( pObject );
		if ( object == m_ObjectLists.end() )
			return;

		// RemovePair mutates both endpoint lists. Snapshot Source's neighbour list first.
		const std::vector< void * > neighbours = object->second;
		for ( void *pOther : neighbours )
			RemovePair( pObject, pOther );
	}

	bool HasObject( void *pObject ) const
	{
		return m_ObjectLists.find( pObject ) != m_ObjectLists.end();
	}

	int GetPairCountForObject( void *pObject ) const
	{
		const auto object = m_ObjectLists.find( pObject );
		return object == m_ObjectLists.end() ? 0 : static_cast< int >( object->second.size() );
	}

	int GetPairListForObject( void *pObject, int nMaxCount, void **ppObjectList ) const
	{
		if ( nMaxCount <= 0 || !ppObjectList )
			return 0;

		const auto object = m_ObjectLists.find( pObject );
		if ( object == m_ObjectLists.end() )
			return 0;

		const int nObjectCount = static_cast< int >( object->second.size() );
		const int nCount = nMaxCount < nObjectCount ? nMaxCount : nObjectCount;
		for ( int i = 0; i < nCount; ++i )
			ppObjectList[ i ] = object->second[ i ];
		return nCount;
	}

private:
	using Pair = std::pair< void *, void * >;

	struct PairHasher
	{
		size_t operator()( const Pair &pair ) const
		{
			size_t seed = std::hash< void * >{}( pair.first );
			seed ^= std::hash< void * >{}( pair.second ) + size_t( 0x9e3779b9 )
				+ ( seed << 6 ) + ( seed >> 2 );
			return seed;
		}
	};

	static Pair CreateSortedPair( void *pObject0, void *pObject1 )
	{
		const std::less< void * > less;
		return less( pObject1, pObject0 )
			? Pair( pObject1, pObject0 )
			: Pair( pObject0, pObject1 );
	}

	void RemoveOneFromObjectList( void *pObject, void *pRemove )
	{
		auto object = m_ObjectLists.find( pObject );
		if ( object == m_ObjectLists.end() )
			return;

		auto item = std::find( object->second.begin(), object->second.end(), pRemove );
		if ( item == object->second.end() )
			return;

		object->second.erase( item );
		if ( object->second.empty() )
			m_ObjectLists.erase( object );
	}

	std::unordered_set< Pair, PairHasher > m_Pairs;
	std::unordered_map< void *, std::vector< void * > > m_ObjectLists;
};
