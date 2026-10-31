//=================================================================================================
//
// GMod 64-bit VPhysics save/restore pointer indirection.
//
// The engine keeps the on-disk physics reference format at 32 bits even though live pointers are
// 64 bits. During save, pointers are assigned stable IDs. During restore, those IDs are rebound to
// the newly-created objects. ID 0 is reserved for the world object by PreSave / PreRestore.
//
//=================================================================================================

#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>

class VJoltSaveRestoreIdTable final
{
public:
	using Id = std::uint32_t;
	static constexpr Id INVALID_ID = ~Id( 0 );
	// ISave/IRestore exchange signed 32-bit integers and reserve negative values for failure.
	static constexpr Id MAX_ID = INT32_MAX;

	void BeginSave( const void *pWorldObject )
	{
		Reset();
		m_Phase = Phase::Saving;
		m_SavePointerIds.emplace( pWorldObject, 0 );
		m_NextSaveId = 1;
	}

	Id GetOrCreateSaveId( const void *pPointer )
	{
		if ( m_Phase != Phase::Saving )
			return INVALID_ID;

		auto iter = m_SavePointerIds.find( pPointer );
		if ( iter != m_SavePointerIds.end() )
			return iter->second;

		if ( m_NextSaveId > MAX_ID )
			return INVALID_ID;

		const Id id = m_NextSaveId++;
		m_SavePointerIds.emplace( pPointer, id );
		return id;
	}

	void EndSave()
	{
		Reset();
	}

	void BeginRestore( void *pWorldObject, int worldType = 0 )
	{
		Reset();
		m_Phase = Phase::Restoring;
		m_RestoreIdPointers.emplace( 0, RestoredPointer{ pWorldObject, worldType } );
	}

	bool RegisterRestoreId( Id id, void *pPointer, int type = 0 )
	{
		if ( m_Phase != Phase::Restoring || id > MAX_ID )
			return false;

		auto result = m_RestoreIdPointers.emplace( id, RestoredPointer{ pPointer, type } );
		return result.second || ( result.first->second.pointer == pPointer
			&& result.first->second.type == type );
	}

	bool LookupRestoreId( Id id, void *&pPointer, int type = 0 ) const
	{
		pPointer = nullptr;
		if ( m_Phase != Phase::Restoring || id > MAX_ID )
			return false;

		auto iter = m_RestoreIdPointers.find( id );
		if ( iter == m_RestoreIdPointers.end() || ( type != 0 && iter->second.type != type ) )
			return false;

		pPointer = iter->second.pointer;
		return true;
	}

	template < typename T >
	bool LookupRestoreId( Id id, T *&pPointer, int type = 0 ) const
	{
		void *pUntyped = nullptr;
		const bool bFound = LookupRestoreId( id, pUntyped, type );
		pPointer = static_cast< T * >( pUntyped );
		return bFound;
	}

	void ForgetRestorePointer( const void *pPointer )
	{
		for ( auto iter = m_RestoreIdPointers.begin(); iter != m_RestoreIdPointers.end(); )
		{
			if ( iter->second.pointer == pPointer )
				iter = m_RestoreIdPointers.erase( iter );
			else
				++iter;
		}
	}

	void EndRestore()
	{
		Reset();
	}

	bool IsSaving() const { return m_Phase == Phase::Saving; }
	bool IsRestoring() const { return m_Phase == Phase::Restoring; }
	std::size_t SaveEntryCount() const { return m_SavePointerIds.size(); }
	std::size_t RestoreEntryCount() const { return m_RestoreIdPointers.size(); }

private:
	struct RestoredPointer
	{
		void *pointer;
		int type;
	};

	enum class Phase
	{
		Idle,
		Saving,
		Restoring,
	};

	void Reset()
	{
		m_SavePointerIds.clear();
		m_RestoreIdPointers.clear();
		m_NextSaveId = 0;
		m_Phase = Phase::Idle;
	}

	Phase m_Phase = Phase::Idle;
	Id m_NextSaveId = 0;
	std::unordered_map< const void *, Id > m_SavePointerIds;
	std::unordered_map< Id, RestoredPointer > m_RestoreIdPointers;
};
