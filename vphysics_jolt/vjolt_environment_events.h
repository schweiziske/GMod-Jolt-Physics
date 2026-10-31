#pragma once

#include <cstdint>
#include <algorithm>
#include <mutex>
#include <utility>
#include <vector>

// Jolt may report body activation from worker threads, while Source game callbacks must run
// from the thread that called IPhysicsEnvironment::Simulate. A handler generation prevents
// queued events from leaking across SetObjectEventHandler replacements.
template < typename Event, typename Listener >
class VJoltDeferredCallbackQueue final
{
public:
	struct Batch
	{
		Listener *pListener = nullptr;
		std::uint64_t nGeneration = 0;
		std::vector< Event > events;
	};

	void SetListener( Listener *pListener )
	{
		std::lock_guard< std::mutex > lock( m_Mutex );
		if ( m_pListener == pListener )
			return;

		m_Events.clear();
		++m_nGeneration;
		m_pListener = pListener;
	}

	Listener *GetListener() const
	{
		std::lock_guard< std::mutex > lock( m_Mutex );
		return m_pListener;
	}

	void Push( Event event )
	{
		std::lock_guard< std::mutex > lock( m_Mutex );
		if ( m_pListener )
			m_Events.push_back( std::move( event ) );
	}

	Batch TakeBatch()
	{
		std::lock_guard< std::mutex > lock( m_Mutex );

		Batch batch;
		batch.pListener = m_pListener;
		batch.nGeneration = m_nGeneration;
		batch.events.reserve( m_Events.size() );
		for ( Event &event : m_Events )
			batch.events.push_back( std::move( event ) );
		m_Events.clear();
		return batch;
	}

	bool IsCurrent( const Batch &batch ) const
	{
		std::lock_guard< std::mutex > lock( m_Mutex );
		return batch.pListener
			&& batch.pListener == m_pListener
			&& batch.nGeneration == m_nGeneration;
	}

	void Discard()
	{
		std::lock_guard< std::mutex > lock( m_Mutex );
		m_Events.clear();
		++m_nGeneration;
	}

	template <typename Predicate>
	void DiscardIf( Predicate predicate )
	{
		std::lock_guard< std::mutex > lock( m_Mutex );
		m_Events.erase( std::remove_if( m_Events.begin(), m_Events.end(), predicate ), m_Events.end() );
	}

private:
	mutable std::mutex m_Mutex;
	std::vector< Event > m_Events;
	Listener *m_pListener = nullptr;
	std::uint64_t m_nGeneration = 0;
};

// IVP owns a physics-environment clock independent of gpGlobals. Jolt has no public equivalent,
// so retain that contract explicitly and do not count internal substeps as external frames.
class VJoltSimulationClock final
{
public:
	void Advance( float flDeltaTime )
	{
		m_flTime += static_cast< double >( flDeltaTime );
	}

	void Reset()
	{
		m_flTime = 0.0;
	}

	float GetTime() const
	{
		return static_cast< float >( m_flTime );
	}

	double GetPreciseTime() const { return m_flTime; }
	void RestoreTime( double time ) { m_flTime = time; }

	float GetNextFrameTime( float flTimestep ) const
	{
		return static_cast< float >( m_flTime + static_cast< double >( flTimestep ) );
	}

private:
	double m_flTime = 0.0;
};
