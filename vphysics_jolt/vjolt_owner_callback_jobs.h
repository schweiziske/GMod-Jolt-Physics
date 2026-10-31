#pragma once

#include <Jolt/Core/JobSystemWithBarrier.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <mutex>
#include <thread>

// The existing pool still owns the solver jobs and dependency graph. Only the
// duration of a synchronous Source callback is serialized on Update's owner.
// Solver stacks remain live: no body snapshots, rewinds or second solver.
class VJoltOwnerCallbackJobs final : public JPH::JobSystem
{
public:
	explicit VJoltOwnerCallbackJobs( JPH::JobSystemWithBarrier &pool )
		: m_Pool( pool ), m_Owner( std::this_thread::get_id() ) { }
	~VJoltOwnerCallbackJobs()
	{
		if ( m_Profile && m_Queued )
			std::fprintf( stderr, "SYNC_CALLBACKS queued=%llu cycles=%llu inline=%llu owner_delay_ms=%.6f drain_ms=%.6f work_ms=%.6f game_ms=%.6f response_ms=%.6f\n",
				m_Queued, m_Cycles, m_Inline, m_OwnerDelayMs, m_DrainMs, m_WorkMs, m_GameMs, m_ResponseMs );
	}
	bool IsProfiling() const { return m_Profile; }
	void RecordEventTimings( double gameMs, double responseMs ) { m_GameMs += gameMs; m_ResponseMs += responseMs; }

	int GetMaxConcurrency() const override { return m_Pool.GetMaxConcurrency(); }
	void CooperativeYield() override
	{
		if ( !( m_JobState.load( std::memory_order_acquire ) & cPaused ) ) return;
		LeaveJob();
		EnterJob();
	}
	JobHandle CreateJob( const char *name, JPH::ColorArg color,
		const JobFunction &function, JPH::uint32 dependencies = 0 ) override
	{
		return m_Pool.CreateJob( name, color, [this, function]()
		{
			EnterJob();
			function();
			LeaveJob();
		}, dependencies );
	}
	Barrier *CreateBarrier() override { m_Barrier = m_Pool.CreateBarrier(); return m_Barrier; }
	void DestroyBarrier( Barrier *barrier ) override
	{
		m_Pool.DestroyBarrier( barrier ); m_Barrier = nullptr;
	}
	void WaitForJobs( Barrier *barrier ) override
	{
		m_Pool.WaitForJobs( barrier, [this]() { ServiceRequests(); } );
	}

	bool Call( const std::function<bool()> &function )
	{
		// A whole contact batch can already be owned by this thread. Individual
		// events within it keep their synchronous semantics without another gate.
		if ( IsInCallback() )
		{
			if ( m_Profile ) ++m_Inline;
			return function();
		}
		Request request{ function };
		{
			std::lock_guard lock( m_Mutex );
			JPH_ASSERT( ( m_JobState.load( std::memory_order_relaxed ) & cActiveMask ) > 0 && m_Barrier != nullptr );
			// Closing the gate and counting entrants share one atomic word. A job
			// either entered before this RMW and must leave, or cannot enter at all.
			const unsigned previous = m_JobState.fetch_or( cPaused, std::memory_order_acq_rel );
			if ( m_Profile )
			{
				++m_Queued;
				if ( !( previous & cPaused ) ) m_PauseBegin = Clock::now();
			}
			m_Requests.push_back( &request );
			m_JobState.fetch_sub( 1, std::memory_order_acq_rel );
			m_Pool.WakeWaiter( m_Barrier );
		}
		m_Changed.notify_all();
		if ( std::this_thread::get_id() == m_Owner ) ServiceRequests();
		{
			std::unique_lock lock( m_Mutex );
			m_Changed.wait( lock, [&]() { return request.done; } );
		}
		EnterJob();
		return request.result;
	}

	// Only the owner reads m_Servicing. It still holds Update's body locks;
	// physics queries in callbacks must therefore use Jolt's NoLock interfaces.
	bool IsInCallback() const { return std::this_thread::get_id() == m_Owner && m_Servicing; }

private:
	static constexpr unsigned cPaused = 1u << 31;
	static constexpr unsigned cActiveMask = cPaused - 1;

	struct Request
	{
		std::function<bool()> function;
		bool result = false, done = false;
	};
	void EnterJob()
	{
		for (;;)
		{
			unsigned state = m_JobState.load( std::memory_order_acquire );
			while ( !( state & cPaused ) )
				if ( m_JobState.compare_exchange_weak( state, state + 1,
					std::memory_order_acquire, std::memory_order_relaxed ) ) return;
			if ( std::this_thread::get_id() == m_Owner )
				ServiceRequests();
			else
			{
				std::unique_lock lock( m_Mutex );
				m_Changed.wait( lock, [&]() { return !( m_JobState.load( std::memory_order_acquire ) & cPaused ); } );
			}
		}
	}
	void LeaveJob()
	{
		const unsigned previous = m_JobState.fetch_sub( 1, std::memory_order_acq_rel );
		JPH_ASSERT( ( previous & cActiveMask ) > 0 );
		if ( previous == ( cPaused | 1u ) )
		{
			// Pair with the waiter's mutex so its nonzero check and wait cannot
			// miss the final departure. Ordinary job completion takes no mutex.
			std::lock_guard lock( m_Mutex );
			m_Changed.notify_all();
		}
	}
	void ServiceRequests()
	{
		JPH_ASSERT( std::this_thread::get_id() == m_Owner );
		if ( !( m_JobState.load( std::memory_order_acquire ) & cPaused ) ) return;
		std::unique_lock lock( m_Mutex );
		if ( m_Requests.empty() ) return;
		const auto serviceBegin = m_Profile ? Clock::now() : Clock::time_point{};
		// No new job enters after a request is queued. Existing jobs finish or
		// cooperatively suspend, including large-island workers waiting on peers.
		m_Changed.wait( lock, [&]() { return ( m_JobState.load( std::memory_order_acquire ) & cActiveMask ) == 0; } );
		const auto workBegin = m_Profile ? Clock::now() : Clock::time_point{};
		if ( m_Profile )
		{
			++m_Cycles;
			m_OwnerDelayMs += Milliseconds( serviceBegin - m_PauseBegin );
			m_DrainMs += Milliseconds( workBegin - serviceBegin );
		}
		m_Servicing = true;
		while ( !m_Requests.empty() )
		{
			Request *request = m_Requests.front();
			lock.unlock();
			const bool result = request->function();
			lock.lock();
			request->result = result;
			request->done = true;
			m_Requests.pop_front();
		}
		m_Servicing = false;
		if ( m_Profile ) m_WorkMs += Milliseconds( Clock::now() - workBegin );
		// No solver job is active here. Publish all callback writes before
		// reopening the gate, including to jobs that arrive without parking.
		m_JobState.store( 0, std::memory_order_release );
		lock.unlock();
		m_Changed.notify_all();
	}
	void QueueJob( Job * ) override { JPH_ASSERT( false ); }
	void QueueJobs( Job **, JPH::uint ) override { JPH_ASSERT( false ); }
	void FreeJob( Job * ) override { JPH_ASSERT( false ); }

	JPH::JobSystemWithBarrier &m_Pool;
	const std::thread::id m_Owner;
	Barrier *m_Barrier = nullptr;
	std::mutex m_Mutex;
	std::condition_variable m_Changed;
	std::deque<Request *> m_Requests;
	std::atomic<unsigned> m_JobState { 0 };
	bool m_Servicing = false;
	using Clock = std::chrono::steady_clock;
	static double Milliseconds( Clock::duration duration ) { return std::chrono::duration<double, std::milli>( duration ).count(); }
	const bool m_Profile = []() { const char *v = std::getenv( "VJOLT_PROFILE_SYNC_COLLISIONS" ); return v && v[0] == '1'; }();
	Clock::time_point m_PauseBegin;
	unsigned long long m_Queued = 0, m_Cycles = 0, m_Inline = 0;
	double m_OwnerDelayMs = 0, m_DrainMs = 0, m_WorkMs = 0, m_GameMs = 0, m_ResponseMs = 0;
};
