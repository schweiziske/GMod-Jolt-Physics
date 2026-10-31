#pragma once

#include <cstdint>

// PhysicsSystem::Update is not transactional: when it reports a contact-capacity error,
// body state has already advanced and some contacts have been skipped. This tracker keeps
// the condition observable without flooding the server console on every bad tick.
class VJoltPhysicsUpdateErrorTracker
{
public:
	bool Record( uint32_t nErrorMask )
	{
		if ( nErrorMask == 0 )
		{
			m_nConsecutiveErrors = 0;
			m_nLastErrorMask = 0;
			return false;
		}

		++m_nTotalErrors;
		++m_nConsecutiveErrors;
		const bool bMaskChanged = nErrorMask != m_nLastErrorMask;
		m_nLastErrorMask = nErrorMask;

		// Report the first occurrence, then 2, 4, 8... persistent occurrences. A newly
		// observed combination of error bits is always reported immediately.
		const bool bPowerOfTwo =
			( m_nConsecutiveErrors & ( m_nConsecutiveErrors - 1 ) ) == 0;
		return bMaskChanged || bPowerOfTwo;
	}

	uint64_t GetTotalErrors() const { return m_nTotalErrors; }
	uint64_t GetConsecutiveErrors() const { return m_nConsecutiveErrors; }
	uint32_t GetLastErrorMask() const { return m_nLastErrorMask; }

private:
	uint64_t m_nTotalErrors = 0;
	uint64_t m_nConsecutiveErrors = 0;
	uint32_t m_nLastErrorMask = 0;
};
