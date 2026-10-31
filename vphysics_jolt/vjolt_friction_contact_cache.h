#pragma once

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/SubShapeIDPair.h>

#include "ankerl/unordered_dense.h"

#include <array>
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <shared_mutex>
#include <vector>

// A friction snapshot in Source enumerates the engine's existing contact points. It must not run
// a new overlap query. IVP's managed friction synapse deliberately survives at a positive gap,
// so a Jolt manifold inside the adapter's IVP-sized speculative range is still a real published
// friction contact even when mPenetrationDepth is negative.
//
// Contact callbacks run on Jolt worker threads. Build the next snapshot in sharded maps while an
// update is running, then publish an immutable body-indexed copy after PhysicsSystem::Update has
// completed. The previously published frame stays visible to Source controllers that run before
// the next update.
class JoltFrictionContactCache final
{
public:
	enum class ContactPointSource : std::uint8_t
	{
		Body1,
		Body2,
	};

	struct Contact
	{
		JPH::BodyID otherBodyID;
		std::uint32_t selfSubShapeID = 0;
		std::uint32_t otherSubShapeID = 0;
		JPH::Vec3 normal = JPH::Vec3::sZero();
		// Solver-frame world point is also consumed by the physgun blocked-chain clamp.
		// Keep it separate from the local point used by public Source contact queries.
		JPH::RVec3 contactPoint = JPH::RVec3::sZero();
		JPH::BodyID contactPointBodyID;
		JPH::Vec3 contactPointLocal = JPH::Vec3::sZero();
		float penetrationDepth = 0.0f;
		float normalImpulse = 0.0f;
		float frictionEnergy = 0.0f;
		float frameDeltaTime = 0.0f;
	};

	struct Statistics
	{
		std::uint64_t currentContacts = 0;
		std::uint64_t contactsCreated = 0;
		std::uint64_t contactsDestroyed = 0;
		std::uint64_t contactsProcessed = 0;
	};

	void BeginFrame( float frameDeltaTime )
	{
		m_BuildingFrameDeltaTime = frameDeltaTime;
		++m_BuildingFrameGeneration;
	}

	float GetBuildingFrameDeltaTime() const
	{
		return m_BuildingFrameDeltaTime;
	}

	Statistics ReadStatistics() const
	{
		Statistics stats;
		stats.contactsCreated = m_ContactsCreated.load( std::memory_order_relaxed );
		stats.contactsDestroyed = m_ContactsDestroyed.load( std::memory_order_relaxed );
		stats.contactsProcessed = m_ContactsProcessed.load( std::memory_order_relaxed );
		for ( const BuildingShard &shard : m_BuildingShards )
		{
			std::lock_guard< std::mutex > lock( shard.mutex );
			stats.currentContacts += shard.contacts.size();
		}
		return stats;
	}

	void ClearStatistics()
	{
		// IVP clear_statistic preserves the current sum_of_mindists while clearing
		// the created/deleted interval counters.
		m_ContactsCreated.store( 0, std::memory_order_relaxed );
		m_ContactsDestroyed.store( 0, std::memory_order_relaxed );
		m_ContactsProcessed.store( 0, std::memory_order_relaxed );
	}

	void RecordContact( const JPH::BodyID &body1ID, const JPH::BodyID &body2ID,
		const JPH::ContactManifold &manifold, bool isSensor,
		ContactPointSource contactPointSource = ContactPointSource::Body1,
		const JPH::Body *pContactPointBody = nullptr )
	{
		const JPH::SubShapeIDPair key( body1ID, manifold.mSubShapeID1,
			body2ID, manifold.mSubShapeID2 );

		// Sensors never enter IVP's friction system. The environment deliberately limits Jolt's
		// speculative range to less than IVP max_distance_for_friction, so a negative depth here
		// represents a separated-but-managed friction synapse rather than an arbitrary broadphase pair.
		if ( isSensor || manifold.mRelativeContactPointsOn1.empty()
			|| manifold.mRelativeContactPointsOn2.empty() )
		{
			BuildingShard &shard = GetShard( key );
			std::lock_guard< std::mutex > lock( shard.mutex );
			auto it = shard.contacts.find( key );
			if ( it != shard.contacts.end() )
				EraseBuildingContact( shard, it );
			return;
		}

		BuildingContact contact;
		contact.body1ID = body1ID;
		contact.body2ID = body2ID;
		contact.normal = manifold.mWorldSpaceNormal.NormalizedOr( JPH::Vec3::sAxisZ() );
		contact.contactPoint = contactPointSource == ContactPointSource::Body1
			? manifold.GetWorldSpaceContactPointOn1( 0 )
			: manifold.GetWorldSpaceContactPointOn2( 0 );
		if ( pContactPointBody )
		{
			contact.contactPointBodyID = pContactPointBody->GetID();
			if ( pContactPointBody->GetShape()->GetSubType() == JPH::EShapeSubType::Sphere )
			{
				// IVP's ball friction path stores ball_center +/- normal * extra_radius, not
				// the tolerance-shifted closest point supplied by the manifold generator.
				const JPH::Vec3 worldDirection = contactPointSource == ContactPointSource::Body1
					? manifold.mWorldSpaceNormal : -manifold.mWorldSpaceNormal;
				const JPH::Vec3 localDirection = pContactPointBody->GetWorldTransform()
					.Multiply3x3Transposed( worldDirection ).NormalizedOr( JPH::Vec3::sAxisZ() );
				const auto *pSphere = static_cast< const JPH::SphereShape * >(
					pContactPointBody->GetShape() );
				contact.contactPointLocal = localDirection * pSphere->GetRadius();
			}
			else
			{
				contact.contactPointLocal = JPH::Vec3(
					pContactPointBody->GetInverseCenterOfMassTransform() * contact.contactPoint );
			}
		}
		contact.penetrationDepth = manifold.mPenetrationDepth;

		BuildingShard &shard = GetShard( key );
		std::lock_guard< std::mutex > lock( shard.mutex );
		auto it = shard.contacts.find( key );
		if ( it == shard.contacts.end() )
		{
			contact.frameDeltaTime = m_BuildingFrameDeltaTime;
			contact.createdFrame = m_BuildingFrameGeneration;
			contact.lastSeenFrame = m_BuildingFrameGeneration;
			auto inserted = shard.contacts.emplace( key, contact ).first;
			LinkBuildingContact( shard, inserted );
			m_ContactsCreated.fetch_add( 1, std::memory_order_relaxed );
		}
		else
		{
			contact.previous = it->second.previous;
			contact.next = it->second.next;
			contact.createdFrame = it->second.createdFrame;
			// Linear-cast bodies and multi-substep updates may report the same manifold more
			// than once. Refresh its geometry without discarding response accumulated by an
			// earlier callback in this same PhysicsSystem::Update.
			if ( it->second.lastSeenFrame == m_BuildingFrameGeneration )
			{
				contact.normalImpulse = it->second.normalImpulse;
				contact.frictionEnergy = it->second.frictionEnergy;
				contact.impactFrame = it->second.impactFrame;
			}
			contact.frameDeltaTime = m_BuildingFrameDeltaTime;
			contact.lastSeenFrame = m_BuildingFrameGeneration;
			it->second = contact;
		}
	}

	void AccumulateContactResponse( const JPH::BodyID &body1ID, const JPH::BodyID &body2ID,
		const JPH::ContactManifold &manifold, float normalImpulse, float frictionEnergy )
	{
		const JPH::SubShapeIDPair key( body1ID, manifold.mSubShapeID1,
			body2ID, manifold.mSubShapeID2 );
		BuildingShard &shard = GetShard( key );
		std::lock_guard< std::mutex > lock( shard.mutex );
		auto it = shard.contacts.find( key );
		if ( it == shard.contacts.end() )
			return;

		it->second.normalImpulse += normalImpulse;
		it->second.frictionEnergy += frictionEnergy;
	}

	void AccumulateSolvedNormalImpulse( const JPH::SubShapeIDPair &key, float normalImpulse,
		bool wasImpact )
	{
		BuildingShard &shard = GetShard( key );
		std::lock_guard< std::mutex > lock( shard.mutex );
		auto it = shard.contacts.find( key );
		if ( it == shard.contacts.end() )
			return;

		// IVP creates a friction synapse with now_friction_pressure == 0 during an impact PSI.
		// Jolt may run several collision steps inside one PhysicsSystem::Update; after the first
		// step the same contact is already persistent. Suppress all pressure from that contact
		// for the rest of this published frame so those later lambdas cannot turn the one-shot
		// impact into Source stress damage. The collision impulse itself remains fully solved.
		if ( wasImpact && it->second.createdFrame == m_BuildingFrameGeneration )
		{
			it->second.impactFrame = m_BuildingFrameGeneration;
			it->second.normalImpulse = 0.0f;
		}
		else if ( it->second.impactFrame != m_BuildingFrameGeneration )
		{
			// StoreAppliedImpulses is called once per collision sub-step. Summing the exact
			// distance-keeper lambdas over the update and dividing by BeginFrame's full dt
			// yields the average force exposed by Source's friction snapshot.
			it->second.normalImpulse += normalImpulse;
		}
	}

	void RemoveContact( const JPH::SubShapeIDPair &key )
	{
		BuildingShard &shard = GetShard( key );
		std::lock_guard< std::mutex > lock( shard.mutex );
		auto it = shard.contacts.find( key );
		if ( it != shard.contacts.end() )
			it->second.lastRemovedFrame = m_BuildingFrameGeneration;
	}

	template < typename RetainUnseenContact >
	void PublishFrame( RetainUnseenContact retainUnseenContact )
	{
		// The environment serializes publication. Source queries keep using the published
		// buffer while we reuse the other table and its contact-vector capacities.
		PublishedMap &next = m_PublishedContactBuffers[ 1 - m_PublishedBufferIndex ];
		for ( auto &entry : next )
			entry.second.clear();
		m_PublishPairStates.clear();

		std::uint64_t processedContacts = 0;

		for ( BuildingShard &shard : m_BuildingShards )
		{
			std::lock_guard< std::mutex > lock( shard.mutex );
			for ( const auto &entry : shard.contacts )
			{
				const BuildingContact &contact = entry.second;
				const std::uint64_t pairKey = MakeBodyPairKey( contact.body1ID, contact.body2ID );
				auto result = m_PublishPairStates.try_emplace( pairKey,
					PublishPairState{ contact.body1ID, contact.body2ID } );
				result.first->second.seenThisFrame |=
					contact.lastSeenFrame == m_BuildingFrameGeneration;
			}
		}

		// Do not hold a contact-cache shard while the callback locks Jolt bodies. Apart from
		// shrinking the critical section, this keeps the cache and body-lock order one-way.
		for ( auto &entry : m_PublishPairStates )
		{
			PublishPairState &state = entry.second;
			if ( !state.seenThisFrame )
				state.retainUnseen = retainUnseenContact( state.body1ID, state.body2ID );
		}

		for ( BuildingShard &shard : m_BuildingShards )
		{
			std::lock_guard< std::mutex > lock( shard.mutex );
			for ( auto it = shard.contacts.begin(); it != shard.contacts.end(); )
			{
				const auto &entry = *it;
				const BuildingContact &contact = entry.second;
				const bool seenThisFrame =
					contact.lastSeenFrame == m_BuildingFrameGeneration;
				if ( seenThisFrame )
					++processedContacts;
				bool keep = seenThisFrame
					&& contact.lastRemovedFrame != m_BuildingFrameGeneration;
				if ( !keep )
				{
					const std::uint64_t pairKey = MakeBodyPairKey( contact.body1ID, contact.body2ID );
					auto pairState = m_PublishPairStates.find( pairKey );
					// Sleep removal arrives in a frame where the inactive pair is not detected at
					// all. If any manifold for this pair was genuinely seen this frame, an old or
					// removed subshape is stale rather than a sleeping IVP friction synapse.
					if ( pairState != m_PublishPairStates.end()
						&& pairState->second.seenThisFrame )
					{
						it = EraseBuildingContact( shard, it );
						continue;
					}
					keep = pairState != m_PublishPairStates.end()
						&& pairState->second.retainUnseen;
				}

				if ( !keep )
				{
					it = EraseBuildingContact( shard, it );
					continue;
				}

				next[ contact.body1ID.GetIndexAndSequenceNumber() ].push_back(
					{ contact.body2ID, entry.first.GetSubShapeID1().GetValue(), entry.first.GetSubShapeID2().GetValue(),
						contact.normal, contact.contactPoint, contact.contactPointBodyID,
						contact.contactPointLocal,
						contact.penetrationDepth,
						contact.normalImpulse, contact.frictionEnergy, contact.frameDeltaTime } );
				next[ contact.body2ID.GetIndexAndSequenceNumber() ].push_back(
					{ contact.body1ID, entry.first.GetSubShapeID2().GetValue(), entry.first.GetSubShapeID1().GetValue(),
						-contact.normal, contact.contactPoint, contact.contactPointBodyID,
						contact.contactPointLocal,
						contact.penetrationDepth,
						contact.normalImpulse, contact.frictionEnergy, contact.frameDeltaTime } );
				++it;
			}
		}
		m_ContactsProcessed.fetch_add( processedContacts, std::memory_order_relaxed );
		for ( auto it = next.begin(); it != next.end(); )
		{
			std::vector< Contact > &contacts = it->second;
			if ( contacts.empty() )
			{
				it = next.erase( it );
				continue;
			}
			if ( !std::is_sorted( contacts.begin(), contacts.end(), ContactLess ) )
				std::sort( contacts.begin(), contacts.end(), ContactLess );
			++it;
		}

		std::unique_lock< std::shared_mutex > lock( m_PublishedMutex );
		// Flipping the index avoids unordered_dense::swap's temporary bucket allocations.
		m_PublishedBufferIndex = 1 - m_PublishedBufferIndex;
	}

	void CopyPublishedContacts( const JPH::BodyID &bodyID, std::vector< Contact > &out ) const
	{
		out.clear();
		std::shared_lock< std::shared_mutex > lock( m_PublishedMutex );
		const PublishedMap &published = PublishedContacts();
		auto it = published.find( bodyID.GetIndexAndSequenceNumber() );
		if ( it != published.end() )
			out = it->second;
	}

	bool CopyFirstPublishedContact( const JPH::BodyID &bodyID, Contact &out ) const
	{
		std::shared_lock< std::shared_mutex > lock( m_PublishedMutex );
		const PublishedMap &published = PublishedContacts();
		auto it = published.find( bodyID.GetIndexAndSequenceNumber() );
		if ( it == published.end() || it->second.empty() )
			return false;
		out = it->second.front();
		return true;
	}

	float GetPublishedNormalImpulseTotal( const JPH::BodyID &bodyID ) const
	{
		float total = 0.0f;
		std::shared_lock< std::shared_mutex > lock( m_PublishedMutex );
		const PublishedMap &published = PublishedContacts();
		auto it = published.find( bodyID.GetIndexAndSequenceNumber() );
		if ( it != published.end() )
			for ( const Contact &contact : it->second )
				total += contact.normalImpulse;
		return total;
	}

	void RemovePublishedContacts( const JPH::BodyID &body1ID, const JPH::BodyID &body2ID )
	{
		RemoveBuildingPair( body1ID, body2ID );

		std::unique_lock< std::shared_mutex > lock( m_PublishedMutex );
		RemoveOneWay( body1ID, body2ID );
		RemoveOneWay( body2ID, body1ID );
	}

	void InvalidateBody( const JPH::BodyID &bodyID )
	{
		for ( BuildingShard &shard : m_BuildingShards )
		{
			std::lock_guard< std::mutex > lock( shard.mutex );
			for (;;)
			{
				auto body = shard.bodies.find( bodyID.GetIndexAndSequenceNumber() );
				if ( body == shard.bodies.end() )
					break;
				// Erasure can move both dense-map entries. Look up the new head rather than
				// retaining a reference across it; unrelated contacts are never traversed.
				EraseBuildingContact( shard, shard.contacts.begin() + body->second.first );
			}
		}

		std::unique_lock< std::shared_mutex > lock( m_PublishedMutex );
		PublishedMap &published = PublishedContacts();
		auto it = published.find( bodyID.GetIndexAndSequenceNumber() );
		if ( it == published.end() )
			return;
		// Erasing a dense-map entry may move another body's vector. Own this list before
		// touching the map, then visit only the neighbors present in the published frame.
		std::vector< Contact > contacts = std::move( it->second );
		published.erase( it );
		for ( std::size_t index = 0; index < contacts.size(); ++index )
		{
			if ( index == 0 || contacts[ index ].otherBodyID != contacts[ index - 1 ].otherBodyID )
				RemoveOneWay( contacts[ index ].otherBodyID, bodyID );
		}
	}

private:
	static constexpr std::uint32_t kInvalidContactIndex = ~std::uint32_t( 0 );

	struct BuildingContact
	{
		JPH::BodyID body1ID;
		JPH::BodyID body2ID;
		JPH::Vec3 normal = JPH::Vec3::sZero();
		JPH::RVec3 contactPoint = JPH::RVec3::sZero();
		JPH::BodyID contactPointBodyID;
		JPH::Vec3 contactPointLocal = JPH::Vec3::sZero();
		float penetrationDepth = 0.0f;
		float normalImpulse = 0.0f;
		float frictionEnergy = 0.0f;
		float frameDeltaTime = 0.0f;
		std::uint64_t createdFrame = 0;
		std::uint64_t lastSeenFrame = 0;
		std::uint64_t lastRemovedFrame = 0;
		std::uint64_t impactFrame = 0;
		// Intrusive lists for the two endpoints, using dense contact indices rather than
		// pointers or separately allocated nodes. Only this contact's shard lock is needed.
		std::array< std::uint32_t, 2 > previous { kInvalidContactIndex, kInvalidContactIndex };
		std::array< std::uint32_t, 2 > next { kInvalidContactIndex, kInvalidContactIndex };
	};

	struct PublishPairState
	{
		JPH::BodyID body1ID;
		JPH::BodyID body2ID;
		bool seenThisFrame = false;
		bool retainUnseen = false;
	};

	using BuildingContactMap = ankerl::unordered_dense::map< JPH::SubShapeIDPair, BuildingContact >;

	struct BodyContacts
	{
		std::uint32_t first = kInvalidContactIndex;
		std::uint32_t count = 0;
	};

	struct BuildingShard
	{
		mutable std::mutex mutex;
		BuildingContactMap contacts;
		ankerl::unordered_dense::map< std::uint32_t, BodyContacts > bodies;
	};

	static std::uint32_t GetEndpoint( const BuildingContact &contact, const JPH::BodyID &bodyID )
	{
		JPH_ASSERT( contact.body1ID == bodyID || contact.body2ID == bodyID );
		return contact.body1ID == bodyID ? 0 : 1;
	}

	// All index helpers run under the existing shard mutex. Insertion appends to the
	// dense map; reallocation preserves indices. Its erase moves the last entry into
	// the removed slot, so EraseBuildingContact also repairs that entry's links.
	static void LinkBuildingContact( BuildingShard &shard, BuildingContactMap::iterator it )
	{
		JPH_ASSERT( shard.contacts.size() < kInvalidContactIndex );
		const auto index = static_cast< std::uint32_t >( it - shard.contacts.begin() );
		BuildingContact &contact = it->second;
		for ( std::uint32_t endpoint = 0; endpoint < 2; ++endpoint )
		{
			if ( endpoint == 1 && contact.body1ID == contact.body2ID )
				continue;
			const JPH::BodyID bodyID = endpoint == 0 ? contact.body1ID : contact.body2ID;
			BodyContacts &body = shard.bodies[ bodyID.GetIndexAndSequenceNumber() ];
			contact.next[ endpoint ] = body.first;
			if ( body.first != kInvalidContactIndex )
			{
				BuildingContact &next = ( shard.contacts.begin() + body.first )->second;
				next.previous[ GetEndpoint( next, bodyID ) ] = index;
			}
			body.first = index;
			++body.count;
		}
	}

	BuildingContactMap::iterator EraseBuildingContact( BuildingShard &shard, BuildingContactMap::iterator it )
	{
		const auto index = static_cast< std::uint32_t >( it - shard.contacts.begin() );
		const auto last = static_cast< std::uint32_t >( shard.contacts.size() - 1 );
		const BuildingContact &contact = it->second;
		for ( std::uint32_t endpoint = 0; endpoint < 2; ++endpoint )
		{
			if ( endpoint == 1 && contact.body1ID == contact.body2ID )
				continue;
			const JPH::BodyID bodyID = endpoint == 0 ? contact.body1ID : contact.body2ID;
			auto body = shard.bodies.find( bodyID.GetIndexAndSequenceNumber() );
			JPH_ASSERT( body != shard.bodies.end() && body->second.count > 0 );
			if ( contact.previous[ endpoint ] == kInvalidContactIndex )
				body->second.first = contact.next[ endpoint ];
			else
			{
				BuildingContact &previous = ( shard.contacts.begin() + contact.previous[ endpoint ] )->second;
				previous.next[ GetEndpoint( previous, bodyID ) ] = contact.next[ endpoint ];
			}
			if ( contact.next[ endpoint ] != kInvalidContactIndex )
			{
				BuildingContact &next = ( shard.contacts.begin() + contact.next[ endpoint ] )->second;
				next.previous[ GetEndpoint( next, bodyID ) ] = contact.previous[ endpoint ];
			}
			if ( --body->second.count == 0 )
				shard.bodies.erase( body );
		}
		if ( index != last )
		{
			const BuildingContact &moved = ( shard.contacts.begin() + last )->second;
			for ( std::uint32_t endpoint = 0; endpoint < 2; ++endpoint )
			{
				if ( endpoint == 1 && moved.body1ID == moved.body2ID )
					continue;
				const JPH::BodyID bodyID = endpoint == 0 ? moved.body1ID : moved.body2ID;
				if ( moved.previous[ endpoint ] == kInvalidContactIndex )
					shard.bodies.find( bodyID.GetIndexAndSequenceNumber() )->second.first = index;
				else
				{
					BuildingContact &previous = ( shard.contacts.begin() + moved.previous[ endpoint ] )->second;
					previous.next[ GetEndpoint( previous, bodyID ) ] = index;
				}
				if ( moved.next[ endpoint ] != kInvalidContactIndex )
				{
					BuildingContact &next = ( shard.contacts.begin() + moved.next[ endpoint ] )->second;
					next.previous[ GetEndpoint( next, bodyID ) ] = index;
				}
			}
		}
		m_ContactsDestroyed.fetch_add( 1, std::memory_order_relaxed );
		return shard.contacts.erase( it );
	}

	static constexpr size_t kShardCount = 64;
	static_assert( ( kShardCount & ( kShardCount - 1 ) ) == 0, "Shard count must be a power of two" );

	BuildingShard &GetShard( const JPH::SubShapeIDPair &key )
	{
		return m_BuildingShards[ key.GetHash() & ( kShardCount - 1 ) ];
	}

	static std::uint64_t MakeBodyPairKey( const JPH::BodyID &body1ID, const JPH::BodyID &body2ID )
	{
		std::uint32_t id1 = body1ID.GetIndexAndSequenceNumber();
		std::uint32_t id2 = body2ID.GetIndexAndSequenceNumber();
		if ( id1 > id2 )
			std::swap( id1, id2 );
		return ( std::uint64_t( id1 ) << 32 ) | id2;
	}

	static bool ContactLess( const Contact &a, const Contact &b )
	{
		if ( a.otherBodyID != b.otherBodyID )
			return a.otherBodyID < b.otherBodyID;
		if ( a.selfSubShapeID != b.selfSubShapeID )
			return a.selfSubShapeID < b.selfSubShapeID;
		return a.otherSubShapeID < b.otherSubShapeID;
	}

	void RemoveBuildingPair( const JPH::BodyID &body1ID, const JPH::BodyID &body2ID )
	{
		for ( BuildingShard &shard : m_BuildingShards )
		{
			std::lock_guard< std::mutex > lock( shard.mutex );
			auto body1 = shard.bodies.find( body1ID.GetIndexAndSequenceNumber() );
			auto body2 = shard.bodies.find( body2ID.GetIndexAndSequenceNumber() );
			if ( body1 == shard.bodies.end() || body2 == shard.bodies.end() )
				continue;
			// A static/world endpoint may touch most of the scene. Walk the smaller list.
			const bool useBody1 = body1->second.count <= body2->second.count;
			const JPH::BodyID bodyID = useBody1 ? body1ID : body2ID;
			std::uint32_t index = ( useBody1 ? body1 : body2 )->second.first;
			while ( index != kInvalidContactIndex )
			{
				auto it = shard.contacts.begin() + index;
				const BuildingContact &contact = it->second;
				std::uint32_t next = contact.next[ GetEndpoint( contact, bodyID ) ];
				if ( ( contact.body1ID == body1ID && contact.body2ID == body2ID )
					|| ( contact.body1ID == body2ID && contact.body2ID == body1ID ) )
				{
					// The next linked contact may be the dense entry moved into this slot.
					if ( next == shard.contacts.size() - 1 )
						next = index;
					EraseBuildingContact( shard, it );
				}
				index = next;
			}
		}
	}

	using PublishedMap = ankerl::unordered_dense::map< std::uint32_t, std::vector< Contact > >;
	using PublishPairStateMap =
		ankerl::unordered_dense::map< std::uint64_t, PublishPairState >;

	// Callers hold m_PublishedMutex; the publisher exclusively owns the other buffer.
	PublishedMap &PublishedContacts() { return m_PublishedContactBuffers[ m_PublishedBufferIndex ]; }
	const PublishedMap &PublishedContacts() const { return m_PublishedContactBuffers[ m_PublishedBufferIndex ]; }

	void RemoveOneWay( const JPH::BodyID &bodyID, const JPH::BodyID &otherBodyID )
	{
		PublishedMap &published = PublishedContacts();
		auto it = published.find( bodyID.GetIndexAndSequenceNumber() );
		if ( it == published.end() )
			return;
		auto &contacts = it->second;
		// Publication sorts by other body first; preserve that order after removing the
		// complete subshape range so first-contact queries remain deterministic.
		auto first = std::lower_bound( contacts.begin(), contacts.end(), otherBodyID,
			[]( const Contact &contact, const JPH::BodyID &id ) { return contact.otherBodyID < id; } );
		auto last = std::upper_bound( first, contacts.end(), otherBodyID,
			[]( const JPH::BodyID &id, const Contact &contact ) { return id < contact.otherBodyID; } );
		contacts.erase( first, last );
		if ( contacts.empty() )
			published.erase( it );
	}

	std::array< BuildingShard, kShardCount > m_BuildingShards;
	std::atomic< std::uint64_t > m_ContactsCreated = 0;
	std::atomic< std::uint64_t > m_ContactsDestroyed = 0;
	std::atomic< std::uint64_t > m_ContactsProcessed = 0;
	float m_BuildingFrameDeltaTime = 0.0f;
	std::uint64_t m_BuildingFrameGeneration = 1;
	PublishPairStateMap m_PublishPairStates;
	mutable std::shared_mutex m_PublishedMutex;
	std::array< PublishedMap, 2 > m_PublishedContactBuffers;
	std::size_t m_PublishedBufferIndex = 0;
};
