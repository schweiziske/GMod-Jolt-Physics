#include "cbase.h"
#include "bridge/vjolt_softbody_internal.h"
#include "bridge/vjolt_softbody_mesh.h"
#include "bridge/vjolt_softbody_query.h"
#include "vjolt_environment.h"
#include "vjolt_interface.h"
#include "vjolt_layers.h"

#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <limits>
#include <memory>
#include <new>
#include <unordered_map>

#include "tier0/memdbgon.h"

namespace
{
using namespace VJoltSoftBody;

// Inputs are differences/sums of angles already bounded to [-PI, PI].
float WrapBendAngle( float angle )
{
	if ( angle > JPH::JPH_PI ) return angle - 2.0f * JPH::JPH_PI;
	if ( angle < -JPH::JPH_PI ) return angle + 2.0f * JPH::JPH_PI;
	return angle;
}

struct SoftBodyRecord
{
	JoltPhysicsEnvironment *environment = nullptr;
	JPH::Body *body = nullptr;
	PreparedMesh mesh;
	VJoltSoftBodyPlasticity plasticity;
	std::vector<float> originalEdgeLengths;
	std::vector<float> originalBendAngles;
	bool retiring = false;

	~SoftBodyRecord()
	{
		if ( body )
		{
			auto &bodies = environment->GetPhysicsSystem()->GetBodyInterfaceNoLock();
			if ( body->IsInBroadPhase() )
			{
				bodies.RemoveBody( body->GetID() );
			}
			bodies.DestroyBody( body->GetID() );
		}
	}

	JPH::SoftBodyMotionProperties &Motion() const
	{
		return *static_cast<JPH::SoftBodyMotionProperties *>( body->GetMotionProperties() );
	}

	void Wake() const
	{
		environment->GetPhysicsSystem()->GetBodyInterfaceNoLock().ActivateBody( body->GetID() );
	}

	bool AcceptsVelocity( JPH::Vec3Arg velocity ) const
	{
		return !velocity.IsNaN() && velocity.LengthSq() <= JPH::Square( Motion().GetMaxLinearVelocity() );
	}

	JPH::Vec3 ToLocal( const VJoltSoftBodyVector &value ) const
	{
		return body->GetRotation().Conjugated() * ToJolt( value );
	}
};

bool IsIdle( const JoltPhysicsEnvironment *environment )
{
	return !environment->IsInSimulation() && !environment->IsInContactCallback();
}

template <typename Value>
bool IsValidRange( uint32_t first, uint32_t count, size_t size, const Value *output )
{
	return first <= size && count <= size - first && ( count == 0 || output != nullptr );
}

class SoftBodyBridge final : public IVJoltSoftBodyBridge, public IVJoltSoftBodyTools
{
public:
	uint32_t GetVersion() const override
	{
		return kVJoltSoftBodyBridgeVersion;
	}

	VJoltSoftBodyHandle Create( int32_t environmentIndex, const VJoltSoftBodyMesh &mesh,
		const VJoltSoftBodySettings &settings, VJoltSoftBodyError *error ) override
	{
		const auto fail = [error]( VJoltSoftBodyError reason ) -> VJoltSoftBodyHandle
		{
			if ( error )
			{
				*error = reason;
			}
			return 0;
		};
		auto *environment = static_cast<JoltPhysicsEnvironment *>(
			JoltPhysicsInterface::GetInstance().GetActiveEnvironmentByIndex( environmentIndex ) );
		if ( !environment )
		{
			return fail( VJoltSoftBodyError::InvalidEnvironment );
		}
		if ( !IsIdle( environment ) )
		{
			return fail( VJoltSoftBodyError::SimulationBusy );
		}
		if ( !ValidateSettings( settings ) )
		{
			return fail( VJoltSoftBodyError::InvalidSettings );
		}
		if ( m_Bodies.size() >= kVJoltSoftBodyMaxBodies
			|| mesh.vertexCount > kVJoltSoftBodyMaxTotalVertices - m_VertexCount
			|| m_NextHandle == std::numeric_limits<VJoltSoftBodyHandle>::max() )
		{
			return fail( VJoltSoftBodyError::LimitExceeded );
		}

		try
		{
			auto record = std::make_unique<SoftBodyRecord>();
			record->environment = environment;
			const auto prepared = PrepareMesh( mesh, settings, record->mesh );
			if ( prepared != VJoltSoftBodyError::None )
			{
				return fail( prepared );
			}

			CreateNativeBody( *record, mesh, settings );
			if ( !record->body )
			{
				return fail( VJoltSoftBodyError::AllocationFailed );
			}
			const auto handle = m_NextHandle++;
			auto *body = record->body;
			m_Bodies.emplace( handle, std::move( record ) );
			m_VertexCount += mesh.vertexCount;
			environment->GetPhysicsSystem()->GetBodyInterfaceNoLock().AddBody(
				body->GetID(), JPH::EActivation::Activate );
			if ( error )
			{
				*error = VJoltSoftBodyError::None;
			}
			return handle;
		}
		catch ( const std::bad_alloc & )
		{
			return fail( VJoltSoftBodyError::AllocationFailed );
		}
	}

	bool IsValid( VJoltSoftBodyHandle handle ) const override
	{
		return Find( handle ) != nullptr;
	}

	bool Destroy( VJoltSoftBodyHandle handle ) override
	{
		auto *record = Find( handle );
		if ( !record )
		{
			return false;
		}
		record->retiring = true;
		if ( record->environment->DeferPhysicsMutation( [this, handle]() { Remove( handle ); } ) )
		{
			return true;
		}
		if ( IsIdle( record->environment ) )
		{
			Remove( handle );
		}
		return true;
	}

	bool GetState( VJoltSoftBodyHandle handle, VJoltSoftBodyState &state ) const override
	{
		const auto *record = FindIdle( handle );
		if ( !record || state.size != sizeof( state ) )
		{
			return false;
		}
		const auto &motion = record->Motion();
		const auto &bounds = record->body->GetWorldSpaceBounds();
		state = {};
		state.vertexCount = static_cast<uint32_t>( motion.GetVertices().size() );
		state.triangleCount = static_cast<uint32_t>( motion.GetFaces().size() );
		state.pinnedCount = record->mesh.pinnedCount;
		state.flags = ( record->body->IsActive() ? VJoltSoftBodyActive : 0u )
			| ( record->mesh.closed ? VJoltSoftBodyClosed : 0u );
		state.mass = record->mesh.totalMass;
		state.pressure = JoltToSource::Energy( motion.GetPressure() );
		state.gravityFactor = motion.GetGravityFactor();
		state.volume = record->mesh.closed ? JoltToSource::Volume( motion.GetVolume() ) : 0.0f;
		state.simulationTime = record->environment->GetSimulationTime();
		state.position = ToSource( JPH::Vec3( record->body->GetCenterOfMassPosition() ) );
		state.boundsMin = ToSource( bounds.mMin );
		state.boundsMax = ToSource( bounds.mMax );
		return true;
	}

	bool GetVertices(
		VJoltSoftBodyHandle handle, uint32_t first, uint32_t count, VJoltSoftBodyVertexState *output ) const override
	{
		const auto *record = FindIdle( handle );
		if ( !record || !IsValidRange( first, count, record->mesh.inverseMasses.size(), output ) )
		{
			return false;
		}
		const auto transform = record->body->GetCenterOfMassTransform();
		const auto &vertices = record->Motion().GetVertices();
		for ( uint32_t offset = 0; offset < count; ++offset )
		{
			const auto index = first + offset;
			const auto &vertex = vertices[index];
			output[offset] = { ToSource( JPH::Vec3( transform * vertex.mPosition ) ),
				ToSource( transform.Multiply3x3( vertex.mVelocity ) ), 1.0f / record->mesh.inverseMasses[index],
				vertex.mInvMass == 0.0f ? 1u : 0u };
		}
		return true;
	}

	bool GetTriangles(
		VJoltSoftBodyHandle handle, uint32_t first, uint32_t count, VJoltSoftBodyTriangle *output ) const override
	{
		const auto *record = FindIdle( handle );
		if ( !record || !IsValidRange( first, count, record->mesh.settings->mFaces.size(), output ) )
		{
			return false;
		}
		for ( uint32_t offset = 0; offset < count; ++offset )
		{
			const auto &face = record->mesh.settings->mFaces[first + offset];
			output[offset] = { face.mVertex[0], face.mVertex[1], face.mVertex[2] };
		}
		return true;
	}

	bool SetPressure( VJoltSoftBodyHandle handle, float pressure ) override
	{
		auto *record = FindIdle( handle );
		if ( !record || !IsFiniteWithin( pressure, 0.0f, kMaxPressure ) || ( pressure > 0.0f && !record->mesh.closed ) )
		{
			return false;
		}
		record->Motion().SetPressure( SourceToJolt::Energy( pressure ) );
		record->Wake();
		return true;
	}

	bool SetGravityFactor( VJoltSoftBodyHandle handle, float factor ) override
	{
		auto *record = FindIdle( handle );
		if ( !record || !IsFiniteWithin( factor, -100.0f, 100.0f ) )
		{
			return false;
		}
		record->Motion().SetGravityFactor( factor );
		record->Wake();
		return true;
	}

	bool SetVertexPinned( VJoltSoftBodyHandle handle, uint32_t index, bool pinned ) override
	{
		auto *record = FindIdle( handle );
		if ( !record || index >= record->mesh.inverseMasses.size() )
		{
			return false;
		}
		auto &motion = record->Motion();
		auto &vertex = motion.GetVertex( index );
		const bool wasPinned = vertex.mInvMass == 0.0f;
		if ( wasPinned != pinned )
		{
			record->mesh.pinnedCount = pinned ? record->mesh.pinnedCount + 1 : record->mesh.pinnedCount - 1;
		}
		vertex.mInvMass = pinned ? 0.0f : record->mesh.inverseMasses[index];
		vertex.mVelocity = JPH::Vec3::sZero();
		motion.CalculateMassAndInertia();
		record->Wake();
		return true;
	}

	bool SetVertexVelocity( VJoltSoftBodyHandle handle, uint32_t index, const VJoltSoftBodyVector &velocity ) override
	{
		auto *record = FindIdle( handle );
		if ( !record || index >= record->mesh.inverseMasses.size() || !IsValidVector( velocity ) )
		{
			return false;
		}
		const auto localVelocity = record->ToLocal( velocity );
		if ( !record->AcceptsVelocity( localVelocity ) )
		{
			return false;
		}
		record->Motion().GetVertex( index ).mVelocity = localVelocity;
		record->Wake();
		return true;
	}

	bool SetVelocity( VJoltSoftBodyHandle handle, const VJoltSoftBodyVector &velocity ) override
	{
		auto *record = FindIdle( handle );
		if ( !record || !IsValidVector( velocity ) || !record->AcceptsVelocity( record->ToLocal( velocity ) ) )
		{
			return false;
		}
		const auto localVelocity = record->ToLocal( velocity );
		for ( auto &vertex : record->Motion().GetVertices() )
		{
			if ( vertex.mInvMass > 0.0f )
			{
				vertex.mVelocity = localVelocity;
			}
		}
		record->Wake();
		return true;
	}

	bool AddImpulse( VJoltSoftBodyHandle handle, const VJoltSoftBodyVector &impulse ) override
	{
		auto *record = FindIdle( handle );
		if ( !record || !IsValidVector( impulse, 1.0e9f ) )
		{
			return false;
		}
		auto &vertices = record->Motion().GetVertices();
		float movingMass = 0.0f;
		for ( const auto &vertex : vertices )
		{
			if ( vertex.mInvMass > 0.0f )
			{
				movingMass += 1.0f / vertex.mInvMass;
			}
		}
		if ( movingMass == 0.0f )
		{
			return false;
		}
		const auto deltaVelocity = record->ToLocal( impulse ) / movingMass;
		for ( const auto &vertex : vertices )
		{
			if ( vertex.mInvMass > 0.0f && !record->AcceptsVelocity( vertex.mVelocity + deltaVelocity ) )
			{
				return false;
			}
		}
		for ( auto &vertex : vertices )
		{
			if ( vertex.mInvMass > 0.0f )
			{
				vertex.mVelocity += deltaVelocity;
			}
		}
		record->Wake();
		return true;
	}

	bool AddVertexImpulse( VJoltSoftBodyHandle handle, uint32_t index, const VJoltSoftBodyVector &impulse ) override
	{
		auto *record = FindIdle( handle );
		if ( !record || index >= record->mesh.inverseMasses.size() || !IsValidVector( impulse, 1.0e9f ) )
		{
			return false;
		}
		auto &vertex = record->Motion().GetVertex( index );
		const auto velocity = vertex.mVelocity + record->ToLocal( impulse ) * vertex.mInvMass;
		if ( vertex.mInvMass == 0.0f || !record->AcceptsVelocity( velocity ) )
		{
			return false;
		}
		vertex.mVelocity = velocity;
		record->Wake();
		return true;
	}

	bool Wake( VJoltSoftBodyHandle handle ) override
	{
		auto *record = FindIdle( handle );
		if ( !record )
		{
			return false;
		}
		record->Wake();
		return true;
	}

	bool Trace( VJoltSoftBodyHandle handle, const VJoltSoftBodyRay &ray, VJoltSoftBodyHit &hit ) const override
	{
		const auto *record = FindIdle( handle );
		try
		{
			return record && VJoltTraceSoftBody( *record->body, record->mesh.closed, ray, hit );
		}
		catch ( const std::bad_alloc & )
		{
			return false;
		}
	}

	bool SetPlasticity( VJoltSoftBodyHandle handle, const VJoltSoftBodyPlasticity &settings ) override
	{
		auto *record = FindIdle( handle );
		if ( !record || settings.size != sizeof( settings )
			|| !IsFiniteWithin( settings.yield, 0.001f, 1.0f )
			|| !IsFiniteWithin( settings.creep, 0.0f, 100.0f )
			|| !IsFiniteWithin( settings.maxStrain, 0.0f, 0.95f ) )
		{
			return false;
		}
		if ( record->originalEdgeLengths.empty() && settings.creep > 0 )
		{
			try
			{
				std::vector<float> lengths, angles;
				lengths.reserve( record->mesh.settings->mEdgeConstraints.size() );
				for ( const auto &edge : record->mesh.settings->mEdgeConstraints )
				{
					lengths.push_back( edge.mRestLength );
				}
				angles.reserve( record->mesh.settings->mDihedralBendConstraints.size() );
				for ( const auto &bend : record->mesh.settings->mDihedralBendConstraints )
				{
					angles.push_back( bend.mInitialAngle );
				}
				record->originalEdgeLengths = std::move( lengths );
				record->originalBendAngles = std::move( angles );
			}
			catch ( const std::bad_alloc & )
			{
				return false;
			}
		}
		record->plasticity = settings;
		return true;
	}

	bool AddImpulseAt( VJoltSoftBodyHandle handle, const VJoltSoftBodyVector &position,
		const VJoltSoftBodyVector &impulse, float radius ) override
	{
		auto *record = FindIdle( handle );
		if ( !record || !IsValidVector( position ) || !IsValidVector( impulse, 1.0e9f )
			|| !IsFiniteWithin( radius, 0.01f, 4096.0f ) )
		{
			return false;
		}
		const JPH::Vec3 local = record->body->GetInverseCenterOfMassTransform() * ToJolt( position );
		const float inverseRadius = 1.0f / SourceToJolt::Distance( radius );
		const auto weight = [&]( const JPH::SoftBodyVertex &vertex )
		{
			return vertex.mInvMass > 0 ? std::max( 0.0f, 1.0f - ( vertex.mPosition - local ).Length() * inverseRadius ) : 0.0f;
		};
		auto &vertices = record->Motion().GetVertices();
		float totalWeight = 0;
		for ( const auto &vertex : vertices )
		{
			totalWeight += weight( vertex );
		}
		if ( totalWeight <= 0 )
		{
			return false;
		}
		const auto localImpulse = record->ToLocal( impulse ) / totalWeight;
		for ( const auto &vertex : vertices )
		{
			if ( !record->AcceptsVelocity( vertex.mVelocity + localImpulse * ( weight( vertex ) * vertex.mInvMass ) ) )
			{
				return false;
			}
		}
		for ( auto &vertex : vertices )
		{
			vertex.mVelocity += localImpulse * ( weight( vertex ) * vertex.mInvMass );
		}
		record->Wake();
		return true;
	}

	void AfterSimulation( JoltPhysicsEnvironment *environment, float deltaTime )
	{
		for ( auto &entry : m_Bodies )
		{
			auto &record = *entry.second;
			const auto &settings = record.plasticity;
			if ( record.environment != environment || record.retiring || settings.creep == 0 || !record.body->IsActive() )
			{
				continue;
			}
			// These settings belong to exactly one body. Update rest lengths only
			// after Jolt has joined its workers; constraint order/topology is fixed.
			const float relaxation = 1.0f - std::exp( -settings.creep * deltaTime );
			const auto &vertices = record.Motion().GetVertices();
			auto &edges = record.mesh.settings->mEdgeConstraints;
			for ( size_t index = 0; index < edges.size(); ++index )
			{
				auto &edge = edges[index];
				const float original = record.originalEdgeLengths[index];
				const float length = ( vertices[edge.mVertex[0]].mPosition - vertices[edge.mVertex[1]].mPosition ).Length();
				const float difference = length - edge.mRestLength;
				const float elastic = settings.yield * edge.mRestLength;
				if ( std::abs( difference ) > elastic && std::isfinite( length ) )
				{
					const float permanent = difference - std::copysign( elastic, difference );
					edge.mRestLength = std::clamp( edge.mRestLength + permanent * relaxation,
						original * ( 1.0f - settings.maxStrain ), original * ( 1.0f + settings.maxStrain ) );
				}
			}
			auto &bends = record.mesh.settings->mDihedralBendConstraints;
			const float elasticAngle = settings.yield * JPH::JPH_PI;
			const float maximumAngle = settings.maxStrain * JPH::JPH_PI;
			for ( size_t index = 0; index < bends.size(); ++index )
			{
				auto &bend = bends[index];
				const auto &x0 = vertices[bend.mVertex[0]].mPosition;
				const auto edge = vertices[bend.mVertex[1]].mPosition - x0;
				const auto n1 = edge.Cross( vertices[bend.mVertex[2]].mPosition - x0 );
				const auto n2 = ( vertices[bend.mVertex[3]].mPosition - x0 ).Cross( edge );
				const float normalProduct = n1.LengthSq() * n2.LengthSq();
				if ( edge.LengthSq() < 1.0e-12f || normalProduct < 1.0e-24f || !std::isfinite( normalProduct ) )
				{
					continue;
				}
				// Match Jolt's signed dihedral angle and wrap across the -PI/PI seam.
				const float angle = JPH::Sign( n2.Cross( n1 ).Dot( edge ) )
					* JPH::ACosApproximate( std::clamp( n1.Dot( n2 ) / std::sqrt( normalProduct ), -1.0f, 1.0f ) );
				const float difference = WrapBendAngle( angle - bend.mInitialAngle );
				if ( std::abs( difference ) > elasticAngle && std::isfinite( angle ) )
				{
					const float original = record.originalBendAngles[index];
					const float current = WrapBendAngle( bend.mInitialAngle - original );
					const float permanent = difference - std::copysign( elasticAngle, difference );
					bend.mInitialAngle = WrapBendAngle( original + std::clamp( current + permanent * relaxation,
						-maximumAngle, maximumAngle ) );
				}
			}
		}
	}

	void BeforeSimulation( JoltPhysicsEnvironment *environment )
	{
		RemoveMatching( [environment]( const SoftBodyRecord &record )
			{ return record.environment == environment && record.retiring; } );
	}

	void EnvironmentDestroyed( JoltPhysicsEnvironment *environment )
	{
		RemoveMatching( [environment]( const SoftBodyRecord &record ) { return record.environment == environment; } );
	}

private:
	static void CreateNativeBody(
		SoftBodyRecord &record, const VJoltSoftBodyMesh &mesh, const VJoltSoftBodySettings &settings )
	{
		const auto rotation = SourceToJolt::Angle( QAngle( settings.angles.x, settings.angles.y, settings.angles.z ) );
		JPH::SoftBodyCreationSettings creation(
			record.mesh.settings, ToJolt( settings.position ), rotation, Layers::MOVING );
		creation.mNumIterations = settings.iterations;
		creation.mPressure = SourceToJolt::Energy( settings.pressure );
		creation.mGravityFactor = settings.gravityFactor;
		creation.mLinearDamping = settings.damping;
		creation.mFriction = settings.friction;
		creation.mRestitution = settings.restitution;
		creation.mVertexRadius = SourceToJolt::Distance( settings.vertexRadius );
		creation.mMaxLinearVelocity = SourceToJolt::Distance( settings.maxVelocity );
		creation.mAllowSleeping = ( settings.flags & VJoltSoftBodyAllowSleeping ) != 0;
		creation.mFacesDoubleSided = ( settings.flags & VJoltSoftBodyDoubleSided ) != 0;
		// UserData stays zero: this body has no Source IPhysicsObject wrapper.
		record.body = record.environment->GetPhysicsSystem()->GetBodyInterfaceNoLock().CreateSoftBody( creation );
		if ( !record.body )
		{
			return;
		}
		auto &motion = record.Motion();
		for ( uint32_t index = 0; index < mesh.vertexCount; ++index )
		{
			auto &vertex = motion.GetVertex( index );
			if ( mesh.vertices[index].pinned )
			{
				vertex.mInvMass = 0.0f;
			}
			else
			{
				vertex.mVelocity = record.ToLocal( settings.velocity );
			}
		}
		motion.CalculateMassAndInertia();
	}

	SoftBodyRecord *Find( VJoltSoftBodyHandle handle ) const
	{
		const auto iterator = m_Bodies.find( handle );
		return iterator != m_Bodies.end() && !iterator->second->retiring ? iterator->second.get() : nullptr;
	}

	SoftBodyRecord *FindIdle( VJoltSoftBodyHandle handle ) const
	{
		auto *record = Find( handle );
		return record && IsIdle( record->environment ) ? record : nullptr;
	}

	void Remove( VJoltSoftBodyHandle handle )
	{
		const auto iterator = m_Bodies.find( handle );
		if ( iterator != m_Bodies.end() )
		{
			m_VertexCount -= static_cast<uint32_t>( iterator->second->mesh.inverseMasses.size() );
			m_Bodies.erase( iterator );
		}
	}

	template <typename Predicate>
	void RemoveMatching( Predicate matches )
	{
		for ( auto iterator = m_Bodies.begin(); iterator != m_Bodies.end(); )
		{
			const auto current = iterator++;
			if ( matches( *current->second ) )
			{
				Remove( current->first );
			}
		}
	}

	std::unordered_map<VJoltSoftBodyHandle, std::unique_ptr<SoftBodyRecord>> m_Bodies;
	VJoltSoftBodyHandle m_NextHandle = 1;
	uint32_t m_VertexCount = 0;
};

SoftBodyBridge g_SoftBodyBridge;
} // namespace

EXPOSE_SINGLE_INTERFACE_GLOBALVAR(
	SoftBodyBridge, IVJoltSoftBodyBridge, VJOLT_SOFTBODY_BRIDGE_VERSION, g_SoftBodyBridge );

EXPOSE_SINGLE_INTERFACE_GLOBALVAR(
	SoftBodyBridge, IVJoltSoftBodyTools, VJOLT_SOFTBODY_TOOLS_VERSION, g_SoftBodyBridge );

void VJoltSoftBodyBeforeSimulation( JoltPhysicsEnvironment *environment )
{
	g_SoftBodyBridge.BeforeSimulation( environment );
}

void VJoltSoftBodyEnvironmentDestroyed( JoltPhysicsEnvironment *environment )
{
	g_SoftBodyBridge.EnvironmentDestroyed( environment );
}

void VJoltSoftBodyAfterSimulation( JoltPhysicsEnvironment *environment, float deltaTime )
{
	g_SoftBodyBridge.AfterSimulation( environment, deltaTime );
}
