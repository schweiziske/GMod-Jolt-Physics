#include "cbase.h"
#include "bridge/vjolt_softbody_mesh.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <set>
#include <unordered_map>

#include "tier0/memdbgon.h"

namespace VJoltSoftBody
{
bool IsFiniteWithin( float value, float minimum, float maximum )
{
	return std::isfinite( value ) && value >= minimum && value <= maximum;
}

bool IsValidVector( const VJoltSoftBodyVector &value, float limit )
{
	return IsFiniteWithin( value.x, -limit, limit ) && IsFiniteWithin( value.y, -limit, limit )
		&& IsFiniteWithin( value.z, -limit, limit );
}

JPH::Vec3 ToJolt( const VJoltSoftBodyVector &value )
{
	return SourceToJolt::Distance( Vector( value.x, value.y, value.z ) );
}

VJoltSoftBodyVector ToSource( JPH::Vec3Arg value )
{
	const Vector source = JoltToSource::Distance( value );
	return { source.x, source.y, source.z };
}

bool ValidateSettings( const VJoltSoftBodySettings &settings )
{
	constexpr uint32_t supportedFlags = VJoltSoftBodyAllowSleeping | VJoltSoftBodyDoubleSided | VJoltSoftBodyDihedralBend;
	return settings.size == sizeof( settings ) && ( settings.flags & ~supportedFlags ) == 0 && settings.iterations >= 1
		&& settings.iterations <= 32 && IsValidVector( settings.position ) && IsValidVector( settings.angles, 36000.0f )
		&& IsValidVector( settings.velocity, settings.maxVelocity )
		&& ToJolt( settings.velocity ).LengthSq() <= JPH::Square( SourceToJolt::Distance( settings.maxVelocity ) )
		&& IsFiniteWithin( settings.edgeCompliance, 0.0f, 1.0f )
		&& IsFiniteWithin( settings.bendCompliance, 0.0f, 1.0f )
		&& IsFiniteWithin( settings.pressure, 0.0f, kMaxPressure )
		&& IsFiniteWithin( settings.gravityFactor, -100.0f, 100.0f ) && IsFiniteWithin( settings.damping, 0.0f, 10.0f )
		&& IsFiniteWithin( settings.friction, 0.0f, 1.0f ) && IsFiniteWithin( settings.restitution, 0.0f, 1.0f )
		&& IsFiniteWithin( settings.vertexRadius, 0.0f, 64.0f )
		&& IsFiniteWithin( settings.maxVelocity, 1.0f, 100000.0f );
}

namespace
{
struct EdgeUse
{
	uint32_t count = 0;
	int direction = 0;
};

class MeshTopology
{
public:
	explicit MeshTopology( uint32_t vertexCount ) : m_Parents( vertexCount ), m_Used( vertexCount, false )
	{
		std::iota( m_Parents.begin(), m_Parents.end(), 0u );
	}

	bool AddTriangle( const VJoltSoftBodyTriangle &triangle )
	{
		std::array<uint32_t, 3> sorted{ triangle.vertices[0], triangle.vertices[1], triangle.vertices[2] };
		std::sort( sorted.begin(), sorted.end() );
		if ( !m_Triangles.insert( sorted ).second )
		{
			return false;
		}

		for ( uint32_t corner = 0; corner < 3; ++corner )
		{
			const uint32_t first = triangle.vertices[corner];
			const uint32_t second = triangle.vertices[( corner + 1 ) % 3];
			const uint64_t key = uint64_t( std::min( first, second ) ) << 32 | std::max( first, second );
			auto &edge = m_Edges[key];
			++edge.count;
			edge.direction += first < second ? 1 : -1;
			if ( edge.count > 2 || ( edge.count == 2 && edge.direction != 0 ) )
			{
				return false;
			}
			m_Used[first] = true;
			m_Parents[Root( first )] = Root( second );
		}
		return true;
	}

	bool IsConnected()
	{
		const uint32_t root = Root( 0 );
		for ( uint32_t vertex = 0; vertex < m_Parents.size(); ++vertex )
		{
			if ( !m_Used[vertex] || Root( vertex ) != root )
			{
				return false;
			}
		}
		return true;
	}

	bool IsClosed() const
	{
		return std::all_of(
			m_Edges.begin(), m_Edges.end(), []( const auto &entry ) { return entry.second.count == 2; } );
	}

private:
	uint32_t Root( uint32_t vertex )
	{
		while ( m_Parents[vertex] != vertex )
		{
			m_Parents[vertex] = m_Parents[m_Parents[vertex]];
			vertex = m_Parents[vertex];
		}
		return vertex;
	}

	std::vector<uint32_t> m_Parents;
	std::vector<bool> m_Used;
	std::set<std::array<uint32_t, 3>> m_Triangles;
	std::unordered_map<uint64_t, EdgeUse> m_Edges;
};

bool HasTriangleArea( const JPH::Vec3 &first, const JPH::Vec3 &second, const JPH::Vec3 &third )
{
	const auto edge1 = second - first;
	const auto edge2 = third - first;
	const float minimumEdgeSquared = JPH::Square( SourceToJolt::Distance( 0.01f ) );
	return edge1.LengthSq() >= minimumEdgeSquared && edge2.LengthSq() >= minimumEdgeSquared
		&& ( third - second ).LengthSq() >= minimumEdgeSquared
		&& edge1.Cross( edge2 ).LengthSq() > 1.0e-12f * edge1.LengthSq() * edge2.LengthSq();
}
} // namespace

VJoltSoftBodyError PrepareMesh(
	const VJoltSoftBodyMesh &mesh, const VJoltSoftBodySettings &settings, PreparedMesh &prepared )
{
	if ( mesh.size != sizeof( mesh ) || !mesh.vertices || !mesh.triangles || mesh.vertexCount < 3
		|| mesh.triangleCount == 0 )
	{
		return VJoltSoftBodyError::InvalidMesh;
	}
	if ( mesh.vertexCount > kVJoltSoftBodyMaxVertices || mesh.triangleCount > kVJoltSoftBodyMaxTriangles )
	{
		return VJoltSoftBodyError::LimitExceeded;
	}

	const auto rotation = SourceToJolt::Angle( QAngle( settings.angles.x, settings.angles.y, settings.angles.z ) );
	prepared.settings = new JPH::SoftBodySharedSettings;
	prepared.settings->mVertices.reserve( mesh.vertexCount );
	prepared.inverseMasses.reserve( mesh.vertexCount );
	for ( uint32_t index = 0; index < mesh.vertexCount; ++index )
	{
		const auto &vertex = mesh.vertices[index];
		if ( !IsValidVector( vertex.position ) || !IsFiniteWithin( vertex.mass, 0.001f, 10000.0f )
			|| vertex.pinned > 1 )
		{
			return VJoltSoftBodyError::InvalidVertex;
		}
		const auto position = ToJolt( vertex.position );
		if ( !IsValidVector( ToSource( ToJolt( settings.position ) + rotation * position ) ) )
		{
			return VJoltSoftBodyError::InvalidVertex;
		}

		JPH::Float3 localPosition;
		position.StoreFloat3( &localPosition );
		const float inverseMass = 1.0f / vertex.mass;
		// Build the complete constraint graph so initially pinned vertices can be
		// released later. Pinning is applied to the runtime vertices after creation.
		prepared.settings->mVertices.emplace_back( localPosition, JPH::Float3( 0, 0, 0 ), inverseMass );
		prepared.inverseMasses.push_back( inverseMass );
		prepared.totalMass += vertex.mass;
		prepared.pinnedCount += vertex.pinned;
	}

	MeshTopology topology( mesh.vertexCount );
	double sixVolume = 0.0;
	const JPH::DVec3 volumeOrigin( ToJolt( mesh.vertices[0].position ) );
	for ( uint32_t index = 0; index < mesh.triangleCount; ++index )
	{
		const auto &triangle = mesh.triangles[index];
		const auto *vertices = triangle.vertices;
		if ( vertices[0] >= mesh.vertexCount || vertices[1] >= mesh.vertexCount || vertices[2] >= mesh.vertexCount
			|| vertices[0] == vertices[1] || vertices[1] == vertices[2] || vertices[2] == vertices[0] )
		{
			return VJoltSoftBodyError::InvalidTriangle;
		}
		const auto first = ToJolt( mesh.vertices[vertices[0]].position );
		const auto second = ToJolt( mesh.vertices[vertices[1]].position );
		const auto third = ToJolt( mesh.vertices[vertices[2]].position );
		if ( !HasTriangleArea( first, second, third ) )
		{
			return VJoltSoftBodyError::InvalidTriangle;
		}
		if ( !topology.AddTriangle( triangle ) )
		{
			return VJoltSoftBodyError::InvalidTopology;
		}
		const auto relativeFirst = JPH::DVec3( first ) - volumeOrigin;
		const auto relativeSecond = JPH::DVec3( second ) - volumeOrigin;
		const auto relativeThird = JPH::DVec3( third ) - volumeOrigin;
		sixVolume += relativeFirst.Cross( relativeSecond ).Dot( relativeThird );
		prepared.settings->AddFace( { vertices[0], vertices[1], vertices[2] } );
	}
	if ( !topology.IsConnected() )
	{
		return VJoltSoftBodyError::InvalidTopology;
	}
	prepared.closed = topology.IsClosed();
	if ( prepared.closed && sixVolume <= SourceToJolt::Volume( 0.000001f ) )
	{
		return VJoltSoftBodyError::InvalidTopology;
	}
	if ( settings.pressure > 0.0f && !prepared.closed )
	{
		return VJoltSoftBodyError::PressureRequiresClosedMesh;
	}

	const JPH::SoftBodySharedSettings::VertexAttributes attributes(
		settings.edgeCompliance, settings.edgeCompliance, settings.bendCompliance );
	const auto bendType = settings.flags & VJoltSoftBodyDihedralBend
		? JPH::SoftBodySharedSettings::EBendType::Dihedral : JPH::SoftBodySharedSettings::EBendType::Distance;
	prepared.settings->CreateConstraints( &attributes, 1, bendType );
	prepared.settings->Optimize();
	return VJoltSoftBodyError::None;
}
} // namespace VJoltSoftBody
