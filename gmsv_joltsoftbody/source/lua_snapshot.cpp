#include "lua_snapshot.h"
#include "lua_mesh.h"

#include <algorithm>
#include <array>

namespace JoltSoftBodyLua
{
using namespace GarrysMod::Lua;

namespace
{
int Failure( ILua *lua, const char *reason )
{
	lua->PushNil();
	lua->PushString( reason );
	return 2;
}

void RawNumber( ILua *lua, int table, const char *name, double value )
{
	lua->PushString( name );
	lua->PushNumber( value );
	lua->RawSet( table );
}

void RawBoolean( ILua *lua, int table, const char *name, bool value )
{
	lua->PushString( name );
	lua->PushBool( value );
	lua->RawSet( table );
}

void ArrayNumber( ILua *lua, int table, uint32_t index, float value )
{
	lua->PushNumber( index );
	lua->PushNumber( value );
	lua->RawSet( table );
}

bool PushArray( ILua *lua, int table, const char *name )
{
	lua->PushString( name );
	lua->RawGet( table );
	return lua->IsType( -1, Type::Table );
}

void WriteState( ILua *lua, const VJoltSoftBodyState &state )
{
	RawNumber( lua, 2, "vertexCount", state.vertexCount );
	RawNumber( lua, 2, "triangleCount", state.triangleCount );
	RawNumber( lua, 2, "pinnedCount", state.pinnedCount );
	RawNumber( lua, 2, "simulationTime", state.simulationTime );
	RawNumber( lua, 2, "minX", state.boundsMin.x );
	RawNumber( lua, 2, "minY", state.boundsMin.y );
	RawNumber( lua, 2, "minZ", state.boundsMin.z );
	RawNumber( lua, 2, "maxX", state.boundsMax.x );
	RawNumber( lua, 2, "maxY", state.boundsMax.y );
	RawNumber( lua, 2, "maxZ", state.boundsMax.z );
	RawNumber( lua, 2, "pressure", state.pressure );
	RawBoolean( lua, 2, "active", ( state.flags & VJoltSoftBodyActive ) != 0 );
}
} // namespace

int ReadStateIntoTable( ILua *lua, IVJoltSoftBodyBridge *bridge, VJoltSoftBodyHandle handle )
{
	VJoltSoftBodyState state;
	if ( !lua->IsType( 2, Type::Table ) || !bridge || !bridge->GetState( handle, state ) )
	{
		return Failure( lua, "ReadState expects a live body and a reusable output table outside simulation" );
	}
	WriteState( lua, state );
	lua->PushBool( true );
	return 1;
}

int ReadSnapshotIntoTable( ILua *lua, IVJoltSoftBodyBridge *bridge, VJoltSoftBodyHandle handle )
{
	if ( !lua->IsType( 2, Type::Table ) )
	{
		return Failure( lua, "ReadSnapshot expects {positions = {}, pinned = {}} to reuse" );
	}
	VJoltSoftBodyState state;
	if ( !bridge || !bridge->GetState( handle, state ) )
	{
		return Failure( lua, "cannot read a retired soft body or a world that is stepping" );
	}
	if ( !PushArray( lua, 2, "positions" ) )
	{
		return Failure( lua, "snapshot.positions must be a flat numeric table" );
	}
	const int positions = lua->Top();
	if ( !PushArray( lua, 2, "pinned" ) )
	{
		return Failure( lua, "snapshot.pinned must be a boolean table" );
	}
	const int pinned = lua->Top();
	const bool skipSleeping = lua->IsType( 3, Type::Bool ) && lua->GetBool( 3 );
	if ( skipSleeping && ( state.flags & VJoltSoftBodyActive ) == 0 )
	{
		lua->Pop( 2 );
		WriteState( lua, state );
		RawBoolean( lua, 2, "updated", false );
		lua->PushBool( true );
		return 1;
	}

	// The caller owns and reuses both arrays. Raw writes avoid __newindex, and
	// fixed native batches keep heap ownership out of Lua's allocation boundary.
	std::array<VJoltSoftBodyVertexState, 128> vertices;
	for ( uint32_t first = 0; first < state.vertexCount; )
	{
		const auto count = std::min( state.vertexCount - first, static_cast<uint32_t>( vertices.size() ) );
		if ( !bridge->GetVertices( handle, first, count, vertices.data() ) )
		{
			return Failure( lua, "native vertex snapshot failed" );
		}
		for ( uint32_t offset = 0; offset < count; ++offset )
		{
			const auto index = first + offset;
			const auto &vertex = vertices[offset];
			ArrayNumber( lua, positions, index * 3 + 1, vertex.position.x );
			ArrayNumber( lua, positions, index * 3 + 2, vertex.position.y );
			ArrayNumber( lua, positions, index * 3 + 3, vertex.position.z );
			lua->PushNumber( index + 1 );
			lua->PushBool( vertex.pinned != 0 );
			lua->RawSet( pinned );
		}
		first += count;
	}
	lua->Pop( 2 );
	WriteState( lua, state );
	RawBoolean( lua, 2, "updated", true );
	lua->PushBool( true );
	return 1;
}

int ReadVertexIntoTable( ILua *lua, IVJoltSoftBodyBridge *bridge, VJoltSoftBodyHandle handle )
{
	uint32_t index;
	if ( !ReadInteger( lua, 2, 1, kVJoltSoftBodyMaxVertices, index ) || !lua->IsType( 3, Type::Table ) )
	{
		return Failure( lua, "ReadVertex expects a one-based vertex index and a reusable output table" );
	}
	VJoltSoftBodyVertexState vertex;
	if ( !bridge || !bridge->GetVertices( handle, index - 1, 1, &vertex ) )
	{
		return Failure( lua, "cannot read vertex: invalid body, index, or simulation state" );
	}
	RawNumber( lua, 3, "x", vertex.position.x );
	RawNumber( lua, 3, "y", vertex.position.y );
	RawNumber( lua, 3, "z", vertex.position.z );
	RawNumber( lua, 3, "vx", vertex.velocity.x );
	RawNumber( lua, 3, "vy", vertex.velocity.y );
	RawNumber( lua, 3, "vz", vertex.velocity.z );
	RawNumber( lua, 3, "mass", vertex.mass );
	RawBoolean( lua, 3, "pinned", vertex.pinned != 0 );
	lua->PushBool( true );
	return 1;
}
} // namespace JoltSoftBodyLua
