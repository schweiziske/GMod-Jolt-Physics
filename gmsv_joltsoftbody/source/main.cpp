#include "lua_mesh.h"
#include "lua_snapshot.h"
#include <vjolt/loader.h>
#include <vjolt/softbody_tools.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <new>
#include <unordered_set>

using namespace GarrysMod::Lua;

namespace
{
using namespace JoltSoftBodyLua;

IVJoltSoftBodyBridge *g_Bridge = nullptr;
IVJoltSoftBodyTools *g_Tools = nullptr;
int g_SoftBodyMeta = 0;
std::unordered_set<VJoltSoftBodyHandle> g_OwnedBodies;

int PushFailure( ILua *lua, const char *reason )
{
	lua->PushNil();
	lua->PushString( g_Bridge ? reason : "matching VJoltSoftBody001 backend is not loaded" );
	return 2;
}

int PushResult( ILua *lua, bool result )
{
	if ( !result )
	{
		return PushFailure( lua, "invalid soft body, unsupported argument, or physics world is stepping" );
	}
	lua->PushBool( true );
	return 1;
}

VJoltSoftBodyHandle ReadHandle( ILua *lua )
{
	const auto *handle = lua->GetUserType<VJoltSoftBodyHandle>( 1, g_SoftBodyMeta );
	return handle ? *handle : 0;
}

void SetNumber( ILua *lua, const char *name, double value )
{
	lua->PushNumber( value );
	lua->SetField( -2, name );
}

void SetBoolean( ILua *lua, const char *name, bool value )
{
	lua->PushBool( value );
	lua->SetField( -2, name );
}

void SetVector( ILua *lua, const char *name, const VJoltSoftBodyVector &value )
{
	Vector vector;
	vector.x = value.x;
	vector.y = value.y;
	vector.z = value.z;
	lua->PushVector( vector );
	lua->SetField( -2, name );
}

void RegisterFunction( ILua *lua, const char *name, CFunc function )
{
	lua->PushCFunction( function );
	lua->SetField( -2, name );
}

bool ReadRange( ILua *lua, uint32_t size, uint32_t &first, uint32_t &count )
{
	first = 1;
	if ( lua->Top() >= 2 && !lua->IsType( 2, Type::Nil ) && !ReadInteger( lua, 2, 1, size + 1, first ) )
	{
		return false;
	}
	--first;
	count = size - first;
	return lua->Top() < 3 || lua->IsType( 3, Type::Nil ) || ReadInteger( lua, 3, 0, count, count );
}

LUA_FUNCTION_STATIC( Available )
{
	LUA->PushBool( g_Bridge != nullptr );
	return 1;
}

LUA_FUNCTION_STATIC( GetVersion )
{
	if ( g_Bridge )
	{
		LUA->PushNumber( g_Bridge->GetVersion() );
	}
	else
	{
		LUA->PushNil();
	}
	return 1;
}

LUA_FUNCTION_STATIC( GetLimits )
{
	LUA->CreateTable();
	SetNumber( LUA, "verticesPerBody", kVJoltSoftBodyMaxVertices );
	SetNumber( LUA, "trianglesPerBody", kVJoltSoftBodyMaxTriangles );
	SetNumber( LUA, "bodies", kVJoltSoftBodyMaxBodies );
	SetNumber( LUA, "totalVertices", kVJoltSoftBodyMaxTotalVertices );
	return 1;
}

LUA_FUNCTION_STATIC( Create )
{
	if ( !g_Bridge )
	{
		return PushFailure( LUA, "soft-body backend unavailable" );
	}
	try
	{
		CreationArguments arguments;
		const char *reason = nullptr;
		if ( !ReadCreationArguments( LUA, arguments, reason ) )
		{
			return PushFailure( LUA, reason );
		}
		VJoltSoftBodyError error;
		const auto handle = g_Bridge->Create( arguments.environment, arguments.Mesh(), arguments.settings, &error );
		if ( !handle )
		{
			return PushFailure( LUA, CreationError( error ) );
		}
		try
		{
			g_OwnedBodies.insert( handle );
		}
		catch ( const std::bad_alloc & )
		{
			g_Bridge->Destroy( handle );
			throw;
		}
		LUA->PushUserType_Value( handle, g_SoftBodyMeta );
		return 1;
	}
	catch ( const std::bad_alloc & )
	{
		return PushFailure( LUA, "unable to allocate soft-body mesh buffers" );
	}
}

LUA_FUNCTION_STATIC( IsValid )
{
	LUA->PushBool( g_Bridge && g_Bridge->IsValid( ReadHandle( LUA ) ) );
	return 1;
}

LUA_FUNCTION_STATIC( Destroy )
{
	auto *handle = LUA->GetUserType<VJoltSoftBodyHandle>( 1, g_SoftBodyMeta );
	if ( !handle || !*handle )
	{
		LUA->PushBool( false );
		return 1;
	}
	const bool destroyed = g_Bridge && g_Bridge->Destroy( *handle );
	g_OwnedBodies.erase( *handle );
	*handle = 0;
	LUA->PushBool( destroyed );
	return 1;
}

LUA_FUNCTION_STATIC( GetState )
{
	VJoltSoftBodyState state;
	if ( !g_Bridge || !g_Bridge->GetState( ReadHandle( LUA ), state ) )
	{
		return PushFailure( LUA, "cannot read a retired soft body or a world that is stepping" );
	}
	LUA->CreateTable();
	SetNumber( LUA, "vertexCount", state.vertexCount );
	SetNumber( LUA, "triangleCount", state.triangleCount );
	SetNumber( LUA, "pinnedCount", state.pinnedCount );
	SetBoolean( LUA, "active", ( state.flags & VJoltSoftBodyActive ) != 0 );
	SetBoolean( LUA, "closed", ( state.flags & VJoltSoftBodyClosed ) != 0 );
	SetNumber( LUA, "mass", state.mass );
	SetNumber( LUA, "pressure", state.pressure );
	SetNumber( LUA, "gravityFactor", state.gravityFactor );
	SetNumber( LUA, "volume", state.volume );
	SetNumber( LUA, "simulationTime", state.simulationTime );
	SetVector( LUA, "pos", state.position );
	SetVector( LUA, "boundsMin", state.boundsMin );
	SetVector( LUA, "boundsMax", state.boundsMax );
	return 1;
}

LUA_FUNCTION_STATIC( GetVertices )
{
	const auto handle = ReadHandle( LUA );
	VJoltSoftBodyState state;
	uint32_t first, count;
	if ( !g_Bridge || !g_Bridge->GetState( handle, state ) || !ReadRange( LUA, state.vertexCount, first, count ) )
	{
		return PushFailure( LUA, "cannot read vertices: invalid soft body, range, or simulation state" );
	}
	// Fixed-size batches avoid native heap buffers across Lua allocations/longjmp.
	std::array<VJoltSoftBodyVertexState, 128> vertices;
	LUA->CreateTable();
	for ( uint32_t offset = 0; offset < count; )
	{
		const uint32_t batch = std::min( count - offset, static_cast<uint32_t>( vertices.size() ) );
		if ( !g_Bridge->GetVertices( handle, first + offset, batch, vertices.data() ) )
		{
			return PushFailure( LUA, "native vertex snapshot failed" );
		}
		for ( uint32_t index = 0; index < batch; ++index )
		{
			LUA->PushNumber( offset + index + 1 );
			LUA->CreateTable();
			SetVector( LUA, "pos", vertices[index].position );
			SetVector( LUA, "velocity", vertices[index].velocity );
			SetNumber( LUA, "mass", vertices[index].mass );
			SetBoolean( LUA, "pinned", vertices[index].pinned != 0 );
			LUA->RawSet( -3 );
		}
		offset += batch;
	}
	return 1;
}

LUA_FUNCTION_STATIC( GetTriangles )
{
	const auto handle = ReadHandle( LUA );
	VJoltSoftBodyState state;
	uint32_t first, count;
	if ( !g_Bridge || !g_Bridge->GetState( handle, state ) || !ReadRange( LUA, state.triangleCount, first, count ) )
	{
		return PushFailure( LUA, "cannot read triangles: invalid soft body, range, or simulation state" );
	}
	std::array<VJoltSoftBodyTriangle, 128> triangles;
	LUA->CreateTable();
	for ( uint32_t offset = 0; offset < count; )
	{
		const uint32_t batch = std::min( count - offset, static_cast<uint32_t>( triangles.size() ) );
		if ( !g_Bridge->GetTriangles( handle, first + offset, batch, triangles.data() ) )
		{
			return PushFailure( LUA, "native triangle snapshot failed" );
		}
		for ( uint32_t index = 0; index < batch; ++index )
		{
			LUA->PushNumber( offset + index + 1 );
			LUA->CreateTable();
			for ( uint32_t corner = 0; corner < 3; ++corner )
			{
				LUA->PushNumber( corner + 1 );
				LUA->PushNumber( triangles[index].vertices[corner] + 1 );
				LUA->RawSet( -3 );
			}
			LUA->RawSet( -3 );
		}
		offset += batch;
	}
	return 1;
}

LUA_FUNCTION_STATIC( ReadSnapshot )
{
	return ReadSnapshotIntoTable( LUA, g_Bridge, ReadHandle( LUA ) );
}

LUA_FUNCTION_STATIC( ReadState )
{
	return ReadStateIntoTable( LUA, g_Bridge, ReadHandle( LUA ) );
}

LUA_FUNCTION_STATIC( ReadVertex )
{
	return ReadVertexIntoTable( LUA, g_Bridge, ReadHandle( LUA ) );
}

LUA_FUNCTION_STATIC( SetPressure )
{
	float pressure;
	return PushResult(
		LUA, g_Bridge && ReadNumber( LUA, 2, pressure ) && g_Bridge->SetPressure( ReadHandle( LUA ), pressure ) );
}

LUA_FUNCTION_STATIC( SetGravityFactor )
{
	float factor;
	return PushResult(
		LUA, g_Bridge && ReadNumber( LUA, 2, factor ) && g_Bridge->SetGravityFactor( ReadHandle( LUA ), factor ) );
}

LUA_FUNCTION_STATIC( SetVertexPinned )
{
	uint32_t vertex;
	return PushResult( LUA,
		g_Bridge && ReadInteger( LUA, 2, 1, kVJoltSoftBodyMaxVertices, vertex ) && LUA->IsType( 3, Type::Bool )
			&& g_Bridge->SetVertexPinned( ReadHandle( LUA ), vertex - 1, LUA->GetBool( 3 ) ) );
}

LUA_FUNCTION_STATIC( SetVertexVelocity )
{
	uint32_t vertex;
	VJoltSoftBodyVector velocity;
	return PushResult( LUA,
		g_Bridge && ReadInteger( LUA, 2, 1, kVJoltSoftBodyMaxVertices, vertex ) && ReadVector( LUA, 3, velocity )
			&& g_Bridge->SetVertexVelocity( ReadHandle( LUA ), vertex - 1, velocity ) );
}

LUA_FUNCTION_STATIC( SetVelocity )
{
	VJoltSoftBodyVector velocity;
	return PushResult(
		LUA, g_Bridge && ReadVector( LUA, 2, velocity ) && g_Bridge->SetVelocity( ReadHandle( LUA ), velocity ) );
}

LUA_FUNCTION_STATIC( AddImpulse )
{
	VJoltSoftBodyVector impulse;
	return PushResult(
		LUA, g_Bridge && ReadVector( LUA, 2, impulse ) && g_Bridge->AddImpulse( ReadHandle( LUA ), impulse ) );
}

LUA_FUNCTION_STATIC( AddVertexImpulse )
{
	uint32_t vertex;
	VJoltSoftBodyVector impulse;
	return PushResult( LUA,
		g_Bridge && ReadInteger( LUA, 2, 1, kVJoltSoftBodyMaxVertices, vertex ) && ReadVector( LUA, 3, impulse )
			&& g_Bridge->AddVertexImpulse( ReadHandle( LUA ), vertex - 1, impulse ) );
}

LUA_FUNCTION_STATIC( Wake )
{
	return PushResult( LUA, g_Bridge && g_Bridge->Wake( ReadHandle( LUA ) ) );
}

LUA_FUNCTION_STATIC( GetHandle )
{
	char text[32];
	std::snprintf( text, sizeof( text ), "%llu", static_cast<unsigned long long>( ReadHandle( LUA ) ) );
	LUA->PushString( text );
	return 1;
}

LUA_FUNCTION_STATIC( Trace )
{
	VJoltSoftBodyRay ray;
	VJoltSoftBodyHit hit;
	if ( !g_Tools || !ReadVector( LUA, 2, ray.start ) || !ReadVector( LUA, 3, ray.delta )
		|| ( LUA->Top() >= 4 && !LUA->IsType( 4, Type::Nil ) && !ReadVector( LUA, 4, ray.extents ) )
		|| !g_Tools->Trace( ReadHandle( LUA ), ray, hit ) )
	{
		return PushFailure( LUA, "soft-body trace unavailable, invalid ray, or physics world is stepping" );
	}
	if ( LUA->Top() >= 5 && LUA->IsType( 5, Type::Table ) )
	{
		LUA->Push( 5 );
	}
	else
	{
		LUA->CreateTable();
	}
	SetBoolean( LUA, "Hit", hit.flags != 0 );
	SetBoolean( LUA, "StartSolid", ( hit.flags & VJoltSoftBodyStartSolid ) != 0 );
	SetBoolean( LUA, "AllSolid", ( hit.flags & VJoltSoftBodyAllSolid ) != 0 );
	SetNumber( LUA, "Fraction", hit.fraction );
	SetNumber( LUA, "FractionLeftSolid", hit.fractionLeftSolid );
	SetNumber( LUA, "Triangle", hit.triangle == UINT32_MAX ? 0 : hit.triangle + 1 );
	SetNumber( LUA, "Vertex", hit.vertex == UINT32_MAX ? 0 : hit.vertex + 1 );
	SetVector( LUA, "HitPos", hit.position );
	SetVector( LUA, "Normal", hit.normal );
	SetVector( LUA, "HitNormal", hit.normal );
	return 1;
}

LUA_FUNCTION_STATIC( SetPlasticity )
{
	VJoltSoftBodyPlasticity settings;
	return PushResult( LUA, g_Tools && ReadNumber( LUA, 2, settings.yield )
		&& ReadNumber( LUA, 3, settings.creep ) && ReadNumber( LUA, 4, settings.maxStrain )
		&& g_Tools->SetPlasticity( ReadHandle( LUA ), settings ) );
}

LUA_FUNCTION_STATIC( AddImpulseAt )
{
	VJoltSoftBodyVector position;
	VJoltSoftBodyVector impulse;
	float radius;
	return PushResult( LUA, g_Tools && ReadVector( LUA, 2, position ) && ReadVector( LUA, 3, impulse )
		&& ReadNumber( LUA, 4, radius ) && g_Tools->AddImpulseAt( ReadHandle( LUA ), position, impulse, radius ) );
}
} // namespace

GMOD_MODULE_OPEN()
{
	const auto discovery = VJoltSDK::DiscoverVersionedInterface<IVJoltSoftBodyBridge>(
		VJOLT_SOFTBODY_BRIDGE_VERSION, kVJoltSoftBodyBridgeVersion );
	g_Bridge = discovery.api;
	g_Tools = VJoltSDK::QueryInterface<IVJoltSoftBodyTools>( discovery.module, VJOLT_SOFTBODY_TOOLS_VERSION );
	if ( g_Tools && g_Tools->GetVersion() != kVJoltSoftBodyToolsVersion )
	{
		g_Tools = nullptr;
	}
	g_SoftBodyMeta = LUA->CreateMetaTable( "JoltSoftBody" );
	LUA->Push( -1 );
	LUA->SetField( -2, "__index" );
	RegisterFunction( LUA, "__gc", Destroy );
	RegisterFunction( LUA, "IsValid", IsValid );
	RegisterFunction( LUA, "Destroy", Destroy );
	RegisterFunction( LUA, "GetState", GetState );
	RegisterFunction( LUA, "GetVertices", GetVertices );
	RegisterFunction( LUA, "GetTriangles", GetTriangles );
	RegisterFunction( LUA, "ReadSnapshot", ReadSnapshot );
	RegisterFunction( LUA, "ReadState", ReadState );
	RegisterFunction( LUA, "ReadVertex", ReadVertex );
	RegisterFunction( LUA, "SetPressure", SetPressure );
	RegisterFunction( LUA, "SetGravityFactor", SetGravityFactor );
	RegisterFunction( LUA, "SetVertexPinned", SetVertexPinned );
	RegisterFunction( LUA, "SetVertexVelocity", SetVertexVelocity );
	RegisterFunction( LUA, "SetVelocity", SetVelocity );
	RegisterFunction( LUA, "AddImpulse", AddImpulse );
	RegisterFunction( LUA, "AddVertexImpulse", AddVertexImpulse );
	RegisterFunction( LUA, "Wake", Wake );
	RegisterFunction( LUA, "GetHandle", GetHandle );
	if ( g_Tools )
	{
		RegisterFunction( LUA, "Trace", Trace );
		RegisterFunction( LUA, "SetPlasticity", SetPlasticity );
		RegisterFunction( LUA, "AddImpulseAt", AddImpulseAt );
	}
	LUA->Pop();
	LUA->PushSpecial( SPECIAL_GLOB );
	LUA->CreateTable();
	RegisterFunction( LUA, "Available", Available );
	RegisterFunction( LUA, "GetVersion", GetVersion );
	RegisterFunction( LUA, "GetLimits", GetLimits );
	RegisterFunction( LUA, "Create", Create );
	// The binding understands the option; older backends reject its creation flag.
	SetBoolean( LUA, "SupportsDihedralBend", true );
	LUA->SetField( -2, "joltsoftbody" );
	LUA->Pop();
	return 0;
}

GMOD_MODULE_CLOSE()
{
	if ( g_Bridge )
	{
		for ( auto handle : g_OwnedBodies )
		{
			g_Bridge->Destroy( handle );
		}
	}
	g_OwnedBodies.clear();
	g_Bridge = nullptr;
	g_Tools = nullptr;
	return 0;
}
