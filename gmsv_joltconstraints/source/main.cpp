#include <GarrysMod/Lua/Interface.h>
#include <vjolt/constraints.h>
#include <vjolt/loader.h>

#include <cmath>
#include <unordered_set>

using namespace GarrysMod::Lua;

void VJoltPathLuaOpen(ILua *lua);
void VJoltPathLuaClose();

namespace
{
IVJoltConstraintsBridge *g_Bridge = nullptr;
int g_MechanismMeta = 0;
std::unordered_set<VJoltConstraintHandle> g_OwnedMechanisms;

int PushFailure( ILua *lua, const char *reason )
{
	lua->PushNil();
	lua->PushString( g_Bridge ? reason : "matching VJoltConstraints001 backend is not loaded" );
	return 2;
}

float ReadNumberField( ILua *lua, int tableIndex, const char *key, float fallback )
{
	lua->GetField( tableIndex, key );
	const float value = lua->IsType( -1, Type::Nil ) ? fallback : static_cast<float>( lua->CheckNumber( -1 ) );
	lua->Pop();
	if ( !std::isfinite( value ) )
	{
		lua->ThrowError( "constraint settings must be finite" );
	}
	return value;
}

bool ReadVectorField( ILua *lua, int tableIndex, const char *key, VJoltConstraintVector &value )
{
	lua->GetField( tableIndex, key );
	const bool provided = !lua->IsType( -1, Type::Nil );
	if ( provided )
	{
		lua->CheckType( -1, Type::Vector );
		const auto *vector = lua->GetUserType<VJoltConstraintVector>( -1, Type::Vector );
		if ( !vector )
		{
			lua->ThrowError( "invalid constraint vector" );
			return false;
		}
		value = *vector;
	}
	lua->Pop();
	return provided;
}

template <typename Settings>
void ReadFrames( ILua *lua, Settings &settings )
{
	lua->CheckType( 3, Type::Table );
	if ( ReadVectorField( lua, 3, "anchor1", settings.anchor1 ) )
	{
		settings.flags &= ~VJoltAnchor1AtCOM;
	}
	if ( ReadVectorField( lua, 3, "anchor2", settings.anchor2 ) )
	{
		settings.flags &= ~VJoltAnchor2AtCOM;
	}
	ReadVectorField( lua, 3, "axis1", settings.axis1 );
	ReadVectorField( lua, 3, "axis2", settings.axis2 );
}

IPhysicsObject *ReadPhysicsObject( ILua *lua, int index )
{
	lua->CheckType( index, Type::PhysObj );
	return lua->GetUserType<IPhysicsObject>( index, Type::PhysObj );
}

VJoltConstraintHandle ReadMechanismHandle( ILua *lua )
{
	const auto *handle = lua->GetUserType<VJoltConstraintHandle>( 1, g_MechanismMeta );
	return handle ? *handle : 0;
}

int PushMechanism( ILua *lua, VJoltConstraintHandle handle )
{
	if ( !handle )
	{
		return PushFailure( lua, "cannot create mechanism: check settings, body support and physics simulation state" );
	}
	g_OwnedMechanisms.insert( handle );
	lua->PushUserType_Value( handle, g_MechanismMeta );
	return 1;
}

void SetNumberField( ILua *lua, const char *key, double value )
{
	lua->PushNumber( value );
	lua->SetField( -2, key );
}

void RegisterFunction( ILua *lua, const char *name, CFunc function )
{
	lua->PushCFunction( function );
	lua->SetField( -2, name );
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

LUA_FUNCTION_STATIC( CreateGear )
{
	auto *firstGear = ReadPhysicsObject( LUA, 1 );
	auto *secondGear = ReadPhysicsObject( LUA, 2 );
	VJoltGearSettings settings;
	ReadFrames( LUA, settings );
	settings.ratio = ReadNumberField( LUA, 3, "ratio", settings.ratio );
	return PushMechanism( LUA, g_Bridge ? g_Bridge->CreateGear( firstGear, secondGear, settings ) : 0 );
}

LUA_FUNCTION_STATIC( CreateRackAndPinion )
{
	auto *pinion = ReadPhysicsObject( LUA, 1 );
	auto *rack = ReadPhysicsObject( LUA, 2 );
	VJoltRackAndPinionSettings settings;
	ReadFrames( LUA, settings );
	settings.travelPerRevolution = ReadNumberField( LUA, 3, "travelPerRevolution", settings.travelPerRevolution );
	settings.minPosition = ReadNumberField( LUA, 3, "minPosition", settings.minPosition );
	settings.maxPosition = ReadNumberField( LUA, 3, "maxPosition", settings.maxPosition );
	return PushMechanism( LUA, g_Bridge ? g_Bridge->CreateRackAndPinion( pinion, rack, settings ) : 0 );
}

LUA_FUNCTION_STATIC( IsValid )
{
	LUA->PushBool( g_Bridge && g_Bridge->IsValid( ReadMechanismHandle( LUA ) ) );
	return 1;
}

LUA_FUNCTION_STATIC( Destroy )
{
	auto *handle = LUA->GetUserType<VJoltConstraintHandle>( 1, g_MechanismMeta );
	if ( !handle || !*handle )
	{
		LUA->PushBool( false );
		return 1;
	}

	const bool removed = g_Bridge && g_Bridge->Destroy( *handle );
	g_OwnedMechanisms.erase( *handle );
	*handle = 0;
	LUA->PushBool( removed );
	return 1;
}

LUA_FUNCTION_STATIC( SetMotor )
{
	const auto handle = ReadMechanismHandle( LUA );
	const float speed = static_cast<float>( LUA->CheckNumber( 2 ) );
	const float torque = static_cast<float>( LUA->CheckNumber( 3 ) );
	if ( !g_Bridge || !g_Bridge->SetMotor( handle, speed, torque ) )
	{
		return PushFailure( LUA, "cannot set motor: invalid mechanism, motor settings, or physics world is stepping" );
	}
	LUA->PushBool( true );
	return 1;
}

LUA_FUNCTION_STATIC( GetState )
{
	VJoltConstraintState state;
	if ( !g_Bridge || !g_Bridge->GetState( ReadMechanismHandle( LUA ), state ) )
	{
		return PushFailure( LUA, "cannot read state: invalid mechanism or physics world is stepping" );
	}

	LUA->CreateTable();
	LUA->PushString( state.kind == VJoltGear ? "gear" : "rack_and_pinion" );
	LUA->SetField( -2, "kind" );
	SetNumberField( LUA, "angle1", state.angle1 );
	SetNumberField( LUA, "angle2", state.angle2 );
	SetNumberField( LUA, "position", state.position );
	SetNumberField( LUA, "angularVelocity1", state.angularVelocity1 );
	SetNumberField( LUA, "angularVelocity2", state.angularVelocity2 );
	SetNumberField( LUA, "linearVelocity", state.linearVelocity );
	SetNumberField( LUA, "motorSpeed", state.motorSpeed );
	SetNumberField( LUA, "maxTorque", state.maxTorque );
	return 1;
}
} // namespace

GMOD_MODULE_OPEN()
{
	const auto discovered = VJoltSDK::DiscoverVersionedInterface<IVJoltConstraintsBridge>(
		VJOLT_CONSTRAINTS_BRIDGE_VERSION, kVJoltConstraintsBridgeVersion );
	g_Bridge = discovered.api;

	g_MechanismMeta = LUA->CreateMetaTable( "JoltMechanism" );
	LUA->Push( -1 );
	LUA->SetField( -2, "__index" );
	RegisterFunction( LUA, "__gc", Destroy );
	RegisterFunction( LUA, "IsValid", IsValid );
	RegisterFunction( LUA, "Destroy", Destroy );
	RegisterFunction( LUA, "SetMotor", SetMotor );
	RegisterFunction( LUA, "GetState", GetState );
	LUA->Pop();

	LUA->PushSpecial( SPECIAL_GLOB );
	LUA->CreateTable();
	RegisterFunction( LUA, "Available", Available );
	RegisterFunction( LUA, "GetVersion", GetVersion );
	RegisterFunction( LUA, "CreateGear", CreateGear );
	RegisterFunction( LUA, "CreateRackAndPinion", CreateRackAndPinion );
	VJoltPathLuaOpen( LUA );
	LUA->SetField( -2, "joltconstraints" );
	LUA->Pop();
	return 0;
}

GMOD_MODULE_CLOSE()
{
	VJoltPathLuaClose();
	if ( g_Bridge )
	{
		for ( const auto handle : g_OwnedMechanisms )
		{
			g_Bridge->Destroy( handle );
		}
	}
	g_OwnedMechanisms.clear();
	g_Bridge = nullptr;
	return 0;
}
