#include "lua_mesh.h"

#include <cmath>
#include <limits>

namespace JoltSoftBodyLua
{
using namespace GarrysMod::Lua;

VJoltSoftBodyMesh CreationArguments::Mesh() const
{
	VJoltSoftBodyMesh mesh;
	mesh.vertexCount = static_cast<uint32_t>( vertices.size() );
	mesh.triangleCount = static_cast<uint32_t>( triangles.size() );
	mesh.vertices = vertices.data();
	mesh.triangles = triangles.data();
	return mesh;
}

bool ReadNumber( ILua *lua, int index, float &value )
{
	if ( !lua->IsType( index, Type::Number ) )
	{
		return false;
	}
	const double number = lua->GetNumber( index );
	if ( !std::isfinite( number ) || std::abs( number ) > std::numeric_limits<float>::max() )
	{
		return false;
	}
	value = static_cast<float>( number );
	return true;
}

bool ReadInteger( ILua *lua, int index, uint32_t minimum, uint32_t maximum, uint32_t &value )
{
	if ( !lua->IsType( index, Type::Number ) )
	{
		return false;
	}
	const double number = lua->GetNumber( index );
	if ( !std::isfinite( number ) || number < minimum || number > maximum || std::floor( number ) != number )
	{
		return false;
	}
	value = static_cast<uint32_t>( number );
	return true;
}

bool ReadVector( ILua *lua, int index, VJoltSoftBodyVector &value )
{
	if ( !lua->IsType( index, Type::Vector ) )
	{
		return false;
	}
	const auto &vector = lua->GetVector( index );
	value = { vector.x, vector.y, vector.z };
	return std::isfinite( value.x ) && std::isfinite( value.y ) && std::isfinite( value.z );
}

namespace
{
int AbsoluteIndex( ILua *lua, int index )
{
	return index > 0 ? index : lua->Top() + index + 1;
}

void GetRawField( ILua *lua, int table, const char *name )
{
	const int absolute = AbsoluteIndex( lua, table );
	lua->PushString( name );
	lua->RawGet( absolute );
}

void GetRawElement( ILua *lua, int table, uint32_t index )
{
	const int absolute = AbsoluteIndex( lua, table );
	lua->PushNumber( index );
	lua->RawGet( absolute );
}

bool ReadNumberField( ILua *lua, int table, const char *name, float &value )
{
	GetRawField( lua, table, name );
	const bool valid = lua->IsType( -1, Type::Nil ) || ReadNumber( lua, -1, value );
	lua->Pop();
	return valid;
}

bool ReadBooleanField( ILua *lua, int table, const char *name, bool &value )
{
	GetRawField( lua, table, name );
	const bool valid = lua->IsType( -1, Type::Nil ) || lua->IsType( -1, Type::Bool );
	if ( lua->IsType( -1, Type::Bool ) )
	{
		value = lua->GetBool( -1 );
	}
	lua->Pop();
	return valid;
}

bool ReadVectorField( ILua *lua, int table, const char *name, VJoltSoftBodyVector &value, bool required = false )
{
	GetRawField( lua, table, name );
	const bool valid = ( !required && lua->IsType( -1, Type::Nil ) ) || ReadVector( lua, -1, value );
	lua->Pop();
	return valid;
}

bool ReadIntegerField( ILua *lua, int table, const char *name, uint32_t minimum, uint32_t maximum, uint32_t &value )
{
	GetRawField( lua, table, name );
	const bool valid = lua->IsType( -1, Type::Nil ) || ReadInteger( lua, -1, minimum, maximum, value );
	lua->Pop();
	return valid;
}

bool ReadSettings( ILua *lua, CreationArguments &arguments )
{
	if ( lua->Top() < 3 || lua->IsType( 3, Type::Nil ) )
	{
		return true;
	}
	if ( !lua->IsType( 3, Type::Table ) )
	{
		return false;
	}
	uint32_t environment = 0;
	auto &settings = arguments.settings;
	bool allowSleeping = true;
	bool doubleSided = true;
	bool dihedralBend = false;
	if ( !ReadIntegerField( lua, 3, "environment", 0, INT32_MAX, environment )
		|| !ReadIntegerField( lua, 3, "iterations", 1, 32, settings.iterations )
		|| !ReadVectorField( lua, 3, "pos", settings.position )
		|| !ReadVectorField( lua, 3, "velocity", settings.velocity )
		|| !ReadNumberField( lua, 3, "edgeCompliance", settings.edgeCompliance )
		|| !ReadNumberField( lua, 3, "bendCompliance", settings.bendCompliance )
		|| !ReadNumberField( lua, 3, "pressure", settings.pressure )
		|| !ReadNumberField( lua, 3, "gravityFactor", settings.gravityFactor )
		|| !ReadNumberField( lua, 3, "damping", settings.damping )
		|| !ReadNumberField( lua, 3, "friction", settings.friction )
		|| !ReadNumberField( lua, 3, "restitution", settings.restitution )
		|| !ReadNumberField( lua, 3, "vertexRadius", settings.vertexRadius )
		|| !ReadNumberField( lua, 3, "maxVelocity", settings.maxVelocity )
		|| !ReadBooleanField( lua, 3, "allowSleeping", allowSleeping )
		|| !ReadBooleanField( lua, 3, "doubleSided", doubleSided )
		|| !ReadBooleanField( lua, 3, "dihedralBend", dihedralBend ) )
	{
		return false;
	}
	GetRawField( lua, 3, "angles" );
	const bool hasAngles = !lua->IsType( -1, Type::Nil );
	if ( hasAngles && !lua->IsType( -1, Type::Angle ) )
	{
		lua->Pop();
		return false;
	}
	if ( hasAngles )
	{
		const auto &angles = lua->GetAngle( -1 );
		settings.angles = { angles.x, angles.y, angles.z };
	}
	lua->Pop();
	arguments.environment = static_cast<int32_t>( environment );
	settings.flags =
		( allowSleeping ? VJoltSoftBodyAllowSleeping : 0u ) | ( doubleSided ? VJoltSoftBodyDoubleSided : 0u )
		| ( dihedralBend ? VJoltSoftBodyDihedralBend : 0u );
	return true;
}

bool ReadVertex( ILua *lua, VJoltSoftBodyVertex &vertex )
{
	if ( lua->IsType( -1, Type::Vector ) )
	{
		return ReadVector( lua, -1, vertex.position );
	}
	if ( !lua->IsType( -1, Type::Table ) )
	{
		return false;
	}
	bool pinned = false;
	const bool valid = ReadVectorField( lua, -1, "pos", vertex.position, true )
		&& ReadNumberField( lua, -1, "mass", vertex.mass ) && ReadBooleanField( lua, -1, "pinned", pinned );
	vertex.pinned = pinned ? 1u : 0u;
	return valid;
}

bool ReadTriangle( ILua *lua, uint32_t vertexCount, VJoltSoftBodyTriangle &triangle )
{
	if ( !lua->IsType( -1, Type::Table ) || lua->ObjLen( -1 ) != 3 )
	{
		return false;
	}
	for ( uint32_t corner = 0; corner < 3; ++corner )
	{
		GetRawElement( lua, -1, corner + 1 );
		uint32_t index;
		const bool valid = ReadInteger( lua, -1, 1, vertexCount, index );
		lua->Pop();
		if ( !valid )
		{
			return false;
		}
		triangle.vertices[corner] = index - 1;
	}
	return true;
}
} // namespace

bool ReadCreationArguments( ILua *lua, CreationArguments &arguments, const char *&error )
{
	// Parsing returns ordinary failures rather than ThrowError/longjmp, allowing
	// the caller's native mesh buffers to be released on every validation failure.
	if ( !lua->IsType( 1, Type::Table ) || !lua->IsType( 2, Type::Table ) )
	{
		error = "Create expects a vertex array, a triangle array, and optional settings";
		return false;
	}
	const int vertexCount = lua->ObjLen( 1 );
	const int triangleCount = lua->ObjLen( 2 );
	if ( vertexCount < 3 || vertexCount > int( kVJoltSoftBodyMaxVertices ) || triangleCount < 1
		|| triangleCount > int( kVJoltSoftBodyMaxTriangles ) )
	{
		error = "mesh requires 3..8192 vertices and 1..16384 triangles";
		return false;
	}
	if ( !ReadSettings( lua, arguments ) )
	{
		error = "invalid soft-body settings: check numeric, Vector, Angle and boolean fields";
		return false;
	}
	arguments.vertices.resize( vertexCount );
	arguments.triangles.resize( triangleCount );
	for ( uint32_t index = 0; index < arguments.vertices.size(); ++index )
	{
		GetRawElement( lua, 1, index + 1 );
		const bool valid = ReadVertex( lua, arguments.vertices[index] );
		lua->Pop();
		if ( !valid )
		{
			error = "each vertex must be a Vector or {pos=Vector, mass=number, pinned=boolean}";
			return false;
		}
	}
	for ( uint32_t index = 0; index < arguments.triangles.size(); ++index )
	{
		GetRawElement( lua, 2, index + 1 );
		const bool valid = ReadTriangle( lua, vertexCount, arguments.triangles[index] );
		lua->Pop();
		if ( !valid )
		{
			error = "each triangle must contain three integer vertex indices, starting at 1";
			return false;
		}
	}
	return true;
}

const char *CreationError( VJoltSoftBodyError error )
{
	switch ( error )
	{
	case VJoltSoftBodyError::None:
		return "no soft-body error";
	case VJoltSoftBodyError::InvalidEnvironment:
		return "the selected physics environment does not exist";
	case VJoltSoftBodyError::SimulationBusy:
		return "soft-body creation is unavailable during physics simulation";
	case VJoltSoftBodyError::InvalidSettings:
		return "soft-body settings are outside the supported ranges";
	case VJoltSoftBodyError::InvalidMesh:
		return "invalid mesh arrays or SDK structure size";
	case VJoltSoftBodyError::InvalidVertex:
		return "vertex positions must be finite and in range; masses must be 0.001..10000 kg";
	case VJoltSoftBodyError::InvalidTriangle:
		return "triangle indices or triangle area are invalid";
	case VJoltSoftBodyError::InvalidTopology:
		return "mesh must be connected, without unused vertices, duplicate faces or inconsistent winding";
	case VJoltSoftBodyError::PressureRequiresClosedMesh:
		return "nonzero pressure requires a closed mesh with outward winding";
	case VJoltSoftBodyError::LimitExceeded:
		return "soft-body mesh or global capacity limit reached";
	case VJoltSoftBodyError::AllocationFailed:
		return "native soft-body allocation failed";
	}
	return "unknown native soft-body creation error";
}
} // namespace JoltSoftBodyLua
