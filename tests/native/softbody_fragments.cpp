#include <Windows.h>
#include "vphysics_interface.h"
#include "icvar.h"
#include <vjolt/softbody.h>
#include <vjolt/softbody_tools.h>
#include <vjolt/softbody_fragment.h>
#include <vjolt/softbody_impact.h>
#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
constexpr float dt = 1.0f / 64;
int checks = 0;
void Check( bool result, const char *message )
{
	++checks;
	if ( !result ) throw std::runtime_error( message );
}
bool Near( float a, float b ) { return std::abs( a - b ) < 0.01f; }
bool Near( VJoltSoftBodyVector a, VJoltSoftBodyVector b )
{
	return Near( a.x, b.x ) && Near( a.y, b.y ) && Near( a.z, b.z );
}
std::vector<VJoltSoftBodyTriangle> Faces( uint32_t rows )
{
	std::vector<VJoltSoftBodyTriangle> faces;
	for ( uint32_t row = 0; row + 1 < rows; ++row )
		for ( uint32_t col = 0; col < 2; ++col )
		{
			const auto a = row * 3 + col;
			faces.push_back( { a, a + 1, a + 3 } );
			faces.push_back( { a + 1, a + 4, a + 3 } );
		}
	return faces;
}
std::vector<VJoltSoftBodyVertexState> Read( IVJoltSoftBodyBridge &bridge, VJoltSoftBodyHandle handle )
{
	VJoltSoftBodyState state;
	Check( bridge.GetState( handle, state ), "read state" );
	std::vector<VJoltSoftBodyVertexState> vertices( state.vertexCount );
	Check( bridge.GetVertices( handle, 0, state.vertexCount, vertices.data() ), "read vertices" );
	for ( const auto &v : vertices )
		Check( v.position.x >= state.boundsMin.x - 0.01f && v.position.x <= state.boundsMax.x + 0.01f
			&& v.position.y >= state.boundsMin.y - 0.01f && v.position.y <= state.boundsMax.y + 0.01f
			&& v.position.z >= state.boundsMin.z - 0.01f && v.position.z <= state.boundsMax.z + 0.01f, "bounds enclose live vertex" );
	return vertices;
}
void ImpactTests( IPhysics &physics, IPhysicsCollision &collision, IVJoltSoftBodyBridge &bridge, IVJoltSoftBodyImpact &impacts )
{
	std::array<float, 2> peak{};
	for ( int trial = 0; trial < 2; ++trial )
	{
		auto *env = physics.CreateEnvironment();
		env->SetGravity( Vector( 0, 0, 0 ) ); env->SetSimulationTimestep( dt );
		auto *floorShape = collision.BBoxToCollide( Vector( -512, -512, -16 ), Vector( 512, 512, 0 ) );
		objectparams_t params{}; params.mass = 1; params.inertia = 1; params.rotInertiaLimit = 0.05f;
		params.enableCollisions = true; params.pName = "impact_floor";
		env->CreatePolyObjectStatic( floorShape, 0, Vector( 0, 0, 0 ), QAngle( 0, 0, 0 ), &params );
		std::array<VJoltSoftBodyVertex, 9> vertices;
		for ( uint32_t i = 0; i < 9; ++i ) vertices[i] = { { float( i % 3 ) * 10, float( i / 3 ) * 10, 0 }, 0.25f, 0 };
		auto faces = Faces( 3 );
		VJoltSoftBodyMesh mesh;
		mesh.vertexCount = 9; mesh.vertices = vertices.data(); mesh.triangleCount = 8; mesh.triangles = faces.data();
		VJoltSoftBodySettings settings; settings.position = { 0, 0, 40 }; settings.damping = 0;
		VJoltSoftBodyError error;
		const auto handle = bridge.Create( 0, mesh, settings, &error );
		Check( handle != 0 && impacts.SetImpactTracking( handle, true ), "enable contact tracking" );
		for ( int i = 0; i < 8; ++i ) env->Simulate( dt );
		VJoltSoftBodyImpact impact;
		Check( impacts.ConsumeImpact( handle, impact ) && impact.contacts == 0, "no contacts produce no feedback" );
		const float speed = trial ? 500.0f : 100.0f;
		bridge.SetVelocity( handle, { 0, 0, -speed } );
		for ( int i = 0; i < 64; ++i )
		{
			env->Simulate( dt );
			Check( impacts.ConsumeImpact( handle, impact ), "consume native impact" );
			if ( impact.energy > peak[trial] )
			{
				peak[trial] = impact.energy;
				Check( impact.contacts > 0 && impact.vertex < 9 && impact.speed > 30, "impact contains contacted vertex and speed" );
				Check( std::abs( impact.position.z ) < 2, "contact location lies on floor" );
			}
			VJoltSoftBodyImpact empty;
			Check( impacts.ConsumeImpact( handle, empty ) && empty.contacts == 0, "impact consumed exactly once" );
		}
		Check( peak[trial] > 0, "falling soft body emits native collision feedback" );
		Check( impacts.SetImpactTracking( handle, false ), "disable contact tracking" );
		bridge.SetVelocity( handle, { 0, 0, -speed } ); env->Simulate( dt );
		Check( impacts.ConsumeImpact( handle, impact ) && impact.contacts == 0, "disabled tracking stays empty" );
		physics.DestroyEnvironment( env ); collision.DestroyCollide( floorShape );
		Check( !impacts.ConsumeImpact( handle, impact ), "retired handle rejects impact reads" );
	}
	Check( peak[1] > peak[0] * 4, "hard collision produces more energy than gentle collision" );
}

void DentTests( IPhysics &physics, IVJoltSoftBodyBridge &bridge, IVJoltSoftBodyTools &tools )
{
	std::array<float, 2> dent{};
	for ( int trial = 0; trial < 2; ++trial )
	{
		auto *env = physics.CreateEnvironment();
		env->SetGravity( Vector( 0, 0, 0 ) ); env->SetSimulationTimestep( dt );
		std::array<VJoltSoftBodyVertex, 9> vertices;
		for ( uint32_t i = 0; i < 9; ++i ) vertices[i] = { { float( i % 3 ) * 10, float( i / 3 ) * 10, 0 }, 0.25f, 1 };
		auto faces = Faces( 3 );
		VJoltSoftBodyMesh mesh;
		mesh.vertexCount = 9; mesh.vertices = vertices.data(); mesh.triangleCount = 8; mesh.triangles = faces.data();
		VJoltSoftBodySettings settings; settings.position = { 0, 0, 100 };
		settings.flags |= VJoltSoftBodyDihedralBend;
		settings.edgeCompliance = 0.000001f; settings.bendCompliance = 0.0001f;
		VJoltSoftBodyError error;
		const auto handle = bridge.Create( 0, mesh, settings, &error );
		Check( handle != 0, "create dent test shell" );
		VJoltSoftBodyPlasticity plastic; plastic.yield = 0.01f; plastic.creep = trial ? 20.0f : 0; plastic.maxStrain = 0.9f;
		Check( tools.SetPlasticity( handle, plastic ), "set elastic / plastic shell" );
		bridge.SetVertexVelocity( handle, 4, { 0, 0, 10 / dt } ); env->Simulate( dt );
		bridge.SetVertexVelocity( handle, 4, {} );
		for ( int i = 0; i < 24; ++i ) env->Simulate( dt );
		bridge.SetVertexPinned( handle, 4, false );
		for ( int i = 0; i < 96; ++i ) env->Simulate( dt );
		dent[trial] = std::abs( Read( bridge, handle )[4].position.z - 100 );
		physics.DestroyEnvironment( env );
	}
	Check( dent[0] < 0.2f, "elastic shell recovers after indentation" );
	Check( dent[1] > 3, "plastic shell retains permanent indentation" );
}

void Run( IPhysics &physics, IVJoltSoftBodyBridge &bridge, IVJoltSoftBodyFragment &fragments, IVJoltSoftBodyTools &tools )
{
	for ( bool dihedral : { false, true } )
	{
		auto *env = physics.CreateEnvironment();
		env->SetGravity( Vector( 0, 0, 0 ) );
		env->SetAirDensity( 0 );
		env->SetSimulationTimestep( dt );
		std::array<VJoltSoftBodyVertex, 15> vertices;
		for ( uint32_t i = 0; i < vertices.size(); ++i )
			vertices[i] = { { float( i % 3 ) * 10, float( i / 3 ) * 10, 0 }, 0.001f, 1 };
		auto faces = Faces( 5 );
		VJoltSoftBodyMesh mesh;
		mesh.vertices = vertices.data(); mesh.vertexCount = 15;
		mesh.triangles = faces.data(); mesh.triangleCount = uint32_t( faces.size() );
		VJoltSoftBodySettings settings;
		settings.position = { 100, 200, 300 }; settings.angles = { 0, 90, 0 };
		settings.damping = 0.1f; settings.vertexRadius = 0;
		settings.edgeCompliance = 0; settings.bendCompliance = 0.001f;
		if ( dihedral ) settings.flags |= VJoltSoftBodyDihedralBend;
		VJoltSoftBodyError error;
		const auto parent = bridge.Create( 0, mesh, settings, &error );
		Check( parent != 0, "create rotated parent" );
		auto original = Read( bridge, parent );
		for ( uint32_t i = 0; i < 15; ++i )
		{
			// Stretch to 150% while kinematic, keeping nonzero live velocities.
			const auto p = original[i].position;
			Check( bridge.SetVertexVelocity( parent, i, { ( p.x - 100 ) * 0.5f / dt,
				( p.y - 200 ) * 0.5f / dt, 0 } ), "move pinned vertex" );
		}
		env->Simulate( dt );
		auto live = Read( bridge, parent );
		std::array<uint32_t, 9> sources{ 0, 1, 2, 3, 4, 5, 6, 7, 8 };
		std::array<float, 9> fractions{ 1, 1, 1, 1, 1, 1, 0.5f, 0.5f, 0.5f };
		auto childFaces = Faces( 3 );
		// Invalid requests cannot retire or mutate the original body.
		sources[1] = 0;
		Check( !fragments.CreateFragment( parent, 9, sources.data(), fractions.data(), 8, childFaces.data(), &error ), "reject duplicate source" );
		sources[1] = 1;
		fractions[0] = 2;
		Check( !fragments.CreateFragment( parent, 9, sources.data(), fractions.data(), 8, childFaces.data(), &error ), "reject mass inflation" );
		fractions[0] = 1;
		std::swap( childFaces[0].vertices[0], childFaces[0].vertices[1] );
		Check( !fragments.CreateFragment( parent, 9, sources.data(), fractions.data(), 8, childFaces.data(), &error ), "reject foreign/reversed face" );
		childFaces = Faces( 3 );
		Check( bridge.IsValid( parent ), "parent survives invalid requests" );
		const auto first = fragments.CreateFragment( parent, 9, sources.data(), fractions.data(), 8, childFaces.data(), &error );
		Check( first != 0, "create first fragment (sub-milligram seam mass)" );
		auto one = Read( bridge, first );
		for ( uint32_t i = 0; i < 9; ++i )
		{
			Check( Near( one[i].position, live[i].position ), "fragment preserves current world position" );
			Check( Near( one[i].velocity, live[i].velocity ), "fragment preserves current world velocity" );
			Check( one[i].pinned == live[i].pinned, "fragment preserves pin" );
			Check( std::abs( one[i].mass - live[i].mass * fractions[i] ) < 1e-8f, "fragment preserves apportioned mass" );
		}
		for ( uint32_t i = 0; i < 9; ++i ) { sources[i] = i + 6; fractions[i] = i < 3 ? 0.5f : 1; }
		const auto second = fragments.CreateFragment( parent, 9, sources.data(), fractions.data(), 8, childFaces.data(), &error );
		Check( second != 0, "create second fragment" );
		auto two = Read( bridge, second );
		float mass = 0;
		for ( const auto &v : one ) mass += v.mass;
		for ( const auto &v : two ) mass += v.mass;
		Check( std::abs( mass - 0.015f ) < 1e-7f, "combined mass conserved" );
		Check( bridge.Destroy( parent ), "commit parent retirement" );
		for ( uint32_t i = 0; i < 9; ++i )
		{
			bridge.SetVertexVelocity( first, i, {} ); bridge.SetVertexPinned( first, i, false );
			bridge.SetVertexVelocity( second, i, {} ); bridge.SetVertexPinned( second, i, false );
		}
		for ( int tick = 0; tick < 32; ++tick ) env->Simulate( dt );
		one = Read( bridge, first );
		const float dx = one[0].position.x - one[1].position.x, dy = one[0].position.y - one[1].position.y;
		Check( std::sqrt( dx * dx + dy * dy ) < 12, "fragment relaxes to original rest shape, not deformed split pose" );
		// Repeated fragmentation and plasticity initialization remain valid.
		VJoltSoftBodyPlasticity plastic; plastic.creep = 2;
		Check( tools.SetPlasticity( first, plastic ), "plasticity on fragment" );
		std::array<uint32_t, 6> nestedSources{ 0, 1, 2, 3, 4, 5 };
		std::array<float, 6> nestedFractions{ 1, 1, 1, 0.5f, 0.5f, 0.5f };
		auto nestedFaces = Faces( 2 );
		const auto nested = fragments.CreateFragment( first, 6, nestedSources.data(), nestedFractions.data(), 4, nestedFaces.data(), &error );
		Check( nested != 0, "fragment can be split again" );
		bridge.Destroy( first );
		env->Simulate( dt );
		Read( bridge, nested );
		physics.DestroyEnvironment( env );
		Check( !bridge.IsValid( nested ) && !bridge.IsValid( second ), "environment cleanup retires fragments" );
	}
}
}

int wmain( int argc, wchar_t **argv )
{
	if ( argc != 3 ) { std::cerr << "usage: softbody_fragments <backend DLL> <GMod root>\n"; return 2; }
	std::cout << std::unitbuf;
	SetErrorMode( SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX );
	const auto backend = std::filesystem::absolute( argv[1] );
	const auto runtimePath = std::filesystem::absolute( argv[2] ) / L"bin/win64";
	const DWORD flags = LOAD_LIBRARY_SEARCH_DEFAULT_DIRS | LOAD_LIBRARY_SEARCH_USER_DIRS;
	SetDefaultDllDirectories( flags ); AddDllDirectory( runtimePath.c_str() ); AddDllDirectory( backend.parent_path().c_str() );
	const auto runtime = LoadLibraryExW( ( runtimePath / L"vstdlib.dll" ).c_str(), nullptr, flags );
	const auto cvarFactory = runtime ? reinterpret_cast<CreateInterfaceFn>( GetProcAddress( runtime, "CreateInterface" ) ) : nullptr;
	const auto library = LoadLibraryExW( backend.c_str(), nullptr, flags );
	if ( !library || !cvarFactory ) { std::cerr << "load error=" << GetLastError() << '\n'; return 3; }
	const auto factory = reinterpret_cast<CreateInterfaceFn>( GetProcAddress( library, "CreateInterface" ) );
	auto *physics = static_cast<IPhysics *>( factory( VPHYSICS_INTERFACE_VERSION, nullptr ) );
	auto *bridge = static_cast<IVJoltSoftBodyBridge *>( factory( VJOLT_SOFTBODY_BRIDGE_VERSION, nullptr ) );
	auto *tools = static_cast<IVJoltSoftBodyTools *>( factory( VJOLT_SOFTBODY_TOOLS_VERSION, nullptr ) );
	auto *fragments = static_cast<IVJoltSoftBodyFragment *>( factory( VJOLT_SOFTBODY_FRAGMENT_VERSION, nullptr ) );
	auto *impacts = static_cast<IVJoltSoftBodyImpact *>( factory( VJOLT_SOFTBODY_IMPACT_VERSION, nullptr ) );
	auto *collision = static_cast<IPhysicsCollision *>( factory( VPHYSICS_COLLISION_INTERFACE_VERSION, nullptr ) );
	auto *surfaces = static_cast<IPhysicsSurfaceProps *>( factory( VPHYSICS_SURFACEPROPS_INTERFACE_VERSION, nullptr ) );
	if ( !physics || !bridge || !tools || !fragments || !impacts || !collision || !surfaces
		|| !physics->Connect( cvarFactory ) || physics->Init() != INIT_OK ) return 4;
	surfaces->ParseSurfaceData( "softbody", "\"default\" { \"friction\" \"0.5\" \"elasticity\" \"0\" }" );
	int result = 0;
	try
	{
		Run( *physics, *bridge, *fragments, *tools );
		ImpactTests( *physics, *collision, *bridge, *impacts );
		DentTests( *physics, *bridge, *tools );
		std::cout << "PASS fragment / impact checks=" << checks << '\n';
	}
	catch ( const std::exception &error ) { std::cerr << "FAIL after " << checks << " checks: " << error.what() << '\n'; result = 1; }
	physics->Shutdown(); physics->Disconnect(); FreeLibrary( library ); FreeLibrary( runtime );
	return result;
}
