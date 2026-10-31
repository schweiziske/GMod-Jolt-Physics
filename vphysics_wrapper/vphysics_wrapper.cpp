//=================================================================================================
//
// The base physics DLL interface
// 
// This is a thin CPU-agnostic wrapper for the actual Volt DLLs which are named
// vphysics_jolt_sse2.dll, vphysics_jolt_sse42.dll and vphysics_jolt_avx2.dll
//
//=================================================================================================

#include "tier0/basetypes.h"
#include "tier1/interface.h"
#include "vphysics_interface.h"
#include "cpu_features.h"

#ifdef _WIN32
#include <intrin.h>
#else
#include <cpuid.h>
#endif

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

class PhysicsWrapper final : public CBaseAppSystem<IPhysics>
{
public:
	bool Connect( CreateInterfaceFn factory ) override;
	void Disconnect() override;

	InitReturnVal_t Init() override;
	void Shutdown() override;
	void *QueryInterface( const char *pInterfaceName ) override;

#if PLATFORM_64BITS && ( defined( GAME_GMOD ) || !defined( GAME_SDK2013 ) ) && ( !defined( GMOD_SDK_PUBLIC ) || !GMOD_SDK_PUBLIC )
	const AppSystemInfo_t *GetDependencies( ) override { return NULL; };
	AppSystemTier_t GetTier() override { return APP_SYSTEM_TIER2; };
	void Reconnect( CreateInterfaceFn factory, const char *pInterfaceName ) override {};
#endif

	IPhysicsEnvironment *CreateEnvironment() override;
	void DestroyEnvironment( IPhysicsEnvironment *pEnvironment ) override;
	IPhysicsEnvironment *GetActiveEnvironmentByIndex( int index ) override;

	IPhysicsObjectPairHash *CreateObjectPairHash() override;
	void DestroyObjectPairHash( IPhysicsObjectPairHash *pHash ) override;

	IPhysicsCollisionSet *FindOrCreateCollisionSet( unsigned int id, int maxElementCount ) override;
	IPhysicsCollisionSet *FindCollisionSet( unsigned int id ) override;
	void DestroyAllCollisionSets() override;

#if GAME_GMOD
	bool IsValidPhysicsObject( IPhysicsObject* pObject ) override;
#endif

public:
	static PhysicsWrapper &GetInstance() { return s_PhysicsInterface; }

private:
	bool InitWrapper();

	CSysModule *m_pActualPhysicsModule;
	IPhysics *m_pActualPhysicsInterface;

	static PhysicsWrapper s_PhysicsInterface;
};

PhysicsWrapper PhysicsWrapper::s_PhysicsInterface;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR( PhysicsWrapper, IPhysics, VPHYSICS_INTERFACE_VERSION, PhysicsWrapper::GetInstance() );

//-------------------------------------------------------------------------------------------------

static void GetCPUID( int *pInfo, int func, int subfunc )
{
#ifdef _WIN32
	__cpuidex( pInfo, func, subfunc );
#else
	__cpuid_count( func, subfunc, pInfo[0], pInfo[1], pInfo[2], pInfo[3] );
#endif
}

static uint64_t GetXCR0()
{
#ifdef _WIN32
	return _xgetbv( 0 );
#else
	uint32_t low, high;
	__asm__ volatile( "xgetbv" : "=a"( low ), "=d"( high ) : "c"( 0 ) );
	return ( uint64_t( high ) << 32 ) | low;
#endif
}

static CPULevel_t GetCPULevel()
{
	int cpuInfo[4] = {};
	VJoltCPUFeatures features;
	GetCPUID( cpuInfo, 0, 0 );
	const int numFuncs = cpuInfo[0];
	if ( numFuncs < 1 )
		return CPU_HAS_SSE2;
	GetCPUID( cpuInfo, 1, 0 );
	features.leaf1ECX = uint32_t( cpuInfo[2] );
	// XGETBV itself is illegal until both CPU and OS advertise XSAVE support.
	constexpr uint32_t xsave = ( 1u << 26 ) | ( 1u << 27 );
	if ( ( features.leaf1ECX & xsave ) == xsave )
		features.xcr0 = GetXCR0();
	if ( numFuncs >= 7 )
	{
		GetCPUID( cpuInfo, 7, 0 );
		features.leaf7EBX = uint32_t( cpuInfo[1] );
	}
	GetCPUID( cpuInfo, int( 0x80000000u ), 0 );
	if ( uint32_t( cpuInfo[0] ) >= 0x80000001u )
	{
		GetCPUID( cpuInfo, int( 0x80000001u ), 0 );
		features.extendedECX = uint32_t( cpuInfo[2] );
	}
	return VJoltSelectCPULevel( features );
}

static const char *GetModuleFromCPULevel( CPULevel_t level )
{
	switch ( level )
	{
		case CPU_HAS_AVX2:		return "vphysics_jolt_avx2" DLL_EXT_STRING;
		case CPU_HAS_SSE42:		return "vphysics_jolt_sse42" DLL_EXT_STRING;
		default:				return "vphysics_jolt_sse2" DLL_EXT_STRING;
	}
}

// Tries to load the actual vphysics DLL
bool PhysicsWrapper::InitWrapper()
{
	if ( m_pActualPhysicsInterface )
		return true;

	const char *pModuleName = GetModuleFromCPULevel( GetCPULevel() );

	if ( !Sys_LoadInterface( pModuleName, VPHYSICS_INTERFACE_VERSION, &m_pActualPhysicsModule, (void **)&m_pActualPhysicsInterface ) )
		return false;

	return true;
}

bool PhysicsWrapper::Connect( CreateInterfaceFn factory )
{
	if ( !InitWrapper() )
		return false;

	return m_pActualPhysicsInterface->Connect( factory );
}

void PhysicsWrapper::Disconnect()
{
	m_pActualPhysicsInterface->Disconnect();

	Sys_UnloadModule( m_pActualPhysicsModule );
}

//-------------------------------------------------------------------------------------------------

InitReturnVal_t PhysicsWrapper::Init()
{
	return m_pActualPhysicsInterface->Init();
}

void PhysicsWrapper::Shutdown()
{
	m_pActualPhysicsInterface->Shutdown();
}

void *PhysicsWrapper::QueryInterface( const char *pInterfaceName )
{
	// This function can be called before Connect, so try and load the real DLL early
	if ( !InitWrapper() )
		return nullptr;

	return m_pActualPhysicsInterface->QueryInterface( pInterfaceName );
}

//-------------------------------------------------------------------------------------------------

IPhysicsEnvironment *PhysicsWrapper::CreateEnvironment()
{
	return m_pActualPhysicsInterface->CreateEnvironment();
}

void PhysicsWrapper::DestroyEnvironment( IPhysicsEnvironment *pEnvironment )
{
	m_pActualPhysicsInterface->DestroyEnvironment( pEnvironment );
}

IPhysicsEnvironment *PhysicsWrapper::GetActiveEnvironmentByIndex( int index )
{
	return m_pActualPhysicsInterface->GetActiveEnvironmentByIndex( index );
}

//-------------------------------------------------------------------------------------------------

IPhysicsObjectPairHash *PhysicsWrapper::CreateObjectPairHash()
{
	return m_pActualPhysicsInterface->CreateObjectPairHash();
}

void PhysicsWrapper::DestroyObjectPairHash( IPhysicsObjectPairHash *pHash )
{
	m_pActualPhysicsInterface->DestroyObjectPairHash( pHash );
}

//-------------------------------------------------------------------------------------------------

IPhysicsCollisionSet *PhysicsWrapper::FindOrCreateCollisionSet( unsigned int id, int maxElementCount )
{
	return m_pActualPhysicsInterface->FindOrCreateCollisionSet( id, maxElementCount );
}

IPhysicsCollisionSet *PhysicsWrapper::FindCollisionSet( unsigned int id )
{
	return m_pActualPhysicsInterface->FindCollisionSet( id );
}

void PhysicsWrapper::DestroyAllCollisionSets()
{
	m_pActualPhysicsInterface->DestroyAllCollisionSets();
}

#if GAME_GMOD
bool PhysicsWrapper::IsValidPhysicsObject( IPhysicsObject* pObject )
{
	return m_pActualPhysicsInterface->IsValidPhysicsObject( pObject );
}
#endif
