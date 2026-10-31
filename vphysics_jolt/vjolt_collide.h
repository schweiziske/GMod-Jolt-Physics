//=================================================================================================
//
// CPhysCollide and friends!
//
//=================================================================================================

#pragma once

#include "coordsize.h"

inline constexpr char VJOLT_SOURCE_MATERIAL_PREFIX[] = "VPhysicsMaterial:";

//-------------------------------------------------------------------------------------------------

static constexpr float kMaxConvexRadius = SourceToJolt::Distance( DIST_EPSILON * 2.0f );

struct JoltSourceQueryTriangle
{
	Vector vertices[ 3 ];
	uint8 materialIndex = 0;
};

struct JoltSourceQueryConvex
{
	unsigned int gameData = 0;
	std::vector< JoltSourceQueryTriangle > triangles;
	// IVP point-array order and metres in IVP axes (x, -z, y). Keeping these
	// before tessellation preserves support-map ties and transform rounding.
	std::vector< Vector > traceVertices;
	// For large IVP ledges the original support search walks compact edges.
	// Each group of three entries is one triangle; the pair is vertex/opposite.
	std::vector< std::pair< uint32, uint32 > > traceEdges;
	bool traceCachedSupport = true;
	std::array< int, 8 > traceStartEdges {};
	int traceLoadIndex = -1; // Load-time leaf identity; not serialized.
};

struct JoltSourceTraceNode
{
	Vector center;
	float radius = 0;
	int left = -1;
	int right = -1;
	int convex = -1;
};

// Per-shape state stashed via JPH::Shape::mUserData (as a pointer-cast).
struct JoltCollideUserData
{
	Vector orthographicAreas { 1.0f, 1.0f, 1.0f };
	// Source buoyancy evaluates pressure on the actual submerged collision
	// surface. Cache the immutable model-space triangle soup here so fluid
	// controllers do not have to tessellate the same shape every simulation
	// step. Vertices are stored in Jolt metres, relative to the collision
	// model's original object origin (not its potentially overridden COM).
	std::vector<JPH::Float3> sourceBuoyancyTriangles;
	bool sourceBuoyancyTrianglesCached = false;
	// Original IVP mass data carried by .phy compact surfaces. The center is
	// converted to Jolt axes and remains in metres; rotational inertia is the
	// diagonal tensor per kilogram in m^2, also converted to Jolt axes.
	JPH::Vec3 sourceMassCenter { JPH::Vec3::sZero() };
	JPH::Vec3 sourceRotationInertiaPerMass { JPH::Vec3::sZero() };
	bool   hasSourceMassProperties = false;
	// Source exposes the cooked IVP ledges through ICollisionQuery. Keep that
	// topology separate from Jolt's immutable tessellation so triangle material
	// edits are shared by all query objects and survive collide serialization.
	std::vector< JoltSourceQueryConvex > sourceQueryConvexes;
	bool sourceQueryConvexesCached = false;
	std::vector< JoltSourceTraceNode > sourceTraceNodes;
	float sourceTraceRadius = 0;
	bool sourceTraceCached = false;
	// Runtime ownership only; not part of collide serialization. Compound and
	// restored decorated roots own metadata allocated for their child shapes,
	// while wrappers made by DuplicateAndScale merely borrow the source shape.
	bool ownsChildUserData = false;
	// Runtime-only Source collide kind used by OutputDebugInfo. Virtual meshes
	// have their own public diagnostic string instead of compact-surface stats.
	bool isVirtualMesh = false;
	// Mirror of the compact ledge-tree outer-hull diagnostic (persisted since collide v4).
	bool hasOuterHull = false;
	int    vcollideIndex     = 0;
	int    gameData          = 0;
};

// Dummy helper class to go back and forth between CPhysCollide* and JPH::Shape*. The
// only "data" on the shape we own is the JoltCollideUserData* in its UserData slot.
class CPhysCollide
{
	~CPhysCollide() = delete;

public:
	JPH::Shape* ToShape()
	{
		return reinterpret_cast<JPH::Shape*>(this);
	}

	const JPH::Shape *ToShape() const
	{
		return reinterpret_cast< const JPH::Shape * >( this );
	}

	//-------------------------------------------------------------------------------------------------

	static CPhysCollide *FromShape( JPH::Shape *pCollide )
	{
		return reinterpret_cast< CPhysCollide * >( pCollide );
	}

	static const CPhysCollide *FromShape( const JPH::Shape *pCollide )
	{
		return reinterpret_cast< const CPhysCollide * >( pCollide );
	}

	//-------------------------------------------------------------------------------------------------

	JoltCollideUserData *GetUserData()
	{
		return reinterpret_cast<JoltCollideUserData *>( ToShape()->GetUserData() );
	}

	const JoltCollideUserData *GetUserData() const
	{
		return reinterpret_cast<const JoltCollideUserData *>( ToShape()->GetUserData() );
	}

	void SetUserData( JoltCollideUserData *pData )
	{
		ToShape()->SetUserData( reinterpret_cast<uint64>( pData ) );
	}
};

//-------------------------------------------------------------------------------------------------

// Dummy helper class to go back and forth between CPhysConvex* and JPH::ConvexShape*.
class CPhysConvex
{
	~CPhysConvex() = delete;

public:
	JPH::ConvexShape* ToConvexShape()
	{
		return reinterpret_cast< JPH::ConvexShape* >( this );
	}

	const JPH::ConvexShape *ToConvexShape() const
	{
		return reinterpret_cast< const JPH::ConvexShape * >( this );
	}

	//-------------------------------------------------------------------------------------------------

	static CPhysConvex *FromConvexShape( JPH::ConvexShape *pCollide )
	{
		return reinterpret_cast< CPhysConvex * >( pCollide );
	}

	static const CPhysConvex *FromConvexShape( const JPH::ConvexShape *pCollide )
	{
		return reinterpret_cast< const CPhysConvex * >( pCollide );
	}

	//-------------------------------------------------------------------------------------------------

	CPhysCollide *ToPhysCollide()
	{
		return reinterpret_cast< CPhysCollide * >( this );
	}

	JoltCollideUserData *GetUserData()
	{
		return reinterpret_cast<JoltCollideUserData *>( ToConvexShape()->GetUserData() );
	}

	const JoltCollideUserData *GetUserData() const
	{
		return reinterpret_cast<const JoltCollideUserData *>( ToConvexShape()->GetUserData() );
	}

	void SetUserData( JoltCollideUserData *pData )
	{
		ToConvexShape()->SetUserData( reinterpret_cast<uint64>( pData ) );
	}
};

//-------------------------------------------------------------------------------------------------

class CPhysPolysoup
{
public:
	JPH::TriangleList Triangles;
};

//-------------------------------------------------------------------------------------------------

// Josh: Suprise! This is not an app system! Just an interface...
class JoltPhysicsCollision final : public IPhysicsCollision 
{
public:
	CPhysConvex		*ConvexFromVerts( Vector **pVerts, int vertCount ) override;
	CPhysConvex		*ConvexFromPlanes( float *pPlanes, int planeCount, float mergeDistance ) override;
	float			ConvexVolume( CPhysConvex *pConvex ) override;

	float			ConvexSurfaceArea( CPhysConvex *pConvex ) override;
	void			SetConvexGameData( CPhysConvex *pConvex, unsigned int gameData ) override;
	void			ConvexFree( CPhysConvex *pConvex ) override;
	CPhysConvex		*BBoxToConvex( const Vector &mins, const Vector &maxs ) override;
	CPhysConvex		*ConvexFromConvexPolyhedron( const CPolyhedron &ConvexPolyhedron ) override;
	void			ConvexesFromConvexPolygon( const Vector &vPolyNormal, const Vector *pPoints, int iPointCount, CPhysConvex **pOutput ) override;

	CPhysPolysoup	*PolysoupCreate() override;
	void			PolysoupDestroy( CPhysPolysoup *pSoup ) override;
	void			PolysoupAddTriangle( CPhysPolysoup *pSoup, const Vector &a, const Vector &b, const Vector &c, int materialIndex7bits ) override;
	CPhysCollide	*ConvertPolysoupToCollide( CPhysPolysoup *pSoup, bool useMOPP ) override;
	
	CPhysCollide	*ConvertConvexToCollide( CPhysConvex **pConvex, int convexCount ) override;
	CPhysCollide	*ConvertConvexToCollideParams( CPhysConvex **pConvex, int convexCount, const convertconvexparams_t &convertParams ) override;
	void			DestroyCollide( CPhysCollide *pCollide ) override;

	int				CollideSize( CPhysCollide *pCollide ) override;
	int				CollideWrite( char *pDest, CPhysCollide *pCollide, bool bSwap = false ) override;
	CPhysCollide	*UnserializeCollide( char *pBuffer, int size, int index ) override;

	float			CollideVolume( CPhysCollide *pCollide ) override;
	float			CollideSurfaceArea( CPhysCollide *pCollide ) override;

	Vector			CollideGetExtent( const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, const Vector &direction ) override;

	void			CollideGetAABB( Vector *pMins, Vector *pMaxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles ) override;

	void			CollideGetMassCenter( CPhysCollide *pCollide, Vector *pOutMassCenter ) override;
	void			CollideSetMassCenter( CPhysCollide *pCollide, const Vector &massCenter ) override;
	Vector			CollideGetOrthographicAreas( const CPhysCollide *pCollide ) override;
	void			CollideSetOrthographicAreas( CPhysCollide *pCollide, const Vector &areas ) override;

	int				CollideIndex( const CPhysCollide *pCollide ) override;

	CPhysCollide	*BBoxToCollide( const Vector &mins, const Vector &maxs ) override;
	int				GetConvexesUsedInCollideable( const CPhysCollide *pCollideable, CPhysConvex **pOutputArray, int iOutputArrayLimit ) override;


	void			TraceBox( const Vector &start, const Vector &end, const Vector &mins, const Vector &maxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr ) override;
	void			TraceBox( const Ray_t &ray, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr ) override;
	void			TraceBox( const Ray_t &ray, unsigned int contentsMask, IConvexInfo *pConvexInfo, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr ) override;

	void			TraceCollide( const Vector &start, const Vector &end, const CPhysCollide *pSweepCollide, const QAngle &sweepAngles, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr ) override;

	bool			IsBoxIntersectingCone( const Vector &boxAbsMins, const Vector &boxAbsMaxs, const truncatedcone_t &cone ) override;

	void			VCollideLoad( vcollide_t *pOutput, int solidCount, const char *pBuffer, int size, bool swap = false ) override;
	void			VCollideUnload( vcollide_t *pVCollide ) override;

	IVPhysicsKeyParser	*VPhysicsKeyParserCreate( const char *pKeyData ) override;
	IVPhysicsKeyParser	*VPhysicsKeyParserCreate( vcollide_t *pVCollide ) override_asw;
	void				VPhysicsKeyParserDestroy( IVPhysicsKeyParser *pParser ) override;

	int				CreateDebugMesh( CPhysCollide const *pCollisionModel, Vector **outVerts ) override;
	void			DestroyDebugMesh( int vertCount, Vector *outVerts ) override;

	ICollisionQuery *CreateQueryModel( CPhysCollide *pCollide ) override;
	void			DestroyQueryModel( ICollisionQuery *pQuery ) override;

	IPhysicsCollision *ThreadContextCreate() override;
	void			ThreadContextDestroy( IPhysicsCollision *pThreadContex ) override;

	CPhysCollide	*CreateVirtualMesh( const virtualmeshparams_t &params ) override;
	bool			SupportsVirtualMesh() override;


	bool			GetBBoxCacheSize( int *pCachedSize, int *pCachedCount ) override;

	
	CPolyhedron		*PolyhedronFromConvex( CPhysConvex * const pConvex, bool bUseTempPolyhedron ) override;

	void			OutputDebugInfo( const CPhysCollide *pCollide ) override;
	unsigned int	ReadStat( int statID ) override;

	float			CollideGetRadius( const CPhysCollide *pCollide ) override_asw;

	void			*VCollideAllocUserData( vcollide_t *pVCollide, size_t userDataSize ) override_asw;
	void			VCollideFreeUserData( vcollide_t *pVCollide ) override_asw;
	void			VCollideCheck( vcollide_t *pVCollide, const char *pName ) override_asw;

	bool			TraceBoxAA( const Ray_t &ray, const CPhysCollide *pCollide, trace_t *ptr ) override_csgo;

	void			DuplicateAndScale( vcollide_t *pOut, const vcollide_t *pIn, float flScale ) override_csgo;

public:
	static JoltPhysicsCollision& GetInstance() { return s_PhysicsCollision; }

private:
	static JoltPhysicsCollision s_PhysicsCollision;
};

JPH::RefConst<JPH::Shape> CreateCOMOverrideShape( const JPH::Shape* pShape, JPH::Vec3Arg comOverride );

void JoltEnsureSourceQueryData( JPH::Shape *pShape );

// Resolve the effective Source surface-prop index for the particular triangle
// touched by a Jolt manifold. A zero compact-triangle material falls back to
// the object's default material, matching IVP_Contact_Point::get_material_info.
int JoltResolveSourceContactMaterial( const JPH::Body &body,
	const JPH::SubShapeID &subShapeID, JPH::RVec3Arg worldContactPoint,
	JPH::Vec3Arg worldSurfaceNormal, int objectMaterialIndex );
