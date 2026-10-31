//========= Copyright Valve Corporation, All rights reserved. ============//
// Source sweep/simplex core. Adaptation provenance is recorded in the trace audit.
#pragma once
namespace VJoltSourceTrace
{
// Match the scalar Source normalization, including its zero-length guard.
static float Normalize( Vector &v )
{
	const float length = std::sqrt( ( v.x * v.x + v.y * v.y ) + v.z * v.z );
	const float inverse = 1.0f / ( length + FLT_EPSILON );
	v.x *= inverse;
	v.y *= inverse;
	v.z *= inverse;
	return length;
}
class ITraceObject
{
public:
	virtual int SupportMap( const Vector &, Vector * ) const = 0;
	virtual Vector GetVertByIndex( int ) const = 0;
	virtual float Radius() const = 0;
};
struct simplexvert_t
{
	Vector position;
	unsigned short testIndex : 15;
	unsigned short sweepIndex : 1;
	unsigned short obstacleIndex;
};

struct simplex_t
{
	simplexvert_t verts[ 4 ];
	int vertCount;

	inline bool PointSimplex( const simplexvert_t &newPoint, Vector *pOut );
	inline bool EdgeSimplex( const simplexvert_t &newPoint, int outIndex, const Vector &edge, Vector *pOut );
	inline bool TriangleSimplex( const simplexvert_t &newPoint, int outIndex, const Vector &faceNormal, Vector *pOut );

	bool SolveGJKSet( const simplexvert_t &newPoint, Vector *pOut );
	bool SolveVoronoiRegion2( const simplexvert_t &newPoint, Vector *pOut );
	bool SolveVoronoiRegion3( const simplexvert_t &newPoint, Vector *pOut );
	bool SolveVoronoiRegion4( const simplexvert_t &newPoint, Vector *pOut );

	Vector ClipRayToTetrahedronBase( const Vector &dir );
	Vector ClipRayToTetrahedron( const Vector &dir );
	float ClipRayToTriangle( const Vector &dir, float epsilon );
};

class CTraceAABB : public ITraceObject
{
public:
	CTraceAABB( const Vector &hlmins, const Vector &hlmaxs, bool isPoint );
	virtual int SupportMap( const Vector &dir, Vector *pOut ) const;
	virtual Vector GetVertByIndex( int index ) const;
	virtual float Radius( void ) const
	{
		return m_radius;
	}

private:
	float m_x[ 2 ];
	float m_y[ 2 ];
	float m_z[ 2 ];
	float m_radius;
	bool m_empty;
};

CTraceAABB::CTraceAABB( const Vector &hlmins, const Vector &hlmaxs, bool isPoint )
{
	if ( isPoint )
	{
		m_x[ 0 ] = m_x[ 1 ] = 0;
		m_y[ 0 ] = m_y[ 1 ] = 0;
		m_z[ 0 ] = m_z[ 1 ] = 0;
		m_radius = 0;
		m_empty = true;
	}
	else
	{
		m_x[ 0 ] = hlmaxs[ 0 ];
		m_x[ 1 ] = hlmins[ 0 ];
		m_y[ 0 ] = hlmaxs[ 1 ];
		m_y[ 1 ] = hlmins[ 1 ];
		m_z[ 0 ] = hlmaxs[ 2 ];
		m_z[ 1 ] = hlmins[ 2 ];
		m_radius = hlmaxs.Length();
		m_empty = false;
	}
}

int CTraceAABB::SupportMap( const Vector &dir, Vector *pOut ) const
{
	Vector out;

	if ( m_empty )
	{
		pOut->Init();
		return 0;
	}
	// index is formed by the 3-bit bitfield SzSySx (negative is 1, positive is 0)
	int x = ( ( *( (unsigned int *)&dir.x ) ) & 0x80000000UL ) >> 31;
	int y = ( ( *( (unsigned int *)&dir.y ) ) & 0x80000000UL ) >> 31;
	int z = ( ( *( (unsigned int *)&dir.z ) ) & 0x80000000UL ) >> 31;
	pOut->x = m_x[ x ];
	pOut->y = m_y[ y ];
	pOut->z = m_z[ z ];
	return ( z << 2 ) | ( y << 1 ) | x;
}

Vector CTraceAABB::GetVertByIndex( int index ) const
{
	Vector out;
	out.x = m_x[ ( index & 1 ) ];
	out.y = m_y[ ( index & 2 ) >> 1 ];
	out.z = m_z[ ( index & 4 ) >> 2 ];

	return out;
}

//-----------------------------------------------------------------------------
// Purpose: Implementation for Trace against an IVP object
//-----------------------------------------------------------------------------
class CTraceRay
{
public:
	CTraceRay( const Vector &hlstart, const Vector &hlend );
	CTraceRay( const Ray_t &ray );
	CTraceRay( const Ray_t &ray, const Vector &offset );
	void Init( const Vector &hlstart, const Vector &delta );
	int SupportMap( const Vector &dir, Vector *pOut ) const;
	Vector GetVertByIndex( int index ) const
	{
		return ( index ) ? m_end : m_start;
	}
	float Radius( void ) const
	{
		return m_length * 0.5f;
	}

	void Reset( float fraction );

	Vector m_start;
	Vector m_end;
	Vector m_delta;
	Vector m_dir;

	float m_length;
	float m_baseLength;
	float m_ooBaseLength;
	float m_bestDist;
};
CTraceRay::CTraceRay( const Vector &hlstart, const Vector &hlend )
{
	Init( hlstart, hlend - hlstart );
}

void CTraceRay::Init( const Vector &hlstart, const Vector &delta )
{
	m_start = hlstart;
	m_end = hlstart + delta;
	m_delta = delta;
	m_dir = delta;
	float len = DotProduct( delta, delta );
	// don't use fast/sse sqrt here we need the precision
	m_length = sqrt( len );
	m_ooBaseLength = 0.0f;
	if ( m_length > 0 )
	{
		m_ooBaseLength = 1.0f / m_length;
		m_dir *= m_ooBaseLength;
	}
	m_baseLength = m_length;
	m_bestDist = 0.f;
}

CTraceRay::CTraceRay( const Ray_t &ray )
{
	Init( ray.m_Start, ray.m_Delta );
}

CTraceRay::CTraceRay( const Ray_t &ray, const Vector &offset )
{
	Vector start;
	VectorAdd( ray.m_Start, offset, start );
	Init( start, ray.m_Delta );
}

void CTraceRay::Reset( float fraction )
{
	// recompute from base values for max precision
	m_length = m_baseLength * fraction;
	m_end = m_start + fraction * m_delta;
	m_bestDist = 0.f;
}

int CTraceRay::SupportMap( const Vector &dir, Vector *pOut ) const
{
	if ( DotProduct( dir, m_delta ) > 0 )
	{
		*pOut = m_end;
		return 1;
	}
	*pOut = m_start;
	return 0;
}

static const char *map_nullname = "**empty**";
static csurface_t nullsurface = { map_nullname, 0 };

static void CM_ClearTrace( trace_t *trace )
{
	memset( trace, 0, sizeof( *trace ) );
	trace->fraction = 1.f;
	trace->fractionleftsolid = 0;
	trace->surface = nullsurface;
}

class CDefConvexInfo : public IConvexInfo
{
public:
	IConvexInfo *GetPtr()
	{
		return this;
	}

	virtual unsigned int GetContents( int convexGameData )
	{
		return CONTENTS_SOLID;
	}
};

class CTraceSolver
{
public:
	CTraceSolver( trace_t *ptr, ITraceObject *sweepobject, CTraceRay *ray, ITraceObject *obstacle, const Vector &axis )
	{
		m_pTotalTrace = ptr;
		m_sweepObject = sweepobject;
		m_sweepObjectRadius = m_sweepObject->Radius();
		m_obstacle = obstacle;
		m_ray = ray;
		m_traceLength = 0;
		m_totalTraceLength = max( ray->m_baseLength, 1e-8f );
		m_pointClosestToIntersection = axis;
		m_epsilon = DIST_EPSILON;
	}

	bool SweepSingleConvex( void );
	bool SolveMeshIntersection( simplex_t &simplex, float &distance );
	float SolveMeshIntersection2D( simplex_t &simplex );
	virtual void DoSweep( void )
	{
		SweepSingleConvex();
		*m_pTotalTrace = m_trace;
	}

	void SetEpsilon( float epsilon )
	{
		m_epsilon = epsilon;
	}

public:
	trace_t m_trace;

	Vector m_pointClosestToIntersection;
	ITraceObject *m_sweepObject;
	ITraceObject *m_obstacle;
	CTraceRay *m_ray;
	trace_t *m_pTotalTrace;
	float m_traceLength;
	float m_totalTraceLength;
	float m_sweepObjectRadius;
	float m_epsilon;

private:
	CTraceSolver( const CTraceSolver & );
};

static void CalculateSeparatingPlane( trace_t *, ITraceObject *, CTraceRay *, ITraceObject *, simplex_t & );
#define Normalize_FastLowPrecision Normalize
#define TEST_EPSILON ( 1.0f / 256.0f )
#define CHECK_TOI_CALCS 0
bool CTraceSolver::SweepSingleConvex( void )
{
	simplex_t simplex;
	simplexvert_t vert;
	Vector tmp;

	simplex.vertCount = 0;

	if ( m_pointClosestToIntersection == vec3_origin )
	{
		m_pointClosestToIntersection.Init( 1, 0, 0 );
	}

	float testLen = 1;
	Vector dir = -m_pointClosestToIntersection;
	Normalize_FastLowPrecision( dir );
	// safe loop, max 100 iterations
	for ( int i = 0; i < 100; i++ )
	{
		// map the direction into the minkowski sum, get a new surface point
		vert.testIndex = m_sweepObject->SupportMap( dir, &vert.position );
		vert.sweepIndex = m_ray->SupportMap( dir, &tmp );
		VectorAdd( vert.position, tmp, vert.position );
		vert.obstacleIndex = m_obstacle->SupportMap( -dir, &tmp );
		VectorSubtract( vert.position, tmp, vert.position );

		testLen = DotProduct( dir, vert.position );
		// found a separating axis, no intersection
		if ( testLen < 0 )
		{
			// make sure we're separated by at least m_epsilon
			testLen = fabs( testLen );
			if ( testLen < m_epsilon && m_ray->m_length > 0 )
			{
				// not separated by enough
				Vector normal = dir;
				if ( testLen > 0 )
				{
					// try to find a better separating plane or clip the ray to the current one
					for ( int j = 0; j < 20; j++ )
					{
						Vector lastVert = vert.position;
						simplex.SolveGJKSet( vert, &m_pointClosestToIntersection );
						dir = -m_pointClosestToIntersection;
						Normalize_FastLowPrecision( dir );
						// map the direction into the minkowski sum, get a new surface point
						vert.testIndex = m_sweepObject->SupportMap( dir, &vert.position );
						vert.sweepIndex = m_ray->SupportMap( dir, &tmp );
						VectorAdd( vert.position, tmp, vert.position );
						vert.obstacleIndex = m_obstacle->SupportMap( -dir, &tmp );
						VectorSubtract( vert.position, tmp, vert.position );

						// found a separating axis, no intersection
						float est = -DotProduct( dir, vert.position );
						if ( est > m_epsilon ) // big enough separation, no hit
							return false;
						// take plane with the most separation
						if ( est > testLen )
						{
							testLen = est;
							normal = dir;
						}
						float last = -DotProduct( dir, lastVert );
						// search is not converging, exit.
						if ( ( est - last ) > -1e-4f )
							break;
					}
				}

				// This trace is going to miss, but not by enough.
				// Hit the separating plane instead
				float dot = -DotProduct( m_ray->m_delta, normal );
				if ( dot < -( m_epsilon * 0.1 ) || ( dot < -1e-4f && testLen < ( m_epsilon * 0.9 ) ) )
				{
					CM_ClearTrace( &m_trace );
					float backupDistance = m_epsilon - testLen;
					backupDistance = -( backupDistance * m_ray->m_baseLength ) / dot;
					m_traceLength = m_ray->m_length - backupDistance;
					if ( m_traceLength < 0 )
					{
						m_traceLength = 0;
						// try sliding along the surface of the minkowski sum
						backupDistance = SolveMeshIntersection2D( simplex );
						if ( m_ray->m_length > backupDistance )
						{
							m_traceLength = m_ray->m_length - backupDistance;
						}
					}
					m_trace.plane.normal = -normal;
					// this is fixed up by the outer code
					// m_trace.endpos = m_ray->m_start*(1.f-m_trace.fraction) + m_ray->m_end*m_trace.fraction;
					m_trace.contents = CONTENTS_SOLID;
					return true;
				}
			}
			return false;
		}

		// contains the origin
		if ( simplex.SolveGJKSet( vert, &m_pointClosestToIntersection ) )
		{
			CM_ClearTrace( &m_trace );
			// now solve for t along the sweep
			if ( m_ray->m_length != 0 )
			{
				float dist;
				if ( SolveMeshIntersection( simplex, dist ) && dist < m_ray->m_length )
				{
					m_traceLength = ( m_ray->m_length - dist );
					CalculateSeparatingPlane( &m_trace, m_sweepObject, m_ray, m_obstacle, simplex );
					float dot = DotProduct( m_ray->m_dir, m_trace.plane.normal );
					if ( dot < 0 )
					{
						m_traceLength += ( m_epsilon / dot );
					}

					if ( m_traceLength < 0 )
					{
						m_traceLength = 0;
					}
					// m_trace.fraction = m_traceLength * m_ray->m_ooBaseLength;
					// m_trace.endpos = m_ray->m_start*(1.f-m_trace.fraction) + m_ray->m_end*m_trace.fraction;
					m_trace.contents = CONTENTS_SOLID;
				}
				else
				{
					// UNDONE: This case happens when you start solid as well as when a false
					// intersection is detected at the very end of the trace
					m_trace.startsolid = true;
					m_trace.allsolid = true;
					m_traceLength = 0;
				}
			}
			else
			{
				m_trace.startsolid = true;
				m_trace.allsolid = true;
				m_traceLength = 0;
			}
			return true;
		}
		dir = -m_pointClosestToIntersection;
		Normalize_FastLowPrecision( dir );
	}

	// BUGBUG: The solution never converged - something is probably wrong!
	AssertMsg( false, "Solution never converged." );
	return false;
}

// NEW SWEPT GJK SOLVER 2/16/2006
// convenience routines - just makes the code a little simpler.
FORCEINLINE bool simplex_t::TriangleSimplex( const simplexvert_t &newPoint, int outIndex, const Vector &faceNormal,
											 Vector *pOut )
{
	vertCount = 3;
	verts[ outIndex ] = newPoint;
	*pOut = -faceNormal;
	return false;
}
FORCEINLINE bool simplex_t::EdgeSimplex( const simplexvert_t &newPoint, int outIndex, const Vector &edge, Vector *pOut )
{
	vertCount = 2;
	verts[ outIndex ] = newPoint;
	Vector cross;
	CrossProduct( edge, newPoint.position, cross );
	CrossProduct( cross, edge, *pOut );
	return false;
}

FORCEINLINE bool simplex_t::PointSimplex( const simplexvert_t &newPoint, Vector *pOut )
{
	vertCount = 1;
	verts[ 0 ] = newPoint;
	*pOut = newPoint.position;
	return false;
}

// In general the voronoi region routines have comments referring to the verts
// All of the code assumes that vert A is the new vert being added to the set
// verts B, C, D are the previous set.  If BCD is a triangle it is assumed to be
// counter-clockwise winding order.  This must be maintained by the code!

// parametric value for closes point on a line segment (p0->p1) to the origin.
bool simplex_t::SolveVoronoiRegion2( const simplexvert_t &newPoint, Vector *pOut )
{
	// solve the line segment AB (where A is the new point)
	Vector AB = verts[ 0 ].position - newPoint.position;
	float d = DotProduct( AB, newPoint.position );
	if ( d < 0 )
	{
		return EdgeSimplex( newPoint, 1, AB, pOut );
	}
	else
	{
		return PointSimplex( newPoint, pOut );
	}
}

// UNDONE: Collapse these routines into a single general routine?
bool simplex_t::SolveVoronoiRegion3( const simplexvert_t &newPoint, Vector *pOut )
{
	// solve the triangle ABC (where A is the new point)
	Vector AB = verts[ 0 ].position - newPoint.position;
	Vector AC = verts[ 1 ].position - newPoint.position;
	Vector ABC;
	CrossProduct( AB, AC, ABC );
	Vector ABCxAC;
	CrossProduct( ABC, AC, ABCxAC );
	float d = DotProduct( ABCxAC, newPoint.position );
	// edge AC or edgeAB / A?
	if ( d < 0 )
	{
		d = DotProduct( AC, newPoint.position );
		if ( d < 0 )
		{
			// edge AC
			return EdgeSimplex( newPoint, 0, AC, pOut );
		}
	}
	else
	{
		// face or A / edge AB?
		Vector ABxABC;
		CrossProduct( AB, ABC, ABxABC );
		d = DotProduct( ABxABC, newPoint.position );
		if ( d > 0 )
		{
			// closest to face
			vertCount = 3;
			d = DotProduct( ABC, newPoint.position );
			// in front of face, return opposite direction
			if ( d < 0 )
			{
				verts[ 2 ] = newPoint;
				*pOut = -ABC;
				return false;
			}
			verts[ 2 ] = verts[ 1 ]; // swap to keep CCW
			verts[ 1 ] = newPoint;
			*pOut = ABC;
			return false;
		}
	}
	// edge AB or A
	d = DotProduct( AB, newPoint.position );
	if ( d < 0 )
	{
		return EdgeSimplex( newPoint, 1, AB, pOut );
	}
	return PointSimplex( newPoint, pOut );
}

bool simplex_t::SolveVoronoiRegion4( const simplexvert_t &newPoint, Vector *pOut )
{
	// solve the tetrahedron ABCD (where A is the new point)

	// compute edge vectors
	Vector AB = verts[ 0 ].position - newPoint.position;
	Vector AC = verts[ 1 ].position - newPoint.position;
	Vector AD = verts[ 2 ].position - newPoint.position;

	// compute face normals
	Vector ABC, ACD, ADB;
	CrossProduct( AB, AC, ABC );
	CrossProduct( AC, AD, ACD );
	CrossProduct( AD, AB, ADB );

	// edge plane normals
	Vector ABCxAC, ABxABC;
	CrossProduct( ABC, AC, ABCxAC );
	CrossProduct( AB, ABC, ABxABC );
	Vector ACDxAD, ACxACD;
	CrossProduct( ACD, AD, ACDxAD );
	CrossProduct( AC, ACD, ACxACD );
	Vector ADBxAB, ADxADB;
	CrossProduct( ADB, AB, ADBxAB );
	CrossProduct( AD, ADB, ADxADB );

	int faceFlags = 0;
	float d;
	d = DotProduct( ABC, newPoint.position );
	if ( d < 0 )
	{
		faceFlags |= 0x1;
	}
	d = DotProduct( ACD, newPoint.position );
	if ( d < 0 )
	{
		faceFlags |= 0x2;
	}
	d = DotProduct( ADB, newPoint.position );
	if ( d < 0 )
	{
		faceFlags |= 0x4;
	}

	switch ( faceFlags )
	{
	case 0:
		// inside all 3 faces, we're done
		verts[ 3 ] = newPoint;
		vertCount = 4;
		return true;
	case 1:
		// ABC only, solve as a triangle
		return SolveVoronoiRegion3( newPoint, pOut );
	case 2:
		// ACD only, solve as a triangle
		verts[ 0 ] = verts[ 2 ]; // collapse BCD to DC
		return SolveVoronoiRegion3( newPoint, pOut );
	case 4:
		// ADB only, solve as a triangle
		verts[ 1 ] = verts[ 2 ]; // collapse BCD to BD
		return SolveVoronoiRegion3( newPoint, pOut );
	case 3: {
		// in front of ABC & ACD
		d = DotProduct( ABCxAC, newPoint.position );
		if ( d < 0 )
		{
			d = DotProduct( ACxACD, newPoint.position );
			if ( d < 0 )
			{
				d = DotProduct( AC, newPoint.position );
				if ( d < 0 )
				{
					// edge AC
					return EdgeSimplex( newPoint, 0, AC, pOut );
				}
				// point A
				return PointSimplex( newPoint, pOut );
			}
			else
			{
				d = DotProduct( ACDxAD, newPoint.position );
				if ( d < 0 )
				{
					// edge AD
					verts[ 0 ] = verts[ 2 ]; // collapse BCD to D
					return EdgeSimplex( newPoint, 1, AD, pOut );
				}
				// face ACD
				return TriangleSimplex( newPoint, 0, ACD, pOut );
			}
		}
		else
		{
			d = DotProduct( ABxABC, newPoint.position );
			if ( d < 0 )
			{
				d = DotProduct( AB, newPoint.position );
				if ( d < 0 )
				{
					// edge AB
					return EdgeSimplex( newPoint, 1, AB, pOut );
				}
				return PointSimplex( newPoint, pOut );
			}
			return TriangleSimplex( newPoint, 2, ABC, pOut );
		}
	}
	break;
	case 5: {
		// in front of ADB & ABC
		d = DotProduct( ADBxAB, newPoint.position );
		if ( d < 0 )
		{
			d = DotProduct( ABxABC, newPoint.position );
			if ( d < 0 )
			{
				d = DotProduct( AB, newPoint.position );
				if ( d < 0 )
				{
					// edge AB
					return EdgeSimplex( newPoint, 1, AB, pOut );
				}
				// point A
				return PointSimplex( newPoint, pOut );
			}
			else
			{
				d = DotProduct( ABCxAC, newPoint.position );
				if ( d < 0 )
				{
					// edge AC
					return EdgeSimplex( newPoint, 0, AC, pOut );
				}
				// face ABC
				return TriangleSimplex( newPoint, 2, ABC, pOut );
			}
		}
		else
		{
			d = DotProduct( ADxADB, newPoint.position );
			if ( d < 0 )
			{
				d = DotProduct( AD, newPoint.position );
				if ( d < 0 )
				{
					// edge AD
					verts[ 0 ] = verts[ 2 ]; // collapse BCD to D
					return EdgeSimplex( newPoint, 1, AD, pOut );
				}
				return PointSimplex( newPoint, pOut );
			}
			return TriangleSimplex( newPoint, 1, ADB, pOut );
		}
	}
	break;
	case 6: {
		// in front of ACD & ADB
		d = DotProduct( ACDxAD, newPoint.position );
		if ( d < 0 )
		{
			d = DotProduct( ADxADB, newPoint.position );
			if ( d < 0 )
			{
				d = DotProduct( AD, newPoint.position );
				if ( d < 0 )
				{
					// edge AD
					verts[ 0 ] = verts[ 2 ]; // collapse BCD to D
					return EdgeSimplex( newPoint, 1, AD, pOut );
				}
				// point A
				return PointSimplex( newPoint, pOut );
			}
			else
			{
				d = DotProduct( ADBxAB, newPoint.position );
				if ( d < 0 )
				{
					// edge AB
					return EdgeSimplex( newPoint, 1, AB, pOut );
				}
				// face ADB
				return TriangleSimplex( newPoint, 1, ADB, pOut );
			}
		}
		else
		{
			d = DotProduct( ACxACD, newPoint.position );
			if ( d < 0 )
			{
				d = DotProduct( AC, newPoint.position );
				if ( d < 0 )
				{
					// edge AC
					return EdgeSimplex( newPoint, 0, AC, pOut );
				}
				return PointSimplex( newPoint, pOut );
			}
			return TriangleSimplex( newPoint, 0, ACD, pOut );
		}
	}
	break;
	case 7: {
		d = DotProduct( AB, newPoint.position );
		if ( d < 0 )
		{
			return EdgeSimplex( newPoint, 1, AB, pOut );
		}
		else
		{
			d = DotProduct( AC, newPoint.position );
			if ( d < 0 )
			{
				return EdgeSimplex( newPoint, 0, AC, pOut );
			}
			else
			{
				d = DotProduct( AD, newPoint.position );
				if ( d < 0 )
				{
					verts[ 0 ] = verts[ 2 ]; // collapse BCD to D
					return EdgeSimplex( newPoint, 1, AD, pOut );
				}
				return PointSimplex( newPoint, pOut );
			}
		}
	}
	}
	verts[ 3 ] = newPoint;
	vertCount = 4;
	return true;
}

bool simplex_t::SolveGJKSet( const simplexvert_t &w, Vector *pOut )
{

#if 0
	for ( int v = 0; v < vertCount; v++ )
	{
		for ( int v2 = v+1; v2 < vertCount; v2++ )
		{
			if ( (verts[v].obstacleIndex == verts[v2].obstacleIndex) &&
				(verts[v].sweepIndex == verts[v2].sweepIndex) &&
				(verts[v].testIndex == verts[v2].testIndex) )
			{
				// same vert in the list twice!  degenerate
				Assert(0);
			}
		}
	}
#endif

	switch ( vertCount )
	{
	case 0:
		vertCount = 1;
		verts[ 0 ] = w;
		*pOut = w.position;
		return false;
	case 1:
		return SolveVoronoiRegion2( w, pOut );
	case 2:
		return SolveVoronoiRegion3( w, pOut );
	case 3:
		return SolveVoronoiRegion4( w, pOut );
	}

	return true;
}

void CalculateSeparatingPlane( trace_t *ptr, ITraceObject *sweepObject, CTraceRay *ray, ITraceObject *obstacle,
							   simplex_t &simplex )
{
	int testCount = 1, obstacleCount = 1;
	unsigned int testIndex[ 4 ], obstacleIndex[ 4 ];
	testIndex[ 0 ] = simplex.verts[ 0 ].testIndex;
	obstacleIndex[ 0 ] = simplex.verts[ 0 ].obstacleIndex;
	Assert( simplex.vertCount <= 4 );
	int i, j;
	for ( i = 1; i < simplex.vertCount; i++ )
	{
		for ( j = 0; j < obstacleCount; j++ )
		{
			if ( obstacleIndex[ j ] == simplex.verts[ i ].obstacleIndex )
				break;
		}
		if ( j == obstacleCount )
		{
			obstacleIndex[ obstacleCount++ ] = simplex.verts[ i ].obstacleIndex;
		}

		for ( j = 0; j < testCount; j++ )
		{
			if ( testIndex[ j ] == simplex.verts[ i ].testIndex )
				break;
		}
		if ( j == testCount )
		{
			testIndex[ testCount++ ] = simplex.verts[ i ].testIndex;
		}
	}

	if ( simplex.vertCount < 3 )
	{
		if ( simplex.vertCount == 2 && testCount == 2 )
		{
			// edge / point
			Vector t0 = sweepObject->GetVertByIndex( testIndex[ 0 ] );
			Vector t1 = sweepObject->GetVertByIndex( testIndex[ 1 ] );
			Vector edge = t1 - t0;
			Vector tangent = CrossProduct( edge, ray->m_delta );
			ptr->plane.normal = CrossProduct( edge, tangent );
			Normalize( ptr->plane.normal );
			ptr->plane.dist = DotProduct( t0 + ptr->endpos, ptr->plane.normal );
			return;
		}
	}
	if ( testCount == 3 )
	{
		// face / xxx
		Vector t0 = sweepObject->GetVertByIndex( testIndex[ 0 ] );
		Vector t1 = sweepObject->GetVertByIndex( testIndex[ 1 ] );
		Vector t2 = sweepObject->GetVertByIndex( testIndex[ 2 ] );
		ptr->plane.normal = CrossProduct( t1 - t0, t2 - t0 );
		Normalize( ptr->plane.normal );
		if ( DotProduct( ptr->plane.normal, ray->m_delta ) > 0 )
		{
			ptr->plane.normal = -ptr->plane.normal;
		}
		ptr->plane.dist = DotProduct( t0 + ptr->endpos, ptr->plane.normal );
	}
	else if ( testCount == 2 && obstacleCount == 2 )
	{
		// edge / edge
		Vector t0 = sweepObject->GetVertByIndex( testIndex[ 0 ] );
		Vector t1 = sweepObject->GetVertByIndex( testIndex[ 1 ] );
		Vector t2 = obstacle->GetVertByIndex( obstacleIndex[ 0 ] );
		Vector t3 = obstacle->GetVertByIndex( obstacleIndex[ 1 ] );
		ptr->plane.normal = CrossProduct( t1 - t0, t3 - t2 );
		Normalize( ptr->plane.normal );
		if ( DotProduct( ptr->plane.normal, ray->m_delta ) > 0 )
		{
			ptr->plane.normal = -ptr->plane.normal;
		}
		ptr->plane.dist = DotProduct( t0 + ptr->endpos, ptr->plane.normal );
	}
	else if ( obstacleCount == 3 )
	{
		// xxx / face
		Vector t0 = obstacle->GetVertByIndex( obstacleIndex[ 0 ] );
		Vector t1 = obstacle->GetVertByIndex( obstacleIndex[ 1 ] );
		Vector t2 = obstacle->GetVertByIndex( obstacleIndex[ 2 ] );
		ptr->plane.normal = CrossProduct( t1 - t0, t2 - t0 );
		Normalize( ptr->plane.normal );
		if ( DotProduct( ptr->plane.normal, ray->m_delta ) > 0 )
		{
			ptr->plane.normal = -ptr->plane.normal;
		}
		ptr->plane.dist = DotProduct( t0, ptr->plane.normal );
	}
	else
	{
		ptr->plane.normal = -ray->m_dir;
		if ( simplex.vertCount )
		{
			ptr->plane.dist =
				DotProduct( ptr->plane.normal, obstacle->GetVertByIndex( simplex.verts[ 0 ].obstacleIndex ) );
		}
		else
			ptr->plane.dist = 0.f;
	}
}

// clip a direction vector to a plane (ray begins at the origin)
inline float Clip( const Vector &dir, const Vector &pos, const Vector &normal )
{
	float dist = DotProduct( pos, normal );
	float cosTheta = DotProduct( dir, normal );
	if ( cosTheta > 0 )
		return dist / cosTheta;

	// parallel or not facing the plane
	return 1e16f;
}

// This is the first iteration of solving time of intersection.
// It needs to test all 4 faces of the tetrahedron to find the one the ray leaves through
// this is done by finding the closest plane by clipping the ray to each plane
Vector simplex_t::ClipRayToTetrahedronBase( const Vector &dir )
{
	Vector AB = verts[ 0 ].position - verts[ 3 ].position;
	Vector AC = verts[ 1 ].position - verts[ 3 ].position;
	Vector AD = verts[ 2 ].position - verts[ 3 ].position;
	Vector BC = verts[ 1 ].position - verts[ 0 ].position;
	Vector BD = verts[ 2 ].position - verts[ 0 ].position;

	// compute face normals
	Vector ABC, ACD, ADB, BCD;
	CrossProduct( AB, AC, ABC );
	CrossProduct( AC, AD, ACD );
	CrossProduct( AD, AB, ADB );
	CrossProduct( BD, BC, BCD ); // flipped to point out of the tetrahedron

	// NOTE: These cancel out in the clipping equation
	// Normalize(ABC);
	// Normalize(ACD);
	// Normalize(ADB);
	// Normalize(BCD);

	// Assert valid tetrahedron/winding order
#if CHECK_TOI_CALCS
	Assert( DotProduct( verts[ 2 ].position, ABC ) < DotProduct( verts[ 3 ].position, ABC ) );
	Assert( DotProduct( verts[ 0 ].position, ACD ) < DotProduct( verts[ 3 ].position, ACD ) );
	Assert( DotProduct( verts[ 1 ].position, ADB ) < DotProduct( verts[ 3 ].position, ADB ) );
	Assert( DotProduct( verts[ 3 ].position, BCD ) < DotProduct( verts[ 0 ].position, BCD ) );
#endif

	float dists[ 4 ];
	dists[ 0 ] = Clip( dir, verts[ 3 ].position, ABC );
	dists[ 1 ] = Clip( dir, verts[ 3 ].position, ACD );
	dists[ 2 ] = Clip( dir, verts[ 3 ].position, ADB );
	dists[ 3 ] = Clip( dir, verts[ 0 ].position, BCD );
	float dmin = dists[ 3 ];
	int best = 3;
	for ( int i = 0; i < 3; i++ )
	{
		if ( dists[ i ] < dmin )
		{
			best = i;
			dmin = dists[ i ];
		}
	}

	vertCount = 3;
	// push back down to a triangle
	// Note that we need to preserve winding so that the vector order assumptions above are still valid!
	switch ( best )
	{
	case 0:
		verts[ 2 ] = verts[ 3 ];
		return ABC;
	case 1:
		verts[ 0 ] = verts[ 3 ];
		return ACD;
	case 2:
		verts[ 1 ] = verts[ 3 ];
		return ADB;
	case 3:
		// swap 1 & 2
		verts[ 3 ] = verts[ 1 ];
		verts[ 1 ] = verts[ 2 ];
		verts[ 2 ] = verts[ 3 ];
		return BCD;
	}
	Assert( 0 ); // failed!
	return vec3_origin;
}

// for subsequent iterations you don't need to test one of the faces (face you previously left through)
// so only test the three faces involving the new point A
Vector simplex_t::ClipRayToTetrahedron( const Vector &dir )
{
	Vector AB = verts[ 0 ].position - verts[ 3 ].position;
	Vector AC = verts[ 1 ].position - verts[ 3 ].position;
	Vector AD = verts[ 2 ].position - verts[ 3 ].position;

	// compute face normals
	Vector ABC, ACD, ADB;
	CrossProduct( AB, AC, ABC );
	CrossProduct( AC, AD, ACD );
	CrossProduct( AD, AB, ADB );

	// NOTE: These cancel out in the clipping equation
	// Normalize(ABC);
	// Normalize(ACD);
	// Normalize(ADB);

	// Assert valid tetrahedron/winding order
#if CHECK_TOI_CALCS
	Assert( DotProduct( verts[ 2 ].position, ABC ) < DotProduct( verts[ 3 ].position, ABC ) );
	Assert( DotProduct( verts[ 0 ].position, ACD ) < DotProduct( verts[ 3 ].position, ACD ) );
	Assert( DotProduct( verts[ 1 ].position, ADB ) < DotProduct( verts[ 3 ].position, ADB ) );
#endif

	float dists[ 3 ];
	dists[ 0 ] = Clip( dir, verts[ 3 ].position, ABC );
	dists[ 1 ] = Clip( dir, verts[ 3 ].position, ACD );
	dists[ 2 ] = Clip( dir, verts[ 3 ].position, ADB );

	float dmin = dists[ 2 ];
	int best = 2;
	for ( int i = 0; i < 2; i++ )
	{
		if ( dists[ i ] < dmin )
		{
			best = i;
			dmin = dists[ i ];
		}
	}

	vertCount = 3;
	// push back down to a triangle
	// Note that we need to preserve winding so that the vector order assumptions above are still valid!
	switch ( best )
	{
	case 0:
		verts[ 2 ] = verts[ 3 ];
		return ABC;
	case 1:
		verts[ 0 ] = verts[ 3 ];
		return ACD;
	case 2:
		verts[ 1 ] = verts[ 3 ];
		return ADB;
	}
	return vec3_origin;
}

float simplex_t::ClipRayToTriangle( const Vector &dir, float epsilon )
{
	Vector AB = verts[ 0 ].position - verts[ 2 ].position;
	Vector AC = verts[ 1 ].position - verts[ 2 ].position;
	Vector BC = verts[ 1 ].position - verts[ 0 ].position;

	// compute face normal
	Vector ABC;
	CrossProduct( AB, AC, ABC ); // this points toward the origin
	Normalize( ABC );
	Vector edgeAB, edgeAC, edgeBC;
	// these should point out of the triangle
	CrossProduct( AB, ABC, edgeAB );
	CrossProduct( ABC, AC, edgeAC );
	CrossProduct( BC, ABC, edgeBC );

	// NOTE: These cancel out in the clipping equation
	Normalize( edgeAB );
	Normalize( edgeAC );
	Normalize( edgeBC );

#if CHECK_TOI_CALCS
	Assert( DotProduct( verts[ 2 ].position, ABC ) < 0 ); // points toward
	// Assert valid triangle/winding order (all normals point away from the origin)
	Assert( DotProduct( verts[ 2 ].position, edgeAB ) > 0 );
	Assert( DotProduct( verts[ 2 ].position, edgeAC ) > 0 );
	Assert( DotProduct( verts[ 0 ].position, edgeBC ) > 0 );
#endif

	float dists[ 3 ];
	dists[ 0 ] = Clip( dir, verts[ 0 ].position, edgeAB );
	dists[ 1 ] = Clip( dir, verts[ 1 ].position, edgeAC );
	dists[ 2 ] = Clip( dir, verts[ 1 ].position, edgeBC );
	Vector *normals[ 3 ] = { &edgeAB, &edgeAC, &edgeBC };

	float dmin = dists[ 0 ];
	int best = 0;
	if ( dists[ 1 ] < dmin )
	{
		dmin = dists[ 1 ];
		best = 1;
	}
	if ( dists[ 2 ] < dmin )
	{
		best = 2;
		dmin = dists[ 2 ];
	}
	float dot = DotProduct( dir, *normals[ best ] );
	if ( dot <= 0 )
		return 1e16f;
	dmin += epsilon / dot;

	return dmin;
}

// Solve for time of intersection along the sweep
// Do this by iteratively clipping the ray to the tetrahedron containing the origin
// when a triangle is found intersecting the ray, reduce the simplex to that triangle
// and then re-expand it to a tetrahedron using the support point normal to the triangle (away from the origin)
// iterate until no new points can be found.  That's the surface of the sum.
bool CTraceSolver::SolveMeshIntersection( simplex_t &simplex, float &distance )
{
	Vector tmp;
	Assert( simplex.vertCount == 4 );
	if ( simplex.vertCount < 4 )
		return false;

	Vector v = simplex.ClipRayToTetrahedronBase( m_ray->m_dir );

	simplexvert_t vert;
	// safe loop, max 100 iterations
	for ( int i = 0; i < 100; i++ )
	{
		Normalize( v );
		vert.testIndex = m_sweepObject->SupportMap( v, &vert.position );
		vert.sweepIndex = m_ray->SupportMap( v, &tmp );
		VectorAdd( vert.position, tmp, vert.position );
		vert.obstacleIndex = m_obstacle->SupportMap( -v, &tmp );
		VectorSubtract( vert.position, tmp, vert.position );

		// map the new separating axis (NOTE: This test is inverted from the GJK - we are trying to get out, not in)
		// found a separating axis, we've moved the sweep back enough
		float dist = DotProduct( v, simplex.verts[ 0 ].position ) + TEST_EPSILON;
		if ( DotProduct( v, vert.position ) <= dist )
		{
			Vector BC = simplex.verts[ 1 ].position - simplex.verts[ 0 ].position;
			Vector BD = simplex.verts[ 2 ].position - simplex.verts[ 0 ].position;

			// compute face normals
			Vector BCD;
			CrossProduct( BC, BD, BCD );
			// NOTE: This cancels out inside Clip()
			// Normalize( BCD );
			// clip ray to triangle
			distance = Clip( m_ray->m_dir, simplex.verts[ 0 ].position, BCD );
			return true;
		}

		// add the new vert
		simplex.verts[ simplex.vertCount ] = vert;
		simplex.vertCount++;
		v = simplex.ClipRayToTetrahedron( m_ray->m_dir );
	}

	Assert( 0 );
	return false;
}

// similar to SolveMeshIntersection, but solves projected into the 2D triangle simplex remaining
// this is used for the near miss case
float CTraceSolver::SolveMeshIntersection2D( simplex_t &simplex )
{
	AssertMsg( simplex.vertCount == 3, "simplex.vertCount != 3: %d", simplex.vertCount );
	if ( simplex.vertCount != 3 )
		return 0.0f;

	// note: This should really do this iteratively in case the triangle is coplanar with another triangle that
	// is between this one and the edge of the sum in this plane
	float dist = simplex.ClipRayToTriangle( m_ray->m_dir, m_epsilon );
	return dist;
}

#undef Normalize_FastLowPrecision
#undef TEST_EPSILON
#undef CHECK_TOI_CALCS
} // namespace VJoltSourceTrace
