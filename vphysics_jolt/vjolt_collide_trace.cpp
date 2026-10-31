// Source-compatible public VPhysics traces over preserved collision geometry.
#include "cbase.h"
#include "coordsize.h"
#include "vjolt_collide.h"
#include "vjolt_source_trace.inl"

// memdbgon must be the last include file in a .cpp file!
#include "tier0/memdbgon.h"

void JoltPhysicsCollision::TraceBox( const Vector &start, const Vector &end, const Vector &mins, const Vector &maxs,
									 const CPhysCollide *pCollide, const Vector &collideOrigin,
									 const QAngle &collideAngles, trace_t *ptr )
{
	Ray_t ray;
	ray.Init( start, end, mins, maxs );
	VJoltSourceTrace::TraceBox( ray, MASK_ALL, nullptr, pCollide, collideOrigin, collideAngles, ptr );
}

void JoltPhysicsCollision::TraceBox( const Ray_t &ray, const CPhysCollide *pCollide, const Vector &collideOrigin,
									 const QAngle &collideAngles, trace_t *ptr )
{
	VJoltSourceTrace::TraceBox( ray, MASK_ALL, nullptr, pCollide, collideOrigin, collideAngles, ptr );
}

void JoltPhysicsCollision::TraceBox( const Ray_t &ray, unsigned int contentsMask, IConvexInfo *pConvexInfo,
									 const CPhysCollide *pCollide, const Vector &collideOrigin,
									 const QAngle &collideAngles, trace_t *ptr )
{
	VJoltSourceTrace::TraceBox( ray, contentsMask, pConvexInfo, pCollide, collideOrigin, collideAngles, ptr );
}

void JoltPhysicsCollision::TraceCollide( const Vector &start, const Vector &end, const CPhysCollide *pSweepCollide,
										 const QAngle &sweepAngles, const CPhysCollide *pCollide,
										 const Vector &collideOrigin, const QAngle &collideAngles, trace_t *pTrace )
{
	VJoltSourceTrace::TraceCollide( start, end, pSweepCollide, sweepAngles, pCollide, collideOrigin, collideAngles,
									pTrace );
}
