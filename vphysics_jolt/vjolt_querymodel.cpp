
#include "cbase.h"

#include "vjolt_collide.h"
#include "vjolt_querymodel.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

JoltCollisionQuery::JoltCollisionQuery( JPH::Shape *pShape )
	: m_pShape ( pShape )
{
	JoltEnsureSourceQueryData( pShape );
}

//-------------------------------------------------------------------------------------------------

int JoltCollisionQuery::ConvexCount()
{
	const JoltCollideUserData *pData = m_pShape
		? reinterpret_cast< const JoltCollideUserData * >( m_pShape->GetUserData() ) : nullptr;
	return pData ? int( pData->sourceQueryConvexes.size() ) : 0;
}

int JoltCollisionQuery::TriangleCount( int convexIndex )
{
	const JoltCollideUserData *pData = m_pShape
		? reinterpret_cast< const JoltCollideUserData * >( m_pShape->GetUserData() ) : nullptr;
	return pData && convexIndex >= 0 && convexIndex < int( pData->sourceQueryConvexes.size() )
		? int( pData->sourceQueryConvexes[ convexIndex ].triangles.size() ) : 0;
}

//-------------------------------------------------------------------------------------------------

unsigned int JoltCollisionQuery::GetGameData( int convexIndex )
{
	const JoltCollideUserData *pData = m_pShape
		? reinterpret_cast< const JoltCollideUserData * >( m_pShape->GetUserData() ) : nullptr;
	return pData && convexIndex >= 0 && convexIndex < int( pData->sourceQueryConvexes.size() )
		? pData->sourceQueryConvexes[ convexIndex ].gameData : 0;
}

//-------------------------------------------------------------------------------------------------

void JoltCollisionQuery::GetTriangleVerts( int convexIndex, int triangleIndex, Vector *verts )
{
	if ( !verts )
		return;

	const JoltCollideUserData *pData = m_pShape
		? reinterpret_cast< const JoltCollideUserData * >( m_pShape->GetUserData() ) : nullptr;
	if ( !pData || convexIndex < 0 || convexIndex >= int( pData->sourceQueryConvexes.size() ) ||
		triangleIndex < 0 || triangleIndex >= int( pData->sourceQueryConvexes[ convexIndex ].triangles.size() ) )
	{
		verts[ 0 ] = verts[ 1 ] = verts[ 2 ] = vec3_origin;
		return;
	}

	const JoltSourceQueryTriangle &triangle =
		pData->sourceQueryConvexes[ convexIndex ].triangles[ triangleIndex ];
	verts[ 0 ] = triangle.vertices[ 0 ];
	verts[ 1 ] = triangle.vertices[ 1 ];
	verts[ 2 ] = triangle.vertices[ 2 ];
}

void JoltCollisionQuery::SetTriangleVerts( int convexIndex, int triangleIndex, const Vector *verts )
{
	// The Source interface itself documents this method as non-functional.
	( void )convexIndex;
	( void )triangleIndex;
	( void )verts;
}

//-------------------------------------------------------------------------------------------------

int JoltCollisionQuery::GetTriangleMaterialIndex( int convexIndex, int triangleIndex )
{
	const JoltCollideUserData *pData = m_pShape
		? reinterpret_cast< const JoltCollideUserData * >( m_pShape->GetUserData() ) : nullptr;
	if ( !pData || convexIndex < 0 || convexIndex >= int( pData->sourceQueryConvexes.size() ) ||
		triangleIndex < 0 || triangleIndex >= int( pData->sourceQueryConvexes[ convexIndex ].triangles.size() ) )
		return 0;

	return pData->sourceQueryConvexes[ convexIndex ].triangles[ triangleIndex ].materialIndex;
}

void JoltCollisionQuery::SetTriangleMaterialIndex( int convexIndex, int triangleIndex, int index7bits )
{
	JoltCollideUserData *pData = m_pShape
		? reinterpret_cast< JoltCollideUserData * >( m_pShape->GetUserData() ) : nullptr;
	if ( !pData || convexIndex < 0 || convexIndex >= int( pData->sourceQueryConvexes.size() ) ||
		triangleIndex < 0 || triangleIndex >= int( pData->sourceQueryConvexes[ convexIndex ].triangles.size() ) )
		return;

	// IVP stores this in a 7-bit bitfield. Release builds therefore wrap rather
	// than clamp: 128 -> 0, 129 -> 1, -1 -> 127.
	pData->sourceQueryConvexes[ convexIndex ].triangles[ triangleIndex ].materialIndex =
		uint8( index7bits ) & 0x7f;
}

//-------------------------------------------------------------------------------------------------
