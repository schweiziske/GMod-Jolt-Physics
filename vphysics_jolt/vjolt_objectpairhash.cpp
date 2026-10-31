
#include "cbase.h"

#include "vjolt_objectpairhash.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

JoltPhysicsObjectPairHash::JoltPhysicsObjectPairHash()
{
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObjectPairHash::AddObjectPair( void *pObject0, void *pObject1 )
{
	m_Storage.AddPair( pObject0, pObject1 );
}

void JoltPhysicsObjectPairHash::RemoveObjectPair( void *pObject0, void *pObject1 )
{
	m_Storage.RemovePair( pObject0, pObject1 );
}

bool JoltPhysicsObjectPairHash::IsObjectPairInHash( void *pObject0, void *pObject1 )
{
	return m_Storage.HasPair( pObject0, pObject1 );
}

void JoltPhysicsObjectPairHash::RemoveAllPairsForObject( void *pObject0 )
{
	m_Storage.RemoveAllPairsForObject( pObject0 );
}

bool JoltPhysicsObjectPairHash::IsObjectInHash( void *pObject0 )
{
	return m_Storage.HasObject( pObject0 );
}

//-------------------------------------------------------------------------------------------------

int JoltPhysicsObjectPairHash::GetPairCountForObject( void *pObject0 )
{
	return m_Storage.GetPairCountForObject( pObject0 );
}

int JoltPhysicsObjectPairHash::GetPairListForObject( void *pObject0, int nMaxCount, void **ppObjectList )
{
	return m_Storage.GetPairListForObject( pObject0, nMaxCount, ppObjectList );
}
