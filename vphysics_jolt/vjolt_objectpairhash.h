
#pragma once

#include "vjolt_objectpairhash_storage.h"

class JoltPhysicsObjectPairHash : public IPhysicsObjectPairHash
{
public:
	JoltPhysicsObjectPairHash();

	void AddObjectPair( void *pObject0, void *pObject1 ) override;
	void RemoveObjectPair( void *pObject0, void *pObject1 ) override;
	bool IsObjectPairInHash( void *pObject0, void *pObject1 ) override;
	void RemoveAllPairsForObject( void *pObject0 ) override;
	bool IsObjectInHash( void *pObject0 ) override;

	int GetPairCountForObject( void *pObject0 ) override;
	int GetPairListForObject( void *pObject0, int nMaxCount, void **ppObjectList ) override;

private:
	VJoltObjectPairStorage m_Storage;
};
