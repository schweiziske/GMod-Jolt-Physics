
#include "cbase.h"

#include "vjolt_surfaceprops.h"

#include "vjolt_keyvalues_schema.h"
#include "vjolt_keyvalues_parser.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

const JoltKVSchemaFunc_t FillBaseProp =
{
	sizeof( surfacedata_t ),
	[]( KeyValues *pProp, void *pPtr, size_t size )
	{
		surfacedata_t *pSurfaceDataPtr = reinterpret_cast< surfacedata_t * >( pPtr );

		int nSurfaceIndex = JoltPhysicsSurfaceProps::GetInstance().GetSurfaceIndex( pProp->GetString() );
		if ( nSurfaceIndex == -1 )
		{
			Log_Warning( LOG_VJolt, "You must specify the base material %s before it can be used. Defaulting to 'default' as a base.\n", pProp->GetString() );
			// Always the default material.
			nSurfaceIndex = 0;
		}

		*pSurfaceDataPtr = *JoltPhysicsSurfaceProps::GetInstance().GetSurfaceData( nSurfaceIndex );
	}
};

const JoltKVSchemaFunc_t FillStringProp =
{
	0, // Varies.
	[]( KeyValues *pProp, void *pPtr, size_t size )
	{
		char *pszStringPtr = reinterpret_cast< char * >( pPtr );
		V_strncpy( pszStringPtr, pProp->GetString(), static_cast< int >( size ) );
	}
};

const JoltKVSchemaFunc_t FillIntProp =
{
	sizeof( int ),
	[]( KeyValues *pProp, void *pPtr, size_t size )
	{
		int *pIntPtr = reinterpret_cast< int * >( pPtr );
		*pIntPtr = pProp->GetInt();
	}
};

const JoltKVSchemaFunc_t FillFloatProp =
{
	sizeof( float ),
	[]( KeyValues *pProp, void *pPtr, size_t size )
	{
		float *pFloatPtr = reinterpret_cast< float * >( pPtr );
		*pFloatPtr = pProp->GetFloat();
	}
};

const JoltKVSchemaFunc_t FillUnsignedCharProp =
{
	sizeof( unsigned char ),
	[]( KeyValues *pProp, void *pPtr, size_t size )
	{
		unsigned char *pCharPtr = reinterpret_cast< unsigned char * >( pPtr );
		*pCharPtr = static_cast< unsigned char >( pProp->GetInt() );
	}
};

const JoltKVSchemaFunc_t FillBoolProp =
{
	sizeof( bool ),
	[]( KeyValues *pProp, void *pPtr, size_t size )
	{
		bool *pBoolPtr = reinterpret_cast< bool * >( pPtr );
		*pBoolPtr = pProp->GetBool();
	}
};

const JoltKVSchemaFunc_t FillVectorProp =
{
	sizeof( Vector ),
	[]( KeyValues *pProp, void *pPtr, size_t size )
	{
		Vector *pVectorPtr = reinterpret_cast< Vector * >( pPtr );
		Vector value;
		if ( sscanf( pProp->GetString(), "%f %f %f", &value.x, &value.y, &value.z ) == 3 )
			*pVectorPtr = value;
	}
};

const JoltKVSchemaFunc_t FillVector4DProp =
{
	sizeof( Vector4D ),
	[]( KeyValues *pProp, void *pPtr, size_t size )
	{
		Vector4D *pVector4DPtr = reinterpret_cast< Vector4D * >( pPtr );
		Vector4D value;
		if ( sscanf( pProp->GetString(), "%f %f %f %f", &value.x, &value.y, &value.z, &value.w ) == 4 )
			*pVector4DPtr = value;
	}
};

const JoltKVSchemaFunc_t FillQAngleProp =
{
	sizeof( QAngle ),
	[]( KeyValues *pProp, void *pPtr, size_t size )
	{
		QAngle *pQAnglePtr = reinterpret_cast< QAngle * >( pPtr );
		QAngle value;
		if ( sscanf( pProp->GetString(), "%f %f %f", &value.x, &value.y, &value.z ) == 3 )
			*pQAnglePtr = value;
	}
};

const JoltKVSchemaFunc_t FillIntPairProp =
{
	sizeof( JoltPhysicsIntPair ),
	[]( KeyValues *pProp, void *pPtr, size_t size )
	{
		JoltPhysicsIntPair *pIntPairPtr = reinterpret_cast< JoltPhysicsIntPair * >( pPtr );
		JoltPhysicsIntPair value;
		if ( sscanf( pProp->GetString(), "%d , %d", &value.Index0, &value.Index1 ) == 2 )
			*pIntPairPtr = value;
	}
};

const JoltKVSchemaFunc_t FillGameMaterialProp =
{
	sizeof( unsigned short ),
	[]( KeyValues *pProp, void *pPtr, size_t size )
	{
		const char *pValue = pProp->GetString();
		unsigned short *pShortPtr = reinterpret_cast< unsigned short * >( pPtr );

		if ( V_strlen( pValue ) == 1 && !V_isdigit( pValue[ 0 ] ) )
			*pShortPtr = FastASCIIToUpper( pValue[ 0 ] );
		else
			*pShortPtr = pProp->GetInt();
	}
};

const JoltKVSchemaFunc_t FillSoundProp =
{
	sizeof( unsigned short ),
	[]( KeyValues *pProp, void *pPtr, size_t size )
	{
		const char *pValue = pProp->GetString();
		unsigned short *pShortPtr = reinterpret_cast< unsigned short * >( pPtr );

		*pShortPtr = JoltPhysicsSurfaceProps::GetInstance().RegisterSound( pValue );
	}
};

const JoltKVSchemaFunc_t FillSurfaceProp =
{
	sizeof( int ),
	[]( KeyValues *pProp, void *pPtr, size_t size )
	{
		const char* pValue = pProp->GetString();
		int *pIntPtr = reinterpret_cast< int * >( pPtr );

		*pIntPtr = JoltPhysicsSurfaceProps::GetInstance().GetSurfaceIndex( pValue );
	}
};

//-------------------------------------------------------------------------------------------------

void ParseJoltKVSchema( KeyValues *pKV, const JoltKVSchemaProp_t *pDescs, uint count, void *pObj, void *pUnknownKeyObj, IVPhysicsKeyHandler *pUnknownKeyHandler )
{
	if ( !pKV || !pObj )
		return;

    for ( KeyValues* pProp = pKV->GetFirstSubKey(); pProp != nullptr; pProp = pProp->GetNextKey() )
    {
		bool bHandled = false;
        const char *pName = pProp->GetName();
        
        for ( uint i = 0; i < count; i++ )
        {
            const JoltKVSchemaProp_t &desc = pDescs[ i ];

            if ( !V_stricmp( pName, desc.pszName ) )
            {
				bHandled = true;
				// TYPE_NONE also identifies empty blocks. A scalar with a block
				// name must not consume an axle slot, nor may a block zero a value.
				if ( ( pProp->GetDataType() == KeyValues::TYPE_NONE ) != desc.func.isBlock )
					continue;
                VJoltAssertMsg( desc.func.ptr_size == 0 || desc.size == desc.func.ptr_size, "Desc element size does not match function size");

				int *pArraySize = nullptr;
				if ( desc.arrayOffset != static_cast<size_t>( ~0llu ) )
					pArraySize = reinterpret_cast< int * >( reinterpret_cast< char * >( pObj ) + desc.arrayOffset );
				if ( pArraySize && ( *pArraySize < 0 || static_cast<size_t>( *pArraySize ) >= desc.arrayCapacity ) )
					continue;

				char* pElement = reinterpret_cast< char * >( pObj ) + desc.offset;
				if ( pArraySize )
					pElement += *pArraySize * desc.size;

                desc.func.ReadFunc( pProp, reinterpret_cast< void * >( pElement ), desc.size);
				if ( desc.fixupFunc )
					desc.fixupFunc( pObj );
				if ( pArraySize )
					( *pArraySize )++;
            }
        }

		if ( !bHandled && pUnknownKeyHandler )
			pUnknownKeyHandler->ParseKeyValue( pUnknownKeyObj, pName,
				pProp->GetDataType() == KeyValues::TYPE_NONE ? "{" : pProp->GetString() );
    }
}

void ParseJoltKVCustom( KeyValues *pKV, void *pUnknownKeyObj, IVPhysicsKeyHandler* pUnknownKeyHandler )
{
	if ( !pKV || !pUnknownKeyHandler )
		return;
	// Josh:
	// Parse out custom KV entries like "vehicle_sounds" etc
	// out recursively.

	for ( KeyValues* pProp = pKV->GetFirstSubKey(); pProp != nullptr; pProp = pProp->GetNextKey() )
	{
		pUnknownKeyHandler->ParseKeyValue( pUnknownKeyObj, pProp->GetName(),
			pProp->GetDataType() == KeyValues::TYPE_NONE ? "{" : pProp->GetString() );

		ParseJoltKVCustom( pProp, pUnknownKeyObj, pUnknownKeyHandler );
	}
}

//-------------------------------------------------------------------------------------------------

KeyValues *HeaderlessKVBufferToKeyValues( const char *pszBuffer, const char *pszSetName, bool lowercase )
{
	vjolt_keyvalues::ParseError error;
	KeyValues *pszKV = vjolt_keyvalues::ParseHeaderless( pszBuffer, error, lowercase );
	if ( !pszKV )
	{
		Log_Warning( LOG_VJolt, "%s: invalid physics key data near byte %zu: %s.\n",
			pszSetName, error.offset, error.message );
	}
	return pszKV;
}
