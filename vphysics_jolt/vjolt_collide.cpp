
#include "cbase.h"

#include "coordsize.h"
#include "mathlib/polyhedron.h"

#include <Jolt/Core/UnorderedSet.h>
#include <map>
#include <Jolt/Geometry/ClosestPoint.h>
#include <Jolt/Geometry/ConvexHullBuilder.h>
#include <Jolt/Physics/Collision/PhysicsMaterialSimple.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/DecoratedShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/TriangleShape.h>

#include "vjolt_parse.h"
#include "vjolt_querymodel.h"
#include "vjolt_surfaceprops.h"

#include "vjolt_collide.h"
#include "vjolt_source_cone_overlap.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

JoltPhysicsCollision JoltPhysicsCollision::s_PhysicsCollision;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR( JoltPhysicsCollision, IPhysicsCollision, VPHYSICS_COLLISION_INTERFACE_VERSION, JoltPhysicsCollision::GetInstance() );

//-------------------------------------------------------------------------------------------------

namespace
{
	constexpr uint32 VJOLT_SOURCE_CONVEX_BASE_VALID = 0x80000000u;
	constexpr uint32 VJOLT_SOURCE_CONVEX_BASE_MASK = 0x7fffffffu;
	constexpr double VJOLT_HALFSPACE_TOLERANCE = 0.0001;
	constexpr double VJOLT_HALFSPACE_DETERMINANT_EPSILON = 1.0e-10;

	struct VJoltHalfspacePlane
	{
		double normal[ 3 ];
		double dist;
	};

	struct VJoltHalfspacePoint
	{
		double value[ 3 ];
	};

	double HalfspaceDot( const VJoltHalfspacePlane &plane, const VJoltHalfspacePoint &point )
	{
		return plane.normal[ 0 ] * point.value[ 0 ]
			+ plane.normal[ 1 ] * point.value[ 1 ]
			+ plane.normal[ 2 ] * point.value[ 2 ];
	}

	double HalfspacePointDistanceSquared( const VJoltHalfspacePoint &a,
		const VJoltHalfspacePoint &b )
	{
		const double x = a.value[ 0 ] - b.value[ 0 ];
		const double y = a.value[ 1 ] - b.value[ 1 ];
		const double z = a.value[ 2 ] - b.value[ 2 ];
		return x * x + y * y + z * z;
	}

	bool IntersectHalfspacePlanes( const VJoltHalfspacePlane &a,
		const VJoltHalfspacePlane &b, const VJoltHalfspacePlane &c,
		VJoltHalfspacePoint &point )
	{
		const double crossBC[ 3 ] =
		{
			b.normal[ 1 ] * c.normal[ 2 ] - b.normal[ 2 ] * c.normal[ 1 ],
			b.normal[ 2 ] * c.normal[ 0 ] - b.normal[ 0 ] * c.normal[ 2 ],
			b.normal[ 0 ] * c.normal[ 1 ] - b.normal[ 1 ] * c.normal[ 0 ]
		};
		const double determinant = a.normal[ 0 ] * crossBC[ 0 ]
			+ a.normal[ 1 ] * crossBC[ 1 ] + a.normal[ 2 ] * crossBC[ 2 ];
		if ( std::fabs( determinant ) < VJOLT_HALFSPACE_DETERMINANT_EPSILON )
			return false;

		const double crossCA[ 3 ] =
		{
			c.normal[ 1 ] * a.normal[ 2 ] - c.normal[ 2 ] * a.normal[ 1 ],
			c.normal[ 2 ] * a.normal[ 0 ] - c.normal[ 0 ] * a.normal[ 2 ],
			c.normal[ 0 ] * a.normal[ 1 ] - c.normal[ 1 ] * a.normal[ 0 ]
		};
		const double crossAB[ 3 ] =
		{
			a.normal[ 1 ] * b.normal[ 2 ] - a.normal[ 2 ] * b.normal[ 1 ],
			a.normal[ 2 ] * b.normal[ 0 ] - a.normal[ 0 ] * b.normal[ 2 ],
			a.normal[ 0 ] * b.normal[ 1 ] - a.normal[ 1 ] * b.normal[ 0 ]
		};
		const double inverseDeterminant = 1.0 / determinant;
		for ( int axis = 0; axis < 3; ++axis )
		{
			point.value[ axis ] = ( a.dist * crossBC[ axis ]
				+ b.dist * crossCA[ axis ] + c.dist * crossAB[ axis ] )
				* inverseDeterminant;
		}
		return true;
	}

	double HalfspaceHullDistanceSquared( JPH::Vec3Arg a, JPH::Vec3Arg b )
	{
		const double x = double( a.GetX() ) - b.GetX();
		const double y = double( a.GetY() ) - b.GetY();
		const double z = double( a.GetZ() ) - b.GetZ();
		return x * x + y * y + z * z;
	}

	bool RemoveSourceDegenerateHullPoints( std::vector< JPH::Vec3 > &vertices )
	{
		// IVP's point-soup builder retries qhull after removing a point from every
		// sliver facet. P_MIN_EDGE_LEN is 0.01 m in ivu_types.hxx. Without this
		// pass, sub-centimetre bevels retain short edges that Source deliberately
		// collapses, changing ConvexFromPlanes even when mergeDistance is smaller.
		constexpr double minimumEdgeLength = 0.01;
		for ( size_t iteration = 0; iteration < vertices.size(); ++iteration )
		{
			JPH::ConvexHullBuilder::Positions positions;
			positions.reserve( vertices.size() );
			for ( JPH::Vec3Arg vertex : vertices )
				positions.push_back( vertex );

			JPH::ConvexHullBuilder builder( positions );
			const char *pError = nullptr;
			const JPH::ConvexHullBuilder::EResult result = builder.Initialize(
				INT_MAX, 0.0f, pError );
			if ( result != JPH::ConvexHullBuilder::EResult::Success
				&& result != JPH::ConvexHullBuilder::EResult::MaxVerticesReached )
				return false;

			std::vector< bool > remove( vertices.size(), false );
			for ( const JPH::ConvexHullBuilder::Face *pFace : builder.GetFaces() )
			{
				std::vector< int > faceVertices;
				const JPH::ConvexHullBuilder::Edge *pEdge = pFace->mFirstEdge;
				if ( !pEdge )
					continue;
				do
				{
					faceVertices.push_back( pEdge->mStartIdx );
					pEdge = pEdge->mNextEdge;
				}
				while ( pEdge != pFace->mFirstEdge );

				// Jolt exposes merged polygon faces while the old qhull path exposes
				// facets. A deterministic fan lets us apply the same IVP sliver test
				// and reproduce its minimum-edge collapse threshold and vertex count.
				for ( size_t triangle = 1; triangle + 1 < faceVertices.size(); ++triangle )
				{
					const int indices[ 3 ] =
					{
						faceVertices[ 0 ], faceVertices[ triangle ],
						faceVertices[ triangle + 1 ]
					};
					if ( remove[ indices[ 0 ] ] || remove[ indices[ 1 ] ]
						|| remove[ indices[ 2 ] ] )
						continue;

					const JPH::Vec3 &a = vertices[ indices[ 0 ] ];
					const JPH::Vec3 &b = vertices[ indices[ 1 ] ];
					const JPH::Vec3 &c = vertices[ indices[ 2 ] ];
					const JPH::Vec3 cross = ( b - a ).Cross( c - a );
					const double areaSum = std::sqrt( double( cross.LengthSq() ) );
					const double edgeSum = HalfspaceHullDistanceSquared( a, b )
						+ HalfspaceHullDistanceSquared( b, c )
						+ HalfspaceHullDistanceSquared( c, a );
					if ( areaSum >= 0.5 * minimumEdgeLength * std::sqrt( edgeSum ) )
						continue;

					int farthestFromFirst = 0;
					double farthestDistance = -1.0;
					for ( int candidate = 0; candidate < 3; ++candidate )
					{
						const double distance = HalfspaceHullDistanceSquared(
							vertices[ indices[ 0 ] ], vertices[ indices[ candidate ] ] );
						if ( distance > farthestDistance )
						{
							farthestDistance = distance;
							farthestFromFirst = candidate;
						}
					}

					int farthestFromFarthest = 0;
					farthestDistance = -1.0;
					for ( int candidate = 0; candidate < 3; ++candidate )
					{
						const double distance = HalfspaceHullDistanceSquared(
							vertices[ indices[ farthestFromFirst ] ],
							vertices[ indices[ candidate ] ] );
						if ( distance > farthestDistance )
						{
							farthestDistance = distance;
							farthestFromFarthest = candidate;
						}
					}

					for ( int candidate = 0; candidate < 3; ++candidate )
					{
						if ( candidate != farthestFromFirst && candidate != farthestFromFarthest )
						{
							remove[ indices[ candidate ] ] = true;
							break;
						}
					}
				}
			}

			if ( std::none_of( remove.begin(), remove.end(), []( bool value ) { return value; } ) )
				return true;

			size_t writeIndex = 0;
			for ( size_t readIndex = 0; readIndex < vertices.size(); ++readIndex )
			{
				if ( !remove[ readIndex ] )
					vertices[ writeIndex++ ] = vertices[ readIndex ];
			}
			vertices.resize( writeIndex );
			if ( vertices.size() < 4 )
				return false;
		}
		return false;
	}

	class VJoltCollideWriteStream final : public JPH::StreamOut
	{
	public:
		explicit VJoltCollideWriteStream( char *pDestination ) : m_pDestination( pDestination ) {}

		void WriteBytes( const void *pData, size_t size ) override
		{
			if ( m_pDestination )
				memcpy( m_pDestination + m_Size, pData, size );
			m_Size += size;
		}

		bool IsFailed() const override { return false; }
		size_t GetSize() const { return m_Size; }

	private:
		char *m_pDestination = nullptr;
		size_t m_Size = 0;
	};

	class VJoltCollideReadStream final : public JPH::StreamIn
	{
	public:
		VJoltCollideReadStream( const char *pData, size_t size ) : m_pData( pData ), m_Size( size ) {}

		void ReadBytes( void *pOutput, size_t size ) override
		{
			if ( m_Offset > m_Size || size > m_Size - m_Offset )
			{
				memset( pOutput, 0, size );
				m_bEOF = true;
				m_bFailed = true;
				return;
			}

			memcpy( pOutput, m_pData + m_Offset, size );
			m_Offset += size;
		}

		bool IsEOF() const override { return m_bEOF; }
		bool IsFailed() const override { return m_bFailed; }
		size_t GetRemaining() const { return m_Offset <= m_Size ? m_Size - m_Offset : 0; }

	private:
		const char *m_pData = nullptr;
		size_t m_Size = 0;
		size_t m_Offset = 0;
		bool m_bEOF = false;
		bool m_bFailed = false;
	};

	void ReplaceRestoredUserDataRecursive( JPH::Shape *pShape,
		JPH::UnorderedSet< JPH::Shape * > &visited )
	{
		if ( !pShape || !visited.insert( pShape ).second )
			return;

		JPH::ShapeList children;
		pShape->SaveSubShapeState( children );
		for ( const JPH::RefConst< JPH::Shape > &child : children )
			ReplaceRestoredUserDataRecursive(
				const_cast< JPH::Shape * >( child.GetPtr() ), visited );

		auto *pData = new JoltCollideUserData();
		pShape->SetUserData( reinterpret_cast< uint64 >( pData ) );
	}

	JoltCollideUserData *ReplaceRestoredUserData( JPH::Shape *pShape )
	{
		JPH::UnorderedSet< JPH::Shape * > visited;
		ReplaceRestoredUserDataRecursive( pShape, visited );
		auto *pRootData = pShape ? reinterpret_cast< JoltCollideUserData * >(
			pShape->GetUserData() ) : nullptr;
		if ( pRootData )
			pRootData->ownsChildUserData = visited.size() > 1;
		return pRootData;
	}

	void DeleteCollideUserDataRecursive( JPH::Shape *pShape,
		JPH::UnorderedSet< JPH::Shape * > &visitedShapes,
		JPH::UnorderedSet< JoltCollideUserData * > &visitedData )
	{
		if ( !pShape || !visitedShapes.insert( pShape ).second )
			return;

		JPH::ShapeList children;
		pShape->SaveSubShapeState( children );
		for ( const JPH::RefConst< JPH::Shape > &child : children )
			DeleteCollideUserDataRecursive(
				const_cast< JPH::Shape * >( child.GetPtr() ), visitedShapes, visitedData );

		auto *pData = reinterpret_cast< JoltCollideUserData * >( pShape->GetUserData() );
		if ( pData && visitedData.insert( pData ).second )
			delete pData;
		pShape->SetUserData( 0 );
	}

	void EnableSourceTriangleBackFacesRecursive( JPH::Shape *pShape,
		JPH::UnorderedSet< JPH::Shape * > &visited )
	{
		if ( !pShape || !visited.insert( pShape ).second )
			return;
		if ( pShape->GetSubType() == JPH::EShapeSubType::Triangle )
			static_cast< JPH::TriangleShape * >( pShape )->SetCollideWithBackFaces( true );

		JPH::ShapeList children;
		pShape->SaveSubShapeState( children );
		for ( const JPH::RefConst< JPH::Shape > &child : children )
			EnableSourceTriangleBackFacesRecursive(
				const_cast< JPH::Shape * >( child.GetPtr() ), visited );
	}

	void EnableSourceTriangleBackFacesRecursive( JPH::Shape *pShape )
	{
		JPH::UnorderedSet< JPH::Shape * > visited;
		EnableSourceTriangleBackFacesRecursive( pShape, visited );
	}

	JPH::RefConst< JPH::PhysicsMaterial > CreateSourcePhysicsMaterial( int materialIndex )
	{
		char name[ 64 ];
		V_snprintf( name, sizeof( name ), "%s%d", VJOLT_SOURCE_MATERIAL_PREFIX,
			uint8( materialIndex ) & 0x7f );
		return new JPH::PhysicsMaterialSimple( name, JPH::Color::sGrey );
	}

	int DecodeSourcePhysicsMaterial( const JPH::PhysicsMaterial *pMaterial )
	{
		const char *pName = pMaterial ? pMaterial->GetDebugName() : nullptr;
		constexpr size_t prefixLength = sizeof( VJOLT_SOURCE_MATERIAL_PREFIX ) - 1;
		if ( !pName || V_strncmp( pName, VJOLT_SOURCE_MATERIAL_PREFIX, prefixLength ) )
			return 0;

		char *pEnd = nullptr;
		const long value = strtol( pName + prefixLength, &pEnd, 10 );
		return pEnd != pName + prefixLength && *pEnd == '\0'
			? int( uint8( value ) & 0x7f ) : 0;
	}

	const JPH::Shape *UnwrapDecoratedShape( const JPH::Shape *pShape )
	{
		while ( pShape && pShape->GetType() == JPH::EShapeType::Decorated )
			pShape = static_cast< const JPH::DecoratedShape * >( pShape )->GetInnerShape();
		return pShape;
	}

	size_t CachedSourceConvexCount( const JPH::Shape *pShape )
	{
		if ( !pShape )
			return 0;
		const auto *pData = reinterpret_cast< const JoltCollideUserData * >(
			pShape->GetUserData() );
		return pData ? pData->sourceQueryConvexes.size() : 0;
	}

	bool ResolveSourceConvexIndex( const JPH::Shape *pShape, JPH::SubShapeID subShapeID,
		size_t baseIndex, size_t &outIndex )
	{
		pShape = UnwrapDecoratedShape( pShape );
		if ( !pShape )
			return false;

		if ( pShape->GetType() == JPH::EShapeType::Compound )
		{
			const auto *pCompound = static_cast< const JPH::CompoundShape * >( pShape );
			if ( !pCompound->IsSubShapeIDValid( subShapeID ) )
				return false;

			JPH::SubShapeID remainder;
			const uint childIndex = pCompound->GetSubShapeIndexFromID( subShapeID, remainder );
			const uint32 taggedBase = pCompound->GetCompoundUserData( childIndex );
			if ( taggedBase & VJOLT_SOURCE_CONVEX_BASE_VALID )
			{
				baseIndex += taggedBase & VJOLT_SOURCE_CONVEX_BASE_MASK;
			}
			else
			{
				for ( uint i = 0; i < childIndex; ++i )
				{
					const JPH::Shape *pSibling = pCompound->GetSubShape( i ).mShape.GetPtr();
					const size_t siblingCount = CachedSourceConvexCount( pSibling );
					baseIndex += siblingCount ? siblingCount : 1;
				}
			}

			return ResolveSourceConvexIndex(
				pCompound->GetSubShape( childIndex ).mShape.GetPtr(), remainder,
				baseIndex, outIndex );
		}

		if ( pShape->GetSubType() == JPH::EShapeSubType::Mesh )
		{
			const auto *pMesh = static_cast< const JPH::MeshShape * >( pShape );
			outIndex = baseIndex + pMesh->GetTriangleUserData( subShapeID );
			return true;
		}

		outIndex = baseIndex;
		return true;
	}

	bool ResolveSourceTriangleMaterial( const JoltCollideUserData &data,
		size_t convexIndex, JPH::Vec3Arg localContactPoint,
		JPH::Vec3Arg localSurfaceNormal, int &outMaterialIndex )
	{
		if ( convexIndex >= data.sourceQueryConvexes.size() )
			return false;

		const JoltSourceQueryConvex &convex = data.sourceQueryConvexes[ convexIndex ];
		if ( convex.triangles.empty() )
			return false;

		const uint8 firstMaterial = convex.triangles.front().materialIndex;
		bool uniformMaterial = true;
		for ( const JoltSourceQueryTriangle &triangle : convex.triangles )
		{
			if ( triangle.materialIndex != firstMaterial )
			{
				uniformMaterial = false;
				break;
			}
		}
		if ( uniformMaterial )
		{
			outMaterialIndex = firstMaterial;
			return true;
		}

		const JPH::Vec3 surfaceNormal = localSurfaceNormal.NormalizedOr( JPH::Vec3::sAxisZ() );
		const float normalPenaltyScale = Square( SourceToJolt::Distance( 1.0f ) );
		float bestScore = FLT_MAX;
		int bestMaterial = firstMaterial;
		for ( const JoltSourceQueryTriangle &triangle : convex.triangles )
		{
			const JPH::Vec3 a = SourceToJolt::Distance( triangle.vertices[ 0 ] );
			const JPH::Vec3 b = SourceToJolt::Distance( triangle.vertices[ 1 ] );
			const JPH::Vec3 c = SourceToJolt::Distance( triangle.vertices[ 2 ] );
			// CCollisionQuery exposes IVP edges in 2,1,0 order, so the outward
			// normal uses the reverse of the usual query-winding cross product.
			const JPH::Vec3 triangleNormal = ( c - a ).Cross( b - a )
				.NormalizedOr( surfaceNormal );
			uint32 closestSet = 0;
			const JPH::Vec3 closest = JPH::ClosestPoint::GetClosestPointOnTriangle(
				a - localContactPoint, b - localContactPoint, c - localContactPoint,
				closestSet );
			const float alignment = Clamp<float>( triangleNormal.Dot( surfaceNormal ), -1.0f, 1.0f );
			const float score = closest.LengthSq() + ( 1.0f - alignment ) * normalPenaltyScale;
			if ( score < bestScore )
			{
				bestScore = score;
				bestMaterial = triangle.materialIndex;
			}
		}

		outMaterialIndex = bestMaterial;
		return true;
	}

	float SourceQuerySurfaceArea( JPH::Shape *pShape )
	{
		if ( !pShape )
			return 0.0f;
		JoltEnsureSourceQueryData( pShape );
		const auto *pData = reinterpret_cast< const JoltCollideUserData * >(
			pShape->GetUserData() );
		if ( !pData )
			return 0.0f;

		double area = 0.0;
		for ( const JoltSourceQueryConvex &convex : pData->sourceQueryConvexes )
		{
			for ( const JoltSourceQueryTriangle &triangle : convex.triangles )
			{
				const Vector edge0 = triangle.vertices[ 1 ] - triangle.vertices[ 0 ];
				const Vector edge1 = triangle.vertices[ 2 ] - triangle.vertices[ 0 ];
				const double crossX = double( edge0.y ) * edge1.z - double( edge0.z ) * edge1.y;
				const double crossY = double( edge0.z ) * edge1.x - double( edge0.x ) * edge1.z;
				const double crossZ = double( edge0.x ) * edge1.y - double( edge0.y ) * edge1.x;
				area += 0.5 * std::sqrt(
					crossX * crossX + crossY * crossY + crossZ * crossZ );
			}
		}
		return float( area );
	}

	void BuildTraceVertices( JoltSourceQueryConvex &convex )
	{
		if ( !convex.traceVertices.empty() )
			return;
		for ( const auto &triangle : convex.triangles )
			for ( const Vector &vertex : triangle.vertices )
			{
				const Vector point( vertex.x * 0.0254f, -vertex.z * 0.0254f, vertex.y * 0.0254f );
				if ( std::find( convex.traceVertices.begin(), convex.traceVertices.end(), point ) ==
					 convex.traceVertices.end() )
					convex.traceVertices.push_back( point );
			}
	}

	int BuildTraceTree( JoltCollideUserData &data, size_t begin, size_t end )
	{
		const int index = int( data.sourceTraceNodes.size() );
		data.sourceTraceNodes.emplace_back();
		Vector lo( FLT_MAX, FLT_MAX, FLT_MAX ), hi( -FLT_MAX, -FLT_MAX, -FLT_MAX );
		for ( size_t i = begin; i < end; ++i )
			for ( const Vector &point : data.sourceQueryConvexes[ i ].traceVertices )
			{
				VectorMin( lo, point, lo );
				VectorMax( hi, point, hi );
			}
		auto &node = data.sourceTraceNodes[ index ];
		node.center = ( lo + hi ) * 0.5f;
		node.radius = ( hi - lo ).Length() * 0.5f + 0.001f;
		if ( end - begin == 1 )
			node.convex = int( begin );
		else
		{
			const size_t middle = begin + ( end - begin ) / 2;
			const int left = BuildTraceTree( data, begin, middle );
			const int right = BuildTraceTree( data, middle, end );
			data.sourceTraceNodes[ index ].left = left;
			data.sourceTraceNodes[ index ].right = right;
		}
		return index;
	}

	void PrepareTraceData( JoltCollideUserData &data )
	{
		if ( data.sourceTraceCached )
			return;
		for ( auto &convex : data.sourceQueryConvexes )
		{
			BuildTraceVertices( convex );
			if ( convex.traceCachedSupport )
				continue;
			const auto &edges = convex.traceEdges;
			auto previous = []( int edge ) { return edge / 3 * 3 + ( edge % 3 + 2 ) % 3; };
			for ( int octant = 0; octant < 8; ++octant )
			{
				const Vector direction( octant & 1 ? -1.f : 1.f, octant & 2 ? -1.f : 1.f, octant & 4 ? -1.f : 1.f );
				int edge = 0, best = edges[ 0 ].first, bestEdge = 0;
				float bestDot = DotProduct( convex.traceVertices[ best ], direction );
				for ( size_t i = 0; i < edges.size() / 3; ++i )
				{
					edge = previous( edge );
					const int stop = edges[ edge ].first;
					int vertex = stop;
					size_t remaining = edges.size();
					do
					{
						const float dot = DotProduct( convex.traceVertices[ vertex ], direction );
						if ( dot > bestDot )
						{
							bestDot = dot;
							best = vertex;
							bestEdge = edge;
							break;
						}
						edge = previous( edges[ edge ].second );
						vertex = edges[ edge ].first;
					} while ( vertex != stop && --remaining );
					if ( vertex != best )
						break;
				}
				convex.traceStartEdges[ octant ] = bestEdge;
			}
		}
		if ( data.sourceTraceNodes.empty() && !data.sourceQueryConvexes.empty() )
		{
			BuildTraceTree( data, 0, data.sourceQueryConvexes.size() );
			data.sourceTraceRadius = 0;
			for ( const auto &convex : data.sourceQueryConvexes )
				for ( const Vector &point : convex.traceVertices )
					data.sourceTraceRadius = Max( data.sourceTraceRadius, point.Length() );
		}
		data.sourceTraceCached = true;
	}

	void BuildQueryConvexFromShape( const JPH::Shape *pShape, JoltSourceQueryConvex &convex )
	{
		if ( !pShape )
			return;

		JPH::AllHitCollisionCollector< JPH::TransformedShapeCollector > collector;
		JPH::ShapeFilter filter;
		pShape->CollectTransformedShapes( JPH::AABox::sBiggest(),
			pShape->GetCenterOfMass() * JoltToSource::Factor, JPH::Quat::sIdentity(),
			JPH::Vec3::sReplicate( JoltToSource::Factor ), JPH::SubShapeIDCreator(),
			collector, filter );

		static constexpr int kRequestedTriangles = 256;
		for ( const JPH::TransformedShape &shape : collector.mHits )
		{
			JPH::TransformedShape::GetTrianglesContext context;
			shape.GetTrianglesStart( context, JPH::AABox::sBiggest(), JPH::Vec3::sZero() );
			for ( ;; )
			{
				JPH::Float3 vertices[ kRequestedTriangles * 3 ];
				const JPH::PhysicsMaterial *materials[ kRequestedTriangles ];
				const int triangleCount = shape.GetTrianglesNext(
					context, kRequestedTriangles, vertices, materials );
				if ( triangleCount <= 0 )
					break;

				for ( int i = 0; i < triangleCount; ++i )
				{
					JoltSourceQueryTriangle &triangle = convex.triangles.emplace_back();
					// CCollisionQuery walks IVP compact edges 2,1,0.
					triangle.vertices[ 0 ] = Vector( vertices[ i * 3 + 2 ].x,
						vertices[ i * 3 + 2 ].y, vertices[ i * 3 + 2 ].z );
					triangle.vertices[ 1 ] = Vector( vertices[ i * 3 + 1 ].x,
						vertices[ i * 3 + 1 ].y, vertices[ i * 3 + 1 ].z );
					triangle.vertices[ 2 ] = Vector( vertices[ i * 3 + 0 ].x,
						vertices[ i * 3 + 0 ].y, vertices[ i * 3 + 0 ].z );
					triangle.materialIndex = uint8( DecodeSourcePhysicsMaterial( materials[ i ] ) );
				}
			}
		}
	}

	void AppendTransformedQueryConvexes( const JPH::CompoundShape *pCompound, uint subShapeIndex,
		const JoltCollideUserData &subData, std::vector< JoltSourceQueryConvex > &out )
	{
		const JPH::CompoundShape::SubShape &subShape = pCompound->GetSubShape( subShapeIndex );
		const JPH::Shape *pChildShape = subShape.mShape.GetPtr();
		const JPH::Vec3 rootCenterOfMass = pCompound->GetCenterOfMass();
		const JPH::Vec3 childCenterOfMass = pChildShape->GetCenterOfMass();
		const JPH::Vec3 childPositionCOM = subShape.GetPositionCOM();
		const JPH::Quat childRotation = subShape.GetRotation();
		const JPH::Vec3 childOriginInRoot = rootCenterOfMass + childPositionCOM
			- childRotation * childCenterOfMass;
		const bool preservesSourceObjectSpace =
			std::fabs( childRotation.Dot( JPH::Quat::sIdentity() ) ) > 1.0f - 1.0e-6f
			&& childOriginInRoot.LengthSq() < 1.0e-12f;

		for ( const JoltSourceQueryConvex &sourceConvex : subData.sourceQueryConvexes )
		{
			if ( preservesSourceObjectSpace )
			{
				out.push_back( sourceConvex );
				continue;
			}

			JoltSourceQueryConvex &convex = out.emplace_back();
			convex.gameData = sourceConvex.gameData;
			convex.traceLoadIndex = sourceConvex.traceLoadIndex;
			for ( const Vector &point : sourceConvex.traceVertices )
			{
				const JPH::Vec3 transformed =
					childOriginInRoot + childRotation * JPH::Vec3( point.x, point.z, -point.y );
				convex.traceVertices.emplace_back( transformed.GetX(), -transformed.GetZ(), transformed.GetY() );
			}
			convex.traceEdges = sourceConvex.traceEdges;
			convex.traceCachedSupport = sourceConvex.traceCachedSupport;
			convex.triangles.reserve( sourceConvex.triangles.size() );
			for ( const JoltSourceQueryTriangle &sourceTriangle : sourceConvex.triangles )
			{
				JoltSourceQueryTriangle &triangle = convex.triangles.emplace_back( sourceTriangle );
				for ( int vertex = 0; vertex < 3; ++vertex )
				{
					const JPH::Vec3 childVertex = SourceToJolt::Distance( sourceTriangle.vertices[ vertex ] );
					const JPH::Vec3 rootVertex = rootCenterOfMass + childPositionCOM
						+ childRotation * ( childVertex - childCenterOfMass );
					triangle.vertices[ vertex ] = JoltToSource::Distance( rootVertex );
				}
			}
		}
	}

	void WriteCollideMetadata( JPH::StreamOut &stream, const JoltCollideUserData &data )
	{
		stream.Write( uint32( data.hasOuterHull ? 1 : 0 ) );
		stream.Write( data.sourceTraceRadius );
		stream.Write( uint32( data.sourceTraceNodes.size() ) );
		for ( const auto &node : data.sourceTraceNodes )
		{
			stream.WriteBytes( &node.center, sizeof( Vector ) );
			stream.Write( node.radius );
			stream.Write( node.left );
			stream.Write( node.right );
			stream.Write( node.convex );
		}
		stream.Write( data.orthographicAreas.x );
		stream.Write( data.orthographicAreas.y );
		stream.Write( data.orthographicAreas.z );
		JPH::Float3 massCenter;
		JPH::Float3 inertiaPerMass;
		data.sourceMassCenter.StoreFloat3( &massCenter );
		data.sourceRotationInertiaPerMass.StoreFloat3( &inertiaPerMass );
		stream.Write( massCenter );
		stream.Write( inertiaPerMass );
		stream.Write( uint32( data.hasSourceMassProperties ? 1 : 0 ) );
		stream.Write( int32( data.vcollideIndex ) );
		stream.Write( int32( data.gameData ) );
		stream.Write( uint32( data.sourceQueryConvexes.size() ) );
		for ( const JoltSourceQueryConvex &convex : data.sourceQueryConvexes )
		{
			stream.Write( uint32( convex.traceCachedSupport ) );
			stream.Write( uint32( convex.traceEdges.size() ) );
			for ( const auto &edge : convex.traceEdges )
			{
				stream.Write( edge.first );
				stream.Write( edge.second );
			}
			stream.Write( uint32( convex.traceVertices.size() ) );
			stream.WriteBytes( convex.traceVertices.data(), convex.traceVertices.size() * sizeof( Vector ) );
			stream.Write( uint32( convex.gameData ) );
			stream.Write( uint32( convex.triangles.size() ) );
			for ( const JoltSourceQueryTriangle &triangle : convex.triangles )
			{
				stream.WriteBytes( triangle.vertices, sizeof( triangle.vertices ) );
				stream.Write( uint32( triangle.materialIndex ) );
			}
		}
	}

	bool ReadCollideMetadata( VJoltCollideReadStream &stream, JoltCollideUserData &data, int version )
	{
		if ( version >= 4 )
		{
			uint32 hasOuterHull = 0;
			stream.Read( hasOuterHull );
			if ( stream.IsFailed() || hasOuterHull > 1 )
				return false;
			data.hasOuterHull = hasOuterHull != 0;
		}
		if ( version >= 3 )
		{
			stream.Read( data.sourceTraceRadius );
			uint32 count = 0;
			stream.Read( count );
			if ( stream.IsFailed() || count > stream.GetRemaining() / 28 )
				return false;
			data.sourceTraceNodes.resize( count );
			for ( auto &node : data.sourceTraceNodes )
			{
				stream.ReadBytes( &node.center, sizeof( Vector ) );
				stream.Read( node.radius );
				stream.Read( node.left );
				stream.Read( node.right );
				stream.Read( node.convex );
			}
		}
		stream.Read( data.orthographicAreas.x );
		stream.Read( data.orthographicAreas.y );
		stream.Read( data.orthographicAreas.z );
		JPH::Float3 massCenter;
		JPH::Float3 inertiaPerMass;
		stream.Read( massCenter );
		stream.Read( inertiaPerMass );
		data.sourceMassCenter = JPH::Vec3( massCenter.x, massCenter.y, massCenter.z );
		data.sourceRotationInertiaPerMass = JPH::Vec3(
			inertiaPerMass.x, inertiaPerMass.y, inertiaPerMass.z );
		uint32 hasMassProperties = 0;
		int32 vcollideIndex = 0;
		int32 gameData = 0;
		stream.Read( hasMassProperties );
		stream.Read( vcollideIndex );
		stream.Read( gameData );
		data.hasSourceMassProperties = hasMassProperties != 0;
		data.vcollideIndex = vcollideIndex;
		data.gameData = gameData;

		uint32 convexCount = 0;
		stream.Read( convexCount );
		// Every convex needs at least gameData + triangleCount. Validate against
		// bytes that actually remain, not the original buffer size, so repeated
		// malicious counts cannot reserve quadratic amounts of memory.
		if ( stream.IsFailed() || convexCount > stream.GetRemaining() / 8 )
			return false;
		data.sourceQueryConvexes.clear();
		data.sourceQueryConvexes.reserve( convexCount );
		for ( uint32 convexIndex = 0; convexIndex < convexCount; ++convexIndex )
		{
			JoltSourceQueryConvex &convex = data.sourceQueryConvexes.emplace_back();
			if ( version >= 3 )
			{
				uint32 cached = 0, edgeCount = 0;
				stream.Read( cached );
				stream.Read( edgeCount );
				convex.traceCachedSupport = cached != 0;
				if ( stream.IsFailed() || edgeCount > stream.GetRemaining() / 8 || edgeCount % 3 )
					return false;
				convex.traceEdges.resize( edgeCount );
				for ( auto &edge : convex.traceEdges )
				{
					stream.Read( edge.first );
					stream.Read( edge.second );
				}
				uint32 count = 0;
				stream.Read( count );
				if ( stream.IsFailed() || count > stream.GetRemaining() / sizeof( Vector ) )
					return false;
				convex.traceVertices.resize( count );
				stream.ReadBytes( convex.traceVertices.data(), count * sizeof( Vector ) );
				for ( const auto &point : convex.traceVertices )
					if ( !point.IsValid() )
						return false;
				for ( const auto &edge : convex.traceEdges )
					if ( edge.first >= count || edge.second >= edgeCount )
						return false;
				if ( !convex.traceCachedSupport && !edgeCount )
					return false;
			}
			uint32 convexGameData = 0;
			uint32 triangleCount = 0;
			stream.Read( convexGameData );
			stream.Read( triangleCount );
			if ( stream.IsFailed() || triangleCount > stream.GetRemaining() / 40 )
				return false;

			convex.gameData = convexGameData;
			convex.triangles.reserve( triangleCount );
			for ( uint32 triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex )
			{
				JoltSourceQueryTriangle &triangle = convex.triangles.emplace_back();
				uint32 materialIndex = 0;
				stream.ReadBytes( triangle.vertices, sizeof( triangle.vertices ) );
				stream.Read( materialIndex );
				triangle.materialIndex = uint8( materialIndex ) & 0x7f;
				if ( stream.IsFailed() )
					return false;
			}
		}
		std::vector<uint8> parents( data.sourceTraceNodes.size(), 0 );
		for ( size_t i = 0; i < data.sourceTraceNodes.size(); ++i )
		{
			const auto &node = data.sourceTraceNodes[ i ];
			if ( !node.center.IsValid() || !std::isfinite( node.radius ) || node.radius < 0 )
				return false;
			if ( node.convex >= 0 )
			{
				if ( size_t( node.convex ) >= data.sourceQueryConvexes.size() || node.left != -1 || node.right != -1 )
					return false;
			}
			else if ( node.left == -1 && node.right == -1 )
				continue;
			else if ( node.left <= int( i ) || node.right <= int( i ) || node.left == node.right ||
					  size_t( node.left ) >= data.sourceTraceNodes.size() ||
					  size_t( node.right ) >= data.sourceTraceNodes.size() )
				return false;
			else if ( parents[ node.left ]++ || parents[ node.right ]++ )
				return false;
		}
		if ( !std::isfinite( data.sourceTraceRadius ) || data.sourceTraceRadius < 0 )
			return false;
		data.sourceQueryConvexesCached = true;
		PrepareTraceData( data );
		return !stream.IsFailed();
	}
}

int JoltResolveSourceContactMaterial( const JPH::Body &body,
	const JPH::SubShapeID &subShapeID, JPH::RVec3Arg worldContactPoint,
	JPH::Vec3Arg worldSurfaceNormal, int objectMaterialIndex )
{
	const JPH::Shape *pShape = body.GetShape();
	const auto *pData = pShape ? reinterpret_cast< const JoltCollideUserData * >(
		pShape->GetUserData() ) : nullptr;
	if ( !pShape || !pData || pData->sourceQueryConvexes.empty() )
		return objectMaterialIndex;

	const JPH::RMat44 inverseWorld = body.GetWorldTransform().InversedRotationTranslation();
	const JPH::Vec3 localContactPoint = JPH::Vec3( inverseWorld * worldContactPoint );
	const JPH::Vec3 localSurfaceNormal = body.GetWorldTransform()
		.Multiply3x3Transposed( worldSurfaceNormal );

	size_t convexIndex = 0;
	int triangleMaterial = 0;
	if ( ResolveSourceConvexIndex( pShape, subShapeID, 0, convexIndex )
		&& ResolveSourceTriangleMaterial( *pData, convexIndex,
			localContactPoint, localSurfaceNormal, triangleMaterial ) )
	{
		return triangleMaterial != 0
			? JoltPhysicsSurfaceProps::GetInstance().RemapSourceTriangleMaterialIndex(
				triangleMaterial )
			: objectMaterialIndex;
	}

	return objectMaterialIndex;
}

void JoltEnsureSourceQueryData( JPH::Shape *pShape )
{
	if ( !pShape )
		return;

	JoltCollideUserData *pData = reinterpret_cast< JoltCollideUserData * >( pShape->GetUserData() );
	if ( !pData )
	{
		pData = new JoltCollideUserData();
		pShape->SetUserData( reinterpret_cast< uint64 >( pData ) );
	}
	if ( pData->sourceQueryConvexesCached )
	{
		PrepareTraceData( *pData );
		return;
	}

	pData->sourceQueryConvexes.clear();
	if ( pShape->GetType() == JPH::EShapeType::Compound )
	{
		JPH::CompoundShape *pCompound = static_cast< JPH::CompoundShape * >( pShape );
		for ( uint i = 0; i < pCompound->GetNumSubShapes(); ++i )
		{
			const size_t convexBase = pData->sourceQueryConvexes.size();
			if ( convexBase <= VJOLT_SOURCE_CONVEX_BASE_MASK )
				pCompound->SetCompoundUserData( i,
					VJOLT_SOURCE_CONVEX_BASE_VALID | uint32( convexBase ) );
			JPH::Shape *pChildShape = const_cast< JPH::Shape * >(
				pCompound->GetSubShape( i ).mShape.GetPtr() );
			JoltEnsureSourceQueryData( pChildShape );
			const JoltCollideUserData *pChildData = reinterpret_cast< const JoltCollideUserData * >(
				pChildShape->GetUserData() );
			if ( pChildData )
				AppendTransformedQueryConvexes( pCompound, i, *pChildData,
					pData->sourceQueryConvexes );
		}
	}
	else
	{
		JoltSourceQueryConvex &convex = pData->sourceQueryConvexes.emplace_back();
		convex.gameData = static_cast< unsigned int >( pData->gameData );
		BuildQueryConvexFromShape( pShape, convex );
	}

	PrepareTraceData( *pData );
	pData->sourceQueryConvexesCached = true;
}

//-------------------------------------------------------------------------------------------------

// Creates a shape from shape settings, resolving the reference. Every shape created
// via this helper carries a freshly-allocated JoltCollideUserData* in its UserData
// slot; freed in DestroyCollide. Sub-shapes inside compounds also go through here,
// so each gets its own struct (per-ledge gameData is set after the call).
template < typename ShapeType, typename T >
ShapeType *ShapeSettingsToShape( const T& settings )
{
	auto result = settings.Create();
	if ( !result.IsValid() )
	{
		// Josh:
		// Need to handle degenerate convexes and stuff.
		const char *error = result.HasError()
			? result.GetError().c_str()
			: "Unknown";

		Log_Warning( LOG_VJolt, "Failed to create shape: %s.\n", error );
		return nullptr;
	}

	auto *pShape = static_cast< ShapeType * >( ToDanglingRef( result.Get() ) );
	pShape->SetUserData( reinterpret_cast<uint64>( new JoltCollideUserData() ) );
	return pShape;
}

// Creates a JPH::ConvexShape from shape settings, and casts to a CPhysConvex
template < typename T >
CPhysConvex *ShapeSettingsToPhysConvex( const T& settings )
{
	return CPhysConvex::FromConvexShape( ShapeSettingsToShape< JPH::ConvexShape >( settings ) );
}

// Creates a JPH::Shape from shape settings, and casts to a CPhysCollide
template < typename T >
CPhysCollide *ShapeSettingsToPhysCollide( const T& settings )
{
	return CPhysCollide::FromShape( ShapeSettingsToShape< JPH::Shape >( settings ) );
}

//-------------------------------------------------------------------------------------------------

CPhysConvex *JoltPhysicsCollision::ConvexFromVerts( Vector **pVerts, int vertCount )
{
	// This uses this and not a CUtlVector or std::vector or whatever for two reasons:
	// 1: This is a single allocation we can then toss away instead of growing
	// 2: We do not want to initialize these vectors on allocation
	// 3: Automatic deletion when out of scope
	// Please do not change me to either.
	std::unique_ptr< JPH::Vec3[] > verts = std::make_unique< JPH::Vec3[] >( vertCount );
	for ( int i = 0; i < vertCount; i++ )
		verts[ i ] = SourceToJolt::Distance( *pVerts[ i ] );

	JPH::ConvexHullShapeSettings settings( verts.get(), vertCount, kMaxConvexRadius, nullptr /* material */ );
	settings.mHullTolerance = 0.0f;
	return ShapeSettingsToPhysConvex( settings );
}

CPhysConvex *JoltPhysicsCollision::ConvexFromPlanes( float *pPlanes, int planeCount, float mergeDistance )
{
	struct listplane_t
	{
		float normal[3];
		float dist;
	};

	if ( !pPlanes || planeCount < 4 )
		return nullptr;

	const listplane_t *pList = reinterpret_cast< const listplane_t * >( pPlanes );
	std::vector< VJoltHalfspacePlane > halfspaces;
	halfspaces.reserve( planeCount );
	for ( int i = 0; i < planeCount; ++i )
	{
		VJoltHalfspacePlane plane
		{
			{
				double( pList[ i ].normal[ 0 ] ),
				double( pList[ i ].normal[ 1 ] ),
				double( pList[ i ].normal[ 2 ] )
			},
			double( SourceToJolt::Distance( pList[ i ].dist ) )
		};
		if ( !std::isfinite( plane.normal[ 0 ] ) || !std::isfinite( plane.normal[ 1 ] )
			|| !std::isfinite( plane.normal[ 2 ] ) || !std::isfinite( plane.dist ) )
			return nullptr;

		bool discardPlane = false;
		std::vector< size_t > planesToDelete;
		for ( size_t oldIndex = 0; oldIndex < halfspaces.size(); ++oldIndex )
		{
			const VJoltHalfspacePlane &oldPlane = halfspaces[ oldIndex ];
			const double parallelDot = plane.normal[ 0 ] * oldPlane.normal[ 0 ]
				+ plane.normal[ 1 ] * oldPlane.normal[ 1 ]
				+ plane.normal[ 2 ] * oldPlane.normal[ 2 ];
			if ( parallelDot > 1.0 - VJOLT_HALFSPACE_TOLERANCE )
			{
				// IVP keeps the plane closest to the enclosed volume. With Source's
				// outward normals this is the plane with the smaller distance.
				if ( oldPlane.dist < plane.dist )
				{
					discardPlane = true;
					break;
				}
				planesToDelete.push_back( oldIndex );
			}
		}
		if ( discardPlane )
			continue;

		halfspaces.push_back( plane );
		for ( auto it = planesToDelete.rbegin(); it != planesToDelete.rend(); ++it )
			halfspaces.erase( halfspaces.begin() + *it );
	}

	if ( halfspaces.size() < 4 )
		return nullptr;

	const double mergeDistanceJolt = double( SourceToJolt::Distance( mergeDistance ) );
	const double mergeDistanceSquared = mergeDistanceJolt * mergeDistanceJolt;
	std::vector< VJoltHalfspacePoint > points;
	for ( size_t i = 0; i < halfspaces.size(); ++i )
	{
		for ( size_t j = i + 1; j < halfspaces.size(); ++j )
		{
			for ( size_t k = j + 1; k < halfspaces.size(); ++k )
			{
				VJoltHalfspacePoint point;
				if ( !IntersectHalfspacePlanes(
					halfspaces[ i ], halfspaces[ j ], halfspaces[ k ], point ) )
					continue;

				bool outside = false;
				for ( const VJoltHalfspacePlane &plane : halfspaces )
				{
					if ( plane.dist - HalfspaceDot( plane, point )
						< -VJOLT_HALFSPACE_TOLERANCE )
					{
						outside = true;
						break;
					}
				}
				if ( outside )
					continue;

				bool merged = false;
				for ( const VJoltHalfspacePoint &oldPoint : points )
				{
					if ( HalfspacePointDistanceSquared( point, oldPoint )
						< mergeDistanceSquared )
					{
						merged = true;
						break;
					}
				}
				if ( !merged )
					points.push_back( point );
			}
		}
	}

	if ( points.size() < 4 )
		return nullptr;

	std::vector< JPH::Vec3 > vertices;
	vertices.reserve( points.size() );
	for ( const VJoltHalfspacePoint &point : points )
	{
		vertices.emplace_back( float( point.value[ 0 ] ), float( point.value[ 1 ] ),
			float( point.value[ 2 ] ) );
	}
	if ( !RemoveSourceDegenerateHullPoints( vertices ) )
		return nullptr;

	JPH::ConvexHullShapeSettings settings(
		vertices.data(), int( vertices.size() ), kMaxConvexRadius, nullptr );
	settings.mHullTolerance = 0.0f;
	return ShapeSettingsToPhysConvex( settings );
}

float JoltPhysicsCollision::ConvexVolume( CPhysConvex *pConvex )
{
	return JoltToSource::Volume( pConvex->ToConvexShape()->GetVolume() );
}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsCollision::ConvexSurfaceArea( CPhysConvex *pConvex )
{
	return pConvex ? SourceQuerySurfaceArea( pConvex->ToPhysCollide()->ToShape() ) : 0.0f;
}

void JoltPhysicsCollision::SetConvexGameData( CPhysConvex *pConvex, unsigned int gameData )
{
	JoltCollideUserData *pData = pConvex->GetUserData();
	if ( pData )
	{
		pData->gameData = static_cast<int>( gameData );
		if ( pData->sourceQueryConvexesCached && pData->sourceQueryConvexes.size() == 1 )
			pData->sourceQueryConvexes[ 0 ].gameData = gameData;
	}
}

void JoltPhysicsCollision::ConvexFree( CPhysConvex *pConvex )
{
	if ( pConvex )
		DestroyCollide( pConvex->ToPhysCollide() );
}

CPhysConvex *JoltPhysicsCollision::BBoxToConvex( const Vector &mins, const Vector &maxs )
{
	JPH::AABox aabox = SourceToJolt::AABBBounds( mins, maxs );

	JPH::BoxShape *pBoxShape = new JPH::BoxShape( aabox.GetExtent(), kMaxConvexRadius, nullptr /* material */ );

	JPH::RotatedTranslatedShapeSettings rotatedSettings( aabox.mMin + aabox.GetExtent(), JPH::Quat::sIdentity(), pBoxShape );
	CPhysConvex *result = ShapeSettingsToPhysConvex( rotatedSettings );
	if ( result )
	{
		JoltEnsureSourceQueryData( result->ToPhysCollide()->ToShape() );
		auto &vertices = result->GetUserData()->sourceQueryConvexes[ 0 ].traceVertices;
		vertices.clear();
		// The shared IVP BBoxToConvex compact-ledge template's point order.
		for ( int corner : { 7, 2, 6, 3, 5, 4, 0, 1 } )
			vertices.emplace_back( ( corner & 1 ? maxs.x : mins.x ) * 0.0254f,
								   -( corner & 4 ? maxs.z : mins.z ) * 0.0254f,
								   ( corner & 2 ? maxs.y : mins.y ) * 0.0254f );
	}
	return result;
}

CPhysConvex *JoltPhysicsCollision::ConvexFromConvexPolyhedron( const CPolyhedron &ConvexPolyhedron )
{
	std::unique_ptr< JPH::Vec3[] > pPoints = std::make_unique< JPH::Vec3[] >( ConvexPolyhedron.iVertexCount );

	// This loop fills me with rage
	for ( unsigned short i = 0; i < ConvexPolyhedron.iVertexCount; ++i )
		pPoints[i] = SourceToJolt::Distance( ConvexPolyhedron.pVertices[i] );

	JPH::ConvexHullShapeSettings settings( pPoints.get(), ConvexPolyhedron.iVertexCount, kMaxConvexRadius, nullptr /* material */);
	settings.mHullTolerance = 0.0f; // Slart: Otherwise some polyhedrons crash :(
	CPhysConvex *pPhysConvex = ShapeSettingsToPhysConvex( settings );

	return pPhysConvex;
}

void JoltPhysicsCollision::ConvexesFromConvexPolygon( const Vector &vPolyNormal, const Vector *pPoints, int iPointCount, CPhysConvex **pOutput )
{
	// Source's implementation never reads the supplied normal. It converts the
	// polygon to a front/back alternating strip and turns every strip window into
	// a closed IVP ledge containing both triangle windings.
	( void )vPolyNormal;
	if ( !pPoints || !pOutput || iPointCount < 3 )
		return;

	std::vector< JPH::Vec3 > points;
	points.reserve( iPointCount );
	for ( int i = 0; i < iPointCount; ++i )
		points.push_back( SourceToJolt::Distance( pPoints[ i ] ) );

	std::vector< int > strip;
	strip.reserve( iPointCount );
	strip.push_back( 0 );
	strip.push_back( iPointCount - 1 );
	int forward = 1;
	int reverse = iPointCount - 2;
	while ( forward <= reverse )
	{
		strip.push_back( forward++ );
		if ( forward <= reverse )
			strip.push_back( reverse-- );
	}

	for ( int outputIndex = 0; outputIndex < iPointCount - 2; ++outputIndex )
	{
		const int indexA = strip[ outputIndex + 0 ];
		const int indexB = strip[ outputIndex + 1 ];
		const int indexC = strip[ outputIndex + 2 ];
		const JPH::Vec3 a = points[ indexA ];
		const JPH::Vec3 b = points[ indexB ];
		const JPH::Vec3 c = points[ indexC ];

		// convert_triangle_to_compace_ledge rejects an unnormalised plane whose
		// squared cross-product length is below P_DOUBLE_RES (1e-12).
		const double abX = double( b.GetX() ) - a.GetX();
		const double abY = double( b.GetY() ) - a.GetY();
		const double abZ = double( b.GetZ() ) - a.GetZ();
		const double acX = double( c.GetX() ) - a.GetX();
		const double acY = double( c.GetY() ) - a.GetY();
		const double acZ = double( c.GetZ() ) - a.GetZ();
		const double crossX = abY * acZ - abZ * acY;
		const double crossY = abZ * acX - abX * acZ;
		const double crossZ = abX * acY - abY * acX;
		if ( crossX * crossX + crossY * crossY + crossZ * crossZ < 1.0e-12 )
		{
			pOutput[ outputIndex ] = nullptr;
			continue;
		}

		JPH::TriangleShapeSettings settings( a, b, c, 0.0f, nullptr );
		CPhysConvex *pConvex = ShapeSettingsToPhysConvex( settings );
		pOutput[ outputIndex ] = pConvex;
		if ( !pConvex )
			continue;
		static_cast< JPH::TriangleShape * >(
			pConvex->ToConvexShape() )->SetCollideWithBackFaces( true );

		// CCollisionQuery walks the closed ledge's two IVP triangles in this exact
		// order: (c,b,a), then (c,a,b). Keep this independent of Jolt's one-face
		// tessellation so area, materials, serialization and query order match.
		JoltCollideUserData *pData = pConvex->GetUserData();
		JoltSourceQueryConvex &queryConvex = pData->sourceQueryConvexes.emplace_back();
		queryConvex.triangles.resize( 2 );
		const Vector &sourceA = pPoints[ indexA ];
		const Vector &sourceB = pPoints[ indexB ];
		const Vector &sourceC = pPoints[ indexC ];
		queryConvex.triangles[ 0 ].vertices[ 0 ] = sourceC;
		queryConvex.triangles[ 0 ].vertices[ 1 ] = sourceB;
		queryConvex.triangles[ 0 ].vertices[ 2 ] = sourceA;
		queryConvex.triangles[ 1 ].vertices[ 0 ] = sourceC;
		queryConvex.triangles[ 1 ].vertices[ 1 ] = sourceA;
		queryConvex.triangles[ 1 ].vertices[ 2 ] = sourceB;
		pData->sourceQueryConvexesCached = true;
	}
}

//-------------------------------------------------------------------------------------------------

CPhysPolysoup *JoltPhysicsCollision::PolysoupCreate()
{
	return new CPhysPolysoup;
}

void JoltPhysicsCollision::PolysoupDestroy( CPhysPolysoup *pSoup )
{
	delete pSoup;
}

void JoltPhysicsCollision::PolysoupAddTriangle( CPhysPolysoup *pSoup, const Vector &a, const Vector &b, const Vector &c, int materialIndex7bits )
{
	pSoup->Triangles.push_back( JPH::Triangle( SourceToJolt::DistanceFloat3( a ),
		SourceToJolt::DistanceFloat3( b ), SourceToJolt::DistanceFloat3( c ),
		uint8( materialIndex7bits ) & 0x7f ) );
}

CPhysCollide *JoltPhysicsCollision::ConvertPolysoupToCollide( CPhysPolysoup *pSoup, bool useMOPP )
{
	// GMod x64 builds Source with ENABLE_IVP_MOPP disabled, so both values use
	// the compact-surface ledge-soup path. Preserve that public behavior while
	// Jolt's MeshShape supplies the equivalent triangle acceleration structure.
	( void )useMOPP;

	// ConvertPolysoupToCollide does NOT free the Polysoup.
	JPH::TriangleList cookedTriangles = pSoup->Triangles;
	std::array< int, 128 > materialSlots;
	materialSlots.fill( -1 );
	materialSlots[ 0 ] = 0;
	JPH::PhysicsMaterialList materials;
	materials.push_back( JPH::PhysicsMaterial::sDefault );
	for ( JPH::Triangle &triangle : cookedTriangles )
	{
		triangle.mUserData = uint32( &triangle - cookedTriangles.data() );
		const int sourceMaterial = int( triangle.mMaterialIndex & 0x7f );
		int &slot = materialSlots[ sourceMaterial ];
		if ( slot < 0 )
		{
			// MeshShape has five material bits. Preserve all 128 Source values in
			// query metadata; encode the first 31 non-default values in Jolt and let
			// the contact compatibility resolver handle any overflow.
			if ( materials.size() < 32 )
			{
				slot = int( materials.size() );
				materials.push_back( CreateSourcePhysicsMaterial( sourceMaterial ) );
			}
			else
			{
				slot = 0;
			}
		}
		triangle.mMaterialIndex = uint32( slot );
	}

	JPH::MeshShapeSettings settings( std::move( cookedTriangles ), std::move( materials ) );
	settings.mPerTriangleUserData = true;
	CPhysCollide *pCollide = ShapeSettingsToPhysCollide( settings );
	if ( !pCollide )
		return nullptr;

	JoltCollideUserData *pData = pCollide->GetUserData();
	pData->sourceQueryConvexes.reserve( pSoup->Triangles.size() );
	std::map<std::array<float, 3>, size_t> pointIndices;
	for ( const JPH::Triangle &sourceTriangle : pSoup->Triangles )
	{
		JoltSourceQueryConvex &convex = pData->sourceQueryConvexes.emplace_back();
		std::array<std::pair<size_t, Vector>, 3> ordered;
		for ( int i = 0; i < 3; ++i )
		{
			const auto &point = sourceTriangle.mV[ i ];
			const std::array<float, 3> key{ point.x, -point.z, point.y };
			const auto entry = pointIndices.emplace( key, pointIndices.size() );
			ordered[ i ] = { entry.first->second, Vector( key[ 0 ], key[ 1 ], key[ 2 ] ) };
		}
		std::sort( ordered.begin(), ordered.end(), []( const auto &a, const auto &b ) { return a.first < b.first; } );
		for ( const auto &point : ordered )
			convex.traceVertices.push_back( point.second );
		convex.triangles.resize( 2 );
		const Vector a = JoltToSource::Distance( sourceTriangle.mV[ 0 ] );
		const Vector b = JoltToSource::Distance( sourceTriangle.mV[ 1 ] );
		const Vector c = JoltToSource::Distance( sourceTriangle.mV[ 2 ] );
		convex.triangles[ 0 ].vertices[ 0 ] = c;
		convex.triangles[ 0 ].vertices[ 1 ] = b;
		convex.triangles[ 0 ].vertices[ 2 ] = a;
		convex.triangles[ 1 ].vertices[ 0 ] = c;
		convex.triangles[ 1 ].vertices[ 1 ] = a;
		convex.triangles[ 1 ].vertices[ 2 ] = b;
		convex.triangles[ 0 ].materialIndex = uint8( sourceTriangle.mMaterialIndex ) & 0x7f;
		convex.triangles[ 1 ].materialIndex = uint8( sourceTriangle.mMaterialIndex ) & 0x7f;
	}
	pData->sourceQueryConvexesCached = true;
	pData->hasOuterHull = pData->sourceQueryConvexes.size() > 1;
	PrepareTraceData( *pData );
	return pCollide;
}

//-------------------------------------------------------------------------------------------------

CPhysCollide *JoltPhysicsCollision::ConvertConvexToCollide( CPhysConvex **pConvex, int convexCount )
{
	// If we only have one convex shape, we can just use that directly,
	// without making a compound shape.
	if ( convexCount == 1 )
	{
		JoltEnsureSourceQueryData( pConvex[ 0 ]->ToPhysCollide()->ToShape() );
		return pConvex[0]->ToPhysCollide();
	}

	JPH::StaticCompoundShapeSettings settings;
	for ( int i = 0; i < convexCount; i++ )
		settings.AddShape( JPH::Vec3::sZero(), JPH::Quat::sIdentity(), pConvex[i]->ToConvexShape() );

	CPhysCollide *pCollide = ShapeSettingsToPhysCollide( settings );
	if ( pCollide && pCollide->GetUserData() )
	{
		pCollide->GetUserData()->ownsChildUserData = true;
		JoltEnsureSourceQueryData( pCollide->ToShape() );
	}

	// This function also 'frees' the convexes.
	for ( int i = 0; i < convexCount; i++ )
		pConvex[i]->ToConvexShape()->Release();

	return pCollide;
}

CPhysCollide *JoltPhysicsCollision::ConvertConvexToCollideParams( CPhysConvex **pConvex, int convexCount, const convertconvexparams_t &convertParams )
{
	CPhysCollide *pCollide = ConvertConvexToCollide( pConvex, convexCount );
	if ( pCollide && pCollide->GetUserData() )
	{
		pCollide->GetUserData()->hasOuterHull = convertParams.buildOuterConvexHull
			|| convertParams.pForcedOuterHull != nullptr;
	}
	return pCollide;
}

void JoltPhysicsCollision::DestroyCollide( CPhysCollide *pCollide )
{
	if ( !pCollide )
		return;

	JPH::Shape *pShape = pCollide->ToShape();
	JoltCollideUserData *pRootData = pCollide->GetUserData();
	if ( pRootData && pRootData->ownsChildUserData )
	{
		JPH::UnorderedSet< JPH::Shape * > visitedShapes;
		JPH::UnorderedSet< JoltCollideUserData * > visitedData;
		visitedShapes.insert( pShape );
		visitedData.insert( pRootData );
		JPH::ShapeList children;
		pShape->SaveSubShapeState( children );
		for ( const JPH::RefConst< JPH::Shape > &child : children )
			DeleteCollideUserDataRecursive(
				const_cast< JPH::Shape * >( child.GetPtr() ), visitedShapes, visitedData );
	}
	delete pRootData;
	pShape->SetUserData( 0 );
	pShape->Release();
}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsCollision::CollideVolume( CPhysCollide *pCollide )
{
	return JoltToSource::Volume( pCollide->ToShape()->GetVolume() );
}

float JoltPhysicsCollision::CollideSurfaceArea( CPhysCollide *pCollide )
{
	return pCollide ? SourceQuerySurfaceArea( pCollide->ToShape() ) : 0.0f;
}

//-------------------------------------------------------------------------------------------------

Vector JoltPhysicsCollision::CollideGetExtent( const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, const Vector &direction )
{
	if ( !pCollide )
		return collideOrigin;

	const JPH::Shape *pShape = pCollide->ToShape();

	const JPH::Vec3 vecDirection = SourceToJolt::Unitless( direction );
	if ( vecDirection.IsNearZero() )
		return collideOrigin;
	const JPH::Quat rotation = SourceToJolt::Angle( collideAngles );
	const JPH::Vec3 positionCOM = SourceToJolt::Distance( collideOrigin ) + rotation * pShape->GetCenterOfMass();
	JPH::AllHitCollisionCollector<JPH::TransformedShapeCollector> collector;
	JPH::ShapeFilter filter;
	pShape->CollectTransformedShapes( JPH::AABox::sBiggest(), positionCOM, rotation,
		JPH::Vec3::sOne(), JPH::SubShapeIDCreator(), collector, filter );

	float flMaxDot = -FLT_MAX;
	JPH::Vec3 vecMaxExtent = SourceToJolt::Distance( collideOrigin );
	const auto consider = [&]( JPH::Vec3Arg vertex )
	{
		const float dot = vertex.Dot( vecDirection );
		if ( dot > flMaxDot )
		{
			vecMaxExtent = vertex;
			flMaxDot = dot;
		}
	};
	for ( const auto &leaf : collector.mHits )
	{
		if ( leaf.mShape->GetType() == JPH::EShapeType::Convex )
		{
			// Support points also cover spheres and retain decorated-shape transforms/scales.
			const auto *convex = static_cast<const JPH::ConvexShape *>( leaf.mShape.GetPtr() );
			JPH::ConvexShape::SupportBuffer buffer;
			const auto *support = convex->GetSupportFunction(
				JPH::ConvexShape::ESupportMode::IncludeConvexRadius, buffer, leaf.GetShapeScale() );
			const JPH::Vec3 localDirection = leaf.mShapeRotation.Conjugated() * vecDirection;
			consider( leaf.GetCenterOfMassTransform() * support->GetSupport( localDirection ) );
			continue;
		}

		// Mesh leaves have no convex support function; inspect their actual vertices.
		constexpr int batchSize = 128;
		JPH::Float3 vertices[batchSize * 3];
		JPH::TransformedShape::GetTrianglesContext context;
		leaf.GetTrianglesStart( context, JPH::AABox::sBiggest(), JPH::Vec3::sZero() );
		for ( int count; ( count = leaf.GetTrianglesNext( context, batchSize, vertices ) ) > 0; )
			for ( int i = 0; i < count * 3; ++i )
				consider( JPH::Vec3( vertices[i].x, vertices[i].y, vertices[i].z ) );
	}

	return JoltToSource::Distance( vecMaxExtent );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsCollision::CollideGetAABB( Vector *pMins, Vector *pMaxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles )
{
	JPH::Vec3 position = SourceToJolt::Distance( collideOrigin );
	JPH::Quat rotation = SourceToJolt::Angle( collideAngles );

	const JPH::Shape *pShape = pCollide->ToShape();

	const JPH::Mat44 translation = JPH::Mat44::sRotationTranslation( rotation, position + rotation * pShape->GetCenterOfMass() );
	const JPH::AABox worldBounds = pShape->GetWorldSpaceBounds( translation, JPH::Vec3::sReplicate( 1.0f ) );

	JoltToSource::AABBBounds( worldBounds, *pMins, *pMaxs );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsCollision::CollideGetMassCenter( CPhysCollide *pCollide, Vector *pOutMassCenter )
{
	const JoltCollideUserData *pData = pCollide->GetUserData();
	const JPH::Vec3 massCenter = pData && pData->hasSourceMassProperties
		? pData->sourceMassCenter
		: pCollide->ToShape()->GetCenterOfMass();
	*pOutMassCenter = JoltToSource::Distance( massCenter );
}

void JoltPhysicsCollision::CollideSetMassCenter( CPhysCollide *pCollide, const Vector &massCenter )
{
	if ( !pCollide )
		return;

	JoltCollideUserData *pData = pCollide->GetUserData();
	if ( !pData )
	{
		pData = new JoltCollideUserData();
		pCollide->SetUserData( pData );
	}

	if ( !pData->hasSourceMassProperties )
	{
		JPH::MassProperties massProperties = pCollide->ToShape()->GetMassProperties();
		massProperties.ScaleToMass( 1.0f );
		pData->sourceRotationInertiaPerMass = JPH::Vec3(
			massProperties.mInertia( 0, 0 ), massProperties.mInertia( 1, 1 ),
			massProperties.mInertia( 2, 2 ) );
		pData->hasSourceMassProperties = true;
	}

	pData->sourceMassCenter = SourceToJolt::Distance( massCenter );
}

Vector JoltPhysicsCollision::CollideGetOrthographicAreas( const CPhysCollide *pCollide )
{
	// Per IPhysicsCollision contract: returns 1,1,1 if not precomputed. The IVP
	// deserializer fills this in from compactsurfaceheader_t::dragAxisAreas.
	if ( !pCollide )
		return Vector( 1.0f, 1.0f, 1.0f );

	const JoltCollideUserData *pData = pCollide->GetUserData();
	return pData ? pData->orthographicAreas : Vector( 1.0f, 1.0f, 1.0f );
}

void JoltPhysicsCollision::CollideSetOrthographicAreas( CPhysCollide *pCollide, const Vector &areas )
{
	if ( !pCollide )
		return;

	JoltCollideUserData *pData = pCollide->GetUserData();
	if ( pData )
		pData->orthographicAreas = areas;
}

//-------------------------------------------------------------------------------------------------

int JoltPhysicsCollision::CollideIndex( const CPhysCollide *pCollide )
{
	[[maybe_unused]] const JPH::Shape *shape = pCollide->ToShape();
	VJoltAssert( ( shape->GetType() == JPH::EShapeType::Convex && shape->GetSubType() == JPH::EShapeSubType::ConvexHull ) || ( shape->GetType() == JPH::EShapeType::Compound && shape->GetSubType() == JPH::EShapeSubType::StaticCompound ) );
	const JoltCollideUserData *pData = pCollide->GetUserData();
	return pData ? pData->vcollideIndex : 0;
}

//-------------------------------------------------------------------------------------------------

CPhysCollide *JoltPhysicsCollision::BBoxToCollide( const Vector &mins, const Vector &maxs )
{
	return BBoxToConvex( mins, maxs )->ToPhysCollide();
}

int JoltPhysicsCollision::GetConvexesUsedInCollideable( const CPhysCollide *pCollideable, CPhysConvex **pOutputArray, int iOutputArrayLimit )
{
	const JPH::Shape *pShape = pCollideable->ToShape();
	JPH::EShapeType shapeType = pShape->GetType();
	if ( shapeType != JPH::EShapeType::Compound )
	{
		pOutputArray[0] = const_cast<CPhysConvex *>( CPhysConvex::FromConvexShape( static_cast<const JPH::ConvexShape *>( pShape ) ) );
		return 1;
	}

	const JPH::StaticCompoundShape *pCompoundShape = static_cast<const JPH::StaticCompoundShape *>( pShape );
	const JPH::StaticCompoundShape::SubShapes &subShapes = pCompoundShape->GetSubShapes();

	const uint maxNumShapes = Min<uint>( pCompoundShape->GetNumSubShapes(), iOutputArrayLimit );
	for ( uint i = 0; i < maxNumShapes; ++i )
		pOutputArray[i] = const_cast<CPhysConvex *>( CPhysConvex::FromConvexShape( static_cast<const JPH::ConvexShape *>( subShapes[i].mShape.GetPtr() ) ) );

	return maxNumShapes;
}

//-------------------------------------------------------------------------------------------------

bool JoltPhysicsCollision::IsBoxIntersectingCone( const Vector &boxAbsMins, const Vector &boxAbsMaxs, const truncatedcone_t &cone )
{
	return vjolt_source_cone_overlap::IsBoxIntersectingCone(
		boxAbsMins, boxAbsMaxs, cone );
}

//-------------------------------------------------------------------------------------------------

//
// See studiobyteswap from the public 2013 SDK for more info about these defines.
// https://github.com/ValveSoftware/source-sdk-2013/blob/master/sp/src/common/studiobyteswap.cpp
//

namespace ivp_compat
{
	struct collideheader_t
	{
		int		vphysicsID;
		short	version;
		short	modelType;
	};

	struct compactsurfaceheader_t
	{
		int		surfaceSize;
		Vector	dragAxisAreas;
		int		axisMapSize;
	};

	struct moppsurfaceheader_t
	{
		int moppSize;
	};

	struct compactsurface_t
	{
		float	mass_center[3];
		float	rotation_inertia[3];
		float	upper_limit_radius;

		unsigned int	max_factor_surface_deviation	: 8;
		int		        byte_size						: 24;
		int		        offset_ledgetree_root;
		int		        dummy[3];
	};

	struct compactmopp_t
	{
		float	mass_center[3];
		float	rotation_inertia[3];
		float	upper_limit_radius;

		unsigned int max_factor_surface_deviation	: 8;
		int          byte_size						: 24;
		int	         offset_ledgetree_root;
		int          offset_ledges;
		int          size_convex_hull;
		int	         dummy;
	};

	struct compactledge_t
	{
		int		c_point_offset;

		union
		{
			int	ledgetree_node_offset;
			int	client_data;
		};

		struct
		{
			uint	has_children_flag	: 2;
			int		is_compact_flag		: 2;
			uint	dummy				: 4;
			uint	size_div_16			: 24; 
		};

		short	n_triangles;
		short	for_future_use;
	};

	struct compactedge_t
	{
		uint    start_point_index	: 16;
		int		opposite_index		: 15;
		uint	is_virtual			: 1;
	};

	struct compacttriangle_t
	{
		uint	tri_index		: 12;
		uint	pierce_index	: 12;
		uint	material_index	: 7;
		uint	is_virtual		: 1;
		compactedge_t c_three_edges[3];
	};

	struct compactledgenode_t
	{
		int		offset_right_node;
		int		offset_compact_ledge;
		float	center[3];
		float	radius;
		unsigned char box_sizes[3];
		unsigned char free_0;

		const compactledge_t *GetCompactLedge() const
		{
			VJoltAssert( this->offset_right_node == 0 );
			return ( compactledge_t * )( ( char * )this + this->offset_compact_ledge );
		}

		const compactledgenode_t *GetLeftChild() const
		{
			VJoltAssert( this->offset_right_node );
			return this + 1;
		}

		const compactledgenode_t *GetRightChild() const
		{
			VJoltAssert( this->offset_right_node );
			return ( compactledgenode_t * )( ( char * )this + this->offset_right_node );
		}

		bool IsTerminal() const
		{
			return this->offset_right_node == 0;
		}

		const compactledge_t *GetCompactHull() const
		{
			if ( this->offset_compact_ledge )
				return ( compactledge_t * )( ( char * )this + this->offset_compact_ledge );
			else
				return nullptr;
		}
	};

	static constexpr int	IVP_COMPACT_SURFACE_SUPER_LEGACY	= 0; // Really old .phy files, don't have anything here, and were just serialized compact headers directly.
	static constexpr int	IVP_COMPACT_SURFACE_ID				= MAKEID('I','V','P','S');
	static constexpr int	IVP_COMPACT_SURFACE_ID_SWAPPED		= MAKEID('S','P','V','I');
	static constexpr int	IVP_COMPACT_MOPP_ID					= MAKEID('M','O','P','P');
	static constexpr int	VPHYSICS_COLLISION_ID				= MAKEID('V','P','H','Y');
	static constexpr int	JOLT_COLLISION_ID					= MAKEID('J','P','H','Y');
	static constexpr short	VPHYSICS_COLLISION_VERSION			= 0x0100;

	enum
	{
		COLLIDE_POLY = 0,
		COLLIDE_MOPP = 1,
		COLLIDE_BALL = 2,
		COLLIDE_VIRTUAL = 3,
	};

	JPH::ConvexShape *IVPLedgeToConvexShape( const compactledge_t *pLedge, int vcollideIndex, int traceLoadIndex )
	{
		if ( !pLedge->n_triangles )
			return nullptr;

		const char *pVertices = reinterpret_cast< const char * >( pLedge ) + pLedge->c_point_offset;
		const compacttriangle_t *pTriangles = reinterpret_cast< const compacttriangle_t * >( pLedge + 1 );

		const int nVertCount = pLedge->n_triangles * 3;

		std::unique_ptr< JPH::Vec3[] > verts = std::make_unique< JPH::Vec3[] >( nVertCount );

		// Each triangle
		for ( int i = 0; i < pLedge->n_triangles; i++ )
		{
			// For each point of the current triangle
			for ( int j = 0; j < 3; j++ )
			{
				static constexpr size_t IVPAlignedVectorSize = 16;

				const int nIndex = pTriangles[ i ].c_three_edges[ j ].start_point_index;
				const float *pVertex = reinterpret_cast< const float * >( pVertices + ( nIndex * IVPAlignedVectorSize ) );

				verts[ ( i * 3 ) + j ] = JPH::Vec3( pVertex[ 0 ], pVertex[ 2 ], -pVertex[ 1 ] );
			}
		}

		int commonMaterialIndex = pTriangles[ 0 ].material_index;
		for ( int i = 1; i < pLedge->n_triangles; ++i )
		{
			if ( int( pTriangles[ i ].material_index ) != commonMaterialIndex )
			{
				commonMaterialIndex = 0;
				break;
			}
		}
		JPH::RefConst< JPH::PhysicsMaterial > material = commonMaterialIndex != 0
			? CreateSourcePhysicsMaterial( commonMaterialIndex ) : nullptr;
		JPH::ConvexHullShapeSettings settings{ verts.get(), nVertCount, kMaxConvexRadius,
			material.GetPtr() };
		settings.mHullTolerance = 0.0f;
		JPH::ConvexShape *pConvexShape = ShapeSettingsToShape< JPH::ConvexShape >( settings );
		if ( !pConvexShape )
		{
			// Wow that sucks, just mock up a small sphere to subsitute.
			// This can happen for models with extremely broken collision hulls.
			// If we don't do this, we'll crash later on because older versions of Source are missing
			// an important nullptr check.
			// A better solution would be to generate a valid convex hull from the points provided.
			JPH::SphereShapeSettings sphereSettings( 1.0f );
			pConvexShape = ShapeSettingsToShape< JPH::ConvexShape >( sphereSettings );
			if ( !pConvexShape )
			{
				// This should never fail, but catching anyway
				return nullptr;
			}
		}
		
		// Per-ledge gameData is the IVP client_data (engine contents flag).
		// vcollideIndex is propagated from the parent solid.
		JoltCollideUserData *pData = CPhysCollide::FromShape( pConvexShape )->GetUserData();
		pData->gameData = static_cast<int>( pLedge->client_data );
		pData->vcollideIndex = vcollideIndex;
		JoltSourceQueryConvex &convex = pData->sourceQueryConvexes.emplace_back();
		convex.gameData = static_cast< unsigned int >( pLedge->client_data );
		convex.traceLoadIndex = traceLoadIndex;
		std::vector<int> pointIndices;
		for ( int i = 0; i < pLedge->n_triangles; ++i )
			for ( int j = 0; j < 3; ++j )
				pointIndices.push_back( pTriangles[ i ].c_three_edges[ j ].start_point_index );
		std::sort( pointIndices.begin(), pointIndices.end() );
		pointIndices.erase( std::unique( pointIndices.begin(), pointIndices.end() ), pointIndices.end() );
		for ( int index : pointIndices )
		{
			const float *point = reinterpret_cast<const float *>( pVertices + index * 16 );
			convex.traceVertices.emplace_back( point[ 0 ], point[ 1 ], point[ 2 ] );
		}
		convex.traceCachedSupport = pLedge->n_triangles <= 512 && pointIndices.back() - pointIndices.front() + 1 < 128;
		if ( !convex.traceCachedSupport )
		{
			const char *firstEdge = reinterpret_cast<const char *>( &pTriangles[ 0 ].c_three_edges[ 0 ] );
			for ( int i = 0; i < pLedge->n_triangles; ++i )
				for ( int j = 0; j < 3; ++j )
				{
					const auto &edge = pTriangles[ i ].c_three_edges[ j ];
					const int vertex =
						int( std::lower_bound( pointIndices.begin(), pointIndices.end(), edge.start_point_index ) -
							 pointIndices.begin() );
					const char *opposite = reinterpret_cast<const char *>( &edge ) + edge.opposite_index * 4;
					const int offset = int( opposite - firstEdge );
					convex.traceEdges.emplace_back( vertex, ( offset / 16 ) * 3 + ( offset % 16 ) / 4 );
				}
		}
		convex.triangles.reserve( pLedge->n_triangles );
		for ( int i = 0; i < pLedge->n_triangles; ++i )
		{
			JoltSourceQueryTriangle &triangle = convex.triangles.emplace_back();
			triangle.vertices[ 0 ] = JoltToSource::Distance( verts[ i * 3 + 2 ] );
			triangle.vertices[ 1 ] = JoltToSource::Distance( verts[ i * 3 + 1 ] );
			triangle.vertices[ 2 ] = JoltToSource::Distance( verts[ i * 3 + 0 ] );
			triangle.materialIndex = uint8( pTriangles[ i ].material_index ) & 0x7f;
		}
		pData->sourceQueryConvexesCached = true;
		PrepareTraceData( *pData );

		return pConvexShape;
	}

	void GetAllIVPEdges( const compactledgenode_t *pNode, CUtlVector< const compactledge_t * >& vecOut )
	{
		if ( !pNode )
			return;

		if ( !pNode->IsTerminal() )
		{
			GetAllIVPEdges( pNode->GetLeftChild(), vecOut );
			GetAllIVPEdges( pNode->GetRightChild(), vecOut );
		}
		else
			vecOut.AddToTail( pNode->GetCompactLedge() );
	}

	bool HasIVPOuterHull( const compactledgenode_t *pNode )
	{
		if ( !pNode || pNode->IsTerminal() )
			return false;
		if ( pNode->GetCompactHull() )
			return true;
		return HasIVPOuterHull( pNode->GetLeftChild() )
			|| HasIVPOuterHull( pNode->GetRightChild() );
	}

	void SetSourceMassProperties( CPhysCollide *pCollide, const compactsurface_t *pSurface )
	{
		if ( !pCollide || !pSurface )
			return;

		JoltCollideUserData *pData = pCollide->GetUserData();
		if ( !pData )
			return;

		// IVP object space maps to Source/Jolt as (x, z, -y). Inertia is an
		// unsigned axis quantity, so only the Y/Z components are permuted.
		pData->sourceMassCenter = JPH::Vec3(
			pSurface->mass_center[0],
			pSurface->mass_center[2],
			-pSurface->mass_center[1] );
		pData->sourceRotationInertiaPerMass = JPH::Vec3(
			pSurface->rotation_inertia[0],
			pSurface->rotation_inertia[2],
			pSurface->rotation_inertia[1] );
		pData->hasSourceMassProperties = true;
	}

	int CopyTraceTree( const compactledgenode_t *source, JoltCollideUserData &data, const std::vector<int> &indices,
					   int &leaf )
	{
		const int index = int( data.sourceTraceNodes.size() );
		JoltSourceTraceNode node;
		node.center.Init( source->center[ 0 ], source->center[ 1 ], source->center[ 2 ] );
		node.radius = source->radius;
		data.sourceTraceNodes.push_back( node );
		if ( source->IsTerminal() )
		{
			data.sourceTraceNodes[ index ].convex = indices[ leaf++ ];
		}
		else
		{
			const int left = CopyTraceTree( source->GetLeftChild(), data, indices, leaf );
			const int right = CopyTraceTree( source->GetRightChild(), data, indices, leaf );
			data.sourceTraceNodes[ index ].left = left;
			data.sourceTraceNodes[ index ].right = right;
		}
		return index;
	}

	CPhysCollide *DeserializeIVP_Poly( const compactsurface_t* pSurface, int vcollideIndex )
	{
		const compactledgenode_t *pFirstLedgeNode = reinterpret_cast< const compactledgenode_t * >(
				reinterpret_cast< const char * >( pSurface ) + pSurface->offset_ledgetree_root );

		CUtlVector< const compactledge_t * > ledges;
		GetAllIVPEdges( pFirstLedgeNode, ledges );

		VJoltAssert( !ledges.IsEmpty() );

		CPhysCollide *pCollide = nullptr;
		if ( ledges.Count() != 1 )
		{
			JPH::StaticCompoundShapeSettings settings{};
			// One compound convex per ledge.
			for ( int i = 0; i < ledges.Count(); i++ )
			{
				const JPH::Shape *pShape = IVPLedgeToConvexShape( ledges[ i ], vcollideIndex, i );
				// Josh:
				// Some models have degenerate convexes which fails to make
				// a subshape in Jolt, so we need to ignore those ledges.
				if ( pShape )
					settings.AddShape( JPH::Vec3::sZero(), JPH::Quat::sIdentity(), pShape );
			}
			JPH::Shape *shape = ShapeSettingsToShape<JPH::Shape>( settings );
			pCollide = CPhysCollide::FromShape( shape );
			pCollide->GetUserData()->vcollideIndex = vcollideIndex;
			pCollide->GetUserData()->ownsChildUserData = true;
			// Contacts can occur before anyone asks for ICollisionQuery. Build the
			// flattened ledge metadata now so a multi-convex .phy uses its triangle
			// materials from the very first simulation frame.
			JoltEnsureSourceQueryData( shape );
		}
		else
		{
			// Single-ledge solid: the convex IS the root. Its UserData (already
			// allocated by IVPLedgeToConvexShape) carries both per-ledge gameData
			// and the propagated vcollideIndex -- nothing more to set here.
			JPH::ConvexShape *pShape = IVPLedgeToConvexShape( ledges[ 0 ], vcollideIndex, 0 );
			pCollide = CPhysCollide::FromShape( pShape );
		}

		SetSourceMassProperties( pCollide, pSurface );
		if ( pCollide && pCollide->GetUserData() )
		{
			auto &data = *pCollide->GetUserData();
			data.hasOuterHull = HasIVPOuterHull( pFirstLedgeNode );
			data.sourceTraceNodes.clear();
			std::vector<int> indices( ledges.Count(), -1 );
			for ( size_t i = 0; i < data.sourceQueryConvexes.size(); ++i )
			{
				const int index = data.sourceQueryConvexes[ i ].traceLoadIndex;
				if ( index >= 0 && size_t( index ) < indices.size() )
					indices[ index ] = int( i );
			}
			int leaf = 0;
			CopyTraceTree( pFirstLedgeNode, data, indices, leaf );
			data.sourceTraceRadius = pSurface->upper_limit_radius +
									 std::sqrt( pSurface->mass_center[ 0 ] * pSurface->mass_center[ 0 ] +
												pSurface->mass_center[ 1 ] * pSurface->mass_center[ 1 ] +
												pSurface->mass_center[ 2 ] * pSurface->mass_center[ 2 ] + 1e-20f );
			data.sourceTraceCached = true;
		}
		return pCollide;
	}

	CPhysCollide *DeserializeIVP_Poly( const collideheader_t *pCollideHeader, int vcollideIndex )
	{
		const compactsurfaceheader_t *pSurfaceHeader = reinterpret_cast< const compactsurfaceheader_t* >( pCollideHeader + 1 );
		const compactsurface_t *pSurface = reinterpret_cast< const compactsurface_t* >( pSurfaceHeader + 1 );

		CPhysCollide *pCollide = DeserializeIVP_Poly( pSurface, vcollideIndex );
		if ( pCollide )
			pCollide->GetUserData()->orthographicAreas = pSurfaceHeader->dragAxisAreas;
		return pCollide;
	}

	CPhysCollide *Deserialize( const char *pCursor, int solidSize, unsigned int i )
	{
		if ( !pCursor || solidSize < int( sizeof( collideheader_t ) ) )
			return nullptr;

		const collideheader_t *pCollideHeader = reinterpret_cast<const collideheader_t *>( pCursor );
		const int vcollideIndex = static_cast<int>( i );

		if ( pCollideHeader->vphysicsID == VPHYSICS_COLLISION_ID )
		{
			// This is the main path that everything falls down for a modern
			// .phy file with the collide header.

			if ( pCollideHeader->version != VPHYSICS_COLLISION_VERSION )
				Log_Warning( LOG_VJolt, "Solid with unknown version: 0x%x, may crash!\n", pCollideHeader->version );

			switch ( pCollideHeader->modelType )
			{
				case COLLIDE_POLY:
					return DeserializeIVP_Poly( pCollideHeader, vcollideIndex );
					break;

				case COLLIDE_MOPP:
					Log_Warning( LOG_VJolt, "Unsupported solid type COLLIDE_MOPP on solid %d. Skipping...\n", i );
					break;
	
				case COLLIDE_BALL:
					Log_Warning( LOG_VJolt, "Unsupported solid type COLLIDE_BALL on solid %d. Skipping...\n", i );
					break;

				case COLLIDE_VIRTUAL:
					Log_Warning( LOG_VJolt, "Unsupported solid type COLLIDE_VIRTUAL on solid %d. Skipping...\n", i );
					break;

				default:
					Log_Warning( LOG_VJolt, "Unsupported solid type 0x%x on solid %d. Skipping...\n", (int)pCollideHeader->modelType, i );
					break;
			}
		}
		else if ( pCollideHeader->vphysicsID == JOLT_COLLISION_ID )
		{
			if ( pCollideHeader->version >= 2 && pCollideHeader->version <= 4 )
			{
				VJoltCollideReadStream stream( reinterpret_cast< const char * >( pCollideHeader + 1 ),
					solidSize - sizeof( collideheader_t ) );
				JoltCollideUserData restoredData;
				if ( !ReadCollideMetadata( stream, restoredData, pCollideHeader->version ) )
					return nullptr;

				JPH::Shape::IDToShapeMap shapeMap;
				JPH::Shape::IDToMaterialMap materialMap;
				JPH::Shape::ShapeResult shape = JPH::Shape::sRestoreWithChildren(
					stream, shapeMap, materialMap );
				if ( shape.HasError() || stream.IsFailed() || !shape.Get() )
				{
					Log_Warning( LOG_VJolt, "Jolt collide deserialize error: %s\n",
						shape.HasError() ? shape.GetError().c_str() : "truncated stream" );
					return nullptr;
				}

				JPH::Shape *pRestoredShape = ToDanglingRef( shape.Get() );
				EnableSourceTriangleBackFacesRecursive( pRestoredShape );
				JoltCollideUserData *pRootData = ReplaceRestoredUserData( pRestoredShape );
				*pRootData = std::move( restoredData );
				pRootData->vcollideIndex = vcollideIndex;
				if ( pCollideHeader->version < 4 && pRestoredShape->GetSubType() == JPH::EShapeSubType::Mesh
					&& pRootData->sourceQueryConvexes.size() > 1 )
				{
					pRootData->hasOuterHull = true;
				}
				JPH::ShapeList restoredChildren;
				pRestoredShape->SaveSubShapeState( restoredChildren );
				pRootData->ownsChildUserData = !restoredChildren.empty();

				if ( pRestoredShape->GetType() == JPH::EShapeType::Compound )
				{
					const JPH::CompoundShape *pCompound =
						static_cast< const JPH::CompoundShape * >( pRestoredShape );
					const uint count = Min< uint >( pCompound->GetNumSubShapes(),
						uint( pRootData->sourceQueryConvexes.size() ) );
					for ( uint j = 0; j < count; ++j )
					{
						JPH::Shape *pChildShape = const_cast< JPH::Shape * >(
							pCompound->GetSubShape( j ).mShape.GetPtr() );
						JoltCollideUserData *pChildData = reinterpret_cast< JoltCollideUserData * >(
							pChildShape->GetUserData() );
						if ( pChildData )
							pChildData->gameData = int( pRootData->sourceQueryConvexes[ j ].gameData );
					}
				}
				return CPhysCollide::FromShape( pRestoredShape );
			}

			if ( pCollideHeader->version != 1 )
			{
				Log_Warning( LOG_VJolt, "Unsupported Jolt collide version %d.\n",
					pCollideHeader->version );
				return nullptr;
			}

			struct Reader : JPH::StreamIn, CUtlBuffer
			{
				using CUtlBuffer::CUtlBuffer;

				void ReadBytes( void *outData, size_t inNumBytes ) override
				{
					Get( outData, static_cast<int>(inNumBytes) );
				}

				bool IsEOF() const override
				{
					return TellGet() == TellMaxPut();
				}

				bool IsFailed() const override
				{
					return !IsValid();
				}
			} reader( pCollideHeader + 1, solidSize - sizeof( collideheader_t ), CUtlBuffer::READ_ONLY );

			JPH::Shape::ShapeResult shape = JPH::Shape::sRestoreFromBinaryState( reader );
			if ( shape.HasError() )
			{
				Log_Warning( LOG_VJolt, "Jolt collide deserialize error: %s\n", shape.GetError().c_str() );
				return nullptr;
			}
			const int subShapeCount = reader.GetInt();
			JPH::ShapeList subShapes;
			for ( int j = 0; j < subShapeCount; ++j )
			{
				JPH::Shape::ShapeResult subShape = JPH::Shape::sRestoreFromBinaryState( reader );
				if ( subShape.HasError() )
				{
					Log_Warning( LOG_VJolt, "Jolt collide deserialize error: %s\n", shape.GetError().c_str() );
					return nullptr;
				}
				subShapes.emplace_back( subShape.Get() );
			}
			shape.Get()->RestoreSubShapeState( subShapes.data(), subShapeCount );
			JPH::Shape *pRestoredShape = ToDanglingRef( shape.Get() );
			EnableSourceTriangleBackFacesRecursive( pRestoredShape );
			JoltCollideUserData *pData = ReplaceRestoredUserData( pRestoredShape );
			pData->vcollideIndex = vcollideIndex;
			JoltEnsureSourceQueryData( pRestoredShape );
			if ( pRestoredShape->GetSubType() == JPH::EShapeSubType::Mesh
				&& pData->sourceQueryConvexes.size() > 1 )
			{
				pData->hasOuterHull = true;
			}
			return CPhysCollide::FromShape( pRestoredShape );
		}
		else
		{
			// This must be a legacy style .phy where it is just a dumped compact surface.
			// Some props in shipping HL2 still use this format, as they have a .phy, even after their
			// .qc had the $collisionmodel removed, as they didn't get the stale .phy in the game files deleted.

			const compactsurface_t *pCompactSurface = reinterpret_cast<const compactsurface_t *>( pCursor );
			const int legacyModelType = pCompactSurface->dummy[2];
			switch ( legacyModelType )
			{
				case IVP_COMPACT_SURFACE_SUPER_LEGACY:
				case IVP_COMPACT_SURFACE_ID:
				case IVP_COMPACT_SURFACE_ID_SWAPPED:
					return DeserializeIVP_Poly( pCompactSurface, vcollideIndex );
					break;

				case IVP_COMPACT_MOPP_ID:
					Log_Warning( LOG_VJolt, "Unsupported legacy solid type IVP_COMPACT_MOPP_ID on solid %d. Skipping...\n", i );
					break;

				default:
					Log_Warning( LOG_VJolt, "Unsupported legacy solid type 0x%x on solid %d. Skipping...\n", legacyModelType, i);
					break;
			}
		}
		return nullptr;
	}
}

void JoltPhysicsCollision::VCollideLoad( vcollide_t *pOutput, int solidCount, const char *pBuffer, int size, bool swap /*= false*/ )
{
	if ( !pOutput )
		return;
	V_memset( pOutput, 0, sizeof( *pOutput ) );

	if ( swap )
	{
		Log_Error( LOG_VJolt, "If you got here. Tell me what you did!\n" );
		return;
	}
	if ( solidCount < 0 || size < 0 || ( size > 0 && !pBuffer )
		|| size_t( solidCount ) > size_t( size ) / sizeof( int ) )
	{
		Log_Warning( LOG_VJolt, "Rejected malformed VCollide header.\n" );
		return;
	}

	pOutput->solidCount = solidCount;
	pOutput->solids = solidCount > 0 ? new CPhysCollide*[ solidCount ]() : nullptr;

	size_t offset = 0;
	bool malformed = false;
	for ( int i = 0; i < solidCount; i++ )
	{
		if ( offset > size_t( size ) || size_t( size ) - offset < sizeof( int ) )
		{
			malformed = true;
			break;
		}

		int solidSize = 0;
		V_memcpy( &solidSize, pBuffer + offset, sizeof( solidSize ) );
		offset += sizeof( solidSize );
		if ( solidSize < 0 || size_t( solidSize ) > size_t( size ) - offset )
		{
			malformed = true;
			break;
		}

		pOutput->solids[ i ] = ivp_compat::Deserialize( pBuffer + offset, solidSize, i );
		offset += size_t( solidSize );
	}

	if ( malformed )
	{
		for ( int i = 0; i < solidCount; ++i )
			DestroyCollide( pOutput->solids[ i ] );
		delete[] pOutput->solids;
		V_memset( pOutput, 0, sizeof( *pOutput ) );
		Log_Warning( LOG_VJolt, "Rejected truncated or oversized VCollide payload.\n" );
		return;
	}

	// The rest of the buffer is KV.
	const int keyValuesSize = int( size_t( size ) - offset );

	pOutput->pKeyValues = new char[ keyValuesSize + 1 ];
	if ( keyValuesSize > 0 )
		V_memcpy( pOutput->pKeyValues, pBuffer + offset, keyValuesSize );
	pOutput->pKeyValues[ keyValuesSize ] = '\0';

	pOutput->descSize = keyValuesSize;
	pOutput->isPacked = false;
#ifdef GAME_ASW_OR_NEWER
	pOutput->pUserData = nullptr;
#endif

}

void JoltPhysicsCollision::VCollideUnload( vcollide_t *pVCollide )
{
	VCollideFreeUserData( pVCollide );
	for ( int i = 0; i < pVCollide->solidCount; i++ )
		DestroyCollide( pVCollide->solids[ i ] );

	delete[] pVCollide->solids;
	delete[] pVCollide->pKeyValues;
	V_memset( pVCollide, 0, sizeof( *pVCollide ) );
}

int JoltPhysicsCollision::CollideSize( CPhysCollide *pCollide )
{
	if ( !pCollide )
		return 0;

	JPH::Shape *pShape = pCollide->ToShape();
	JoltEnsureSourceQueryData( pShape );
	VJoltCollideWriteStream stream( nullptr );
	const ivp_compat::collideheader_t header{ ivp_compat::JOLT_COLLISION_ID, 4, 0 };
	stream.Write( header );
	WriteCollideMetadata( stream, *pCollide->GetUserData() );
	JPH::Shape::ShapeToIDMap shapeMap;
	JPH::Shape::MaterialToIDMap materialMap;
	pShape->SaveWithChildren( stream, shapeMap, materialMap );
	return stream.GetSize() > size_t( INT_MAX ) ? 0 : int( stream.GetSize() );
}

int JoltPhysicsCollision::CollideWrite( char *pDest, CPhysCollide *pCollide, bool bSwap /*= false*/ )
{
	if ( bSwap )
	{
		Log_Error( LOG_VJolt, "If you got here. Tell me what you did!\n" );
		return 0;
	}

	if ( !pCollide )
		return 0;

	JPH::Shape *pShape = pCollide->ToShape();
	JoltEnsureSourceQueryData( pShape );
	VJoltCollideWriteStream stream( pDest );
	const ivp_compat::collideheader_t header{ ivp_compat::JOLT_COLLISION_ID, 4, 0 };
	stream.Write( header );
	WriteCollideMetadata( stream, *pCollide->GetUserData() );
	JPH::Shape::ShapeToIDMap shapeMap;
	JPH::Shape::MaterialToIDMap materialMap;
	pShape->SaveWithChildren( stream, shapeMap, materialMap );
	return stream.GetSize() > size_t( INT_MAX ) ? 0 : int( stream.GetSize() );
}

CPhysCollide *JoltPhysicsCollision::UnserializeCollide( char *pBuffer, int size, int index )
{
	return ivp_compat::Deserialize( pBuffer, size, index );
}

//-------------------------------------------------------------------------------------------------

IVPhysicsKeyParser *JoltPhysicsCollision::VPhysicsKeyParserCreate( const char *pKeyData )
{
	return CreateVPhysicsKeyParser( pKeyData, false );
}

IVPhysicsKeyParser *JoltPhysicsCollision::VPhysicsKeyParserCreate( vcollide_t *pVCollide )
{
	return CreateVPhysicsKeyParser( pVCollide->pKeyValues, pVCollide->isPacked );
}

void JoltPhysicsCollision::VPhysicsKeyParserDestroy( IVPhysicsKeyParser *pParser )
{
	DestroyVPhysicsKeyParser( pParser );
}

//-------------------------------------------------------------------------------------------------

int JoltPhysicsCollision::CreateDebugMesh( CPhysCollide const *pCollisionModel, Vector **outVerts )
{
	if ( !outVerts )
		return 0;
	*outVerts = nullptr;
	if ( !pCollisionModel )
		return 0;
	const JPH::Shape *pShape = pCollisionModel->ToShape();

	JPH::AllHitCollisionCollector<JPH::TransformedShapeCollector> collector;
	JPH::ShapeFilter filter;
	pShape->CollectTransformedShapes( JPH::AABox::sBiggest(), pShape->GetCenterOfMass() * JoltToSource::Factor, JPH::Quat::sIdentity(), JPH::Vec3::sReplicate( JoltToSource::Factor ), JPH::SubShapeIDCreator(), collector, filter );

	// GetStats counts stored triangles, not tessellated primitives such as spheres.
	// Collect into a bounded scratch buffer before appending to the output.
	constexpr int batchSize = 128;
	JPH::Float3 batch[batchSize * 3];
	std::vector<Vector> vertices;
	for ( auto &shape : collector.mHits )
	{
		JPH::Shape::GetTrianglesContext ctx;
		shape.GetTrianglesStart( ctx, JPH::AABox::sBiggest(), JPH::Vec3::sZero() );
		for ( ;; )
		{
			const int count = shape.GetTrianglesNext( ctx, batchSize, batch );
			if ( count == 0 )
				break;
			if ( vertices.size() > size_t( INT_MAX ) - size_t( count ) * 3 )
				return 0;
			for ( int i = 0; i < count * 3; i += 3 )
				for ( int corner : { 2, 1, 0 } ) // VPhysics winding.
				{
					const JPH::Float3 &vertex = batch[i + corner];
					vertices.emplace_back( vertex.x, vertex.y, vertex.z );
				}
		}
	}
	if ( vertices.empty() )
		return 0;
	Vector *pVerts = new Vector[vertices.size()];
	std::copy( vertices.begin(), vertices.end(), pVerts );
	*outVerts = pVerts;
	return int( vertices.size() );
}

void JoltPhysicsCollision::DestroyDebugMesh( int vertCount, Vector *outVerts )
{
	delete[] outVerts;
}

//-------------------------------------------------------------------------------------------------

ICollisionQuery *JoltPhysicsCollision::CreateQueryModel( CPhysCollide *pCollide )
{
	return new JoltCollisionQuery( pCollide->ToShape() );
}

void JoltPhysicsCollision::DestroyQueryModel( ICollisionQuery *pQuery )
{
	delete pQuery;
}

//-------------------------------------------------------------------------------------------------

IPhysicsCollision *JoltPhysicsCollision::ThreadContextCreate()
{
	return this;
}

void JoltPhysicsCollision::ThreadContextDestroy( IPhysicsCollision *pThreadContex )
{
	// Does nothing in VPhysics.
}

//-------------------------------------------------------------------------------------------------

CPhysCollide *JoltPhysicsCollision::CreateVirtualMesh( const virtualmeshparams_t &params )
{
	IVirtualMeshEvent *event = params.pMeshEventHandler;

	virtualmeshlist_t meshList;

	event->GetVirtualMesh( params.userData, &meshList );

	JPH::VertexList vertexList;
	vertexList.resize( meshList.vertexCount );
	for ( int i = 0; i < meshList.vertexCount; ++i )
		vertexList[i] = SourceToJolt::DistanceFloat3( meshList.pVerts[i] );

	JPH::IndexedTriangleList indexedTriangleList;
	indexedTriangleList.resize( meshList.triangleCount * 2 );

	for ( int i = 0; i < meshList.triangleCount; ++i )
	{
		// Add both windings to make this two-faced.
		// Probably doesn't matter too much but matches what used to happen.
		indexedTriangleList[i*2+0].mIdx[0] = meshList.indices[i*3+0];
		indexedTriangleList[i*2+0].mIdx[1] = meshList.indices[i*3+1];
		indexedTriangleList[i*2+0].mIdx[2] = meshList.indices[i*3+2];

		indexedTriangleList[i*2+1].mIdx[2] = meshList.indices[i*3+0];
		indexedTriangleList[i*2+1].mIdx[1] = meshList.indices[i*3+1];
		indexedTriangleList[i*2+1].mIdx[0] = meshList.indices[i*3+2];
	}

	JPH::MeshShapeSettings settings( vertexList, indexedTriangleList );
	CPhysCollide *pCollide = ShapeSettingsToPhysCollide( settings );
	if ( pCollide && pCollide->GetUserData() )
		pCollide->GetUserData()->isVirtualMesh = true;
	return pCollide;
}

bool JoltPhysicsCollision::SupportsVirtualMesh()
{
	return true;
}

//-------------------------------------------------------------------------------------------------

bool JoltPhysicsCollision::GetBBoxCacheSize( int *pCachedSize, int *pCachedCount )
{
	// Josh: We don't use a bbox cache as we have box shapes directly,
	// and this is only used for debug stats.
	*pCachedSize = 0;
	*pCachedCount = 0;
	return true;
}

//-------------------------------------------------------------------------------------------------

CPolyhedron *JoltPhysicsCollision::PolyhedronFromConvex( CPhysConvex * const pConvex, bool bUseTempPolyhedron )
{
	if ( !pConvex )
		return nullptr;

	JPH::Shape *pShape = pConvex->ToConvexShape();
	JoltEnsureSourceQueryData( pShape );
	const JoltCollideUserData *pData = pConvex->GetUserData();
	if ( !pData || pData->sourceQueryConvexes.size() != 1 )
		return nullptr;

	const std::vector< JoltSourceQueryTriangle > &sourceTriangles =
		pData->sourceQueryConvexes.front().triangles;
	if ( sourceTriangles.empty() || sourceTriangles.size() > USHRT_MAX )
		return nullptr;

	struct IndexedTriangle
	{
		unsigned short vertices[ 3 ];
	};
	struct IndexedLine
	{
		unsigned short low;
		unsigned short high;

		bool operator<( const IndexedLine &other ) const
		{
			return low != other.low ? low < other.low : high < other.high;
		}
		bool operator==( const IndexedLine &other ) const
		{
			return low == other.low && high == other.high;
		}
	};

	std::vector< Vector > vertices;
	std::vector< IndexedTriangle > triangles;
	vertices.reserve( sourceTriangles.size() );
	triangles.reserve( sourceTriangles.size() );
	for ( const JoltSourceQueryTriangle &sourceTriangle : sourceTriangles )
	{
		IndexedTriangle &triangle = triangles.emplace_back();
		for ( int vertexIndex = 0; vertexIndex < 3; ++vertexIndex )
		{
			const Vector &sourceVertex = sourceTriangle.vertices[ vertexIndex ];
			auto found = std::find( vertices.begin(), vertices.end(), sourceVertex );
			if ( found == vertices.end() )
			{
				if ( vertices.size() == USHRT_MAX )
					return nullptr;
				triangle.vertices[ vertexIndex ] = static_cast< unsigned short >( vertices.size() );
				vertices.push_back( sourceVertex );
			}
			else
			{
				triangle.vertices[ vertexIndex ] = static_cast< unsigned short >(
					found - vertices.begin() );
			}
		}
	}

	std::vector< IndexedLine > lines;
	lines.reserve( triangles.size() * 3 );
	for ( const IndexedTriangle &triangle : triangles )
	{
		for ( int edge = 0; edge < 3; ++edge )
		{
			const unsigned short first = triangle.vertices[ edge ];
			const unsigned short second = triangle.vertices[ (edge + 1) % 3 ];
			if ( first == second )
				return nullptr;
			lines.push_back( first < second
				? IndexedLine { first, second } : IndexedLine { second, first } );
		}
	}
	std::sort( lines.begin(), lines.end() );
	for ( size_t first = 0; first < lines.size(); )
	{
		size_t end = first + 1;
		while ( end < lines.size() && lines[ end ] == lines[ first ] )
			++end;
		if ( end - first != 2 )
			return nullptr;
		lines[ first / 2 ] = lines[ first ];
		first = end;
	}
	lines.resize( lines.size() / 2 );

	// A Source CPhysConvex is a closed triangulated ledge: every line is shared
	// by exactly two triangles, so lineCount * 2 == polygonCount * 3. Refuse a
	// malformed/open query mesh instead of allocating the IVP-sized arrays and
	// then writing past them like the original implementation would.
	if ( vertices.empty() || lines.empty() || lines.size() > USHRT_MAX )
		return nullptr;

	const unsigned short vertexCount = static_cast< unsigned short >( vertices.size() );
	const unsigned short lineCount = static_cast< unsigned short >( lines.size() );
	const unsigned short polygonCount = static_cast< unsigned short >( triangles.size() );
	const size_t indexCountValue = triangles.size() * 3;
	if ( indexCountValue > USHRT_MAX )
		return nullptr;
	const unsigned short indexCount = static_cast< unsigned short >( indexCountValue );

	CPolyhedron *pPolyhedron = bUseTempPolyhedron
		? GetTempPolyhedron( vertexCount, lineCount, indexCount, polygonCount )
		: CPolyhedron_AllocByNew::Allocate( vertexCount, lineCount, indexCount, polygonCount );
	if ( !pPolyhedron )
		return nullptr;

	std::copy( vertices.begin(), vertices.end(), pPolyhedron->pVertices );
	for ( unsigned short i = 0; i < lineCount; ++i )
	{
		pPolyhedron->pLines[ i ].iPointIndices[ 0 ] = lines[ i ].low;
		pPolyhedron->pLines[ i ].iPointIndices[ 1 ] = lines[ i ].high;
	}

	for ( unsigned short polygonIndex = 0; polygonIndex < polygonCount; ++polygonIndex )
	{
		const IndexedTriangle &triangle = triangles[ polygonIndex ];
		Polyhedron_IndexedPolygon_t &polygon = pPolyhedron->pPolygons[ polygonIndex ];
		polygon.iFirstIndex = static_cast< unsigned short >( polygonIndex * 3 );
		polygon.iIndexCount = 3;
		const Vector &a = vertices[ triangle.vertices[ 0 ] ];
		const Vector &b = vertices[ triangle.vertices[ 1 ] ];
		const Vector &c = vertices[ triangle.vertices[ 2 ] ];
		polygon.polyNormal = (c - a).Cross( b - a );
		polygon.polyNormal.NormalizeInPlace();

		for ( int edge = 0; edge < 3; ++edge )
		{
			const unsigned short first = triangle.vertices[ edge ];
			const unsigned short second = triangle.vertices[ (edge + 1) % 3 ];
			const IndexedLine key = first < second
				? IndexedLine { first, second } : IndexedLine { second, first };
			const auto line = std::lower_bound( lines.begin(), lines.end(), key );
			VJoltAssert( line != lines.end() && *line == key );
			Polyhedron_IndexedLineReference_t &reference =
				pPolyhedron->pIndices[ polygon.iFirstIndex + edge ];
			reference.iLineIndex = static_cast< unsigned short >( line - lines.begin() );
			// This is Source's exact reverse-line flag calculation.
			reference.iEndPointIndex = first > second ? 0 : 1;
		}
	}

	return pPolyhedron;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsCollision::OutputDebugInfo( const CPhysCollide *pCollide )
{
	if ( !pCollide )
	{
		Msg( "CollisionModel: null\n" );
		return;
	}

	JPH::Shape *pShape = const_cast< JPH::Shape * >( pCollide->ToShape() );
	JoltEnsureSourceQueryData( pShape );
	const JoltCollideUserData *pData = pCollide->GetUserData();
	if ( pData && pData->isVirtualMesh )
	{
		Msg( "Virtual mesh!\n" );
		return;
	}

	const size_t convexCount = pData ? pData->sourceQueryConvexes.size() : 0;
	Msg( "CollisionModel: Compact Surface: %u convex pieces %s outer hull\n",
		unsigned( convexCount ), pData && pData->hasOuterHull ? "with" : "no" );
}

unsigned int JoltPhysicsCollision::ReadStat( int statID )
{
	// Josh:
	// This always returns 0 in VPhysics.
	// It was used by the HL2 Beta physgun at one point for...
	// something...
	return 0;
}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsCollision::CollideGetRadius( const CPhysCollide *pCollide )
{
	return pCollide->ToShape()->GetInnerRadius();
}

//-------------------------------------------------------------------------------------------------

void *JoltPhysicsCollision::VCollideAllocUserData( vcollide_t *pVCollide, size_t userDataSize )
{
#ifdef GAME_ASW_OR_NEWER
	VCollideFreeUserData( pVCollide );

	if ( userDataSize )
		pVCollide->pUserData = malloc( userDataSize );

	return pVCollide->pUserData;
#else
	return nullptr;
#endif
}

void JoltPhysicsCollision::VCollideFreeUserData( vcollide_t *pVCollide )
{
#ifdef GAME_ASW_OR_NEWER
	if ( pVCollide->pUserData )
	{
		free( pVCollide->pUserData );
		pVCollide->pUserData = nullptr;
	}
#endif
}

void JoltPhysicsCollision::VCollideCheck( vcollide_t *pVCollide, const char *pName )
{
	// Josh:
	// A thing to spew warnings about non-optimal solids in IVP.
	// Entirely useless for us.
}

bool JoltPhysicsCollision::TraceBoxAA( const Ray_t &ray, const CPhysCollide *pCollide, trace_t *ptr )
{
	TraceBox( ray, pCollide, vec3_origin, vec3_angle, ptr );
	return true;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsCollision::DuplicateAndScale( vcollide_t *pOut, const vcollide_t *pIn, float flScale )
{
	CPhysCollide **pSolids = new CPhysCollide * [pIn->solidCount];
	for ( unsigned short i = 0; i < pIn->solidCount; i++ )
	{
		JPH::Shape *pShape = pIn->solids[i]->ToShape();
		JoltEnsureSourceQueryData( pShape );
		const JPH::Vec3 scale = JPH::Vec3::sReplicate( flScale );
		JPH::Shape *pScaledShape = nullptr;
		if ( std::isfinite( flScale ) && flScale != 0.0f )
		{
			JPH::Shape::ShapeResult scaled = pShape->ScaleShape( scale );
			if ( scaled.IsValid() )
			{
				// Even a unit-scale copy needs its own metadata and an owning reference.
				if ( scaled.Get().GetPtr() == pShape )
					scaled.Set( new JPH::ScaledShape( pShape, scale ) );
				pScaledShape = ToDanglingRef( scaled.Get() );
			}
		}

		if ( !pScaledShape )
		{
			pSolids[i] = nullptr;
			continue;
		}

		const JoltCollideUserData *pSourceData = pIn->solids[i]->GetUserData();
		auto *pScaledData = pSourceData
			? new JoltCollideUserData( *pSourceData ) : new JoltCollideUserData();
		pScaledData->ownsChildUserData = false;
		pScaledData->sourceTraceRadius *= std::fabs( flScale );
		for ( auto &node : pScaledData->sourceTraceNodes )
		{
			node.center *= flScale;
			node.radius *= std::fabs( flScale );
		}
		// Source rebuilds the scaled collide from geometry. Runtime overrides made
		// through CollideSetMassCenter/CollideSetOrthographicAreas are not copied:
		// the new mass center comes from scaled geometry and drag areas reset.
		pScaledData->orthographicAreas = Vector( 1.0f, 1.0f, 1.0f );
		pScaledData->sourceMassCenter = JPH::Vec3::sZero();
		pScaledData->sourceRotationInertiaPerMass = JPH::Vec3::sZero();
		pScaledData->hasSourceMassProperties = false;
		for ( JPH::Float3 &vertex : pScaledData->sourceBuoyancyTriangles )
		{
			vertex.x *= flScale;
			vertex.y *= flScale;
			vertex.z *= flScale;
		}
		for ( JoltSourceQueryConvex &convex : pScaledData->sourceQueryConvexes )
		{
			for ( Vector &vertex : convex.traceVertices )
				vertex *= flScale;
			for ( JoltSourceQueryTriangle &triangle : convex.triangles )
				for ( Vector &vertex : triangle.vertices )
					vertex *= flScale;
		}
		pScaledShape->SetUserData( reinterpret_cast< uint64 >( pScaledData ) );
		pSolids[i] = CPhysCollide::FromShape( pScaledShape );
	}

	// pKeyValues is consumed as a C string by VPhysicsKeyParserCreate.
	// Keep the terminator when duplicating a collide, just like VCollideLoad does.
	char *pKeyValues = new char[ pIn->descSize + 1 ];
	if ( pIn->descSize > 0 && pIn->pKeyValues )
		V_memcpy( pKeyValues, pIn->pKeyValues, pIn->descSize );
	pKeyValues[ pIn->descSize ] = '\0';

	*pOut = vcollide_t
	{
		.solidCount = pIn->solidCount,
		.isPacked   = pIn->isPacked,
		.descSize   = pIn->descSize,
		.solids     = pSolids,
		.pKeyValues = pKeyValues,
#ifdef GAME_ASW_OR_NEWER
		.pUserData  = nullptr,
#endif
	};
}

//-------------------------------------------------------------------------------------------------

JPH::RefConst<JPH::Shape> CreateCOMOverrideShape( const JPH::Shape* pShape, JPH::Vec3Arg comOverride )
{
	JPH::Vec3 comOffset = comOverride - pShape->GetCenterOfMass();

	if ( comOffset.IsNearZero() )
		return pShape;

	auto *pWrapper = new JPH::OffsetCenterOfMassShape( pShape, comOffset );
	// The collision model owns this metadata. A temporary body-only wrapper merely
	// forwards it and is itself lifetime-managed by JPH::RefConst.
	pWrapper->SetUserData( pShape->GetUserData() );

	return pWrapper;
}
