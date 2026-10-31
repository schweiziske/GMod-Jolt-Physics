#pragma once
#ifdef _MSC_VER
#pragma float_control( precise, on, push )
#pragma fp_contract( off )
#endif
#include "vjolt_source_trace_solver.inl"
namespace VJoltSourceTrace
{
static matrix3x4_t Rotation( const QAngle &angles )
{
	const float sy = std::sin( DEG2RAD( angles.y ) ), cy = std::cos( DEG2RAD( angles.y ) );
	const float sp = std::sin( DEG2RAD( angles.x ) ), cp = std::cos( DEG2RAD( angles.x ) );
	const float sr = std::sin( DEG2RAD( angles.z ) ), cr = std::cos( DEG2RAD( angles.z ) );
	matrix3x4_t rotation;
	rotation.Init( Vector( cp * cy, cp * sy, -sp ),
				   Vector( -( cr * sy - sr * sp * cy ), -( -cr * cy - sr * sp * sy ), sr * cp ),
				   Vector( cr * sp * cy + sr * sy, cr * sp * sy - sr * cy, cr * cp ), vec3_origin );
	return rotation;
}
class ModelConvex final : public ITraceObject
{
public:
	CUtlVectorFixedGrowable<Vector, 128> vertices;
	const JoltSourceQueryConvex &source;
	matrix3x4_t rotation;
	static int Previous( int edge )
	{
		return edge / 3 * 3 + ( edge % 3 + 2 ) % 3;
	}
	int Walk( const Vector &direction, int edge ) const
	{
		const auto &edges = source.traceEdges;
		int best = int( edges[ edge ].first ), bestEdge = edge;
		float bestDot = DotProduct( source.traceVertices[ best ], direction );
		CUtlVectorFixedGrowable<unsigned char, 256> visited;
		visited.SetCount( int( source.traceVertices.size() ) );
		memset( visited.Base(), 0, visited.Count() );
		visited[ best ] = 1;
		for ( int i = 0; i < int( edges.size() / 3 ); ++i )
		{
			edge = Previous( edge );
			int stop = int( edges[ edge ].first ), vertex = stop;
			size_t remaining = edges.size();
			do
			{
				if ( !visited[ vertex ] )
				{
					visited[ vertex ] = 1;
					float dot = DotProduct( source.traceVertices[ vertex ], direction );
					if ( dot > bestDot )
					{
						bestDot = dot;
						best = vertex;
						bestEdge = edge;
						break;
					}
				}
				edge = Previous( int( edges[ edge ].second ) );
				vertex = int( edges[ edge ].first );
			} while ( vertex != stop && --remaining );
			if ( vertex != best )
				break;
		}
		return bestEdge;
	}
	ModelConvex( const JoltSourceQueryConvex &convex, const QAngle &angles ) : source( convex )
	{
		rotation = Rotation( angles );
		matrix3x4_t basis;
		for ( int row = 0; row < 3; ++row )
		{
			basis[ row ][ 0 ] = rotation[ row ][ 0 ] * JoltToSource::Factor;
			basis[ row ][ 1 ] = -rotation[ row ][ 2 ] * JoltToSource::Factor;
			basis[ row ][ 2 ] = rotation[ row ][ 1 ] * JoltToSource::Factor;
			basis[ row ][ 3 ] = 0;
		}
		vertices.EnsureCapacity( int( convex.traceVertices.size() ) );
		for ( const auto &point : convex.traceVertices )
		{
			Vector v;
			for ( int row = 0; row < 3; ++row )
				v[ row ] = ( point.x * basis[ row ][ 0 ] + point.y * basis[ row ][ 1 ] ) + point.z * basis[ row ][ 2 ];
			vertices.AddToTail( v );
		}
		// Source pads the final SIMD batch by repeating its last vertex. The
		// repeated indices also participate in the four-lane tie reduction.
		if ( source.traceCachedSupport && vertices.Count() )
			while ( vertices.Count() & 3 )
				vertices.AddToTail( vertices.Tail() );
	}
	int SupportMap( const Vector &d, Vector *out ) const override
	{
		if ( !source.traceCachedSupport )
		{
			const Vector local(
				( d.x * rotation[ 0 ][ 0 ] + ( -d.z ) * ( -rotation[ 2 ][ 0 ] ) ) + d.y * rotation[ 1 ][ 0 ],
				( d.x * ( -rotation[ 0 ][ 2 ] ) + ( -d.z ) * rotation[ 2 ][ 2 ] ) + d.y * ( -rotation[ 1 ][ 2 ] ),
				( d.x * rotation[ 0 ][ 1 ] + ( -d.z ) * ( -rotation[ 2 ][ 1 ] ) ) + d.y * rotation[ 1 ][ 1 ] );
			int octant = ( local.x < 0 ? 1 : 0 ) + ( local.y < 0 ? 2 : 0 ) + ( local.z < 0 ? 4 : 0 );
			int index = int( source.traceEdges[ Walk( local, source.traceStartEdges[ octant ] ) ].first );
			*out = vertices[ index ];
			return index;
		}
		int ids[ 4 ] = {};
		float dots[ 4 ] = { -FLT_MAX, -FLT_MAX, -FLT_MAX, -FLT_MAX };
		for ( int i = 0; i < vertices.Count(); ++i )
		{
			float dot = DotProduct( vertices[ i ], d );
			int lane = i & 3;
			if ( dot > dots[ lane ] )
			{
				dots[ lane ] = dot;
				ids[ lane ] = i;
			}
		}
		int e = dots[ 2 ] > dots[ 0 ] ? 2 : 0, o = dots[ 3 ] > dots[ 1 ] ? 3 : 1;
		int best = ids[ dots[ o ] > dots[ e ] ? o : e ];
		*out = vertices[ best ];
		return best;
	}
	Vector GetVertByIndex( int i ) const override
	{
		return vertices[ i ];
	}
	float Radius() const override
	{
		return 0;
	}
};
static void SweepModel( CTraceSolver &solver, const CPhysCollide *collide, const QAngle &angles, unsigned mask,
						IConvexInfo *info, trace_t *out )
{
	const auto &data = *collide->GetUserData();
	const auto rotation = Rotation( angles );
	auto local = [ & ]( const Vector &v ) {
		Vector p = v * 0.0254f;
		return Vector( ( p.x * rotation[ 0 ][ 0 ] + ( -p.z ) * ( -rotation[ 2 ][ 0 ] ) ) + p.y * rotation[ 1 ][ 0 ],
					   ( p.x * ( -rotation[ 0 ][ 2 ] ) + ( -p.z ) * rotation[ 2 ][ 2 ] ) +
						   p.y * ( -rotation[ 1 ][ 2 ] ),
					   ( p.x * rotation[ 0 ][ 1 ] + ( -p.z ) * ( -rotation[ 2 ][ 1 ] ) ) + p.y * rotation[ 1 ][ 1 ] );
	};
	const Vector start = local( solver.m_ray->m_start ), delta = local( solver.m_ray->m_delta );
	Vector direction = delta;
	const float lengthSquared = DotProduct( direction, direction );
	if ( lengthSquared >= 1e-30f )
		direction *= 1.0f / std::sqrt( lengthSquared );
	Vector center = start + delta * 0.5f;
	auto hits = [ & ]( const JoltSourceTraceNode &node ) {
		const Vector d = node.center - center;
		// Deliberately match Source: tree radii are IVP metres, while the
		// swept-object radius here is Source inches. Correcting these units
		// changes visited leaves and the carried simplex axis at exact ties.
		const float radius = node.radius + solver.m_sweepObjectRadius;
		if ( solver.m_ray->m_length > 0 )
		{
			Vector cross;
			CrossProduct( direction, d, cross );
			return cross.LengthSqr() < radius * radius;
		}
		return d.LengthSqr() < radius * radius;
	};
	auto leaf = [ & ]( int index ) {
		const auto &cv = data.sourceQueryConvexes[ index ];
		auto contents = info ? info->GetContents( cv.gameData ) : CONTENTS_SOLID;
		if ( !( contents & mask ) || cv.traceVertices.empty() )
			return;
		ModelConvex obstacle( cv, angles );
		solver.m_obstacle = &obstacle;
		if ( solver.SweepSingleConvex() && solver.m_traceLength < solver.m_totalTraceLength )
		{
			out->plane.normal = solver.m_trace.plane.normal;
			out->startsolid = solver.m_trace.startsolid;
			out->allsolid = solver.m_trace.allsolid;
			solver.m_totalTraceLength = solver.m_traceLength;
			out->fraction = solver.m_traceLength * solver.m_ray->m_ooBaseLength;
			out->contents = contents;
			float test = ( solver.m_traceLength + 2 * solver.m_epsilon ) * solver.m_ray->m_ooBaseLength;
			if ( test < 1 )
			{
				solver.m_ray->Reset( test );
				center = start + delta * ( 0.5f * test );
			}
		}
	};
	if ( data.sourceTraceNodes.empty() )
		return;
	CUtlVectorFixedGrowable<int, 64> stack;
	if ( !hits( data.sourceTraceNodes[ 0 ] ) )
		return;
	int index = 0;
	for ( ;; )
	{
		const auto &node = data.sourceTraceNodes[ index ];
		if ( node.convex >= 0 )
			leaf( node.convex );
		else if ( node.left >= 0 && node.right >= 0 )
		{
			const auto &left = data.sourceTraceNodes[ node.left ], &right = data.sourceTraceNodes[ node.right ];
			bool l = hits( left ), r = hits( right );
			if ( l && r )
			{
				bool leftFirst = ( start - left.center ).LengthSqr() < ( start - right.center ).LengthSqr();
				stack.AddToTail( leftFirst ? node.right : node.left );
				index = leftFirst ? node.left : node.right;
				continue;
			}
			if ( l || r )
			{
				index = l ? node.left : node.right;
				continue;
			}
		}
		if ( !stack.Count() )
			break;
		index = stack.Tail();
		stack.Remove( stack.Count() - 1 );
	}
}
static void TraceBox( const Ray_t &src, unsigned int mask, IConvexInfo *info, const CPhysCollide *collide,
					  const Vector &origin, const QAngle &angles, trace_t *out )
{
	CM_ClearTrace( out );
	auto *shape = const_cast<JPH::Shape *>( collide->ToShape() );
	JoltEnsureSourceQueryData( shape );
	CTraceAABB box( -src.m_Extents, src.m_Extents, src.m_IsRay );
	CTraceRay ray( src, -origin );
	CTraceSolver solver( out, &box, &ray, nullptr, ray.m_start );
	SweepModel( solver, collide, angles, mask, info, out );
	out->startpos = src.m_Start + src.m_StartOffset;
	VectorMA( out->startpos, out->fraction, src.m_Delta, out->endpos );
	if ( out->DidHit() )
		out->plane.dist = DotProduct( out->endpos, out->plane.normal );
}
static void TraceCollide( const Vector &start, const Vector &end, const CPhysCollide *swept, const QAngle &sweepAngles,
						  const CPhysCollide *collide, const Vector &origin, const QAngle &angles, trace_t *out )
{
	CM_ClearTrace( out );
	JoltEnsureSourceQueryData( const_cast<JPH::Shape *>( swept->ToShape() ) );
	JoltEnsureSourceQueryData( const_cast<JPH::Shape *>( collide->ToShape() ) );
	CTraceRay ray( start - origin, end - origin );
	const auto &sweepData = *swept->GetUserData();
	// Source enumerates left/right leaves, then sweeps them in reverse order.
	CUtlVectorFixedGrowable<int, 64> pending, order;
	if ( !sweepData.sourceTraceNodes.empty() )
		pending.AddToTail( 0 );
	while ( pending.Count() )
	{
		int index = pending.Tail();
		pending.Remove( pending.Count() - 1 );
		const auto &node = sweepData.sourceTraceNodes[ index ];
		if ( node.convex >= 0 )
			order.AddToTail( node.convex );
		else if ( node.left >= 0 && node.right >= 0 )
		{
			pending.AddToTail( node.right );
			pending.AddToTail( node.left );
		}
	}
	for ( int i = order.Count() - 1; i >= 0; --i )
	{
		const auto &convex = sweepData.sourceQueryConvexes[ order[ i ] ];
		if ( convex.traceVertices.empty() )
			continue;
		trace_t tr;
		CM_ClearTrace( &tr );
		ModelConvex box( convex, sweepAngles );
		CTraceSolver solver( &tr, &box, &ray, nullptr, start - origin );
		solver.SetEpsilon( 0.25f );
		solver.m_sweepObjectRadius = swept->GetUserData()->sourceTraceRadius * JoltToSource::Factor;
		SweepModel( solver, collide, angles, MASK_ALL, nullptr, &tr );
		if ( tr.fraction < out->fraction )
			*out = tr;
	}
	out->endpos = start * ( 1 - out->fraction ) + end * out->fraction;
	if ( out->DidHit() )
		out->plane.dist = DotProduct( out->endpos, out->plane.normal );
}
} // namespace VJoltSourceTrace
#ifdef _MSC_VER
#pragma float_control( pop )
#endif
