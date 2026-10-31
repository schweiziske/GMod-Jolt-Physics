#pragma once

namespace vjolt_source_cone_overlap
{
	class BoxSupport
	{
	public:
		BoxSupport( const Vector &mins, const Vector &maxs )
			: m_Mins( mins ), m_Maxs( maxs )
		{
		}

		Vector GetSupport( const Vector &direction ) const
		{
			return Vector(
				std::signbit( direction.x ) ? m_Mins.x : m_Maxs.x,
				std::signbit( direction.y ) ? m_Mins.y : m_Maxs.y,
				std::signbit( direction.z ) ? m_Mins.z : m_Maxs.z );
		}

	private:
		Vector m_Mins;
		Vector m_Maxs;
	};

	class ConeSupport
	{
	public:
		explicit ConeSupport( const truncatedcone_t &cone )
			: m_Normal( cone.normal )
		{
			float cosTheta = 0.0f;
			SinCos( DEG2RAD( cone.theta ), &m_SinTheta, &cosTheta );
			m_Radius = cone.h * m_SinTheta / cosTheta;
			m_CenterBase = cone.h * m_Normal;
		}

		Vector GetSupport( const Vector &direction ) const
		{
			Vector unitDirection = direction;
			VectorNormalize( unitDirection );
			const float dot = DotProduct( unitDirection, m_Normal );
			if ( dot > -m_SinTheta )
			{
				unitDirection -= m_Normal * dot;
				const float radialLength = VectorNormalize( unitDirection );
				if ( radialLength > 1.0e-4f )
					return m_CenterBase + unitDirection * m_Radius;
				return m_CenterBase;
			}
			return vec3_origin;
		}

	private:
		Vector m_Normal;
		Vector m_CenterBase;
		float m_Radius = 0.0f;
		float m_SinTheta = 0.0f;
	};

	struct Simplex
	{
		bool TriangleSimplex( const Vector &newPoint, int outIndex,
			const Vector &faceNormal, Vector *pOut )
		{
			vertCount = 3;
			verts[ outIndex ] = newPoint;
			*pOut = -faceNormal;
			return false;
		}

		bool EdgeSimplex( const Vector &newPoint, int outIndex,
			const Vector &edge, Vector *pOut )
		{
			vertCount = 2;
			verts[ outIndex ] = newPoint;
			Vector cross;
			CrossProduct( edge, newPoint, cross );
			CrossProduct( cross, edge, *pOut );
			return false;
		}

		bool PointSimplex( const Vector &newPoint, Vector *pOut )
		{
			vertCount = 1;
			verts[ 0 ] = newPoint;
			*pOut = newPoint;
			return false;
		}

		bool SolveVoronoiRegion2( const Vector &newPoint, Vector *pOut )
		{
			const Vector AB = verts[ 0 ] - newPoint;
			if ( DotProduct( AB, newPoint ) < 0.0f )
				return EdgeSimplex( newPoint, 1, AB, pOut );
			return PointSimplex( newPoint, pOut );
		}

		bool SolveVoronoiRegion3( const Vector &newPoint, Vector *pOut )
		{
			const Vector AB = verts[ 0 ] - newPoint;
			const Vector AC = verts[ 1 ] - newPoint;
			Vector ABC;
			CrossProduct( AB, AC, ABC );
			Vector ABCxAC;
			CrossProduct( ABC, AC, ABCxAC );
			float d = DotProduct( ABCxAC, newPoint );
			if ( d < 0.0f )
			{
				d = DotProduct( AC, newPoint );
				if ( d < 0.0f )
					return EdgeSimplex( newPoint, 0, AC, pOut );
			}
			else
			{
				Vector ABxABC;
				CrossProduct( AB, ABC, ABxABC );
				d = DotProduct( ABxABC, newPoint );
				if ( d > 0.0f )
				{
					vertCount = 3;
					d = DotProduct( ABC, newPoint );
					if ( d < 0.0f )
					{
						verts[ 2 ] = newPoint;
						*pOut = -ABC;
						return false;
					}
					verts[ 2 ] = verts[ 1 ];
					verts[ 1 ] = newPoint;
					*pOut = ABC;
					return false;
				}
			}
			d = DotProduct( AB, newPoint );
			if ( d < 0.0f )
				return EdgeSimplex( newPoint, 1, AB, pOut );
			return PointSimplex( newPoint, pOut );
		}

		bool SolveVoronoiRegion4( const Vector &newPoint, Vector *pOut )
		{
			const Vector AB = verts[ 0 ] - newPoint;
			const Vector AC = verts[ 1 ] - newPoint;
			const Vector AD = verts[ 2 ] - newPoint;

			Vector ABC, ACD, ADB;
			CrossProduct( AB, AC, ABC );
			CrossProduct( AC, AD, ACD );
			CrossProduct( AD, AB, ADB );

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
			float d = DotProduct( ABC, newPoint );
			if ( d < 0.0f ) faceFlags |= 0x1;
			d = DotProduct( ACD, newPoint );
			if ( d < 0.0f ) faceFlags |= 0x2;
			d = DotProduct( ADB, newPoint );
			if ( d < 0.0f ) faceFlags |= 0x4;

			switch ( faceFlags )
			{
			case 0:
				verts[ 3 ] = newPoint;
				vertCount = 4;
				return true;
			case 1:
				return SolveVoronoiRegion3( newPoint, pOut );
			case 2:
				verts[ 0 ] = verts[ 2 ];
				return SolveVoronoiRegion3( newPoint, pOut );
			case 4:
				verts[ 1 ] = verts[ 2 ];
				return SolveVoronoiRegion3( newPoint, pOut );
			case 3:
				d = DotProduct( ABCxAC, newPoint );
				if ( d < 0.0f )
				{
					d = DotProduct( ACxACD, newPoint );
					if ( d < 0.0f )
					{
						d = DotProduct( AC, newPoint );
						if ( d < 0.0f )
							return EdgeSimplex( newPoint, 0, AC, pOut );
						return PointSimplex( newPoint, pOut );
					}
					d = DotProduct( ACDxAD, newPoint );
					if ( d < 0.0f )
					{
						verts[ 0 ] = verts[ 2 ];
						return EdgeSimplex( newPoint, 1, AD, pOut );
					}
					return TriangleSimplex( newPoint, 0, ACD, pOut );
				}
				d = DotProduct( ABxABC, newPoint );
				if ( d < 0.0f )
				{
					d = DotProduct( AB, newPoint );
					if ( d < 0.0f )
						return EdgeSimplex( newPoint, 1, AB, pOut );
					return PointSimplex( newPoint, pOut );
				}
				return TriangleSimplex( newPoint, 2, ABC, pOut );
			case 5:
				d = DotProduct( ADBxAB, newPoint );
				if ( d < 0.0f )
				{
					d = DotProduct( ABxABC, newPoint );
					if ( d < 0.0f )
					{
						d = DotProduct( AB, newPoint );
						if ( d < 0.0f )
							return EdgeSimplex( newPoint, 1, AB, pOut );
						return PointSimplex( newPoint, pOut );
					}
					d = DotProduct( ABCxAC, newPoint );
					if ( d < 0.0f )
						return EdgeSimplex( newPoint, 0, AC, pOut );
					return TriangleSimplex( newPoint, 2, ABC, pOut );
				}
				d = DotProduct( ADxADB, newPoint );
				if ( d < 0.0f )
				{
					d = DotProduct( AD, newPoint );
					if ( d < 0.0f )
					{
						verts[ 0 ] = verts[ 2 ];
						return EdgeSimplex( newPoint, 1, AD, pOut );
					}
					return PointSimplex( newPoint, pOut );
				}
				return TriangleSimplex( newPoint, 1, ADB, pOut );
			case 6:
				d = DotProduct( ACDxAD, newPoint );
				if ( d < 0.0f )
				{
					d = DotProduct( ADxADB, newPoint );
					if ( d < 0.0f )
					{
						d = DotProduct( AD, newPoint );
						if ( d < 0.0f )
						{
							verts[ 0 ] = verts[ 2 ];
							return EdgeSimplex( newPoint, 1, AD, pOut );
						}
						return PointSimplex( newPoint, pOut );
					}
					d = DotProduct( ADBxAB, newPoint );
					if ( d < 0.0f )
						return EdgeSimplex( newPoint, 1, AB, pOut );
					return TriangleSimplex( newPoint, 1, ADB, pOut );
				}
				d = DotProduct( ACxACD, newPoint );
				if ( d < 0.0f )
				{
					d = DotProduct( AC, newPoint );
					if ( d < 0.0f )
						return EdgeSimplex( newPoint, 0, AC, pOut );
					return PointSimplex( newPoint, pOut );
				}
				return TriangleSimplex( newPoint, 0, ACD, pOut );
			case 7:
				d = DotProduct( AB, newPoint );
				if ( d < 0.0f )
					return EdgeSimplex( newPoint, 1, AB, pOut );
				d = DotProduct( AC, newPoint );
				if ( d < 0.0f )
					return EdgeSimplex( newPoint, 0, AC, pOut );
				d = DotProduct( AD, newPoint );
				if ( d < 0.0f )
				{
					verts[ 0 ] = verts[ 2 ];
					return EdgeSimplex( newPoint, 1, AD, pOut );
				}
				return PointSimplex( newPoint, pOut );
			}

			verts[ 3 ] = newPoint;
			vertCount = 4;
			return true;
		}

		bool SolveGJKSet( const Vector &newPoint, Vector *pOut )
		{
			switch ( vertCount )
			{
			case 0:
				vertCount = 1;
				verts[ 0 ] = newPoint;
				*pOut = newPoint;
				return false;
			case 1:
				return SolveVoronoiRegion2( newPoint, pOut );
			case 2:
				return SolveVoronoiRegion3( newPoint, pOut );
			case 3:
				return SolveVoronoiRegion4( newPoint, pOut );
			default:
				return true;
			}
		}

		Vector verts[ 4 ];
		int vertCount = 0;
	};

	inline bool IsBoxIntersectingCone( const Vector &boxAbsMins,
		const Vector &boxAbsMaxs, const truncatedcone_t &cone )
	{
		// CTraceAABB's point mode collapses the support map to the ray origin.
		// IsBoxIntersectingCone supplies a zero-length ray at that same origin,
		// so Source's degenerate point result is compiler/direction dependent.
		// Keep the cleared-trace miss semantics instead of reproducing unstable
		// GJK output for a support shape with no position information.
		if ( boxAbsMins == boxAbsMaxs )
			return false;

		// The finite cone becomes exactly its apex. The generic simplex can
		// cycle when the Minkowski support repeats on this zero-volume case.
		if ( cone.h == 0.0f )
		{
			return cone.origin.x > boxAbsMins.x && cone.origin.x < boxAbsMaxs.x
				&& cone.origin.y > boxAbsMins.y && cone.origin.y < boxAbsMaxs.y
				&& cone.origin.z > boxAbsMins.z && cone.origin.z < boxAbsMaxs.z;
		}

		const BoxSupport box( boxAbsMins - cone.origin, boxAbsMaxs - cone.origin );
		const ConeSupport coneSupport( cone );
		Simplex simplex;
		Vector closest = boxAbsMaxs;
		if ( closest == vec3_origin )
			closest.Init( 1.0f, 0.0f, 0.0f );
		Vector direction = -closest;
		VectorNormalize( direction );

		for ( int iteration = 0; iteration < 100; ++iteration )
		{
			const Vector point = box.GetSupport( direction )
				- coneSupport.GetSupport( -direction );
			if ( DotProduct( direction, point ) < 0.0f )
				return false;
			if ( simplex.SolveGJKSet( point, &closest ) )
				return true;
			direction = -closest;
			VectorNormalize( direction );
		}
		return false;
	}
}
