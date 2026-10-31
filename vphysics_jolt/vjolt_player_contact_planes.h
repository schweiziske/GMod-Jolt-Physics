#pragma once

#include <algorithm>
#include <cfloat>
#include <cmath>
#include "mathlib/vector.h"
#include "tier1/utlvector.h"

static constexpr int kMaxPlayerContactNormals = 8;

class PlayerContactNormalList
{
public:
	void AddNormal( const Vector &normal )
	{
		if ( m_Normals.Count() == kMaxPlayerContactNormals )
			return;

		for ( int i = m_Normals.Count(); --i >= 0; )
		{
			if ( DotProduct( m_Normals[i], normal ) > 0.99f )
				return;
		}
		m_Normals.AddToTail( normal );
	}

	Vector ClampVector( const Vector &velocity, float limitVelocity ) const
	{
		if ( m_Normals.Count() >= 2 )
			return ClampAgainstMultiplePlanes( velocity, limitVelocity );
		if ( m_Normals.Count() == 1 )
		{
			const float projection = DotProduct( velocity, m_Normals[0] );
			if ( projection > limitVelocity )
				return velocity + m_Normals[0] * ( limitVelocity - projection );
		}
		return velocity;
	}

private:
	bool IsFeasible( const Vector &velocity, float limitVelocity ) const
	{
		constexpr float kFeasibilityEpsilon = 1.0e-4f;
		for ( int i = 0; i < m_Normals.Count(); ++i )
		{
			if ( DotProduct( velocity, m_Normals[i] )
				> limitVelocity + kFeasibilityEpsilon )
				return false;
		}
		return true;
	}

	void ConsiderCandidate( const Vector &candidate, const Vector &input,
		float limitVelocity, Vector &best, float &bestDistanceSqr ) const
	{
		if ( !candidate.IsValid() || !IsFeasible( candidate, limitVelocity ) )
			return;
		const float distanceSqr = ( candidate - input ).LengthSqr();
		if ( distanceSqr < bestDistanceSqr )
		{
			best = candidate;
			bestDistanceSqr = distanceSqr;
		}
	}

	Vector ClampAgainstMultiplePlanes( const Vector &velocity,
		float limitVelocity ) const
	{
		if ( IsFeasible( velocity, limitVelocity ) )
			return velocity;

		// Contact pressure can select planes even when the requested velocity is
		// separating. Treating two planes as a crease imposes equality on both and
		// traps the player; contacts only constrain motion into the surfaces. Jolt
		// also publishes more speculative planes than IVP. Project onto the convex set
		// n[i].v <= limit. In 3D the closest point has at most three active planes.
		Vector best = vec3_origin;
		float bestDistanceSqr = FLT_MAX;
		ConsiderCandidate( vec3_origin, velocity, limitVelocity,
			best, bestDistanceSqr );
		constexpr float kLambdaEpsilon = 1.0e-5f;

		for ( int i = 0; i < m_Normals.Count(); ++i )
		{
			const Vector &n0 = m_Normals[i];
			const float lambda0 = DotProduct( velocity, n0 ) - limitVelocity;
			if ( lambda0 >= -kLambdaEpsilon )
				ConsiderCandidate( velocity - n0 * lambda0, velocity,
					limitVelocity, best, bestDistanceSqr );

			for ( int j = i + 1; j < m_Normals.Count(); ++j )
			{
				const Vector &n1 = m_Normals[j];
				const float dot01 = DotProduct( n0, n1 );
				const float determinant = 1.0f - dot01 * dot01;
				if ( determinant > 1.0e-8f )
				{
					const float rhs0 = DotProduct( velocity, n0 ) - limitVelocity;
					const float rhs1 = DotProduct( velocity, n1 ) - limitVelocity;
					const float pairLambda0 = ( rhs0 - dot01 * rhs1 ) / determinant;
					const float pairLambda1 = ( rhs1 - dot01 * rhs0 ) / determinant;
					if ( pairLambda0 >= -kLambdaEpsilon
						&& pairLambda1 >= -kLambdaEpsilon )
					{
						ConsiderCandidate( velocity - n0 * pairLambda0
							- n1 * pairLambda1, velocity, limitVelocity,
							best, bestDistanceSqr );
					}
				}

				for ( int k = j + 1; k < m_Normals.Count(); ++k )
				{
					const Vector &n2 = m_Normals[k];
					float matrix[3][4] = {
						{ 1.0f, dot01, DotProduct( n0, n2 ),
							DotProduct( velocity, n0 ) - limitVelocity },
						{ dot01, 1.0f, DotProduct( n1, n2 ),
							DotProduct( velocity, n1 ) - limitVelocity },
						{ DotProduct( n0, n2 ), DotProduct( n1, n2 ), 1.0f,
							DotProduct( velocity, n2 ) - limitVelocity },
					};

					bool solvable = true;
					for ( int column = 0; column < 3; ++column )
					{
						int pivot = column;
						for ( int row = column + 1; row < 3; ++row )
						{
							if ( std::fabs( matrix[row][column] )
								> std::fabs( matrix[pivot][column] ) )
								pivot = row;
						}
						if ( std::fabs( matrix[pivot][column] ) < 1.0e-8f )
						{
							solvable = false;
							break;
						}
						if ( pivot != column )
						{
							for ( int component = column; component < 4; ++component )
								std::swap( matrix[column][component], matrix[pivot][component] );
						}
						const float inversePivot = 1.0f / matrix[column][column];
						for ( int component = column; component < 4; ++component )
							matrix[column][component] *= inversePivot;
						for ( int row = 0; row < 3; ++row )
						{
							if ( row == column )
								continue;
							const float factor = matrix[row][column];
							for ( int component = column; component < 4; ++component )
								matrix[row][component] -= factor * matrix[column][component];
						}
					}
					if ( solvable )
					{
						const float tripleLambda0 = matrix[0][3];
						const float tripleLambda1 = matrix[1][3];
						const float tripleLambda2 = matrix[2][3];
						if ( tripleLambda0 >= -kLambdaEpsilon
							&& tripleLambda1 >= -kLambdaEpsilon
							&& tripleLambda2 >= -kLambdaEpsilon )
						{
							ConsiderCandidate( velocity - n0 * tripleLambda0
								- n1 * tripleLambda1 - n2 * tripleLambda2,
								velocity, limitVelocity, best, bestDistanceSqr );
						}
					}
				}
			}
		}

		return bestDistanceSqr < FLT_MAX ? best : vec3_origin;
	}

	CUtlVectorFixed< Vector, kMaxPlayerContactNormals > m_Normals;
};
