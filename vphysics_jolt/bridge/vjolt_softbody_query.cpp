#include "cbase.h"
#include "bridge/vjolt_softbody_mesh.h"
#include "bridge/vjolt_softbody_query.h"

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/SoftBody/SoftBodyShape.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>

#include <algorithm>
#include <vector>

namespace
{
using namespace VJoltSoftBody;

bool ValidRay( const VJoltSoftBodyRay &ray )
{
	if ( ray.size != sizeof( ray ) || !IsValidVector( ray.start ) || !IsValidVector( ray.delta, 2 * kMaxCoordinate )
		|| !IsValidVector( ray.extents ) || ray.extents.x < 0 || ray.extents.y < 0 || ray.extents.z < 0 )
	{
		return false;
	}
	JPH::Vec3 axes[3];
	for ( uint32_t index = 0; index < 3; ++index )
	{
		const auto &axis = ray.axes[index];
		if ( !IsValidVector( axis, 1.001f ) )
		{
			return false;
		}
		axes[index] = JPH::Vec3( axis.x, axis.y, axis.z );
		if ( std::abs( axes[index].LengthSq() - 1 ) > 0.0001f )
		{
			return false;
		}
	}
	return std::abs( axes[0].Dot( axes[1] ) ) < 0.0001f
		&& std::abs( axes[0].Dot( axes[2] ) ) < 0.0001f
		&& axes[0].Cross( axes[1] ).Dot( axes[2] ) > 0.9999f;
}

bool Contains( const JPH::TransformedShape &shape, JPH::RVec3Arg point )
{
	JPH::AnyHitCollisionCollector<JPH::CollidePointCollector> collector;
	shape.CollidePoint( point, collector );
	return collector.HadHit();
}

// Each triangle overlaps a translating box over one interval. Union those
// intervals and classify the remaining gaps against the closed volume. This
// proves continuous allsolid even on folded meshes; endpoints alone cannot.
bool ContinuousOverlap( const JPH::Body &body, const JPH::TransformedShape &shape,
	const VJoltSoftBodyRay &ray, bool closed, JPH::RVec3Arg start, JPH::Vec3Arg delta )
{
	JPH::Vec3 axes[3];
	for ( uint32_t index = 0; index < 3; ++index )
	{
		const auto &axis = ray.axes[index];
		axes[index] = JPH::Vec3( axis.x, axis.y, axis.z );
	}
	const auto inBox = [&]( JPH::Vec3Arg value )
	{
		return JPH::Vec3( axes[0].Dot( value ), axes[1].Dot( value ), axes[2].Dot( value ) );
	};
	const auto direction = inBox( delta );
	const auto extents = JPH::Vec3::sMax( ToJolt( ray.extents ), JPH::Vec3::sReplicate( 1e-7f ) );
	const auto transform = body.GetCenterOfMassTransform();
	const auto &motion = *static_cast<const JPH::SoftBodyMotionProperties *>( body.GetMotionProperties() );
	std::vector<std::pair<float, float>> intervals;
	for ( const auto &face : motion.GetFaces() )
	{
		JPH::Vec3 points[3];
		for ( uint32_t corner = 0; corner < 3; ++corner )
		{
			points[corner] = inBox( JPH::Vec3( transform * motion.GetVertex( face.mVertex[corner] ).mPosition - start ) );
		}
		float enter = 0, leave = 1;
		const auto project = [&]( JPH::Vec3Arg axis )
		{
			const float length = axis.Length();
			if ( length < 1e-12f ) return true;
			const float a = points[0].Dot( axis ), b = points[1].Dot( axis ), c = points[2].Dot( axis );
			const float radius = extents.Dot( axis.Abs() );
			const float low = std::min( { a, b, c } ) - radius, high = std::max( { a, b, c } ) + radius;
			const float speed = direction.Dot( axis );
			if ( std::abs( speed ) < 1e-12f * length ) return low <= 0 && high >= 0;
			float first = low / speed, last = high / speed;
			if ( first > last ) std::swap( first, last );
			enter = std::max( enter, first );
			leave = std::min( leave, last );
			return enter <= leave;
		};
		if ( !project( JPH::Vec3::sAxisX() ) || !project( JPH::Vec3::sAxisY() ) || !project( JPH::Vec3::sAxisZ() )
			|| !project( ( points[1] - points[0] ).Cross( points[2] - points[0] ) ) ) continue;
		bool overlaps = true;
		for ( uint32_t edge = 0; edge < 3 && overlaps; ++edge )
		{
			const auto value = points[( edge + 1 ) % 3] - points[edge];
			overlaps = project( value.Cross( JPH::Vec3::sAxisX() ) )
				&& project( value.Cross( JPH::Vec3::sAxisY() ) ) && project( value.Cross( JPH::Vec3::sAxisZ() ) );
		}
		if ( overlaps ) intervals.emplace_back( enter, leave );
	}
	std::sort( intervals.begin(), intervals.end() );
	float reach = 0;
	for ( size_t index = 0; index <= intervals.size(); ++index )
	{
		const float next = index < intervals.size() ? intervals[index].first : 1;
		if ( next > reach + 1e-7f && ( !closed || !Contains( shape, start + delta * ( 0.5f * ( reach + next ) ) ) ) )
		{
			return false;
		}
		reach = std::max( reach, index < intervals.size() ? intervals[index].second : 1.0f );
		if ( reach >= 1 ) return true;
	}
	return true;
}

void FaceIdentity( const JPH::Body &body, const JPH::SubShapeID &subShape,
	JPH::RVec3Arg contact, VJoltSoftBodyHit &hit )
{
	const auto &shape = *static_cast<const JPH::SoftBodyShape *>( body.GetShape() );
	const auto &motion = *static_cast<const JPH::SoftBodyMotionProperties *>( body.GetMotionProperties() );
	hit.triangle = shape.GetFaceIndex( subShape );
	const auto &face = motion.GetFace( hit.triangle );
	const JPH::Vec3 local = body.GetInverseCenterOfMassTransform() * contact;
	float closest = FLT_MAX;
	for ( uint32_t vertex : face.mVertex )
	{
		const float distance = ( motion.GetVertex( vertex ).mPosition - local ).LengthSq();
		if ( distance < closest )
		{
			closest = distance;
			hit.vertex = vertex;
		}
	}
}
} // namespace

bool VJoltTraceSoftBody( const JPH::Body &body, bool closed,
	const VJoltSoftBodyRay &ray, VJoltSoftBodyHit &hit )
{
	using namespace VJoltSoftBody;
	if ( !ValidRay( ray ) || hit.size != sizeof( hit ) )
	{
		return false;
	}
	hit = {};
	const auto shape = body.GetTransformedShape();
	const auto start = ToJolt( ray.start );
	const auto delta = ToJolt( ray.delta );
	const bool startsInside = closed && Contains( shape, start );
	const bool isRay = ray.extents.x == 0 && ray.extents.y == 0 && ray.extents.z == 0;
	JPH::SubShapeID subShape;
	JPH::Vec3 normal = JPH::Vec3::sZero();
	JPH::RVec3 contact = start;
	bool hasFace = false;
	if ( isRay )
	{
		JPH::RayCastSettings settings;
		settings.mBackFaceModeTriangles = startsInside ? JPH::EBackFaceMode::CollideWithBackFaces
			: JPH::EBackFaceMode::IgnoreBackFaces;
		JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> collector;
		shape.CastRay( JPH::RRayCast( start, delta ), settings, collector );
		hasFace = collector.HadHit() && collector.mHit.mFraction < 1;
		if ( hasFace )
		{
			subShape = collector.mHit.mSubShapeID2;
			contact = start + delta * collector.mHit.mFraction;
			normal = shape.GetWorldSpaceSurfaceNormal( subShape, contact );
			hit.fraction = collector.mHit.mFraction;
			hit.flags = VJoltSoftBodyHitSurface;
		}
		if ( startsInside )
		{
			hit.flags |= VJoltSoftBodyStartSolid;
			hit.fractionLeftSolid = hasFace ? hit.fraction : 0;
			hit.fraction = 0;
			if ( !hasFace )
			{
				hit.flags |= VJoltSoftBodyAllSolid;
			}
		}
	}
	else
	{
		JPH::BoxShape box( JPH::Vec3::sMax( ToJolt( ray.extents ), JPH::Vec3::sReplicate( 1e-7f ) ), 0 );
		JPH::Mat44 pose;
		for ( uint32_t index = 0; index < 3; ++index )
		{
			const auto &axis = ray.axes[index];
			pose.SetColumn4( index, JPH::Vec4( axis.x, axis.y, axis.z, 0 ) );
		}
		pose.SetColumn4( 3, JPH::Vec4( start, 1 ) );
		JPH::ShapeCastSettings settings;
		settings.mBackFaceModeTriangles = JPH::EBackFaceMode::IgnoreBackFaces;
		settings.mCollisionTolerance = 1e-6f;
		settings.mPenetrationTolerance = 1e-6f;
		settings.mReturnDeepestPoint = true;
		JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector> collector;
		shape.CastShape( JPH::RShapeCast( &box, JPH::Vec3::sReplicate( 1 ), JPH::RMat44( pose ), delta ),
			settings, JPH::RVec3::sZero(), collector );
		hasFace = collector.HadHit() && collector.mHit.mFraction < 1;
		if ( hasFace )
		{
			const auto &result = collector.mHit;
			subShape = result.mSubShapeID2;
			contact = result.mContactPointOn2;
			normal = -result.mPenetrationAxis.NormalizedOr( JPH::Vec3::sAxisZ() );
			hit.fraction = result.mFraction;
			hit.flags = VJoltSoftBodyHitSurface;
			if ( result.mFraction == 0 && result.mPenetrationDepth > 1e-6f )
			{
				hit.flags |= VJoltSoftBodyStartSolid;
			}
		}
		if ( startsInside )
		{
			hit.flags |= VJoltSoftBodyStartSolid;
			hit.fraction = 0;
		}
		if ( hit.flags & VJoltSoftBodyStartSolid )
		{
			if ( delta.IsNearZero() || ContinuousOverlap( body, shape, ray, closed, start, delta ) )
			{
				hit.flags |= VJoltSoftBodyAllSolid;
			}
		}
		else if ( hasFace )
		{
			// Source's hull separation tolerance, in Source units.
			const float approach = delta.Dot( normal );
			if ( approach < 0 )
			{
				hit.fraction = std::max( 0.0f, hit.fraction + SourceToJolt::Distance( 0.03125f ) / approach );
			}
		}
	}
	if ( hasFace )
	{
		if ( normal.Dot( delta ) > 0 )
		{
			normal = -normal;
		}
		const auto sourceNormal = JoltToSource::Unitless( normal );
		hit.normal = { sourceNormal.x, sourceNormal.y, sourceNormal.z };
		hit.planeDistance = JoltToSource::Distance( JPH::Vec3( contact ) ).Dot( sourceNormal );
		FaceIdentity( body, subShape, contact, hit );
	}
	hit.position = ToSource( start + delta * hit.fraction );
	return true;
}
