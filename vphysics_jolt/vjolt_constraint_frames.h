//=================================================================================================
//
// Source VPhysics object-local constraint frames -> Jolt COM-local frames
//
//=================================================================================================

#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Math/Mat44.h>

#include <cmath>

struct JoltFixedConstraintFrames
{
	JPH::Vec3 m_PointReferenceCOM;
	JPH::Vec3 m_AxisXReferenceCOM;
	JPH::Vec3 m_AxisYReferenceCOM;
	JPH::Vec3 m_PointAttachedCOM;
	JPH::Vec3 m_AxisXAttachedCOM;
	JPH::Vec3 m_AxisYAttachedCOM;
};

struct JoltSliderConstraintFrames
{
	JPH::Vec3 m_PointReferenceCOM;
	JPH::Vec3 m_SliderAxisReferenceCOM;
	JPH::Vec3 m_NormalAxisReferenceCOM;
	JPH::Vec3 m_PointAttachedCOM;
	JPH::Vec3 m_SliderAxisAttachedCOM;
	JPH::Vec3 m_NormalAxisAttachedCOM;
};

struct JoltHingeConstraintFrames
{
	JPH::Vec3 m_PointReferenceCOM;
	JPH::Vec3 m_HingeAxisReferenceCOM;
	JPH::Vec3 m_NormalAxisReferenceCOM;
	JPH::Vec3 m_PointAttachedCOM;
	JPH::Vec3 m_HingeAxisAttachedCOM;
	JPH::Vec3 m_NormalAxisAttachedCOM;
};

// Havana does not choose an arbitrary world-space normal for an ordinary hinge. When the
// limited-hinge perpendicular axis is zero, it runs hk_Vector3_Util::perp_vec independently on
// each body's object-local hinge axis, after Source -> IVP coordinate conversion. Converted back
// to Source/Jolt coordinates, its two seeds are +X and -Z (the 0.57 threshold is exact).
inline JPH::Vec3 BuildHavanaHingePerpendicular( JPH::Vec3Arg hingeAxisObject )
{
	const JPH::Vec3 axis = hingeAxisObject.NormalizedOr( JPH::Vec3::sAxisX() );
	const JPH::Vec3 seed = std::fabs( axis.GetX() ) < 0.57f
		? JPH::Vec3::sAxisX()
		: -JPH::Vec3::sAxisZ();
	return seed.Cross( axis ).Normalized();
}

// Source hinge inputs use a world-space point/axis, but Havana immediately stores both in each
// object's local space. The optional limited-hinge perpendicular directions are object-local in
// the actual implementation (despite the stale public-header comment claiming world space).
// Jolt LocalToBodyCOM uses the same local axes, with the local point shifted by the shape COM.
inline JoltHingeConstraintFrames BuildJoltHingeConstraintFrames(
	JPH::Mat44Arg referenceObjectToWorld,
	JPH::Mat44Arg attachedObjectToWorld,
	JPH::Vec3Arg worldPoint,
	JPH::Vec3Arg worldHingeAxis,
	JPH::Vec3Arg referencePerpAxisObject,
	JPH::Vec3Arg attachedPerpAxisObject,
	JPH::Vec3Arg referenceCOM,
	JPH::Vec3Arg attachedCOM )
{
	const JPH::Mat44 worldToReference = referenceObjectToWorld.InversedRotationTranslation();
	const JPH::Mat44 worldToAttached = attachedObjectToWorld.InversedRotationTranslation();
	const JPH::Vec3 hingeAxisWorld = worldHingeAxis.NormalizedOr( JPH::Vec3::sAxisX() );
	const JPH::Vec3 hingeAxisReference = worldToReference.Multiply3x3( hingeAxisWorld ).Normalized();
	const JPH::Vec3 hingeAxisAttached = worldToAttached.Multiply3x3( hingeAxisWorld ).Normalized();

	const JPH::Vec3 normalAxisReference = referencePerpAxisObject.LengthSq() != 0.0f
		? referencePerpAxisObject.Normalized()
		: BuildHavanaHingePerpendicular( hingeAxisReference );
	const JPH::Vec3 normalAxisAttached = attachedPerpAxisObject.LengthSq() != 0.0f
		? attachedPerpAxisObject.Normalized()
		: BuildHavanaHingePerpendicular( hingeAxisAttached );

	return
	{
		JPH::Vec3( worldToReference * worldPoint ) - referenceCOM,
		hingeAxisReference,
		normalAxisReference,
		JPH::Vec3( worldToAttached * worldPoint ) - attachedCOM,
		hingeAxisAttached,
		normalAxisAttached,
	};
}

// Source defines attachedRefXform as:
//
//     p_reference_object = attachedRefXform * p_attached_object
//
// Havana does not choose an arbitrary coincident weld frame. fixed_constraint.cpp evaluates
//
//     target attached object transform = reference object transform * attachedRefXform
//
// and compares that directly with the attached object's actual transform. Its linear Jacobian is
// also built at the target attached-object origin. Jolt stores body transforms at the COM, so the
// exact equivalent frames are attachedRefXform in the reference object's space and identity in the
// attached object's space, with each translation shifted into that body's COM-local space.
inline JoltFixedConstraintFrames BuildJoltFixedConstraintFrames(
	JPH::Mat44Arg attachedToReference,
	JPH::Vec3Arg referenceCOM,
	JPH::Vec3Arg attachedCOM )
{
	return
	{
		attachedToReference.GetTranslation() - referenceCOM,
		attachedToReference.GetAxisX(),
		attachedToReference.GetAxisY(),
		-attachedCOM,
		JPH::Vec3::sAxisX(),
		JPH::Vec3::sAxisY(),
	};
}

// A Source slider uses the same attached-object -> reference-object transform as a fixed
// constraint, but permits translation along slideAxisRef. Its zero coordinate is the requested
// attachedRefXform pose, not necessarily the bodies' live pose when the constraint is created.
// Use the reference object origin as that zero frame and express the same frame in attached space.
inline JoltSliderConstraintFrames BuildJoltSliderConstraintFrames(
	JPH::Mat44Arg attachedToReference,
	JPH::Vec3Arg slideAxisReference,
	JPH::Vec3Arg referenceCOM,
	JPH::Vec3Arg attachedCOM )
{
	const JPH::Mat44 referenceToAttached = attachedToReference.InversedRotationTranslation();
	const JPH::Vec3 sliderAxis = slideAxisReference.NormalizedOr( JPH::Vec3::sAxisX() );
	const JPH::Vec3 normalAxis = sliderAxis.GetNormalizedPerpendicular();

	return
	{
		-referenceCOM,
		sliderAxis,
		normalAxis,
		referenceToAttached.GetTranslation() - attachedCOM,
		referenceToAttached.Multiply3x3( sliderAxis ),
		referenceToAttached.Multiply3x3( normalAxis ),
	};
}
