
#include "cbase.h"

#include "vjolt_layers.h"
#include "vjolt_collide.h"
#include "vjolt_object.h"
#include "vjolt_environment.h"
#include "vjolt_surfaceprops.h"

#include <Jolt/Core/UnorderedSet.h>

#include "vjolt_controller_fluid.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

// Josh: The surfacePlane in fluidparams_t is specified in world-space, but we need to translate
// that to the pFluidObject's local-space so we can handle the fluid object moving
// for eg. func_water_analog.
static cplane_t PlaneToLocalSpace( JoltPhysicsObject* pFluidObject, Vector4D worldSurfacePlane )
{
	const cplane_t worldPlane = { worldSurfacePlane.AsVector3D(), worldSurfacePlane[3] };

	matrix3x4_t objectToWorld;
	pFluidObject->GetPositionMatrix( &objectToWorld );

	cplane_t localPlane;
	MatrixITransformPlane( objectToWorld, worldPlane, localPlane );
	return localPlane;
}

static Vector VelocityToLocalSpace( JoltPhysicsObject *pFluidObject, const Vector &worldVelocity )
{
	matrix3x4_t objectToWorld;
	pFluidObject->GetPositionMatrix( &objectToWorld );

	Vector localVelocity;
	VectorIRotate( worldVelocity, objectToWorld, localVelocity );
	return localVelocity;
}

//-------------------------------------------------------------------------------------------------

JoltPhysicsFluidController::JoltPhysicsFluidController( JPH::PhysicsSystem *pPhysicsSystem, JoltPhysicsObject *pFluidObject, const fluidparams_t *pParams )
	: m_pPhysicsSystem( pPhysicsSystem )
	, m_pFluidObject( pFluidObject )
	, m_Params( *pParams )
	, m_LocalPlane( PlaneToLocalSpace( pFluidObject, pParams->surfacePlane ) )
	, m_LocalCurrentVelocity( VelocityToLocalSpace( pFluidObject, pParams->currentVelocity ) )
	, m_flDensity( pFluidObject->GetMaterialDensity() )
{
	m_pFluidObject->BecomeTrigger();
	m_pFluidObject->SetFluidController( this );
	m_pFluidObject->AddDestroyedListener( this );
}

JoltPhysicsFluidController::~JoltPhysicsFluidController()
{
	ClearCachedObjectsInShape();

	if ( m_pFluidObject )
	{
		m_pFluidObject->RemoveDestroyedListener( this );
		m_pFluidObject->SetFluidController( nullptr );
		m_pFluidObject->RemoveTrigger();
	}
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFluidController::SetGameData( void *pGameData )
{
	m_Params.pGameData = pGameData;
}

void *JoltPhysicsFluidController::GetGameData() const
{
	return m_Params.pGameData;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFluidController::GetSurfacePlane( Vector *pNormal, float *pDist ) const
{
	const cplane_t worldPlane = GetSurfacePlane();

	if ( pNormal )
		*pNormal = worldPlane.normal;

	if ( pDist )
		*pDist = worldPlane.dist;
}

float JoltPhysicsFluidController::GetDensity() const
{
	// Source's ConvertDensityToIVP is an identity conversion.
	return m_flDensity;
}

void JoltPhysicsFluidController::WakeAllSleepingObjects()
{
	for ( JoltPhysicsObject *pObject : m_ObjectsInShape )
		pObject->Wake();
}

int JoltPhysicsFluidController::GetContents() const
{
	return m_Params.contents;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFluidController::OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject )
{
	if ( pObject == m_pFluidObject )
		m_pFluidObject = nullptr;

	Erase( m_ObjectsInShape, pObject );
	m_ObjectStates.erase( pObject );
}

void JoltPhysicsFluidController::RemoveObjectForTransfer( JoltPhysicsObject *pObject )
{
	if ( !pObject || pObject == m_pFluidObject )
		return;

	Erase( m_ObjectsInShape, pObject );
	if ( m_ObjectStates.erase( pObject ) != 0 )
		pObject->RemoveDestroyedListener( this );
}

//-------------------------------------------------------------------------------------------------

static JPH::Vec3 ProjectPoint( const JPH::Plane &plane, JPH::Vec3Arg point )
{
	return point - plane.SignedDistance(point) * plane.GetNormal();
}

static void BuildSourceBuoyancyTriangles(
	const JPH::Shape *pShape, std::vector<JPH::Float3> &triangles )
{
	triangles.clear();
	JPH::Shape::VisitedShapes visitedShapes;
	const JPH::Shape::Stats stats = pShape->GetStatsRecursive( visitedShapes );
	triangles.reserve( size_t( stats.mNumTriangles ) * 3 );

	// Expand compounds and decorators into leaf shapes in the collision model's
	// original object space. Supplying the root COM as the transformed COM keeps
	// object-space origin at zero even when the collision model has a COM offset.
	JPH::AllHitCollisionCollector<JPH::TransformedShapeCollector> collector;
	JPH::ShapeFilter filter;
	pShape->CollectTransformedShapes(
		JPH::AABox::sBiggest(), pShape->GetCenterOfMass(), JPH::Quat::sIdentity(),
		JPH::Vec3::sOne(), JPH::SubShapeIDCreator(), collector, filter );

	static constexpr int kRequestedTriangles = JPH::Shape::cGetTrianglesMinTrianglesRequested;
	for ( const JPH::TransformedShape &shape : collector.mHits )
	{
		JPH::Shape::GetTrianglesContext context;
		shape.GetTrianglesStart( context, JPH::AABox::sBiggest(), JPH::Vec3::sZero() );
		for ( ;; )
		{
			JPH::Float3 vertices[ kRequestedTriangles * 3 ];
			const int count = shape.GetTrianglesNext( context, kRequestedTriangles, vertices, nullptr );
			if ( count == 0 )
				break;
			triangles.insert( triangles.end(), vertices, vertices + count * 3 );
		}
	}
}

static const std::vector<JPH::Float3> &GetSourceBuoyancyTriangles(
	const JoltPhysicsObject *pObject, const JPH::Shape *pBodyShape,
	const JPH::Shape *&pFallbackShape, std::vector<JPH::Float3> &fallbackTriangles )
{
	const CPhysCollide *pCollide = pObject->GetCollide();
	if ( pCollide )
	{
		auto *pData = const_cast<JoltCollideUserData *>( pCollide->GetUserData() );
		if ( pData )
		{
			if ( !pData->sourceBuoyancyTrianglesCached )
			{
				pData->sourceBuoyancyTrianglesCached = true;
				BuildSourceBuoyancyTriangles( pCollide->ToShape(), pData->sourceBuoyancyTriangles );
			}
			return pData->sourceBuoyancyTriangles;
		}
	}

	// CreateSphereObject and a few procedural object paths do not expose a
	// CPhysCollide. Cache their immutable body shape on the per-fluid object
	// state instead.
	if ( pFallbackShape != pBodyShape )
	{
		pFallbackShape = pBodyShape;
		BuildSourceBuoyancyTriangles( pBodyShape, fallbackTriangles );
	}
	return fallbackTriangles;
}

struct SubmergedTriangle
{
	float flArea = 0.0f;
	JPH::Vec3 vCentroid = JPH::Vec3::sZero();
};

static bool ClipTriangleBelowSurface( const JPH::Plane &surface, const JPH::Vec3 vertices[3],
	SubmergedTriangle &out )
{
	JPH::Vec3 clipped[4];
	int clippedCount = 0;

	for ( int i = 0; i < 3; ++i )
	{
		const JPH::Vec3 &a = vertices[i];
		const JPH::Vec3 &b = vertices[( i + 1 ) % 3];
		const float distanceA = surface.SignedDistance( a );
		const float distanceB = surface.SignedDistance( b );
		const bool insideA = distanceA <= 0.0f;
		const bool insideB = distanceB <= 0.0f;

		if ( insideA )
			clipped[clippedCount++] = a;

		if ( insideA != insideB )
		{
			const float denominator = distanceA - distanceB;
			if ( fabsf( denominator ) > 1.0e-12f )
				clipped[clippedCount++] = a + ( b - a ) * ( distanceA / denominator );
		}
	}

	if ( clippedCount < 3 )
		return false;

	float totalArea = 0.0f;
	JPH::Vec3 weightedCentroid = JPH::Vec3::sZero();
	for ( int i = 1; i + 1 < clippedCount; ++i )
	{
		const JPH::Vec3 cross = ( clipped[i] - clipped[0] ).Cross( clipped[i + 1] - clipped[0] );
		const float area = 0.5f * cross.Length();
		if ( area <= 1.0e-12f )
			continue;
		totalArea += area;
		weightedCentroid += ( clipped[0] + clipped[i] + clipped[i + 1] ) * ( area / 3.0f );
	}

	if ( totalArea <= 1.0e-12f )
		return false;

	out.flArea = totalArea;
	out.vCentroid = weightedCentroid / totalArea;
	return true;
}

struct SourcePressureResult
{
	JPH::Vec3 vLinearImpulse = JPH::Vec3::sZero();
	JPH::Vec3 vAngularImpulse = JPH::Vec3::sZero();
	JPH::Vec3 vAngularVelocityChange = JPH::Vec3::sZero();
	float flVisibleArea = 0.0f;
};

static SourcePressureResult CalculateSourceSpherePressure(
	const JPH::Body &body, const JPH::Plane &surface, JPH::Vec3Arg fluidVelocity,
	float fluidDensity, float damping, float deltaTime,
	const std::vector<JPH::Float3> &triangles, float radius )
{
	SourcePressureResult result;
	const JPH::Vec3 center = JPH::Vec3( body.GetCenterOfMassPosition() );
	const JPH::Vec3 relativeVelocity = fluidVelocity - body.GetLinearVelocity();
	const float speedSquared = relativeVelocity.LengthSq();
	const JPH::Vec3 relativeDirection = speedSquared > 1.0e-12f
		? relativeVelocity / sqrtf( speedSquared )
		: JPH::Vec3::sAxisY();
	const float centerDistance = surface.SignedDistance( center );
	JPH::Vec3 pressurePoint = center;
	static constexpr float kPi = 3.14159265358979323846f;

	if ( centerDistance + radius <= 0.0f )
	{
		// IVP_BALL uses the exact projected circle when fully submerged.
		result.flVisibleArea = kPi * radius * radius;
	}
	else if ( centerDistance - radius < 0.0f )
	{
		// For a partially submerged ball, integrate the same projected wet
		// silhouette over Jolt's sphere tessellation. Translation uses one
		// center velocity just like IVP_BALL; angular surface velocity is not
		// folded into the pressure solve.
		const JPH::Quat rotation = body.GetRotation();
		const JPH::Vec3 objectPosition = JPH::Vec3( body.GetPosition() );
		JPH::Vec3 weightedPoint = JPH::Vec3::sZero();
		for ( size_t i = 0; i + 2 < triangles.size(); i += 3 )
		{
			JPH::Vec3 worldVertices[3];
			for ( int vertex = 0; vertex < 3; ++vertex )
				worldVertices[vertex] = objectPosition + rotation
					* JPH::Vec3::sLoadFloat3Unsafe( triangles[i + vertex] );

			JPH::Vec3 normal = ( worldVertices[1] - worldVertices[0] ).Cross(
				worldVertices[2] - worldVertices[0] );
			const float normalLength = normal.Length();
			if ( normalLength <= 1.0e-12f )
				continue;
			normal /= normalLength;
			if ( normal.Dot( relativeDirection ) > 1.0e-10f )
				continue;

			SubmergedTriangle submerged;
			if ( !ClipTriangleBelowSurface( surface, worldVertices, submerged ) )
				continue;
			const float projectedArea = ( std::max )(
				0.0f, -normal.Dot( relativeDirection ) * submerged.flArea );
			result.flVisibleArea += projectedArea;
			weightedPoint += submerged.vCentroid * projectedArea;
		}

		if ( result.flVisibleArea > 1.0e-12f )
			pressurePoint = weightedPoint / result.flVisibleArea;
	}

	if ( result.flVisibleArea <= 0.0f )
		return result;

	if ( speedSquared > 1.0e-12f && fluidDensity > 0.0f )
	{
		static constexpr float kSourceFrictionDamping = 0.05f;
		const float forceMagnitude = result.flVisibleArea * speedSquared
			* 0.5f * fluidDensity * ( damping + 3.0f * kSourceFrictionDamping );
		result.vLinearImpulse = relativeDirection * ( forceMagnitude * deltaTime );
		result.vAngularImpulse = ( pressurePoint - center ).Cross( result.vLinearImpulse );
	}

	// apply_dampening directly subtracts this from IVP's rot_speed_change;
	// it is a velocity delta and deliberately has no delta-time multiplier.
	static constexpr float kBallRotDamping = 0.01f;
	result.vAngularVelocityChange = -body.GetAngularVelocity()
		* ( kBallRotDamping * result.flVisibleArea / ( 2.0f * kPi * radius * radius ) );
	return result;
}

static SourcePressureResult CalculateSourcePressure(
	const JoltPhysicsObject *pObject, const JPH::Body &body, const JPH::Plane &surface,
	JPH::Vec3Arg fluidVelocity, float fluidDensity, float damping, float deltaTime,
	const JPH::Shape *&pFallbackShape, std::vector<JPH::Float3> &fallbackTriangles )
{
	SourcePressureResult result;
	if ( fluidDensity <= 0.0f )
		return result;

	const JPH::Shape *pBodyShape = body.GetShape();
	const std::vector<JPH::Float3> &triangles = GetSourceBuoyancyTriangles(
		pObject, pBodyShape, pFallbackShape, fallbackTriangles );
	if ( pBodyShape->GetSubType() == JPH::EShapeSubType::Sphere )
	{
		const auto *pSphere = static_cast<const JPH::SphereShape *>( pBodyShape );
		return CalculateSourceSpherePressure(
			body, surface, fluidVelocity, fluidDensity, damping,
			deltaTime, triangles, pSphere->GetRadius() );
	}
	const JPH::Quat rotation = body.GetRotation();
	const JPH::Vec3 objectPosition = JPH::Vec3( body.GetPosition() );
	const JPH::Vec3 centerOfMass = JPH::Vec3( body.GetCenterOfMassPosition() );
	const JPH::Vec3 linearVelocity = body.GetLinearVelocity();
	const JPH::Vec3 angularVelocity = body.GetAngularVelocity();
	const float pressureFactor = 0.5f * fluidDensity * damping;
	// physics_fluid.cpp leaves IVP_Template_Buoyancy's default surface
	// friction damping intact. This is distinct from viscosity_factor, which
	// Source explicitly sets to zero for GMod fluids.
	static constexpr float kSourceFrictionDamping = 0.05f;
	const float frictionFactor = 0.5f * fluidDensity * kSourceFrictionDamping;
	JPH::Vec3 moveTorque = JPH::Vec3::sZero();

	for ( size_t i = 0; i + 2 < triangles.size(); i += 3 )
	{
		JPH::Vec3 worldVertices[3];
		for ( int vertex = 0; vertex < 3; ++vertex )
			worldVertices[vertex] = objectPosition + rotation * JPH::Vec3::sLoadFloat3Unsafe( triangles[i + vertex] );

		JPH::Vec3 normal = ( worldVertices[1] - worldVertices[0] ).Cross(
			worldVertices[2] - worldVertices[0] );
		const float normalLength = normal.Length();
		if ( normalLength <= 1.0e-12f )
			continue;
		const float wholeTriangleArea = 0.5f * normalLength;
		normal /= normalLength;

		SubmergedTriangle submerged;
		if ( !ClipTriangleBelowSurface( surface, worldVertices, submerged ) )
			continue;

		const JPH::Vec3 arm = submerged.vCentroid - centerOfMass;
		const JPH::Vec3 pointVelocity = linearVelocity + angularVelocity.Cross( arm );
		const JPH::Vec3 relativeVelocity = fluidVelocity - pointVelocity;
		const float speedSquared = relativeVelocity.LengthSq();
		if ( speedSquared <= 1.0e-12f )
			continue;

		// Havana only damps the face presented to the local flow. Back-facing
		// surfaces contribute nothing, avoiding the whole-AABB drag used by Jolt.
		if ( normal.Dot( relativeVelocity ) > 1.0e-10f )
			continue;

		const float speed = sqrtf( speedSquared );
		const JPH::Vec3 relativeDirection = relativeVelocity / speed;
		const float projectedArea = ( std::max )(
			0.0f, -normal.Dot( relativeDirection ) * submerged.flArea );

		// Havana intentionally uses the whole triangle for its friction term,
		// even when only part of that triangle is below the water plane. The
		// pressure term alone is clipped/projected to the submerged surface.
		const float forceMagnitude = speedSquared
			* ( projectedArea * pressureFactor + wholeTriangleArea * frictionFactor );
		if ( forceMagnitude <= 0.0f )
			continue;
		const JPH::Vec3 pressureForce = relativeDirection * forceMagnitude;
		const JPH::Vec3 pressureImpulse = pressureForce * deltaTime;
		result.vLinearImpulse += pressureImpulse;
		result.vAngularImpulse += arm.Cross( pressureImpulse );
		result.flVisibleArea += projectedArea;

		const JPH::Vec3 moveDirection = relativeDirection
			- normal * normal.Dot( relativeDirection );
		moveTorque += moveDirection.Cross( pressureForce );
	}

	// Match IVP_Controller_Buoyancy::apply_dampening. This is a small
	// orientation-stabilising torque in addition to the natural r x impulse.
	if ( result.flVisibleArea > 0.0f )
	{
		static constexpr float kSourceTorqueFactor = 0.01f;
		const float moveFactor = -kSourceTorqueFactor * sqrtf( result.flVisibleArea ) * 0.1f;
		result.vAngularImpulse += moveTorque * ( moveFactor * deltaTime );
	}

	return result;
}

// Collects every unique body that intersects with the water shape. Forces are
// evaluated afterwards so the collector does not mix query traversal and writes.
class SourceFluidLayerFilter final : public JPH::BroadPhaseLayerFilter, public JPH::ObjectLayerFilter
{
public:
	bool ShouldCollide( JPH::BroadPhaseLayer layer ) const override
	{
		return layer == BroadPhaseLayers::MOVING || layer == BroadPhaseLayers::DEBRIS;
	}
	bool ShouldCollide( JPH::ObjectLayer layer ) const override
	{
		return layer == Layers::MOVING || layer == Layers::DEBRIS;
	}
};

class SourceFluidCollector : public JPH::CollideShapeCollector
{
public:
	SourceFluidCollector( JPH::PhysicsSystem *inSystem, std::vector<JoltPhysicsObject *> &objectsInShape )
		: m_pPhysicsSystem	( inSystem )
		, m_ObjectsInShape	( objectsInShape ) {}

	void AddHit( const ResultType &inResult ) override
	{
		const JPH::BodyID inBodyID = inResult.mBodyID2;

		JPH::BodyLockRead lock( m_pPhysicsSystem->GetBodyLockInterface(), inBodyID );
		if ( !lock.Succeeded() )
			return;
		const JPH::Body &body = lock.GetBody();
		JoltPhysicsObject *pObject = reinterpret_cast<JoltPhysicsObject *>( body.GetUserData() );
		if ( !pObject )
			return;

		// Don't do fluid simulation for these.
		if ( pObject->GetShadowController() || !( pObject->GetCallbackFlags() & CALLBACK_DO_FLUID_SIMULATION ) )
			return;

		if ( m_SeenObjects.insert( pObject ).second )
			m_ObjectsInShape.push_back( pObject );
	}

private:
	JPH::PhysicsSystem *				m_pPhysicsSystem;
	std::vector<JoltPhysicsObject *>	&m_ObjectsInShape;
	JPH::UnorderedSet<JoltPhysicsObject *> m_SeenObjects;
};

void JoltPhysicsFluidController::OnPreSimulate( float deltaTime )
{
	// Retain state and destruction listeners for objects that remain in the fluid.
	m_ObjectsInShape.clear();
	for ( auto &entry : m_ObjectStates )
		entry.second.bSeenThisFrame = false;

	if ( !m_pFluidObject )
	{
		ClearCachedObjectsInShape();
		return;
	}

	const cplane_t surfacePlane = GetSurfacePlane();

	JPH::Plane surface = JPH::Plane(
		SourceToJolt::Unitless( surfacePlane.normal ),
		SourceToJolt::Distance( -surfacePlane.dist ) );

	SourceFluidCollector collector( m_pPhysicsSystem, m_ObjectsInShape );

	// Apply buoyancy to all bodies that intersect with the water
	JPH::CollideShapeSettings collideSettings;
	collideSettings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideOnlyWithActive;

	// Ignore my own body
	// TODO(Josh): Should we use the SourceHitFilter from the player controller here?
	// May make more sense and respect ShouldCollide. Not sure if regular VPhysics does here.
	JPH::IgnoreSingleBodyFilter body_filter( m_pFluidObject->GetBodyID() );

	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	JPH::Mat44 queryTransform = bodyInterface.GetCenterOfMassTransform( m_pFluidObject->GetBodyID() );

	const JPH::Shape *pShape = m_pFluidObject->GetCollide()->ToShape();
	const SourceFluidLayerFilter layerFilter;

	m_pPhysicsSystem->GetNarrowPhaseQueryNoLock().CollideShape(
		pShape, JPH::Vec3::sReplicate( 1.0f ), queryTransform, collideSettings, JPH::Vec3::sZero(), collector,
		layerFilter, layerFilter, body_filter );

	const JPH::Vec3 surfacePoint = ProjectPoint( surface, JPH::Vec3::sZero() );
	const JPH::Vec3 surfaceNormal = surface.GetNormal();
	// Source calls ConvertDirectionToIVP for fluid current, which rotates axes
	// but deliberately does not apply the inches-to-metres distance scale.
	const JPH::Vec3 baseCurrentVelocity = SourceToJolt::Unitless( GetCurrentVelocity() );
	for ( JoltPhysicsObject *pObject : m_ObjectsInShape )
	{
		FluidObjectState &state = FindOrCreateObjectState( pObject );
		state.bSeenThisFrame = true;

		JPH::BodyLockWrite lock( m_pPhysicsSystem->GetBodyLockInterface(), pObject->GetBodyID() );
		if ( !lock.Succeeded() )
			continue;
		JPH::Body &body = lock.GetBody();
		if ( !body.IsActive() || !body.IsDynamic() )
			continue;

		// IVP carries a small amount of the object's previous motion into the
		// surrounding fluid. It also dilutes that motion when newly submerged
		// projected area grows.
		if ( state.flVisibleArea > state.flVisibleAreaOld && state.flVisibleArea > 1.0e-12f )
			state.vRelativeCurrent *= state.flVisibleAreaOld / state.flVisibleArea;
		state.flVisibleAreaOld = state.flVisibleArea;
		state.flVisibleArea = 0.0f;

		const JPH::Vec3 fluidVelocity = baseCurrentVelocity + state.vRelativeCurrent;
		const JPH::Vec3 linearVelocity = body.GetLinearVelocity();
		const float fluidDensity = m_flDensity * pObject->GetBuoyancyRatio();

		float totalVolume = 0.0f;
		float submergedVolume = 0.0f;
		JPH::Vec3 relativeCenterOfBuoyancy = JPH::Vec3::sZero();
		body.GetSubmergedVolume( surfacePoint, surfaceNormal,
			totalVolume, submergedVolume, relativeCenterOfBuoyancy );

		SourcePressureResult pressure = CalculateSourcePressure(
			pObject, body, surface, fluidVelocity, fluidDensity, m_Params.damping, deltaTime,
			state.pFallbackTriangleShape, state.fallbackTriangles );
		state.flVisibleArea = pressure.flVisibleArea;

		JPH::Vec3 buoyancyImpulse = JPH::Vec3::sZero();
		if ( submergedVolume > 1.0e-10f && fluidDensity > 0.0f )
		{
			buoyancyImpulse = -fluidDensity * submergedVolume
				* body.GetMotionProperties()->GetGravityFactor()
				* m_pPhysicsSystem->GetGravity() * deltaTime;
		}

		const JPH::Vec3 linearImpulse = pressure.vLinearImpulse + buoyancyImpulse;
		const JPH::Vec3 angularImpulse = pressure.vAngularImpulse
			+ relativeCenterOfBuoyancy.Cross( buoyancyImpulse );
		JPH::MotionProperties *pMotion = body.GetMotionProperties();
		pMotion->AddLinearVelocityStep( linearImpulse * pMotion->GetInverseMass() );
		pMotion->AddAngularVelocityStep(
			body.GetInverseInertia() * angularImpulse + pressure.vAngularVelocityChange );

		// Source sets viscosity_factor=0 and viscosity_input_factor=0.1.
		// The update observes the pre-impulse core speed because IVP stores the
		// calculated impulses in speed_change until the PSI is committed.
		state.vRelativeCurrent += ( linearVelocity - fluidVelocity ) * ( 0.1f * deltaTime );
	}

	for ( auto it = m_ObjectStates.begin(); it != m_ObjectStates.end(); )
	{
		if ( it->second.bSeenThisFrame )
			++it;
		else
		{
			it->first->RemoveDestroyedListener( this );
			const auto departed = it++;
			m_ObjectStates.erase( departed );
		}
	}
}

//-------------------------------------------------------------------------------------------------

cplane_t JoltPhysicsFluidController::GetSurfacePlane() const
{
	if ( !m_pFluidObject )
		return cplane_t{};

	matrix3x4_t objectToWorld;
	m_pFluidObject->GetPositionMatrix( &objectToWorld );

	cplane_t worldPlane;
	MatrixTransformPlane( objectToWorld, m_LocalPlane, worldPlane );

	return worldPlane;
}

Vector JoltPhysicsFluidController::GetCurrentVelocity() const
{
	if ( !m_pFluidObject )
		return vec3_origin;

	matrix3x4_t objectToWorld;
	m_pFluidObject->GetPositionMatrix( &objectToWorld );
	Vector worldVelocity;
	VectorRotate( m_LocalCurrentVelocity, objectToWorld, worldVelocity );
	return worldVelocity;
}

JoltPhysicsFluidController::FluidObjectState &JoltPhysicsFluidController::FindOrCreateObjectState(
	JoltPhysicsObject *pObject )
{
	auto [it, inserted] = m_ObjectStates.try_emplace( pObject );
	if ( inserted )
	{
		it->second.pObject = pObject;
		pObject->AddDestroyedListener( this );
	}
	return it->second;
}

void JoltPhysicsFluidController::ClearCachedObjectsInShape()
{
	for ( const auto &entry : m_ObjectStates )
		entry.first->RemoveDestroyedListener( this );
	m_ObjectStates.clear();
	m_ObjectsInShape.clear();
}
