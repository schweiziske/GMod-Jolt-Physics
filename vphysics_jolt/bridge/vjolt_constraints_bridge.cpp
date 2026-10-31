#include "cbase.h"
#include "bridge/vjolt_constraints_internal.h"
#include "vjolt_environment.h"
#include "vjolt_interface.h"
#include "vjolt_internal_listeners.h"
#include "vjolt_object.h"

#include <Jolt/Physics/Constraints/GearConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/RackAndPinionConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <unordered_map>

#include "tier0/memdbgon.h"

#if GAME_GMOD
extern bool IsValidPhyiscsObject( IPhysicsObject *object );
#endif

namespace
{
constexpr size_t kMaxMechanisms = 4096;
constexpr float kMaxCoordinate = 1048576.0f;
constexpr float kMinGearRatio = 0.001f;
constexpr float kMaxGearRatio = 1000.0f;
constexpr float kMinRackTravel = 0.01f;
constexpr float kMaxMotorSpeed = 3600.0f;
constexpr float kMaxMotorTorque = 1.0e9f;
constexpr uint32_t kSupportedFlags = VJoltAnchor1AtCOM | VJoltAnchor2AtCOM;

bool IsFiniteWithin( float value, float limit = kMaxCoordinate )
{
	return std::isfinite( value ) && std::abs( value ) <= limit;
}

bool IsFiniteVector( const VJoltConstraintVector &value )
{
	return IsFiniteWithin( value.x ) && IsFiniteWithin( value.y ) && IsFiniteWithin( value.z );
}

bool IsValidAxis( const VJoltConstraintVector &axis )
{
	const double lengthSquared = double( axis.x ) * axis.x + double( axis.y ) * axis.y + double( axis.z ) * axis.z;
	return IsFiniteVector( axis ) && lengthSquared >= 1.0e-8;
}

template <typename Settings>
bool AreValidFrames( const Settings &settings )
{
	return settings.size == sizeof( Settings ) && ( settings.flags & ~kSupportedFlags ) == 0
		&& IsFiniteVector( settings.anchor1 ) && IsFiniteVector( settings.anchor2 ) && IsValidAxis( settings.axis1 )
		&& IsValidAxis( settings.axis2 );
}

bool IsValidGearSettings( const VJoltGearSettings &settings )
{
	return AreValidFrames( settings ) && IsFiniteWithin( settings.ratio, kMaxGearRatio )
		&& std::abs( settings.ratio ) >= kMinGearRatio;
}

bool IsValidRackSettings( const VJoltRackAndPinionSettings &settings )
{
	return AreValidFrames( settings ) && IsFiniteWithin( settings.travelPerRevolution )
		&& std::abs( settings.travelPerRevolution ) >= kMinRackTravel && IsFiniteWithin( settings.minPosition )
		&& IsFiniteWithin( settings.maxPosition ) && settings.minPosition <= 0.0f && settings.maxPosition >= 0.0f
		&& settings.minPosition < settings.maxPosition;
}

bool IsLivePhysicsObject( IPhysicsObject *object )
{
	if ( !object )
	{
		return false;
	}

#if GAME_GMOD
	return IsValidPhyiscsObject( object );
#else
	// Validate a foreign pointer before casting or dereferencing it.
	auto &physics = JoltPhysicsInterface::GetInstance();
	for ( int index = 0; auto *environment = physics.GetActiveEnvironmentByIndex( index ); ++index )
	{
		int count = 0;
		const auto **objects = environment->GetObjectList( &count );
		for ( int objectIndex = 0; objectIndex < count; ++objectIndex )
		{
			if ( objects[objectIndex] == object )
			{
				return true;
			}
		}
	}
	return false;
#endif
}

bool SupportsMechanism( JoltPhysicsObject *object )
{
	return !object->IsStatic() && !object->IsTrigger() && !object->IsFluid() && !object->IsControlledByGame()
		&& !object->GetShadowController() && ( object->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) == 0;
}

JPH::Vec3 ToJoltDirection( const VJoltConstraintVector &axis )
{
	return SourceToJolt::Distance( Vector( axis.x, axis.y, axis.z ) ).Normalized();
}

JPH::RVec3 ResolveAnchor( const JPH::Body &body, const VJoltConstraintVector &anchor, bool useCenterOfMass )
{
	if ( useCenterOfMass )
	{
		return body.GetCenterOfMassPosition();
	}
	return JPH::RVec3( SourceToJolt::Distance( Vector( anchor.x, anchor.y, anchor.z ) ) );
}

JPH::HingeConstraint *CreateWorldHinge( JPH::Body &body, JPH::RVec3Arg anchor, JPH::Vec3Arg axis )
{
	JPH::HingeConstraintSettings settings;
	settings.mPoint1 = settings.mPoint2 = anchor;
	settings.mHingeAxis1 = settings.mHingeAxis2 = axis;
	settings.mNormalAxis1 = settings.mNormalAxis2 = axis.GetNormalizedPerpendicular();
	return static_cast<JPH::HingeConstraint *>( settings.Create( JPH::Body::sFixedToWorld, body ) );
}

JPH::SliderConstraint *CreateWorldSlider(
	JPH::Body &body, JPH::RVec3Arg anchor, JPH::Vec3Arg axis, float minPosition, float maxPosition )
{
	JPH::SliderConstraintSettings settings;
	settings.mPoint1 = settings.mPoint2 = anchor;
	settings.mSliderAxis1 = settings.mSliderAxis2 = axis;
	settings.mNormalAxis1 = settings.mNormalAxis2 = axis.GetNormalizedPerpendicular();
	settings.mLimitsMin = SourceToJolt::Distance( minPosition );
	settings.mLimitsMax = SourceToJolt::Distance( maxPosition );
	return static_cast<JPH::SliderConstraint *>( settings.Create( JPH::Body::sFixedToWorld, body ) );
}

class ConstraintBridge;

struct Mechanism final : IJoltObjectDestroyedListener
{
	ConstraintBridge *owner = nullptr;
	VJoltConstraintHandle handle = 0;
	JoltPhysicsEnvironment *environment = nullptr;
	std::array<JoltPhysicsObject *, 2> objects{};
	std::array<JPH::RefConst<JPH::Shape>, 2> initialShapes;
	std::array<JPH::Vec3, 2> axes;
	JPH::Ref<JPH::HingeConstraint> primaryHinge;
	JPH::Ref<JPH::HingeConstraint> secondaryHinge;
	JPH::Ref<JPH::SliderConstraint> rackSlider;
	JPH::Ref<JPH::TwoBodyConstraint> coupling;
	uint32_t kind = 0;
	bool retiring = false;
	float motorSpeed = 0.0f;
	float maxTorque = 0.0f;

	bool CanSimulate() const
	{
		for ( size_t index = 0; index < objects.size(); ++index )
		{
			if ( !SupportsMechanism( objects[index] )
				|| initialShapes[index].GetPtr() != objects[index]->GetBody()->GetShape() )
			{
				return false;
			}
		}
		return !retiring;
	}

	void WakeObjects() const
	{
		for ( auto *object : objects )
		{
			object->Wake();
		}
	}

	void OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *object ) override;
};

class ConstraintBridge final : public IVJoltConstraintsBridge
{
public:
	uint32_t GetVersion() const override
	{
		return kVJoltConstraintsBridgeVersion;
	}

	VJoltConstraintHandle CreateGear(
		IPhysicsObject *firstGear, IPhysicsObject *secondGear, const VJoltGearSettings &settings ) override
	{
		if ( !IsValidGearSettings( settings ) )
		{
			return 0;
		}

		auto mechanism = PrepareMechanism( firstGear, secondGear, settings );
		if ( !mechanism )
		{
			return 0;
		}

		mechanism->kind = VJoltGear;
		auto &firstBody = *mechanism->objects[0]->GetBody();
		auto &secondBody = *mechanism->objects[1]->GetBody();
		const auto secondAnchor = ResolveAnchor( secondBody, settings.anchor2, settings.flags & VJoltAnchor2AtCOM );
		mechanism->secondaryHinge = CreateWorldHinge( secondBody, secondAnchor, mechanism->axes[1] );

		JPH::GearConstraintSettings couplingSettings;
		couplingSettings.mHingeAxis1 = mechanism->axes[0];
		couplingSettings.mHingeAxis2 = mechanism->axes[1];
		couplingSettings.mRatio = settings.ratio;
		auto *coupling = static_cast<JPH::GearConstraint *>( couplingSettings.Create( firstBody, secondBody ) );
		// Jolt uses these guides to correct accumulated angular drift.
		coupling->SetConstraints( mechanism->primaryHinge, mechanism->secondaryHinge );
		mechanism->coupling = coupling;
		return RegisterMechanism( std::move( mechanism ) );
	}

	VJoltConstraintHandle CreateRackAndPinion(
		IPhysicsObject *pinion, IPhysicsObject *rack, const VJoltRackAndPinionSettings &settings ) override
	{
		if ( !IsValidRackSettings( settings ) )
		{
			return 0;
		}

		auto mechanism = PrepareMechanism( pinion, rack, settings );
		if ( !mechanism )
		{
			return 0;
		}

		mechanism->kind = VJoltRackAndPinion;
		auto &pinionBody = *mechanism->objects[0]->GetBody();
		auto &rackBody = *mechanism->objects[1]->GetBody();
		const auto rackAnchor = ResolveAnchor( rackBody, settings.anchor2, settings.flags & VJoltAnchor2AtCOM );
		mechanism->rackSlider =
			CreateWorldSlider( rackBody, rackAnchor, mechanism->axes[1], settings.minPosition, settings.maxPosition );

		JPH::RackAndPinionConstraintSettings couplingSettings;
		couplingSettings.mHingeAxis = mechanism->axes[0];
		couplingSettings.mSliderAxis = mechanism->axes[1];
		couplingSettings.mRatio = 2.0f * JPH::JPH_PI / SourceToJolt::Distance( settings.travelPerRevolution );
		auto *coupling = static_cast<JPH::RackAndPinionConstraint *>( couplingSettings.Create( pinionBody, rackBody ) );
		coupling->SetConstraints( mechanism->primaryHinge, mechanism->rackSlider );
		mechanism->coupling = coupling;
		return RegisterMechanism( std::move( mechanism ) );
	}

	bool IsValid( VJoltConstraintHandle handle ) const override
	{
		return FindMechanism( handle ) != nullptr;
	}

	bool SetMotor( VJoltConstraintHandle handle, float degreesPerSecond, float maxTorque ) override
	{
		auto *mechanism = FindMechanism( handle );
		if ( !mechanism || mechanism->environment->IsInSimulation()
			|| !IsFiniteWithin( degreesPerSecond, kMaxMotorSpeed ) || !IsFiniteWithin( maxTorque, kMaxMotorTorque )
			|| maxTorque < 0.0f )
		{
			return false;
		}

		mechanism->primaryHinge->GetMotorSettings().SetTorqueLimit( SourceToJolt::Torque( maxTorque ) );
		mechanism->primaryHinge->SetTargetAngularVelocity( SourceToJolt::Angle( degreesPerSecond ) );
		mechanism->primaryHinge->SetMotorState(
			maxTorque == 0.0f ? JPH::EMotorState::Off : JPH::EMotorState::Velocity );
		mechanism->motorSpeed = degreesPerSecond;
		mechanism->maxTorque = maxTorque;
		mechanism->WakeObjects();
		return true;
	}

	bool GetState( VJoltConstraintHandle handle, VJoltConstraintState &state ) const override
	{
		const auto *mechanism = FindMechanism( handle );
		if ( !mechanism || state.size != sizeof( state ) || mechanism->environment->IsInSimulation() )
		{
			return false;
		}

		const auto &firstBody = *mechanism->objects[0]->GetBody();
		const auto &secondBody = *mechanism->objects[1]->GetBody();
		state = {};
		state.kind = mechanism->kind;
		state.angle1 = JoltToSource::Angle( mechanism->primaryHinge->GetCurrentAngle() );
		state.angularVelocity1 = JoltToSource::Angle( firstBody.GetAngularVelocity().Dot( mechanism->axes[0] ) );
		state.motorSpeed = mechanism->motorSpeed;
		state.maxTorque = mechanism->maxTorque;

		if ( mechanism->secondaryHinge )
		{
			state.angle2 = JoltToSource::Angle( mechanism->secondaryHinge->GetCurrentAngle() );
			state.angularVelocity2 = JoltToSource::Angle( secondBody.GetAngularVelocity().Dot( mechanism->axes[1] ) );
		}
		if ( mechanism->rackSlider )
		{
			state.position = JoltToSource::Distance( mechanism->rackSlider->GetCurrentPosition() );
			state.linearVelocity = JoltToSource::Distance( secondBody.GetLinearVelocity().Dot( mechanism->axes[1] ) );
		}
		return true;
	}

	bool Destroy( VJoltConstraintHandle handle ) override
	{
		auto *mechanism = FindMechanism( handle );
		if ( !mechanism )
		{
			return false;
		}

		auto *environment = mechanism->environment;
		mechanism->retiring = true;
		if ( environment->DeferPhysicsMutation( [this, handle]() { RemoveMechanism( handle ); } ) )
		{
			return true;
		}
		if ( !environment->IsInSimulation() )
		{
			RemoveMechanism( handle );
		}
		// Motion-controller callbacks run before the solver. BeforeSimulation removes
		// their retired assemblies; contact callbacks use the deferred queue above.
		return true;
	}

	void BeforeSimulation( JoltPhysicsEnvironment *environment )
	{
		for ( auto iterator = m_Mechanisms.begin(); iterator != m_Mechanisms.end(); )
		{
			const auto current = iterator++;
			const auto &mechanism = *current->second;
			if ( mechanism.environment == environment && !mechanism.CanSimulate() )
			{
				RemoveMechanism( current->first );
			}
		}
	}

	void ReleaseObject( JoltPhysicsObject *object )
	{
		for ( auto iterator = m_Mechanisms.begin(); iterator != m_Mechanisms.end(); )
		{
			const auto current = iterator++;
			const auto &objects = current->second->objects;
			if ( objects[0] == object || objects[1] == object )
			{
				RemoveMechanism( current->first );
			}
		}
	}

	void EnvironmentDestroyed( JoltPhysicsEnvironment *environment )
	{
		for ( auto iterator = m_Mechanisms.begin(); iterator != m_Mechanisms.end(); )
		{
			const auto current = iterator++;
			if ( current->second->environment == environment )
			{
				RemoveMechanism( current->first, nullptr, false );
			}
		}
	}

private:
	friend struct Mechanism;

	Mechanism *FindMechanism( VJoltConstraintHandle handle ) const
	{
		const auto iterator = m_Mechanisms.find( handle );
		if ( iterator == m_Mechanisms.end() || iterator->second->retiring )
		{
			return nullptr;
		}
		return iterator->second.get();
	}

	template <typename Settings>
	std::unique_ptr<Mechanism> PrepareMechanism(
		IPhysicsObject *firstObject, IPhysicsObject *secondObject, const Settings &settings )
	{
		if ( firstObject == secondObject || !IsLivePhysicsObject( firstObject ) || !IsLivePhysicsObject( secondObject )
			|| m_NextHandle == std::numeric_limits<VJoltConstraintHandle>::max()
			|| m_Mechanisms.size() >= kMaxMechanisms )
		{
			return nullptr;
		}

		auto *first = static_cast<JoltPhysicsObject *>( firstObject );
		auto *second = static_cast<JoltPhysicsObject *>( secondObject );
		auto *environment = first->GetJoltEnvironment();
		if ( !environment || environment != second->GetJoltEnvironment() || environment->IsInSimulation()
			|| !SupportsMechanism( first ) || !SupportsMechanism( second ) )
		{
			return nullptr;
		}

		auto mechanism = std::make_unique<Mechanism>();
		mechanism->owner = this;
		mechanism->environment = environment;
		mechanism->objects = { first, second };
		mechanism->axes = { ToJoltDirection( settings.axis1 ), ToJoltDirection( settings.axis2 ) };
		mechanism->initialShapes = { first->GetBody()->GetShape(), second->GetBody()->GetShape() };
		const auto firstAnchor =
			ResolveAnchor( *first->GetBody(), settings.anchor1, settings.flags & VJoltAnchor1AtCOM );
		mechanism->primaryHinge = CreateWorldHinge( *first->GetBody(), firstAnchor, mechanism->axes[0] );
		return mechanism;
	}

	VJoltConstraintHandle RegisterMechanism( std::unique_ptr<Mechanism> mechanism )
	{
		mechanism->handle = m_NextHandle++;
		const auto handle = mechanism->handle;
		auto *system = mechanism->environment->GetPhysicsSystem();
		for ( auto *object : mechanism->objects )
		{
			object->AddDestroyedListener( mechanism.get() );
			object->AddConstraintAttachment();
		}

		system->AddConstraint( mechanism->primaryHinge );
		if ( mechanism->secondaryHinge )
		{
			system->AddConstraint( mechanism->secondaryHinge );
		}
		if ( mechanism->rackSlider )
		{
			system->AddConstraint( mechanism->rackSlider );
		}
		system->AddConstraint( mechanism->coupling );
		mechanism->WakeObjects();
		m_Mechanisms.emplace( handle, std::move( mechanism ) );
		return handle;
	}

	void RemoveMechanism(
		VJoltConstraintHandle handle, JoltPhysicsObject *dyingObject = nullptr, bool wakeObjects = true )
	{
		const auto iterator = m_Mechanisms.find( handle );
		if ( iterator == m_Mechanisms.end() )
		{
			return;
		}

		auto &mechanism = *iterator->second;
		auto *system = mechanism.environment->GetPhysicsSystem();
		system->RemoveConstraint( mechanism.coupling );
		system->RemoveConstraint( mechanism.primaryHinge );
		if ( mechanism.secondaryHinge )
		{
			system->RemoveConstraint( mechanism.secondaryHinge );
		}
		if ( mechanism.rackSlider )
		{
			system->RemoveConstraint( mechanism.rackSlider );
		}

		for ( auto *object : mechanism.objects )
		{
			object->RemoveDestroyedListener( &mechanism );
			object->RemoveConstraintAttachment();
			if ( wakeObjects && object != dyingObject
				&& ( object->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) == 0 )
			{
				object->Wake();
			}
		}
		m_Mechanisms.erase( iterator );
	}

	std::unordered_map<VJoltConstraintHandle, std::unique_ptr<Mechanism>> m_Mechanisms;
	VJoltConstraintHandle m_NextHandle = 1;
};

void Mechanism::OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *object )
{
	owner->RemoveMechanism( handle, object );
}

ConstraintBridge g_ConstraintBridge;
} // namespace

EXPOSE_SINGLE_INTERFACE_GLOBALVAR(
	ConstraintBridge, IVJoltConstraintsBridge, VJOLT_CONSTRAINTS_BRIDGE_VERSION, g_ConstraintBridge );

void VJoltConstraintsBeforeSimulation( JoltPhysicsEnvironment *environment )
{
	g_ConstraintBridge.BeforeSimulation( environment );
}

void VJoltConstraintsReleaseObject( JoltPhysicsObject *object )
{
	g_ConstraintBridge.ReleaseObject( object );
}

void VJoltConstraintsEnvironmentDestroyed( JoltPhysicsEnvironment *environment )
{
	g_ConstraintBridge.EnvironmentDestroyed( environment );
}
