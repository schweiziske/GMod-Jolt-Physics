#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Math/Math.h>

#include <algorithm>
#include <cmath>

// Pure Source shadow-controller math in Jolt coordinates. Keep this independent from
// IPhysicsObject so both IPhysicsShadowController and IPhysicsObject::ComputeShadowControl
// use the same implementation.
namespace VJoltShadowControl
{
	struct Params
	{
		JPH::Vec3 targetPosition = JPH::Vec3::sZero();
		JPH::Quat targetRotation = JPH::Quat::sIdentity();
		JPH::Vec3 lastPosition = JPH::Vec3::sZero();
		JPH::Vec3 lastImpulse = JPH::Vec3::sZero();
		float maxAngular = 0.0f;
		float maxDampAngular = 0.0f;
		float maxSpeed = 0.0f;
		float maxDampSpeed = 0.0f;
		float dampFactor = 1.0f;
		float teleportDistance = 0.0f;
	};

	struct Result
	{
		float secondsToArrival = 0.0f;
		bool teleported = false;
	};

	inline void ComputeController( JPH::Vec3 &currentSpeed, JPH::Vec3Arg delta,
		float maxSpeed, float maxDampSpeed, float scaleDelta, float damping,
		JPH::Vec3 *pOutImpulse = nullptr )
	{
		if ( currentSpeed.LengthSq() < 1.0e-6f )
			currentSpeed = JPH::Vec3::sZero();

		JPH::Vec3 acceleration = JPH::Vec3::sZero();
		if ( maxSpeed > 0.0f )
		{
			acceleration = delta * scaleDelta;
			const float speed = acceleration.Length();
			if ( speed > maxSpeed )
				acceleration *= maxSpeed / speed;
		}

		JPH::Vec3 dampAcceleration = JPH::Vec3::sZero();
		if ( maxDampSpeed > 0.0f )
		{
			dampAcceleration = currentSpeed * -damping;
			const float speed = dampAcceleration.Length();
			if ( speed > maxDampSpeed )
				dampAcceleration *= maxDampSpeed / speed;
		}

		currentSpeed += dampAcceleration + acceleration;
		if ( pOutImpulse )
			*pOutImpulse = acceleration;
	}

	inline JPH::Vec3 QuaternionDeltaAngles( JPH::QuatArg target, JPH::QuatArg current )
	{
		const JPH::Quat delta = ( target * current.Conjugated() ).Normalized();
		const float angleMagnitude = 2.0f * std::acos( std::clamp( delta.GetW(), -1.0f, 1.0f ) );
		float angle = angleMagnitude;
		if ( angle > JPH::JPH_PI )
			angle -= 2.0f * JPH::JPH_PI;

		const JPH::Vec3 axis( delta.GetX(), delta.GetY(), delta.GetZ() );
		const float axisLengthSq = axis.LengthSq();
		return axisLengthSq > 1.0e-12f ? axis * ( angle / std::sqrt( axisLengthSq ) )
			: JPH::Vec3::sZero();
	}

	inline Result Compute( Params &params, JPH::Vec3 &position, JPH::Quat &rotation,
		JPH::Vec3 &linearVelocity, JPH::Vec3 &angularVelocity,
		float secondsToArrival, float deltaTime )
	{
		Result result;
		result.secondsToArrival = secondsToArrival > 0.0f ? secondsToArrival : 0.0f;
		if ( deltaTime <= 0.0f )
			return result;

		const float rawFraction = result.secondsToArrival > 0.0f
			? deltaTime / result.secondsToArrival : 1.0f;
		const float fraction = rawFraction < 1.0f ? rawFraction : 1.0f;
		result.secondsToArrival = result.secondsToArrival > deltaTime
			? result.secondsToArrival - deltaTime : 0.0f;
		if ( fraction <= 0.0f )
			return result;

		JPH::Vec3 deltaPosition = params.targetPosition - position;
		if ( params.teleportDistance > 0.0f )
		{
			const JPH::Vec3 teleportError = params.lastPosition != JPH::Vec3::sZero()
				? position - params.lastPosition : deltaPosition;
			if ( teleportError.LengthSq() > params.teleportDistance * params.teleportDistance )
			{
				position = params.targetPosition;
				rotation = params.targetRotation;
				deltaPosition = JPH::Vec3::sZero();
				result.teleported = true;
			}
		}

		const float scaleDelta = fraction / deltaTime;
		ComputeController( linearVelocity, deltaPosition, params.maxSpeed,
			params.maxDampSpeed, scaleDelta, params.dampFactor, &params.lastImpulse );
		params.lastPosition = position + linearVelocity * deltaTime;

		const JPH::Vec3 deltaAngles = QuaternionDeltaAngles( params.targetRotation, rotation );
		ComputeController( angularVelocity, deltaAngles, params.maxAngular,
			params.maxDampAngular, scaleDelta, params.dampFactor );
		return result;
	}

	inline float LinearDampingFactor( float damping, float deltaTime )
	{
		const float scaled = ( damping > 0.0f ? damping : 0.0f )
			* ( deltaTime > 0.0f ? deltaTime : 0.0f );
		return scaled < 0.25f ? 1.0f - scaled : std::exp( -scaled );
	}

	inline float AngularDampingFactor( float damping, float deltaTime )
	{
		const float scaled = ( damping > 0.0f ? damping : 0.0f )
			* ( deltaTime > 0.0f ? deltaTime : 0.0f );
		// IVP tests the squared length of its three-axis damping vector before choosing
		// the linear approximation or exponential path. Jolt exposes one scalar here.
		return 3.0f * scaled * scaled < 0.5f ? 1.0f - scaled : std::exp( -scaled );
	}

	inline void ApplyGroundCorrection( bool allowTranslation, bool isOnGround,
		JPH::Vec3Arg gravityStep, JPH::Vec3 &linearVelocity, JPH::Vec3 &lastImpulse )
	{
		if ( !allowTranslation || !isOnGround || lastImpulse.GetZ() >= gravityStep.GetZ() )
			return;

		const float correction = gravityStep.GetZ() - lastImpulse.GetZ();
		linearVelocity.SetZ( linearVelocity.GetZ() + correction );
		lastImpulse.SetZ( gravityStep.GetZ() );
	}
}
