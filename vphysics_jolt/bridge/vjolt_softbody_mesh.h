#pragma once

#include <vjolt/softbody.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>
#include <vector>

namespace VJoltSoftBody
{
inline constexpr float kMaxCoordinate = 1048576.0f;
inline constexpr float kMaxPressure = 1.0e9f;

struct PreparedMesh
{
	JPH::Ref<JPH::SoftBodySharedSettings> settings;
	std::vector<float> inverseMasses;
	float totalMass = 0.0f;
	uint32_t pinnedCount = 0;
	bool closed = false;
};

bool IsFiniteWithin( float value, float minimum, float maximum );
bool IsValidVector( const VJoltSoftBodyVector &value, float limit = kMaxCoordinate );
JPH::Vec3 ToJolt( const VJoltSoftBodyVector &value );
VJoltSoftBodyVector ToSource( JPH::Vec3Arg value );
bool ValidateSettings( const VJoltSoftBodySettings &settings );
VJoltSoftBodyError PrepareMesh(
	const VJoltSoftBodyMesh &mesh, const VJoltSoftBodySettings &settings, PreparedMesh &prepared );
} // namespace VJoltSoftBody
