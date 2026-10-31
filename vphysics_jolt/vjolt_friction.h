
#pragma once

#include <cstdint>
#include <vector>

class JoltPhysicsObject;

class JoltPhysicsFrictionSnapshot final : public IPhysicsFrictionSnapshot
{
public:
	explicit JoltPhysicsFrictionSnapshot( JoltPhysicsObject *pObject );

	bool IsValid() override;

	IPhysicsObject *GetObject( int index ) override;
	int GetMaterial( int index ) override;

	void GetContactPoint( Vector &out ) override;

	void GetSurfaceNormal( Vector &out ) override;
	float GetNormalForce() override;
	float GetEnergyAbsorbed() override;

	void RecomputeFriction() override;
	void ClearFrictionForce() override;

	void MarkContactForDelete() override;
	void DeleteAllMarkedContacts( bool wakeObjects ) override;

	void NextFrictionData() override;
	float GetFrictionCoefficient() override;

	// Adapter-only geometry used by the player shadow reconciler. Source's public
	// snapshot omits separation, but Jolt speculative contacts require it to
	// distinguish a real IVP-distance blocker from an early manifold.
	float GetPenetrationDepth() const;

	struct Contact
	{
		JPH::BodyID otherBodyID;
		std::uint32_t selfSubShapeID = 0;
		std::uint32_t otherSubShapeID = 0;
		Vector vNormal;
		Vector vContactPoint;
		JPH::RVec3 worldContactPoint = JPH::RVec3::sZero();
		float flPenetrationDepth;
		float flNormalImpulse;
		float flFrictionEnergy;
		float flFrameDeltaTime;
	};

private:
	JoltPhysicsObject *ResolveOtherObject( const Contact &contact ) const;
	JoltPhysicsObject *m_pSelf = nullptr;
	std::vector<Contact> m_contacts;
	std::vector<JPH::BodyID> m_DeleteList;
	size_t m_index = 0;
};
