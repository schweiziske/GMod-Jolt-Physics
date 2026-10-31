local I = JoltI18n or include("jolt_i18n/init.lua")
local gameplay = include("jolt_gameplay/server.lua")
local blackhole = {}

CreateConVar("sbox_maxjolt_blackholes", "3", FCVAR_ARCHIVE, I.Text("jolt.maximum_active_black_holes_per_player"), 0, 32)
cleanup.Register("jolt_blackholes")

function blackhole.Spawn(owner, position, velocity, settings)
	if not IsValid(owner) or not owner:IsPlayer() or not owner:Alive() then
		return nil, I.Phrase("jolt.a_living_player_is_required")
	end
	if not owner:CheckLimit("jolt_blackholes") then
		return nil -- Sandbox has already sent LimitHit.
	end
	local gravity, reason = gameplay.LoadFeature("gravity")
	if not gravity then
		return nil, reason
	end

	local grenade = ents.Create("sent_jolt_blackhole")
	if not IsValid(grenade) then
		return nil, I.Phrase("jolt.unable_to_create_the_black_hole")
	end
	grenade.FieldSettings = settings
	grenade:SetPos(position)
	grenade:SetOwner(owner)
	grenade:SetCreator(owner)
	grenade:Spawn()
	if not IsValid(grenade) then
		return nil, I.Phrase("jolt.unable_to_initialize_the_black_hole_s_physics_object")
	end

	local physics = grenade:GetPhysicsObject()
	if not IsValid(physics) then
		grenade:Remove()
		return nil, I.Phrase("jolt.unable_to_initialize_the_black_hole_s_physics_object")
	end
	if velocity then
		physics:SetVelocity(velocity)
		physics:AddAngleVelocity(Vector(200, 100, 0))
	else
		physics:EnableMotion(false)
	end
	gameplay.TrackEntity(owner, grenade, "jolt_blackholes", I.Phrase("jolt.jolt_black_hole"))
	return grenade
end

return blackhole
