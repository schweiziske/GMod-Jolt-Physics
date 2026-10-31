local I = JoltI18n or include("jolt_i18n/init.lua")
ENT.Type = "anim"
ENT.Base = "base_anim"
ENT.PrintName = "#jolt.jolt_soft_body"
ENT.Spawnable = false
ENT.DoNotDuplicate = true
ENT.RenderGroup = RENDERGROUP_OPAQUE

function ENT:SetupDataTables()
	self:NetworkVar("Int", 0, "SoftBodyID")
	self:NetworkVar("Bool", 0, "PlayerCollision")
end

-- Movement prediction needs the same filter as the server, including before
-- the separately streamed mesh metadata arrives. Keep tool/weapon traces solid.
hook.Add("ShouldCollide", "JoltSoftBodyPlayerCollision", function(first, second)
	if not IsValid(first) or not IsValid(second) then return end
	if first:GetClass() == "sent_jolt_softbody" and second:IsPlayer() then
		if not first:GetPlayerCollision() then return false end
	elseif second:GetClass() == "sent_jolt_softbody" and first:IsPlayer() then
		if not second:GetPlayerCollision() then return false end
	end
end)

function ENT:TestCollision(start, delta, isBox, extents, mask)
	if mask and bit.band(mask, CONTENTS_SOLID) == 0 then return false end
	local record = self.JoltSoftBodyRecord
	if not record then return false end
	if SERVER then
		-- Also works when Source owns TraceRay (mode 0). Full Jolt Trace clips
		-- registered bodies natively before reaching this gameplay callback.
		local hit = record.body:Trace(start, delta, isBox and extents or nil)
		return hit and hit.Hit and hit or false
	end
	if not record.buffer or not record.render or record.render.revision == 0 then return false end
	return JoltSoftBodies.Collision.Trace(record.buffer.display, record.geometry.indices,
		start, delta, isBox and extents or nil, record.settings.kind ~= 1, record.collision, record.buffer.revision)
end
