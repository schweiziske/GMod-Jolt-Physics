include("shared.lua")

local glow = Material("sprites/light_glow02_add")
local ring = Material("cable/blue_elec")
local pullColor = Color(130, 90, 255)
local burstColor = Color(100, 210, 255)
local coreColor = Color(5, 3, 12)
local segments = 48

function ENT:DrawTranslucent()
	if self:GetPhase() == self.PhaseFuse then
		self:DrawModel()
		return
	end

	local now = CurTime()
	local phaseLength = math.max(self:GetPhaseEnd() - self:GetPhaseStart(), 0.01)
	local progress = math.Clamp((now - self:GetPhaseStart()) / phaseLength, 0, 1)
	local bursting = self:GetPhase() == self.PhaseBurst
	local position = self:GetPos()
	local color = bursting and burstColor or pullColor
	local coreRadius = bursting and 12 * (1 - progress) or 10 + math.sin(now * 8) * 2
	local ringRadius = bursting and 20 + self:GetFieldRadius() * progress or 46 - 22 * progress

	local bounds = self:GetFieldRadius() + 64
	self:SetRenderBounds(Vector(-bounds, -bounds, -bounds), Vector(bounds, bounds, bounds))
	render.SetMaterial(glow)
	render.DrawSprite(position, coreRadius * 9, coreRadius * 9, color)
	render.SetColorMaterial()
	render.DrawSphere(position, coreRadius, 16, 12, coreColor)

	render.SetMaterial(ring)
	for plane = 1, 2 do
		render.StartBeam(segments + 1)
		for segment = 0, segments do
			local angle = segment / segments * math.pi * 2 + now * plane
			local horizontal = math.cos(angle) * ringRadius
			local vertical = math.sin(angle) * ringRadius
			local offset = plane == 1 and Vector(horizontal, vertical, 0) or Vector(horizontal, 0, vertical)
			render.AddBeam(position + offset, bursting and 5 or 2, segment / segments, color)
		end
		render.EndBeam()
	end
end
