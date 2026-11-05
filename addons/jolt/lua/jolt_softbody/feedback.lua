local api = JoltSoftBodies
local feedback = {nextPoll = 0}
local sounds = {
	"physics/cardboard/cardboard_box_impact_soft1.wav",
	"physics/cardboard/cardboard_box_impact_soft2.wav",
	"physics/rubber/rubber_tire_impact_hard1.wav",
	"physics/cardboard/cardboard_box_impact_soft3.wav",
	"physics/metal/metal_sheet_impact_hard2.wav",
}

function feedback.Register(record)
	record.damage = 0
	record.impact = {}
	record.feedbackReady = record.settings.feedback and record.body.SetImpactTracking
		and record.body:SetImpactTracking(true) or false
end

function feedback.Inherit(parent, children)
	for _, child in ipairs(children) do
		child.damage = parent.damage or 0
		child.entity:SetSoftDamage(child.damage)
		-- A fresh broken surface needs another substantial impact to split again.
		child.nextImpact = CurTime() + 0.5
	end
end

function feedback.Strength(settings, impact)
	local threshold = settings.impactthreshold or 80
	if not api.Config.IsFinite(impact.speed) or not api.Config.IsFinite(impact.energy)
		or impact.speed <= threshold or impact.energy <= 0 then return 0 end
	local speed = math.Clamp((impact.speed - threshold) / (threshold * 4), 0, 1)
	-- Weight by participating mass so a single noisy vertex cannot count like a
	-- whole-body impact. This is a gameplay metric, not a material failure model.
	local reference = 0.5 * settings.mass * (threshold * 0.0254) ^ 2
	return speed * math.min(1, math.sqrt(impact.energy / math.max(0.000001, reference)))
end

function feedback.Apply(record, impact, now)
	if not record.settings.feedback or now < (record.nextImpact or 0) then return false end
	local strength = feedback.Strength(record.settings, impact)
	if strength < 0.03 then return false end
	local vertex = math.floor(tonumber(impact.vertex) or 0)
	if vertex < 1 or vertex > record.geometry.vertexCount then return false end
	record.nextImpact = now + 0.15
	record.damage = math.min(1, (record.damage or 0) + strength * 0.25 / (record.settings.toughness or 1))
	local entity = record.entity
	entity:SetSoftDamage(record.damage)
	entity:SetImpactVertex(vertex)
	entity:SetImpactStrength(strength)
	entity:SetImpactTime(now)
	entity:EmitSound(sounds[record.settings.material or 1], 65 + strength * 15,
		math.floor(100 - strength * 15), 0.15 + strength * 0.65, CHAN_AUTO)
	hook.Run("JoltSoftBodyImpact", entity, {strength = strength, damage = record.damage,
		speed = impact.speed, energy = impact.energy, vertex = vertex, position = impact.pos})
	-- Gamemode hooks can remove the object.
	if not IsValid(entity) or api.Server.Records[record.id] ~= record then return true end
	if record.settings.impactbreak and record.damage >= 1 then
		if record.settings.kind == 2 then
			record.leaking = true
		elseif record.geometry.columns then
			local mesh = record.geometry
			if mesh.rowTo - mesh.rowFrom >= 2 then
				record.impactSeam = math.Clamp(math.floor((vertex - 1) / mesh.columns) + mesh.rowFrom,
					mesh.rowFrom + 1, mesh.rowTo - 1)
			end
		end
		if not record.broken then
			record.broken = true
			hook.Run("JoltSoftBodyBroken", entity)
		end
	end
	return true
end

function feedback.SetInflation(record, value)
	if record.settings.kind ~= 2 or not api.Config.IsFinite(value) then return false end
	value = math.Clamp(value, 0, 100)
	local pressure = math.min(1e9, record.settings.mass * 600 * record.settings.width * 0.5 * value / 100)
	if not record.body:SetPressure(pressure) then return false end
	record.settings.inflation = value
	record.entity:SetInflation(value)
	record.forceSample = true
	return true
end

function feedback.Damage(record, position, impulse)
	if not record.settings.feedback or CurTime() < (record.nextImpact or 0) then return end
	local positions, nearest, distance = record.snapshot.positions, 1, math.huge
	for index = 1, record.geometry.vertexCount do
		local at = (index - 1) * 3
		local x, y, z = positions[at + 1] - position.x, positions[at + 2] - position.y, positions[at + 3] - position.z
		local square = x * x + y * y + z * z
		if square < distance then nearest, distance = index, square end
	end
	local mass = record.settings.mass
	local speed = math.min(record.settings.maxVelocity, impulse / math.max(0.001, mass * 0.2))
	feedback.Apply(record, {vertex = nearest, pos = position, speed = speed,
		energy = 0.5 * mass * 0.2 * (speed * 0.0254) ^ 2}, CurTime())
end

function feedback.Think(now)
	if now < feedback.nextPoll then return end
	feedback.nextPoll = now + 0.1
	-- Hooks may remove other records or create fragments while this batch runs.
	local pending = {}
	for index, record in ipairs(api.Server.Order) do pending[index] = record end
	for index = #pending, 1, -1 do
		local record = pending[index]
		if api.Server.Records[record.id] == record and record.feedbackReady and record.body:IsValid() then
			local impact = record.body:ConsumeImpact(record.impact)
			if impact and impact.contacts > 0 then feedback.Apply(record, impact, now) end
		end
		if api.Server.Records[record.id] == record then
			if record.leaking and record.settings.inflation > 0 then
				local elapsed = math.Clamp(now - (record.lastLeak or now - 0.1), 0, 0.25)
				feedback.SetInflation(record, record.settings.inflation - elapsed * 35)
				record.lastLeak = now
			end
			if record.impactSeam and now >= (record.nextImpactSplit or 0) then
				local seam = record.impactSeam
				record.impactSeam = nil
				record.nextImpactSplit = now + 0.5
				api.Tearing.Split(record, seam)
			end
		end
	end
end
return feedback
