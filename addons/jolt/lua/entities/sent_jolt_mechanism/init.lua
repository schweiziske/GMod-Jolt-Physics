AddCSLuaFile("shared.lua")
AddCSLuaFile("cl_init.lua")
include("shared.lua")

function ENT:Initialize()
	self:SetNoDraw(true)
	self:SetMoveType(MOVETYPE_NONE)
	self:SetSolid(SOLID_NONE)
end

function ENT:AttachMechanism(mechanism, members)
	self:DetachMembers()
	self.Mechanism = mechanism
	self.Members = members
	local first, second = members[1], members[2]
	self.Type = "JoltMechanism"
	self.Ent1, self.Ent2 = first.entity, second.entity
	self.Bone1, self.Bone2 = first.bone, second.bone
	self.pl = self:GetCreator()
	constraint.AddConstraintTable(first.entity, self, second.entity)
end

function ENT:DetachMembers()
	for _, member in ipairs(self.Members or {}) do
		if IsValid(member.entity) then
			member.entity:DontDeleteOnRemove(self)
			for i = #(member.entity.Constraints or {}), 1, -1 do
				if member.entity.Constraints[i] == self then
					table.remove(member.entity.Constraints, i)
				end
			end
		end
	end
	self.Members = nil
end

function ENT:HasValidMembers()
	if not self.Mechanism or not self.Mechanism:IsValid() or not self.Members then
		return false
	end
	for _, member in ipairs(self.Members) do
		if not IsValid(member.entity) or not IsValid(member.physics) then
			return false
		end
		if member.entity:GetPhysicsObjectNum(member.bone) ~= member.physics then
			return false
		end
	end
	return true
end

function ENT:SetMotor(speed, torque)
	if not self:HasValidMembers() then
		return nil, "机械组件或其中的物体已失效。"
	end
	return self.Mechanism:SetMotor(speed, torque)
end

function ENT:Think()
	if not self:HasValidMembers() then
		self:Remove()
		return
	end
	self:NextThink(CurTime() + 0.1)
	return true
end

function ENT:OnRemove()
	local mechanism = self.Mechanism
	self.Mechanism = nil
	if mechanism then
		mechanism:Destroy()
	end
	self:DetachMembers()
end
