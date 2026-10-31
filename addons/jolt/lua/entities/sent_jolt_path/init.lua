AddCSLuaFile("shared.lua")
AddCSLuaFile("cl_init.lua")
include("shared.lua")

function ENT:GetNativePath()
    local runtime = JoltPath.Runtime[self]
    return runtime and runtime.native
end
function ENT:Initialize()
    self:SetNoDraw(true)
    self:SetMoveType(MOVETYPE_NONE)
    self:SetSolid(SOLID_NONE)
end
function ENT:UpdateTransmitState() return TRANSMIT_ALWAYS end
function ENT:DetachMembers()
    for _, member in ipairs(self.Members or {}) do
        if IsValid(member) then
            member:DontDeleteOnRemove(self)
            for i = #(member.Constraints or {}), 1, -1 do
                if member.Constraints[i] == self then table.remove(member.Constraints, i) end
            end
        end
    end
    self.Members = nil
end
function ENT:AttachPath(handle, follower, bone, reference, referenceBone, data)
    local old = self:GetNativePath()
    self:DetachMembers()
    self.Data = data
    JoltPath.Runtime[self] = {native = handle, follower = follower:GetPhysicsObjectNum(bone),
        reference = reference and reference:GetPhysicsObjectNum(referenceBone) or nil}
    self.Follower, self.FollowerBone = follower, bone
    self.Reference, self.ReferenceBone = reference, referenceBone
    self.Members = {follower}
    if reference then self.Members[#self.Members + 1] = reference end
    self.ResumeMotor = nil
    self.PendingDupe = nil
    self.Type, self.Ent1, self.Ent2, self.Ent3 = "JoltPath", follower, self, reference or game.GetWorld()
    self.Bone1, self.Bone2, self.Bone3, self.pl = bone, 0, referenceBone, self:GetCreator()
    self.Constraints = {self}
    constraint.AddConstraintTable(follower, self, reference)
    if old then old:Destroy() end
    self:SetPos(JoltPath.WorldNodes(data)[1])
    self:NextThink(CurTime())
end
function ENT:HasValidMembers()
    local runtime = JoltPath.Runtime[self]
    if not runtime or not runtime.native:IsValid() or not IsValid(self.Follower) or not IsValid(runtime.follower)
        or self.Follower:GetPhysicsObjectNum(self.FollowerBone) ~= runtime.follower then return false end
    if self.Reference and (not IsValid(self.Reference) or not IsValid(runtime.reference)
        or self.Reference:GetPhysicsObjectNum(self.ReferenceBone) ~= runtime.reference) then return false end
    return true
end
function ENT:Think()
    if self.PendingDupe and not self:GetNativePath() and CurTime() < self.PendingDupe then self:NextThink(CurTime() + .1); return true end
    if not self:HasValidMembers() then self:Remove(); return end
    local native = self:GetNativePath()
    local state = native:GetState()
    if not state then self:NextThink(CurTime() + .1); return true end
    if self.Data.settings.mode == "pingpong" and not state.closed and state.enabled and state.motor.mode == "velocity" then
        local forward = state.motor.speed > 0
        local margin = math.min(.01, 1 / math.max(state.length, 1))
        if (forward and state.progress >= 1 - margin) or (not forward and state.motor.speed < 0 and state.progress <= margin) then
            local motor = table.Copy(state.motor)
            motor.speed = -motor.speed
            native:SetMotor(motor)
        end
    end
    self:SetProgress(state.progress)
    self:SetPathVelocity(state.velocity)
    self:SetMotorTarget(state.motor.target)
    self:SetMotorSpeed(state.motor.speed)
    self:SetMotorMode(JoltPath.EnumIndex(JoltPath.Modes, state.motor.mode) or 1)
    self:SetPathEnabled(state.enabled)
    self:SetPathPaused(self.ResumeMotor ~= nil and state.motor.mode == "position"
        and math.abs(state.motor.target - (self.HoldTarget or -1)) < .0001)
    self:NextThink(CurTime() + .1)
    return true
end
function ENT:PreEntityCopy()
    if not self:HasValidMembers() then return end
    local position, angles = JoltPath.Frame(self.Data)
    if not self.Reference then position = self.Data.nodes[1] end
    local nodes = {}
    for i, point in ipairs(JoltPath.WorldNodes(self.Data)) do nodes[i] = WorldToLocal(point, angle_zero, position, angles) end
    local settings = JoltPath.Settings(self.Data.settings)
    local state = self:GetNativePath():GetState()
    if state then
        for key, value in pairs(state.motor) do settings[key] = value end
        if self.Data.settings.mode == "pingpong" and state.motor.mode == "velocity" then settings.mode = "pingpong" end
    end
    self:SetPos(position); self:SetAngles(angles)
    self.PathBlueprint = {nodes = nodes, settings = settings}
end
function ENT:OnEntityCopyTableFinish(data)
    for key in pairs(data) do if key ~= "Pos" and key ~= "Angle" and key ~= "Class" then data[key] = nil end end
    data.PathBlueprint = self.PathBlueprint and table.Copy(self.PathBlueprint) or nil
end
function ENT:OnRemove()
    local handle = self:GetNativePath()
    if handle then handle:Destroy() end
    self:DetachMembers()
    if JoltPath then JoltPath.Tracks[self], JoltPath.Runtime[self] = nil, nil end
end
