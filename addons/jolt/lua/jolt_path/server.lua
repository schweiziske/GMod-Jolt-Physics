local I = JoltI18n or include("jolt_i18n/init.lua")
local P = JoltPath
local gameplay = include("jolt_gameplay/server.lua")
P.Tracks = P.Tracks or {}
P.Runtime = P.Runtime or {}
P.Drafts = P.Drafts or setmetatable({}, {__mode = "k"})
P.RequestTimes = P.RequestTimes or setmetatable({}, {__mode = "k"})
CreateConVar("sbox_maxjolt_paths", "8", FCVAR_ARCHIVE, I.Text("jolt.maximum_jolt_path_tracks_per_player"), 0, 64)
cleanup.Register("jolt_paths")
for _, name in ipairs({"jolt_path_data", "jolt_path_draft", "jolt_path_action", "jolt_path_request"}) do util.AddNetworkString(name) end

function P.ActiveTool(player)
    if not IsValid(player) or not player:Alive() then return end
    local weapon = player:GetActiveWeapon()
    if not IsValid(weapon) or weapon:GetClass() ~= "gmod_tool" or weapon:GetMode() ~= "jolt_path" then return end
    return weapon:GetToolObject("jolt_path")
end
function P.GetDraft(player)
    if not P.Drafts[player] then
        P.Drafts[player] = {nodes = {}, settings = P.Settings(), referenceBone = 0, node = 0}
    end
    return P.Drafts[player]
end
function P.ToolSettings(tool)
    local input = {mode = tool:GetClientInfo("mode"), rotation = tool:GetClientInfo("rotation")}
    for _, key in ipairs(P.BoolFields) do input[key] = tool:GetClientNumber(key) ~= 0 end
    for _, field in ipairs(P.NumberFields) do input[field[1]] = tool:GetClientNumber(string.lower(field[1])) end
    input.target = input.target / 100
    return P.Settings(input)
end
function P.SyncDraft(player, applySettings)
    if not IsValid(player) then return end
    local draft = P.GetDraft(player)
    net.Start("jolt_path_draft")
    P.WriteData(draft)
    net.WriteEntity(IsValid(draft.selected) and draft.selected or NULL)
    net.WriteUInt(draft.node or 0, 6)
    net.WriteBool(draft.moving == true)
    net.WriteBool(applySettings == true)
    net.Send(player)
    local tool = P.ActiveTool(player)
    if tool then tool:SetStage(#draft.nodes >= 2 and 1 or 0) end
end
function P.SendTrack(controller, player)
    if not IsValid(controller) or not controller.Data then return end
    net.Start("jolt_path_data")
    net.WriteEntity(controller)
    P.WriteData(controller.Data)
    if IsValid(player) then net.Send(player) else net.Broadcast() end
end
function P.CanModify(player, entity, bone, button)
    if not IsValid(player) or not IsValid(entity) or entity:IsPlayer() or entity:IsNPC() or entity:IsVehicle()
        or entity:GetMoveType() ~= MOVETYPE_VPHYSICS or not P.Finite(bone) or bone % 1 ~= 0 or bone < 0 or bone > 255 then return false end
    local physics = entity:GetPhysicsObjectNum(bone)
    if not IsValid(physics) then return false end
    if entity.CPPICanTool and entity:CPPICanTool(player, "jolt_path") == false then return false end
    local trace = {Hit = true, HitWorld = false, Entity = entity, PhysicsBone = bone,
        HitPos = physics:GetPos(), HitNormal = Vector(0, 0, 1)}
    return hook.Run("CanTool", player, trace, "jolt_path", P.ActiveTool(player), button or 1) ~= false
end
function P.CanEdit(player, controller)
    if not IsValid(controller) or controller:GetClass() ~= "sent_jolt_path" or controller:GetCreator() ~= player
        or not controller:HasValidMembers() then return false end
    if not P.CanModify(player, controller.Follower, controller.FollowerBone) then return false end
    if controller.Reference and not P.CanModify(player, controller.Reference, controller.ReferenceBone) then return false end
    return true
end
function P.FindTrack(player, trace)
    if IsValid(trace.Entity) and trace.Entity:GetClass() == "sent_jolt_path" and P.CanEdit(player, trace.Entity) then return trace.Entity end
    for controller in pairs(P.Tracks) do
        if IsValid(controller) and controller.Follower == trace.Entity and controller.FollowerBone == (trace.PhysicsBone or 0)
            and P.CanEdit(player, controller) then return controller end
    end
end
local function memberFree(entity, bone, except)
    for controller in pairs(P.Tracks) do
        if IsValid(controller) and controller ~= except and controller.Follower == entity and controller.FollowerBone == bone then return false end
    end
    return true
end
local function loadNative()
    local native, reason = gameplay.LoadFeature("constraints")
    if not native then return nil, reason end
    if not native.PathsAvailable or not native.PathsAvailable() then
        return nil, I.Phrase("jolt.requires_a_physics_backend_with_pathconstraint_and_gmsv_joltconstrai")
    end
    return native
end
local function prepare(player, follower, bone, data, existing)
    local settings, reason = P.Settings(data.settings)
    if not settings then return nil, reason end
    if not P.CanModify(player, follower, bone) then return nil, I.Phrase("jolt.no_permission_to_edit_this_object_or_it_is_not_a_regular_physics_obj") end
    if not memberFree(follower, bone, existing) then return nil, I.Phrase("jolt.this_physics_object_already_has_a_track_select_it_with_e_rmb_to_edit") end
    if existing and not P.CanEdit(player, existing) then return nil, I.Phrase("jolt.the_selected_track_is_invalid_or_you_do_not_have_edit_permission") end
    if existing then
        local state = existing:GetNativePath():GetState()
        if not state or not state.enabled then return nil, I.Phrase("jolt.cannot_rebuild_a_track_during_rewind") end
    end
    local reference = IsValid(data.reference) and data.reference or nil
    local referenceBone = data.referenceBone or 0
    if data.reference ~= nil and not IsValid(data.reference) then return nil, I.Phrase("jolt.the_reference_object_was_removed") end
    if reference and (reference == follower or not P.CanModify(player, reference, referenceBone)) then return nil, I.Phrase("jolt.the_reference_must_be_another_object_you_have_permission_to_edit") end
    local valid, nodeError = P.ValidateNodes(data.nodes, settings.closed, false)
    if not valid then return nil, nodeError end
    local nodes = P.WorldNodes(data)
    for _, node in ipairs(nodes) do if not P.ValidVector(node, P.MaxCoordinate) then return nil, I.Phrase("jolt.the_track_exceeds_the_world_coordinate_limits") end end
    local curve, curveError = P.BuildCurve(nodes, settings)
    if not curve then return nil, curveError end
    if not existing and (not player:CheckLimit("jolt_paths") or not player:CheckLimit("constraints")) then return nil end
    local native, nativeError = loadNative()
    if not native then return nil, nativeError end
    return {native = native, physics = follower:GetPhysicsObjectNum(bone), reference = reference,
        referencePhysics = reference and reference:GetPhysicsObjectNum(referenceBone) or nil,
        nodes = nodes, settings = settings, referenceBone = referenceBone}
end
function P.CreateTrack(player, follower, bone, data, existing, duplicating, pending)
    if pending and (not IsValid(pending) or pending:GetClass() ~= "sent_jolt_path" or pending:GetCreator() ~= player
        or not pending.PendingDupe or pending:GetNativePath()) then return nil, I.Phrase("jolt.invalid_track_duplication_controller_entity") end
    local args, reason = prepare(player, follower, bone, data, existing)
    if not args then return nil, reason end
    local originalPosition, originalAngles = args.physics:GetPos(), args.physics:GetAngles()
    local handle, detail = args.native.CreatePath(args.physics, args.nodes, P.NativeSettings(args.settings), args.referencePhysics)
    if not handle then return nil, detail end
    local controller = existing or pending
    if not IsValid(controller) then
        controller = ents.Create("sent_jolt_path")
        if IsValid(controller) then
            controller:SetPos(args.nodes[1]); controller:SetCreator(player); controller:Spawn()
        end
        if not IsValid(controller) then
            handle:Destroy()
            if IsValid(args.physics) then args.physics:SetPos(originalPosition); args.physics:SetAngles(originalAngles) end
            return nil, I.Phrase("jolt.unable_to_create_the_track_controller_entity")
        end
    end
    controller:AttachPath(handle, follower, bone, args.reference, args.referenceBone,
        {nodes = P.CopyNodes(data.nodes), settings = args.settings, reference = args.reference, referenceBone = args.referenceBone})
    P.Tracks[controller] = true
    if not existing then
        player:AddCount("jolt_paths", controller)
        player:AddCleanup("jolt_paths", controller)
        if not duplicating then
            player:AddCount("constraints", controller)
            undo.Create("jolt.jolt_path_track"); undo.AddEntity(controller); undo.SetPlayer(player); undo.Finish()
        end
    end
    P.SendTrack(controller)
    return controller
end
function P.SelectTrack(player, controller)
    if not P.CanEdit(player, controller) then return false, I.Phrase("jolt.aim_at_an_object_on_your_own_track") end
    local data = controller.Data
    local settings = P.Settings(data.settings)
    local state = controller:GetNativePath():GetState()
    if state then
        for key, value in pairs(state.motor) do settings[key] = value end
        if data.settings.mode == "pingpong" and state.motor.mode == "velocity" then settings.mode = "pingpong" end
    end
    P.Drafts[player] = {nodes = P.CopyNodes(data.nodes), reference = data.reference,
        referenceBone = data.referenceBone, settings = settings, selected = controller, node = 0}
    P.SyncDraft(player, true)
    return true
end
local function validTrace(player, trace)
    return trace.Hit and not trace.HitSky and P.ValidVector(trace.HitPos, P.MaxCoordinate)
        and trace.HitPos:DistToSqr(player:GetShootPos()) <= 32767 * 32767
end
local function localPoint(draft, point)
    local position, angles = P.Frame(draft)
    return WorldToLocal(point, angle_zero, position, angles)
end
function P.LeftClick(tool, trace)
    local player = tool:GetOwner()
    if not validTrace(player, trace) then return false end
    local draft = P.GetDraft(player)
    local settings, reason = P.ToolSettings(tool)
    if not settings then return gameplay.Fail(player, reason) end
    draft.settings = settings
    local point = trace.HitPos + trace.HitNormal * math.Clamp(tool:GetClientNumber("offset"), -1024, 1024)
    if not P.ValidVector(point, P.MaxCoordinate) then return false end
    if player:KeyDown(IN_USE) then
        if draft.moving and draft.node > 0 and draft.nodes[draft.node] then
            draft.nodes[draft.node] = localPoint(draft, point); draft.moving = false
        else
            local best, nearest = 32 * 32
            for i, node in ipairs(P.WorldNodes(draft)) do
                local distance = node:DistToSqr(trace.HitPos)
                if distance < best then nearest, best = i, distance end
            end
            if not nearest then return gameplay.Fail(player, I.Phrase("jolt.aim_near_a_node_or_select_one_in_the_panel_s_node_list")) end
            draft.node, draft.moving = nearest, true
        end
    else
        if #draft.nodes >= P.MaxNodes then return gameplay.Fail(player, I.Phrase("jolt.a_track_can_have_at_most_32_nodes")) end
        local index = player:KeyDown(IN_SPEED) and draft.node > 0 and draft.node + 1 or #draft.nodes + 1
        local added = localPoint(draft, point)
        table.insert(draft.nodes, index, added); draft.node = index; draft.moving = false
    end
    P.SyncDraft(player)
    return true
end
function P.RightClick(tool, trace)
    local player, draft = tool:GetOwner(), P.GetDraft(tool:GetOwner())
    if not validTrace(player, trace) then return false end
    if player:KeyDown(IN_USE) then
        local controller = P.FindTrack(player, trace)
        if not controller then return gameplay.Fail(player, I.Phrase("jolt.aim_at_the_object_attached_to_your_own_track")) end
        return P.SelectTrack(player, controller)
    end
    if player:KeyDown(IN_SPEED) then
        local reference, bone = nil, 0
        if not trace.HitWorld then
            if not P.CanModify(player, trace.Entity, trace.PhysicsBone or 0, 2) then return false end
            reference, bone = trace.Entity, trace.PhysicsBone or 0
        end
        local worldNodes = P.WorldNodes(draft)
        draft.reference, draft.referenceBone = reference, bone
        for i, node in ipairs(worldNodes) do draft.nodes[i] = localPoint(draft, node) end
        P.SyncDraft(player)
        return true
    end
    if trace.HitWorld then return gameplay.Fail(player, I.Phrase("jolt.rmb_must_target_the_physics_object_to_attach")) end
    local settings, error = P.ToolSettings(tool)
    if not settings then return gameplay.Fail(player, error) end
    draft.settings = settings
    local existing = P.FindTrack(player, trace)
    if existing and existing ~= draft.selected then return gameplay.Fail(player, I.Phrase("jolt.this_object_already_has_a_track_select_it_with_e_rmb_first")) end
    local controller, reason = P.CreateTrack(player, trace.Entity, trace.PhysicsBone or 0, draft, existing)
    if not controller then return gameplay.Fail(player, reason) end
    draft.selected, draft.moving = controller, false
    P.SyncDraft(player)
    return true
end
local function clear(player)
    local previous = P.GetDraft(player)
    P.Drafts[player] = {nodes = {}, settings = previous.settings, node = 0, referenceBone = 0}
end
function P.RemoveNode(player)
    local draft = P.GetDraft(player)
    if #draft.nodes > 0 then
        local index = draft.node > 0 and draft.node or #draft.nodes
        table.remove(draft.nodes, index); draft.node = math.min(index, #draft.nodes)
    end
    draft.moving = false
end
function P.Reload(tool)
    local player = tool:GetOwner()
    if player:KeyDown(IN_USE) then return false end
    P.RemoveNode(player)
    P.SyncDraft(player)
    return true
end
local function motorSettings(controller, settings)
    local updated = P.Settings(controller.Data.settings)
    for _, field in ipairs(P.NumberFields) do if field[1] ~= "smoothing" then updated[field[1]] = settings[field[1]] end end
    updated.mode, updated.visible = settings.mode, settings.visible
    return updated
end
function P.Control(player, action, settings)
    local controller = P.GetDraft(player).selected
    if not P.CanEdit(player, controller) then return false, I.Phrase("jolt.select_your_own_track_with_e_rmb_first") end
    local state = controller:GetNativePath():GetState()
    if not state or not state.enabled then return false, I.Phrase("jolt.track_is_rewinding_or_invalid_please_try_again_later") end
    if action == "delete" then controller:Remove(); P.GetDraft(player).selected = nil; return true end
    if action == "apply" then
        P.GetDraft(player).settings = settings
        return P.CreateTrack(player, controller.Follower, controller.FollowerBone, P.GetDraft(player), controller)
    end
    local motor = state.motor
    local resume, holdTarget = controller.ResumeMotor, controller.HoldTarget
    if action == "motor" then
        motor = P.NativeSettings(settings)
    elseif action == "pause" then
        if resume and motor.mode == "position" and math.abs(motor.target - (holdTarget or -1)) < .0001 then
            motor = resume; resume = nil
        else
            if motor.maxForce <= 0 then return false, I.Phrase("jolt.holding_position_requires_motor_force_greater_than_0") end
            resume = table.Copy(motor)
            motor = table.Copy(motor); motor.mode, motor.target = "position", state.progress
            holdTarget = motor.target
        end
    elseif action == "reverse" then
        motor.speed = -motor.speed
        if motor.mode == "position" then motor.target = 1 - motor.target end
        resume = nil
    end
    local ok, reason = controller:GetNativePath():SetMotor(motor)
    if not ok then return false, reason end
    controller.ResumeMotor, controller.HoldTarget = resume, holdTarget
    if action == "motor" then
        controller.Data.settings = motorSettings(controller, settings); controller.ResumeMotor = nil
    elseif action == "reverse" then
        controller.Data.settings.speed, controller.Data.settings.target = motor.speed, motor.target
    end
    P.SendTrack(controller)
    return true
end
function P.Action(player, action, value)
    local tool = P.ActiveTool(player)
    if not tool then return false end
    local draft = P.GetDraft(player)
    local settings, reason = P.ToolSettings(tool)
    if not settings then return gameplay.Fail(player, reason) end
    draft.settings = settings
    local ok, detail = true
    if action == "clear" then clear(player)
    elseif action == "remove_node" then P.RemoveNode(player)
    elseif action == "select_node" then
        if not P.Finite(value) or value % 1 ~= 0 or not draft.nodes[value] then return false end
        draft.node, draft.moving = value, true
    elseif action == "set_node" then
        if not value or not draft.nodes[value.index] or not P.ValidVector(value.position, P.MaxLocalCoordinate) then return false end
        draft.nodes[value.index] = value.position; draft.node = value.index; draft.moving = false
    elseif action == "move_up" or action == "move_down" then
        local other = draft.node + (action == "move_up" and -1 or 1)
        if draft.nodes[draft.node] and draft.nodes[other] then
            draft.nodes[draft.node], draft.nodes[other] = draft.nodes[other], draft.nodes[draft.node]; draft.node = other
        end
    elseif action == "import" then
        local trace = player:GetEyeTrace()
        if not value or not validTrace(player, trace) then return false end
        local yaw = math.Clamp(tool:GetClientNumber("presetyaw"), -180, 180)
        if not P.Finite(yaw) then return false end
        local imported = {nodes = {}, referenceBone = 0, settings = value.settings, node = 0}
        local angles = Angle(0, player:EyeAngles().y + yaw, 0)
        for i, v in ipairs(value.nodes) do
            local point = LocalToWorld(v, angle_zero, trace.HitPos, angles)
            if not P.ValidVector(point, P.MaxCoordinate) then return gameplay.Fail(player, I.Phrase("jolt.the_load_position_exceeds_the_map_coordinate_limits")) end
            imported.nodes[i] = point
        end
        P.Drafts[player] = imported; P.SyncDraft(player, true); return true
    elseif action ~= "sync" then ok, detail = P.Control(player, action, settings) end
    if not ok then gameplay.Fail(player, detail) end
    P.SyncDraft(player)
    return ok
end

net.Receive("jolt_path_action", function(length, player)
    if length > 16384 or not P.ActiveTool(player) then return end
    local now = CurTime()
    if now < (P.RequestTimes[player] or 0) then return end
    P.RequestTimes[player] = now + .05
    local action = P.Actions[net.ReadUInt(4) + 1]
    if not action then return end
    local value
    if action == "select_node" then value = net.ReadUInt(6)
    elseif action == "set_node" then value = {index = net.ReadUInt(6), position = P.ReadVector()}
    elseif action == "import" then value = P.ReadData(); if not value then return end end
    P.Action(player, action, value)
end)
local requests = setmetatable({}, {__mode = "k"})
net.Receive("jolt_path_request", function(length, player)
    if length > 32 or not IsValid(player) then return end
    local now = CurTime()
    local bucket = requests[player]
    if not bucket or now - bucket.time > 1 then bucket = {time = now, count = 0}; requests[player] = bucket end
    if bucket.count >= 32 then return end
    bucket.count = bucket.count + 1
    local entity = net.ReadEntity()
    if IsValid(entity) and entity:GetClass() == "sent_jolt_path" then P.SendTrack(entity, player) end
end)

-- A track is also the invisible, non-physical blueprint origin in the standard
-- constraint graph. The stock duplicator transforms this entity's Pos/Angle,
-- then the constraint factory attaches it to the newly created members. This
-- works without overriding Entity:GetTable or callbacks on somebody else's prop.
duplicator.RegisterEntityClass("sent_jolt_path", function(player, position, angles, blueprint)
    if not IsValid(player) or not P.ValidVector(position, P.MaxCoordinate) or not isangle(angles)
        or not P.Finite(angles.p) or not P.Finite(angles.y) or not P.Finite(angles.r)
        or type(blueprint) ~= "table" then return end
    local settings = P.Settings(blueprint.settings)
    if not settings or not P.ValidateNodes(blueprint.nodes, settings.closed, false)
        or not player:CheckLimit("jolt_paths") or not player:CheckLimit("constraints") then return end
    local entity = ents.Create("sent_jolt_path")
    if not IsValid(entity) then return end
    entity.PathBlueprint = {nodes = P.CopyNodes(blueprint.nodes), settings = settings}
    entity.PendingDupe = CurTime() + 2
    entity:SetPos(position); entity:SetAngles(angles); entity:SetCreator(player); entity:Spawn()
    return entity
end, "Pos", "Angle", "PathBlueprint")
duplicator.RegisterConstraint("JoltPath", function(follower, controller, reference, bone, referenceBone, player)
    if not IsValid(player) or not IsValid(follower) or not IsValid(controller) or controller:GetClass() ~= "sent_jolt_path"
        or controller:GetCreator() ~= player or not controller.PendingDupe or controller:GetNativePath() then return end
    local blueprint = controller.PathBlueprint
    if not blueprint then return end
    local config = P.Settings(blueprint.settings)
    if not config then controller:Remove(); return end
    local worldNodes = {}
    for i, point in ipairs(blueprint.nodes) do worldNodes[i] = LocalToWorld(point, angle_zero, controller:GetPos(), controller:GetAngles()) end
    local referenceEntity = IsValid(reference) and not reference:IsWorld() and reference or nil
    local data = {reference = referenceEntity, referenceBone = referenceBone or 0, nodes = worldNodes, settings = config}
    if referenceEntity then
        if not P.CanModify(player, referenceEntity, referenceBone) then controller:Remove(); return end
        for i, point in ipairs(worldNodes) do data.nodes[i] = localPoint(data, point) end
    end
    local snap = config.snap
    config.snap = false -- Pasted members already have their saved pose.
    local result = P.CreateTrack(player, follower, bone, data, nil, true, controller)
    if not result then controller:Remove(); return end
    result.Data.settings.snap = snap
    P.SendTrack(result)
    return result
end, "Ent1", "Ent2", "Ent3", "Bone1", "Bone3", "pl")

hook.Add("PlayerDisconnected", "JoltPathCleanup", function(player)
    P.Drafts[player], P.RequestTimes[player], requests[player] = nil, nil, nil
    for entity in pairs(P.Tracks) do if IsValid(entity) and entity:GetCreator() == player then entity:Remove() end end
end)
hook.Add("PostCleanupMap", "JoltPathDraftCleanup", function()
    for player in pairs(P.Drafts) do clear(player); P.SyncDraft(player) end
end)
