local I = JoltI18n or include("jolt_i18n/init.lua")
if SERVER then
    AddCSLuaFile()
    AddCSLuaFile("jolt_gravity/client.lua")
end
include("jolt_gameplay/field_settings.lua")

JoltGravity = JoltGravity or {}
local G = JoltGravity
G.Fields = G.Fields or {}
G.State = G.State or { world = Vector(0, 0, -600), fields = {}, revision = -1 }

local function finite(n) return isnumber(n) and n == n and math.abs(n) <= 10000000 end
local function validVector(v) return isvector(v) and finite(v.x) and finite(v.y) and finite(v.z) end

-- Shared, read-only gravity sampling for prediction and presentation. The server's
-- physical bodies are integrated by C++, never by this Lua function.
function G.Sample(position)
    if SERVER and joltgravity and joltgravity.Available() then
        local value = joltgravity.Sample(position)
        if value then return value end
    end
    local state, selected = G.State, nil
    for _, field in ipairs(state.fields) do
        local d = position - field.center
        local inside = field.shape == 0 and d:LengthSqr() <= field.radius * field.radius
            or field.shape == 1 and math.abs(d.x) <= field.halfExtent.x
                and math.abs(d.y) <= field.halfExtent.y and math.abs(d.z) <= field.halfExtent.z
        if inside and (not selected or field.priority > selected.priority) then selected = field end
    end
    local gravity = state.world
    if selected then
        if selected.mode == 0 then gravity = selected.gravity
        else
            local d = selected.center - position
            local length = d:Length()
            gravity = length < 0.00001 and Vector(0, 0, 0)
                or d * ((selected.mode == 2 and -1 or 1) * selected.strength / length)
        end
    end
    return gravity
end

if SERVER then
    local gameplay = include("jolt_gameplay/server.lua")
    local native, why = gameplay.LoadFeature("gravity")
    if not native then
        G.LoadError = why
        ErrorNoHalt(I.Text("jolt.jolt_gravity_disabled") .. I.Resolve(why) .. "\n")
        return
    end
    G.LoadError = nil
    util.AddNetworkString("jolt_gravity_snapshot_v2")
    CreateConVar("vjolt_gravity_fields_per_player", "16", FCVAR_ARCHIVE, I.Text("jolt.maximum_gravity_regions_created_per_player_with_the_gravity_weapon"), 1, 128)

    function G.RestoreObjects(previous)
        for _, item in ipairs(previous or {}) do
            if IsValid(item.object) then joltgravity.Configure(item.object, item.config) end
        end
    end

    -- All physics bones are updated as one Lua transaction. Failure restores the
    -- preceding successful writes rather than leaving a half-modified ragdoll.
    function G.SetEntity(entity, gravity, factor)
        if not IsValid(entity) or (gravity ~= nil and not validVector(gravity))
            or (factor ~= nil and (not finite(factor) or math.abs(factor) > 1024)) then
            return nil, I.Phrase("jolt.invalid_entity_gravity_direction_or_multiplier")
        end
        if entity:IsPlayer() or entity:IsNPC() then
            return nil, I.Phrase("jolt.individual_gravity_changes_for_players_and_npcs_are_not_supported")
        end
        local previous = {}
        for bone = 0, entity:GetPhysicsObjectCount() - 1 do
            local object = entity:GetPhysicsObjectNum(bone)
            if IsValid(object) then
                local config, err = joltgravity.GetObject(object)
                if not config then return nil, err end
                previous[#previous + 1] = { object = object, config = config }
            end
        end
        if #previous == 0 then return nil, I.Phrase("jolt.the_selected_entity_has_no_editable_physics_objects") end
        for i, item in ipairs(previous) do
            local cfg = item.config
            local ok, err = joltgravity.Configure(item.object, {
                gravity = gravity, factor = factor or cfg.factor, enabled = cfg.enabled
            })
            if not ok then
                local changed = {}
                for j = 1, i - 1 do changed[j] = previous[j] end
                G.RestoreObjects(changed)
                return nil, err
            end
        end
        return previous
    end

    function G.AddField(player, config)
        G.Fields[player] = G.Fields[player] or {}
        local fields = G.Fields[player]
        for i = #fields, 1, -1 do if not fields[i]:IsValid() then table.remove(fields, i) end end
        local maximum = GetConVar("vjolt_gravity_fields_per_player"):GetInt()
        local allowed = hook.Run("PlayerCheckLimit", player, "jolt_gravityfields", #fields, maximum)
        if allowed == nil then allowed = #fields < maximum end
        if not allowed then
            player:LimitHit("jolt_gravityfields")
            return nil
        end
        local field, err = joltgravity.CreateField(config)
        if not field then return nil, err end
        fields[#fields + 1] = field
        return field
    end

    function G.RemoveLastField(player)
        local fields = G.Fields[player] or {}
        for i = #fields, 1, -1 do
            local field = table.remove(fields, i)
            if field:IsValid() then return field:Destroy() end
        end
        return false
    end

    local broadcastRevision = -1
    local function writeVector(vector)
        net.WriteFloat(vector.x); net.WriteFloat(vector.y); net.WriteFloat(vector.z)
    end
    local function snapshot(player)
        local world = joltgravity.GetWorld()
        local fields = joltgravity.GetFields()
        if not world or not fields then return end
        local revision = joltgravity.GetRevision()
        G.State = { world = world, fields = fields, revision = revision }
        -- At most 64 fields per reliable message, below GMod's message-size limit.
        -- Client swaps the complete revision atomically, never half an update.
        local chunks = math.max(1, math.ceil(#fields / 64))
        for chunk = 1, chunks do
            local first, last = (chunk - 1) * 64 + 1, math.min(chunk * 64, #fields)
            net.Start("jolt_gravity_snapshot_v2")
            net.WriteUInt(2, 8); net.WriteDouble(revision)
            writeVector(world)
            net.WriteUInt(chunk, 5); net.WriteUInt(chunks, 5); net.WriteUInt(math.max(0, last - first + 1), 7)
            for i = first, last do
                local f = fields[i]
                net.WriteString(f.id); net.WriteUInt(f.slot, 11)
                net.WriteUInt(f.shape, 1); net.WriteUInt(f.mode, 2); net.WriteInt(f.priority, 32)
                -- Bit-coordinate vectors can truncate large extents. Keep the
                -- native float components, including the full 32767 range.
                writeVector(f.center); writeVector(f.halfExtent); writeVector(f.gravity)
                net.WriteFloat(f.strength)
            end
            if IsValid(player) then net.Send(player) else net.Broadcast() end
        end
        if not IsValid(player) then broadcastRevision = revision end
    end
    G.SendSnapshot = snapshot
    timer.Create("JoltGravity.Snapshot", 0.1, 0, function()
        -- Sending the joining player a snapshot must not suppress the same
        -- revision's broadcast to existing players. Reload also sends v2 once.
        if joltgravity.GetRevision() ~= broadcastRevision then snapshot() end
    end)
    hook.Add("PlayerInitialSpawn", "JoltGravity.Snapshot", function(player)
        timer.Simple(1, function() if IsValid(player) then snapshot(player) end end)
    end)
    hook.Add("PlayerDisconnected", "JoltGravity.Fields", function(player)
        for _, field in ipairs(G.Fields[player] or {}) do field:Destroy() end
        G.Fields[player] = nil
    end)
    hook.Add("InitPostEntity", "JoltGravity.Baseline", function()
        G.OriginalWorld = joltgravity.GetWorld()
        snapshot()
    end)
    hook.Add("PostCleanupMap", "JoltGravity.Reset", function()
        joltgravity.Reset()
        if G.OriginalWorld then joltgravity.SetWorld(G.OriginalWorld) end
        G.Fields = {}; snapshot()
    end)
else
    I.RegisterAlias("SBoxLimit_jolt_gravityfields", "jolt.gravity_field_limit_reached")
    local function readVector()
        return Vector(net.ReadFloat(), net.ReadFloat(), net.ReadFloat())
    end
    local pending
    net.Receive("jolt_gravity_snapshot_v2", function()
        if net.ReadUInt(8) ~= 2 then return end
        local revision, world = net.ReadDouble(), readVector()
        local chunk, chunks, count = net.ReadUInt(5), net.ReadUInt(5), net.ReadUInt(7)
        if revision < G.State.revision or chunks < 1 or chunks > 16 or chunk < 1 or chunk > chunks or count > 64 then return end
        if not pending or pending.revision ~= revision or pending.expires < CurTime() then
            pending = { revision = revision, world = world, chunks = chunks, pieces = {}, expires = CurTime() + 5 }
        end
        if pending.chunks ~= chunks then return end
        local fields = {}
        for i = 1, count do
            local f = { id = net.ReadString(), slot = net.ReadUInt(11), shape = net.ReadUInt(1), mode = net.ReadUInt(2), priority = net.ReadInt(32) }
            f.center, f.halfExtent, f.gravity = readVector(), readVector(), readVector()
            f.radius, f.strength = f.halfExtent.x, net.ReadFloat()
            fields[i] = f
        end
        pending.pieces[chunk] = fields
        for i = 1, chunks do if not pending.pieces[i] then return end end
        local complete = {}
        for i = 1, chunks do for _, f in ipairs(pending.pieces[i]) do complete[#complete + 1] = f end end
        table.sort(complete, function(a, b) return a.slot < b.slot end)
        G.State = { world = pending.world, fields = complete, revision = revision }
        pending = nil
        hook.Run("JoltGravityChanged", revision)
    end)
    include("jolt_gravity/client.lua")
end
