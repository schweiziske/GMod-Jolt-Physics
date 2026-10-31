-- One H press invokes the secondary action of the currently equipped Jolt tool.
if SERVER then
    AddCSLuaFile()
    util.AddNetworkString("jolt_shortcut_h_v1")
    net.Receive("jolt_shortcut_h_v1", function(length, owner)
        if length ~= MAX_EDICT_BITS or not IsValid(owner) or not owner:IsPlayer() or not owner:Alive()
            or owner:InVehicle() then return end
        local weapon = net.ReadEntity()
        if not IsValid(weapon) or weapon:GetOwner() ~= owner or owner:GetActiveWeapon() ~= weapon then return end
        local class = weapon:GetClass()
        if class ~= "weapon_jolt_gravity" then return end
        local now = CurTime()
        if now < (owner.JoltNextHAction or 0) then return end
        owner.JoltNextHAction = now + .3
        weapon:SetBoxField(not weapon:GetBoxField())
    end)
else
    -- Poll the physical key so this also works in singleplayer, where predicted
    -- PlayerButtonDown hooks are not dispatched to the client in the same way.
    local held = input.IsKeyDown(KEY_H)
    hook.Add("Think", "Jolt.ShortcutH", function()
        local down = input.IsKeyDown(KEY_H)
        local pressed = down and not held
        held = down -- Consume presses in UI too; closing it must not replay H.
        if not pressed then return end
        local owner = LocalPlayer()
        if not IsValid(owner) or not owner:Alive() or owner:InVehicle() or owner:IsTyping()
            or not system.HasFocus() or gui.IsGameUIVisible() or gui.IsConsoleVisible()
            or vgui.CursorVisible() or IsValid(vgui.GetKeyboardFocus()) then return end
        local weapon = owner:GetActiveWeapon()
        if not IsValid(weapon) or weapon:GetOwner() ~= owner then return end
        local class = weapon:GetClass()
        if class == "gmod_tool" then
            if weapon:GetMode() == "jolt_path" and JoltPath and JoltPath.SendAction then
                JoltPath.SendAction("clear") -- Reuse the tool's pending/ack flow.
            end
        elseif class == "weapon_jolt_gravity" then
            net.Start("jolt_shortcut_h_v1")
            net.WriteEntity(weapon)
            net.SendToServer()
        end
    end)
end
