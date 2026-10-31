local I = JoltI18n or include("jolt_i18n/init.lua")
TOOL.Category = "Jolt"
TOOL.Name = "#tool.jolt_path.name"
TOOL.ClientConVar = {closed = "0", snap = "1", visible = "1", smoothing = ".75", rotation = "follow",
    mode = "pingpong", speed = "128", target = "50", maxforce = "100000", friction = "0", frequency = "2", damping = "1",
    offset = "8", presetyaw = "0"}
TOOL.Information = {{name = "left"}, {name = "right"}, {name = "reload"}}
if CLIENT then
    I.RegisterAlias("tool.jolt_path.name", "jolt.path_track_pathconstraint")
    I.RegisterAlias("tool.jolt_path.desc", "jolt.draw_smooth_tracks_for_sliding_return_travel_or_closed_loop_motion")
    I.RegisterAlias("tool.jolt_path.left", "jolt.add_node_e_lmb_select_move_shift_lmb_insert_after_selected_node")
    I.RegisterAlias("tool.jolt_path.right", "jolt.attach_object_e_rmb_select_track_shift_rmb_set_reference_object_worl")
    I.RegisterAlias("tool.jolt_path.reload", "jolt.r_delete_selected_node_h_new_track_use_the_panel_to_apply_pause_reve")
    I.RegisterAlias("Cleanup_jolt_paths", "jolt.jolt_path_track")
    I.RegisterAlias("Cleaned_jolt_paths", "jolt.jolt_path_tracks_cleaned_up")
    I.RegisterAlias("SBoxLimit_jolt_paths", "jolt.jolt_path_track_limit_reached")
end
function TOOL:LeftClick(trace) if CLIENT then return true end return JoltPath.LeftClick(self, trace) end
function TOOL:RightClick(trace) if CLIENT then return true end return JoltPath.RightClick(self, trace) end
function TOOL:Reload() if CLIENT then return true end return JoltPath.Reload(self) end
function TOOL:Deploy()
    if SERVER then JoltPath.SyncDraft(self:GetOwner()) end
    return true
end
function TOOL:Holster()
    if SERVER then
        local player = self:GetOwner()
        if IsValid(player) then JoltPath.GetDraft(player).moving = false; JoltPath.SyncDraft(player) end
    end
    return true
end
function TOOL:DrawHUD() if JoltPath.DrawHUD then JoltPath.DrawHUD() end end
function TOOL.BuildCPanel(panel) JoltPath.BuildPanel(panel) end
