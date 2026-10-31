local I = JoltI18n or include("jolt_i18n/init.lua")
TOOL.Category = "Jolt"
TOOL.Name = "#tool.jolt_gears.name"
TOOL.ClientConVar = {
	mode = "gear",
	ratio = "2",
	travel = "128",
	minposition = "-128",
	maxposition = "128",
	speed = "180",
	torque = "1000000",
}
TOOL.Information = {
	{name = "left", stage = 0},
	{name = "left_1", stage = 1},
	{name = "right"},
	{name = "reload"},
}

if SERVER then
	CreateConVar("sbox_maxjolt_mechanisms", "32", FCVAR_ARCHIVE, I.Text("jolt.maximum_jolt_mechanical_assemblies_per_player"), 0, 256)
	cleanup.Register("jolt_mechanisms")
end

if CLIENT then
	I.RegisterAlias("tool.jolt_gears.name", "jolt.gears_rack")
	I.RegisterAlias("tool.jolt_gears.desc", "jolt.connect_two_objects_with_fixed_rotation_axes_or_a_slide_rail")
	I.RegisterAlias("tool.jolt_gears.left", "jolt.select_the_first_gear_the_clicked_surface_normal_sets_the_rotation_a")
	I.RegisterAlias("tool.jolt_gears.left_1", "jolt.select_the_second_gear_or_rack_the_surface_normal_sets_the_rotation")
	I.RegisterAlias("tool.jolt_gears.right", "jolt.apply_motor_settings_to_your_most_recently_created_assembly")
	I.RegisterAlias("tool.jolt_gears.reload", "jolt.cancel_selection")
	I.RegisterAlias("Cleanup_jolt_mechanisms", "jolt.jolt_mechanical_assembly")
	I.RegisterAlias("Cleaned_jolt_mechanisms", "jolt.jolt_mechanical_assemblies_cleaned_up")
	I.RegisterAlias("SBoxLimit_jolt_mechanisms", "jolt.jolt_mechanical_assembly_limit_reached")
end

local gameplay = SERVER and include("jolt_gameplay/server.lua")

local function canModify(tool, trace, button)
	local player = tool:GetOwner()
	local entity = trace.Entity
	if not IsValid(entity) or entity:IsPlayer() or entity:IsNPC() or entity:IsVehicle() then
		return false
	end
	if entity:GetMoveType() ~= MOVETYPE_VPHYSICS then
		return false
	end
	if entity.CPPICanTool and not entity:CPPICanTool(player, "jolt_gears") then
		return false
	end
	return hook.Run("CanTool", player, trace, "jolt_gears", tool, button) ~= false
end

local function memberTrace(member)
	return {
		Hit = true,
		HitWorld = false,
		Entity = member.entity,
		PhysicsBone = member.bone,
		HitPos = member.physics:GetPos(),
		HitNormal = member.axis,
	}
end

function TOOL:GetMotorSettings()
	local speed = gameplay.ClampNumber(self:GetClientNumber("speed"), 180, -3600, 3600)
	local torque = gameplay.ClampNumber(self:GetClientNumber("torque"), 1000000, 0, 1000000000)
	return speed, torque
end

function TOOL:ReadMembers()
	local members = {}
	for index = 1, 2 do
		local entity = self:GetEnt(index)
		local physics = self:GetPhys(index)
		local bone = self:GetBone(index)
		if not IsValid(entity) or not IsValid(physics) or entity:GetPhysicsObjectNum(bone) ~= physics then
			return nil, I.Phrase("jolt.the_selected_object_was_removed_or_its_physics_object_was_replaced")
		end
		members[index] = {
			entity = entity,
			physics = physics,
			bone = bone,
			-- ToolObj stores the normal in body-local space between clicks.
			axis = self:GetNormal(index):GetNormalized(),
		}
	end
	if members[1].entity == members[2].entity then
		return nil, I.Phrase("jolt.select_two_different_objects")
	end
	return members
end

function TOOL:CreateNativeMechanism(constraints, members)
	local settings = {axis1 = members[1].axis, axis2 = members[2].axis}
	local first = members[1].physics
	local second = members[2].physics
	local mode = self:GetClientInfo("mode")
	if mode == "gear" then
		settings.ratio = gameplay.ClampNumber(self:GetClientNumber("ratio"), 2, -1000, 1000)
		return constraints.CreateGear(first, second, settings)
	end
	if mode == "rack" then
		settings.travelPerRevolution = gameplay.ClampNumber(self:GetClientNumber("travel"), 128, -4096, 4096)
		settings.minPosition = gameplay.ClampNumber(self:GetClientNumber("minposition"), -128, -4096, 0)
		settings.maxPosition = gameplay.ClampNumber(self:GetClientNumber("maxposition"), 128, 0, 4096)
		return constraints.CreateRackAndPinion(first, second, settings)
	end
	return nil, I.Phrase("jolt.choose_dual_gear_or_rack_and_pinion_mode")
end

function TOOL:CreateAssembly(constraints)
	local owner = self:GetOwner()
	local members, reason = self:ReadMembers()
	if not members then
		return gameplay.Fail(owner, reason)
	end
	for _, member in ipairs(members) do
		if not canModify(self, memberTrace(member), 1) then
			return gameplay.Fail(owner, I.Phrase("jolt.you_do_not_have_permission_to_edit_the_selected_objects"))
		end
	end
	if not owner:CheckLimit("jolt_mechanisms") or not owner:CheckLimit("constraints") then
		return false
	end

	local mechanism, detail = self:CreateNativeMechanism(constraints, members)
	if not mechanism then
		return gameplay.Fail(owner, detail)
	end
	local driven, motorError = mechanism:SetMotor(self:GetMotorSettings())
	if not driven then
		mechanism:Destroy()
		return gameplay.Fail(owner, motorError)
	end

	local controller = ents.Create("sent_jolt_mechanism")
	if not IsValid(controller) then
		mechanism:Destroy()
		return gameplay.Fail(owner, I.Phrase("jolt.unable_to_create_the_mechanical_assembly"))
	end
	controller:SetPos(members[1].physics:GetPos())
	controller:SetCreator(owner)
	controller:Spawn()
	if not IsValid(controller) then
		mechanism:Destroy()
		return false
	end
	controller:AttachMechanism(mechanism, members)
	owner:AddCount("constraints", controller)
	gameplay.TrackEntity(owner, controller, "jolt_mechanisms", I.Phrase("jolt.jolt_mechanical_assembly"))
	self.LastMechanism = controller
	return true
end

function TOOL:LeftClick(trace)
	if not IsValid(trace.Entity) or trace.HitWorld then
		return false
	end
	if CLIENT then
		return true
	end

	local owner = self:GetOwner()
	if not canModify(self, trace, 1) then
		return false
	end
	local physics = trace.Entity:GetPhysicsObjectNum(trace.PhysicsBone)
	if not IsValid(physics) then
		return false
	end
	local constraints, reason = gameplay.LoadFeature("constraints")
	if not constraints then
		return gameplay.Fail(owner, reason)
	end

	if self:NumObjects() == 0 then
		self:SetObject(1, trace.Entity, trace.HitPos, physics, trace.PhysicsBone, trace.HitNormal)
		self:SetStage(1)
		return true
	end
	self:SetObject(2, trace.Entity, trace.HitPos, physics, trace.PhysicsBone, trace.HitNormal)
	local created = self:CreateAssembly(constraints)
	self:ClearObjects()
	return created
end

function TOOL:RightClick()
	if CLIENT then
		return true
	end
	local owner = self:GetOwner()
	local controller = self.LastMechanism
	if not IsValid(controller) or controller:GetCreator() ~= owner or not controller:HasValidMembers() then
		return gameplay.Fail(owner, I.Phrase("jolt.create_a_mechanical_assembly_first"))
	end
	for _, member in ipairs(controller.Members) do
		if not canModify(self, memberTrace(member), 2) then
			return gameplay.Fail(owner, I.Phrase("jolt.you_do_not_have_permission_to_edit_this_assembly_s_objects"))
		end
	end
	local applied, reason = controller:SetMotor(self:GetMotorSettings())
	if not applied then
		return gameplay.Fail(owner, reason)
	end
	return true
end

function TOOL:Reload()
	self:ClearObjects()
	return true
end

function TOOL:Holster()
	self:ClearObjects()
	return true
end

function TOOL.BuildCPanel(panel)
	I.Control(panel, "Help", I.Token("jolt.freeze_and_position_two_objects_then_click_their_surfaces_each_rotat"))
	I.Control(panel, "Help", I.Token("jolt.axes_are_fixed_in_world_space_unfreeze_to_run_use_the_no_collide_too"))
	local mode = I.Control(panel, "ComboBox", I.Token("jolt.assembly_type"), "jolt_gears_mode")
	I.Choice(mode, I.Token("jolt.dual_gears"), "gear")
	I.Choice(mode, I.Token("jolt.rack_and_pinion"), "rack")
	I.Control(panel, "NumSlider", I.Token("jolt.tooth_ratio_second_first"), "jolt_gears_ratio", -20, 20, 2)
	I.Control(panel, "Help", I.Token("jolt.for_the_selected_axes_a_positive_ratio_turns_gears_in_opposite_direc"))
	I.Control(panel, "NumSlider", I.Token("jolt.rack_travel_per_revolution"), "jolt_gears_travel", -1024, 1024, 1)
	I.Control(panel, "NumSlider", I.Token("jolt.minimum_rack_position"), "jolt_gears_minposition", -4096, 0, 1)
	I.Control(panel, "NumSlider", I.Token("jolt.maximum_rack_position"), "jolt_gears_maxposition", 0, 4096, 1)
	I.Control(panel, "NumSlider", I.Token("jolt.motor_speed_degrees_second"), "jolt_gears_speed", -720, 720, 0)
	I.Control(panel, "NumSlider", I.Token("jolt.maximum_motor_torque"), "jolt_gears_torque", 0, 10000000, 0)
	I.Control(panel, "Help", I.Token("jolt.rmb_updates_the_latest_assembly_zero_speed_brakes_zero_torque_disabl"))
end
