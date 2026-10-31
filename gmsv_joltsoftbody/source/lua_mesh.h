#pragma once

#include <GarrysMod/Lua/Interface.h>
#include <vjolt/softbody.h>
#include <vector>

namespace JoltSoftBodyLua
{
struct CreationArguments
{
	int32_t environment = 0;
	VJoltSoftBodySettings settings;
	std::vector<VJoltSoftBodyVertex> vertices;
	std::vector<VJoltSoftBodyTriangle> triangles;

	VJoltSoftBodyMesh Mesh() const;
};

bool ReadNumber( GarrysMod::Lua::ILua *lua, int index, float &value );
bool ReadInteger( GarrysMod::Lua::ILua *lua, int index, uint32_t minimum, uint32_t maximum, uint32_t &value );
bool ReadVector( GarrysMod::Lua::ILua *lua, int index, VJoltSoftBodyVector &value );
bool ReadCreationArguments( GarrysMod::Lua::ILua *lua, CreationArguments &arguments, const char *&error );
const char *CreationError( VJoltSoftBodyError error );
} // namespace JoltSoftBodyLua
