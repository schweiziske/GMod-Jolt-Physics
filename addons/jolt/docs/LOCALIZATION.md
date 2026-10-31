# Localization

[Home](../../../README.md)

The five gameplay features (soft bodies, black holes, gravity fields, gears/racks and rails) come in English (`en`) and Simplified Chinese (`zh-CN`). They follow the game's `gmod_language`: `zh`, `zh-CN`, `zh_CN`, `zh-Hans` and `schinese` select Chinese; anything else, including `zh-TW`, falls back to English. Players joining a server need nothing extra, since the language catalogs are sent with the addon's other Lua files.

To switch, change the game language or run `gmod_language en` or `gmod_language zh-CN`. Open panels relabel themselves without losing slider values, combo-box choices, preset names or collapsed sections, and cached HUD text follows. There is no separate Jolt language setting: `jolt_language` is a client userinfo ConVar the addon keeps in sync with `gmod_language`, so the server can reply to each player in their own language.

## Fixing translations

The phrase files are generated from a source catalog that is not part of this repository:

| File | Contents |
| --- | --- |
| `addons/jolt/lua/jolt_i18n/en.lua`, `zh-CN.lua` | Phrase key → text, used by the Lua UI |
| `addons/jolt/lua/jolt_i18n/aliases.lua` | Native tool, cleanup and limit names (`tool.*`, `Cleanup_*`, `SBoxLimit_*`) → phrase keys |
| `addons/jolt/resource/localization/en/jolt.properties`, `zh-CN/jolt.properties` | The same phrases under their native names, read only when the addon is installed locally |

To fix a translation, open an issue, or a pull request that changes the phrase in `en.lua` or `zh-CN.lua` and the matching line in `jolt.properties`. The maintainers carry the change into the source catalog, which regenerates all of these files. Keys are `jolt.` followed by lowercase letters, digits and underscores; every key needs both languages. Keep format specifiers such as `%s` and `%d` identical in both languages, and keep leading and trailing spaces, since many phrases are joined to other text.

## Using `JoltI18n`

`JoltI18n` (`addons/jolt/lua/jolt_i18n/init.lua`) is shared between server and client.

| Function | Use |
| --- | --- |
| `Text(key, ...)` | Text in the current language (English on the server), with `string.format` arguments; for client UI drawn right away |
| `Token(key)` | A `{key}` placeholder resolved later; for stored labels and validation reasons from shared code |
| `Phrase(key)` | A token on the server, text on the client; for messages built in shared code |
| `Format(key, ...)` | Formatted text on the client; on the server, a token carrying the arguments, resolved per player |
| `Control(panel, method, token, ...)`, `Bind(control, method, token)`, `Choice(combo, token, data)` | Create or label VGUI controls so they relabel on a language change |
| `Chat(player, message)` | Resolve tokens in the player's language and print to their chat; use it wherever server text reaches a player |
| `UndoName(token)` | Turn a token into the phrase key to pass to `undo.Create` |
| `RegisterAlias(name, key)` | Map a native name (`tool.*`, `Cleanup_*`, `SBoxLimit_*`) to a phrase key |

The client hook `JoltLanguageChanged` runs after a language change; clear cached HUD strings there. Don't translate protocol IDs, enum values, model paths or preset data. Plain text and unknown tokens pass through unchanged, so error messages from the native modules stay in English.
