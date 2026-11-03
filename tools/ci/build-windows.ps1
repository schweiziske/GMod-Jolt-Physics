[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)]
    [ValidateSet('gmod-public-x86','gmod-public-x64','gmod-beta-x64','sdk2013-mp-x86','sdk2013-sp-x86')]
    [string]$Target,
    [Parameter(Mandatory=$true)][string]$SdkRoot,
    [Parameter(Mandatory=$true)][string]$OutputRoot,
    [string]$GmodCommon = '',
    [string]$CMake = 'cmake',
    [string]$GeneratorInstance = '',
    [ValidateRange(1,64)][int]$Parallel = 4
)
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$SdkRoot = (Resolve-Path -LiteralPath $SdkRoot).Path
$OutputRoot = [IO.Path]::GetFullPath($OutputRoot)
$isGmod = $Target.StartsWith('gmod-')
$is64 = $Target.EndsWith('-x64')
$arch = if ($is64) { 'x64' } else { 'Win32' }
$profile = switch ($Target) {
    'gmod-beta-x64' { 'gmod-beta' }
    'sdk2013-mp-x86' { 'sdk2013-mp' }
    'sdk2013-sp-x86' { 'sdk2013-sp' }
    default { 'gmod-public' }
}
$sdkSource = if ($isGmod) { "$SdkRoot/gmod/src" } else { "$SdkRoot/$profile/src" }
$engine = if ($isGmod) { 'gmod' } else { 'sdk2013' }
$binRelative = if ($is64) { 'bin/win64' } else { 'bin' }
$packageName = switch ($Target) {
    'gmod-public-x86' { 'GMod-Jolt-Physics-Public-Windows-x86' }
    'gmod-public-x64' { 'GMod-Jolt-Physics-Public-Windows-x64' }
    'gmod-beta-x64' { 'GMod-Jolt-Physics-Beta-x86-64-Branch-Windows-x64' }
    'sdk2013-mp-x86' { 'Jolt-SDK2013-MP-Windows-x86' }
    'sdk2013-sp-x86' { 'Jolt-SDK2013-SP-Windows-x86' }
}
$package = "$OutputRoot/package"
$logs = "$OutputRoot/logs"
$downloads = "$OutputRoot/downloads"
# Refuse stale packaging directories so old DLLs cannot enter a new artifact.
if ((Test-Path $package) -or (Test-Path $downloads)) {
    throw 'Choose a fresh OutputRoot (package/downloads already exists).'
}
New-Item -ItemType Directory "$package/$binRelative", $logs, $downloads -Force | Out-Null
if ($isGmod) { $GmodCommon = (Resolve-Path -LiteralPath $GmodCommon).Path }
$generator = @('-G','Visual Studio 17 2022','-A',$arch)
if ($GeneratorInstance) { $generator += "-DCMAKE_GENERATOR_INSTANCE=$GeneratorInstance" }
function Run-CMake([string[]]$Arguments, [string]$LogName) {
    $ErrorActionPreference = 'Continue'
    & $CMake @Arguments *> "$logs/$LogName.log"
    $code = $LASTEXITCODE
    if ($code -ne 0) {
        Get-Content "$logs/$LogName.log" -Tail 60
        throw "CMake failed ($code): $LogName"
    }
}
function Git-Head([string]$Path) {
    $head = & git -C $Path rev-parse HEAD
    if ($LASTEXITCODE -ne 0) { throw "Cannot identify Git revision: $Path" }
    return "$head".Trim()
}
$sourceHead = Git-Head $repo
$sdkHead = Git-Head $SdkRoot
$joltHead = Git-Head "$repo/joltphysics/src"
$commonHead = if ($isGmod) { Git-Head $GmodCommon } else { $null }

# Public and legacy consumers link archives from their SDK profile directories.
# Rebuild those libraries with this runner's toolchain before building backends.
# Beta's libraries are built by the backend's add_subdirectory instead.
if ($profile -ne 'gmod-beta') {
    Run-CMake (@('-S',$SdkRoot,'-B',"$OutputRoot/sdk") + $generator + @(
        "-DSOURCE_SDK_PROFILE=$profile", '-DSDK_BUILD_TESTS=OFF', '-DSDK_WRITE_LEGACY_LIBS=ON')) 'sdk-configure'
    Run-CMake @('--build',"$OutputRoot/sdk",'--config','Release','--target','tier1','mathlib','--parallel',"$Parallel") 'sdk-build'
}
# Keep using the existing backend builder and its flat DLL/PDB staging package.
$backendArguments = @{
    SdkRoot=$sdkSource; BuildRoot="$OutputRoot/backend"; CMake=$CMake;
    Architecture=$arch; EngineBranch=$engine; SdkProfile=$profile;
    GeneratorInstance=$GeneratorInstance; Parallel=$Parallel
}
& "$repo/vphysics_jolt/build-package.ps1" @backendArguments *> "$logs/backend.log"
Get-ChildItem "$OutputRoot/backend/package" -File |
    Where-Object { $_.Extension -in @('.dll','.pdb') } |
    Copy-Item -Destination "$package/$binRelative/"
if ($isGmod) {
    Run-CMake (@('-S',"$repo/gmsv_jolt",'-B',"$OutputRoot/modules") + $generator + @(
        "-DGMOD_COMMON=$GmodCommon", '-DVJOLT_BUILD_SDK_TESTS=OFF')) 'modules-configure'
    Run-CMake @('--build',"$OutputRoot/modules",'--config','Release','--parallel',"$Parallel") 'modules-build'
    New-Item -ItemType Directory "$package/garrysmod/lua/bin" -Force | Out-Null
    Get-ChildItem "$OutputRoot/modules/modules" -Recurse -File |
        Where-Object { $_.Name -like 'gmsv_jolt*' -and $_.Extension -in @('.dll','.pdb') } |
        Copy-Item -Destination "$package/garrysmod/lua/bin/"
}

# Verify architecture from PE headers without executing game binaries on CI.
$dlls = @(Get-ChildItem $package -Recurse -Filter '*.dll')
$expectedCount = if ($isGmod) { 7 } else { 4 }
if ($dlls.Count -ne $expectedCount) { throw "Expected $expectedCount DLLs, found $($dlls.Count)" }
$machine = if ($is64) { 0x8664 } else { 0x14c }
foreach ($dll in $dlls) {
    $bytes = [IO.File]::ReadAllBytes($dll.FullName)
    $pe = [BitConverter]::ToInt32($bytes,0x3c)
    if ([BitConverter]::ToUInt16($bytes,$pe+4) -ne $machine) { throw "Wrong architecture: $($dll.Name)" }
}
$files = @(Get-ChildItem $package -Recurse -File | ForEach-Object {
    [ordered]@{file=$_.FullName.Substring($package.Length+1).Replace('\','/');
        sha256=(Get-FileHash $_.FullName).Hash.ToLowerInvariant();bytes=$_.Length}
})
[ordered]@{
    target=$Target;architecture=$arch;profile=$profile;configuration='Release /MT';
    source=$sourceHead;jolt=$joltHead;sdk=$sdkHead;garrysmod_common=$commonHead;
    sourceStatus=@(& git -C $repo status --short --untracked-files=no);
    runUrl=if($env:GITHUB_ACTIONS){"$env:GITHUB_SERVER_URL/$env:GITHUB_REPOSITORY/actions/runs/$env:GITHUB_RUN_ID"}else{$null};
    files=$files
} | ConvertTo-Json -Depth 6 | Set-Content "$logs/manifest.json" -Encoding utf8
Compress-Archive "$package/*" "$downloads/$packageName.zip"
# v1.0.0 ships the architecture-independent Lua addon as its own ZIP.
if ($Target -eq 'gmod-public-x86') {
    $addon = "$OutputRoot/addon-package"
    New-Item -ItemType Directory "$addon/garrysmod/addons" -Force | Out-Null
    Copy-Item -LiteralPath "$repo/addons/jolt" -Destination "$addon/garrysmod/addons/jolt" -Recurse
    Copy-Item -LiteralPath "$repo/addons/jolt/lua/README.md" -Destination "$addon/README.md"
    Compress-Archive "$addon/*" "$downloads/GMod-Jolt-Physics-Lua-Addon.zip"
}
Get-ChildItem $downloads -Filter '*.zip' | ForEach-Object {
    "$((Get-FileHash $_.FullName).Hash.ToLowerInvariant())  $($_.Name)" |
        Set-Content "$($_.FullName).sha256" -Encoding ascii
}
Write-Output "Packaged $expectedCount DLLs for $Target in $downloads"
