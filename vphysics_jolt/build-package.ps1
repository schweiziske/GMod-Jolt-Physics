param(
    [Parameter(Mandatory = $true)][string]$SdkRoot,
    [Parameter(Mandatory = $true)][string]$BuildRoot,
    [string]$CMake = 'cmake',
    [ValidateSet('Win32', 'x64')][string]$Architecture = 'x64',
    [ValidateSet('gmod', 'sdk2013')][string]$EngineBranch = 'gmod',
    [string]$SdkProfile = '',
    [string]$GeneratorInstance = '',
    [ValidateRange(1, 64)][int]$Parallel = 8
)
$ErrorActionPreference = 'Stop'
function Invoke-CMake([string[]]$Arguments) {
    # Windows PowerShell can treat native stderr warnings as terminating errors
    # when CI captures output. The process exit code determines build success.
    $ErrorActionPreference = 'Continue'
    & $CMake @Arguments
    if ($LASTEXITCODE -ne 0) { throw "CMake failed with exit code $LASTEXITCODE" }
}
$BuildRoot = [IO.Path]::GetFullPath($BuildRoot)
$SdkRoot = (Resolve-Path -LiteralPath $SdkRoot).Path
$packageDir = Join-Path $BuildRoot 'package'
New-Item -ItemType Directory -Path $packageDir -Force | Out-Null
$entries = @()
foreach ($variant in @('avx2', 'sse42', 'sse2')) {
    $variantDir = Join-Path $BuildRoot $variant
    $extra = @("-DVOLT_ENGINE_BRANCH=$EngineBranch")
    if ($SdkProfile) { $extra += "-DSOURCE_SDK_PROFILE=$SdkProfile" }
    if ($GeneratorInstance) { $extra += "-DCMAKE_GENERATOR_INSTANCE=$GeneratorInstance" }
    Invoke-CMake (@('-S', $PSScriptRoot, '-B', $variantDir, '-G', 'Visual Studio 17 2022', '-A', $Architecture,
        "-DVOLT_GMOD_SDK=$SdkRoot", "-DVOLT_CPU_VARIANT=$variant", '-DSDK_BUILD_TESTS=OFF', '-DSDK_WRITE_LEGACY_LIBS=OFF') + $extra)
    $targets = @("vphysics_jolt_$variant")
    if ($variant -eq 'avx2') { $targets += 'vphysics' }
    Invoke-CMake (@('--build', $variantDir, '--config', 'Release', '--target') + $targets + @('--parallel', "$Parallel"))
    foreach ($target in $targets) {
        foreach ($extension in @('dll', 'pdb')) {
            $source = Join-Path $variantDir "Release/$target.$extension"
            $destination = Join-Path $packageDir "$target.$extension"
            Copy-Item -LiteralPath $source -Destination $destination -Force
            $entries += [ordered]@{
                file = "$target.$extension"
                sha256 = (Get-FileHash -LiteralPath $destination -Algorithm SHA256).Hash
                bytes = (Get-Item -LiteralPath $destination).Length
            }
        }
    }
}
$repoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$manifest = [ordered]@{
    builtAt = (Get-Date -Format o)
    sourceRoot = $repoRoot
    mainHead = (& git -C $repoRoot rev-parse HEAD)
    joltHead = (& git -C (Join-Path $repoRoot 'joltphysics/src') rev-parse HEAD)
    mainStatus = @(& git -C $repoRoot status --short)
    joltStatus = @(& git -C (Join-Path $repoRoot 'joltphysics/src') status --short)
    sdkRoot = $SdkRoot
    configuration = "MSVC v143 Release $Architecture /MT; SSE2 wrapper and three separate backends"
    files = $entries
}
$manifest | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $packageDir 'manifest.json') -Encoding utf8
Write-Output "Complete package: $packageDir"
