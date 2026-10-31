param(
    [Parameter(Mandatory = $true)][string]$SdkRoot,
    [Parameter(Mandatory = $true)][string]$BuildRoot,
    [string]$CMake = 'cmake',
    [ValidateRange(1, 64)][int]$Parallel = 8
)
$ErrorActionPreference = 'Stop'
$BuildRoot = [IO.Path]::GetFullPath($BuildRoot)
$SdkRoot = (Resolve-Path -LiteralPath $SdkRoot).Path
$packageDir = Join-Path $BuildRoot 'package'
New-Item -ItemType Directory -Path $packageDir -Force | Out-Null
$entries = @()
foreach ($variant in @('avx2', 'sse42', 'sse2')) {
    $variantDir = Join-Path $BuildRoot $variant
    & $CMake -S $PSScriptRoot -B $variantDir -G 'Visual Studio 17 2022' -A x64 "-DVOLT_GMOD_SDK=$SdkRoot" "-DVOLT_CPU_VARIANT=$variant"
    if ($LASTEXITCODE -ne 0) { throw "Configure failed: $variant" }
    $targets = @("vphysics_jolt_$variant")
    if ($variant -eq 'avx2') { $targets += 'vphysics' }
    & $CMake --build $variantDir --config Release --target @targets --parallel $Parallel
    if ($LASTEXITCODE -ne 0) { throw "Build failed: $variant" }
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
    configuration = 'MSVC v143 Release x64 /MT; SSE2 wrapper and three separate backends'
    files = $entries
}
$manifest | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $packageDir 'manifest.json') -Encoding utf8
Write-Output "Complete package: $packageDir"
