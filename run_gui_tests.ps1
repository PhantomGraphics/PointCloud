<#
.SYNOPSIS
    Build and run PointCloudView JSON scenario tests.

.PARAMETER Scenario
    Specific scenario name to run (e.g. "generate", "file", "pipeline").
    Omit to run all scenarios.

.PARAMETER Build
    Build the project with MSBuild before running.

.PARAMETER Configuration
    Build configuration (Debug / Release). Default: Debug.

.EXAMPLE
    .\run_gui_tests.ps1

.EXAMPLE
    .\run_gui_tests.ps1 -Scenario pipeline

.EXAMPLE
    .\run_gui_tests.ps1 -Build
#>
param(
    [string] $Scenario      = "",
    [switch] $Build,
    [ValidateSet("Debug", "Release")]
    [string] $Configuration = "Debug"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$repoRoot  = Split-Path -Parent $PSScriptRoot
$vcxproj   = Join-Path $PSScriptRoot "PointCloudView\PointCloudView.vcxproj"
$exePath   = Join-Path $PSScriptRoot "PointCloudView\x64\$Configuration\PointCloudView.exe"
$scenDir   = Join-Path $PSScriptRoot "PointCloudView\scenarios"

$allScenarios = @("generate", "file", "pipeline", "filter", "fit", "cluster", "normals", "realworld")

# ---------------------------------------------------------------------------
# Locate MSBuild
# ---------------------------------------------------------------------------
function Find-MSBuild {
    $cmd = Get-Command msbuild -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }

    $candidates = @(
        "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe",
        "C:\Program Files\Microsoft Visual Studio\2022\Professional\MSBuild\Current\Bin\MSBuild.exe",
        "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\MSBuild\Current\Bin\MSBuild.exe"
    )
    $found = $candidates | Where-Object { Test-Path $_ } | Select-Object -First 1
    if (-not $found) {
        Write-Error "MSBuild not found. Run from a Visual Studio Developer Command Prompt or omit -Build."
        exit 1
    }
    return $found
}

# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
if ($Build) {
    $msbuild = Find-MSBuild
    Write-Host "=== Build: PointCloudView $Configuration|x64 ===" -ForegroundColor Cyan
    & $msbuild $vcxproj /p:Configuration=$Configuration /p:Platform=x64 /m /nologo /v:minimal
    if ($LASTEXITCODE -ne 0) {
        Write-Error "Build failed (exit code $LASTEXITCODE)."
        exit 1
    }
    Write-Host "Build succeeded." -ForegroundColor Green
}

if (-not (Test-Path $exePath)) {
    Write-Error "Executable not found: $exePath`nRun with -Build first."
    exit 1
}

# ---------------------------------------------------------------------------
# Run scenarios
# ---------------------------------------------------------------------------
$scenariosToRun = if ($Scenario -ne "") { @($Scenario) } else { $allScenarios }

$passed = 0
$failed = 0
$startAll = Get-Date

foreach ($name in $scenariosToRun) {
    $jsonPath = Join-Path $scenDir "$name.json"
    if (-not (Test-Path $jsonPath)) {
        Write-Warning "Scenario file not found: $jsonPath"
        $failed++
        continue
    }

    # Pass path relative to repo root so the app resolves PointCloud/samples/ correctly.
    $relJson = "PointCloud/PointCloudView/scenarios/$name.json"

    Write-Host "--- $name ---" -ForegroundColor Cyan
    $start = Get-Date

    Push-Location $repoRoot
    try {
        & $exePath "--run-scenario" $relJson
        $exitCode = $LASTEXITCODE
    } finally {
        Pop-Location
    }

    $elapsed = (Get-Date) - $start
    if ($exitCode -eq 0) {
        Write-Host ("[PASS] $name  {0:mm\:ss\.ff}" -f $elapsed) -ForegroundColor Green
        $passed++
    } else {
        Write-Host ("[FAIL] $name  {0:mm\:ss\.ff}" -f $elapsed) -ForegroundColor Red
        $failed++
    }
}

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
$totalElapsed = (Get-Date) - $startAll
Write-Host ""
Write-Host ("Total: $passed passed, $failed failed  [{0:mm\:ss\.ff}]" -f $totalElapsed)

exit $(if ($failed -eq 0) { 0 } else { 1 })
