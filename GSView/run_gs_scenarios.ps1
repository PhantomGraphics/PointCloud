# run_gs_scenarios.ps1 - Run all GSView scenario tests
# Usage: .\PointCloud\GSView\run_gs_scenarios.ps1 [-Configuration Debug|Release]
# Run from the Phantom root or the PointCloud\GSView directory.
#
# Locates the Phantom root by its build markers (CMakePresets.json +
# cmake\PhantomVulkanApp.cmake), matching run_pc_scenarios.ps1 -- the older
# "Phantom2026.sln" lookup pointed at the pre-submodule outer tree whose build
# directory is now stale.

param(
    [string]$Configuration = "Debug"
)

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot  = $scriptDir
while ($repoRoot -and -not (
        (Test-Path (Join-Path $repoRoot "CMakePresets.json")) -and
        (Test-Path (Join-Path $repoRoot "cmake\PhantomVulkanApp.cmake")))) {
    $parent = Split-Path -Parent $repoRoot
    if ($parent -eq $repoRoot) { $repoRoot = $null; break }
    $repoRoot = $parent
}
if (-not $repoRoot) {
    Write-Host "ERROR: Could not locate the Phantom root (CMakePresets.json + cmake\PhantomVulkanApp.cmake)"
    exit 1
}
$preset    = "windows-$($Configuration.ToLower())"
$exe       = Join-Path $repoRoot "build\$preset\PointCloud\GSView.exe"
$scenDir   = Join-Path $scriptDir "scenarios"

if (-not (Test-Path $exe)) {
    Write-Host "ERROR: Executable not found: $exe"
    exit 1
}

# Prerequisite: make sure the synthetic GS sample data the scenarios load actually
# exists. download_gs_samples.ps1 only generates missing files, so this is cheap on
# repeat runs and removes the dependency on a pre-populated samples/gs directory.
$genScript = Join-Path $scriptDir "..\download_gs_samples.ps1"
if (Test-Path $genScript) {
    Write-Host "Ensuring synthetic GS samples exist ..."
    try {
        & $genScript
    } catch {
        Write-Host "ERROR: sample generation failed: $_"
        exit 1
    }
} else {
    Write-Host "WARNING: $genScript not found; assuming samples/gs is already populated"
}

$scenarios = Get-ChildItem "$scenDir\*.json" | Sort-Object Name
if ($scenarios.Count -eq 0) {
    Write-Host "ERROR: No scenario JSON files found in $scenDir"
    exit 1
}

$passed = 0
$failed = 0

foreach ($s in $scenarios) {
    Write-Host "Running: $($s.BaseName)"
    $proc = Start-Process -FilePath $exe `
        -ArgumentList "--run-scenario `"$($s.FullName)`"" `
        -PassThru -Wait -WorkingDirectory $scriptDir
    if ($proc.ExitCode -eq 0) {
        Write-Host "PASSED: $($s.BaseName)"
        $passed++
    } else {
        Write-Host "FAILED: $($s.BaseName) (exit $($proc.ExitCode))"
        $failed++
    }
}

Write-Host ""
Write-Host "Results: $passed/$($scenarios.Count) PASSED, $failed FAILED"
exit $failed
