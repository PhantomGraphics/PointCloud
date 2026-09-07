# run_pc_scenarios.ps1 - Run all PointCloudView scenario tests
# Usage: .\PointCloud\PointCloudView\run_pc_scenarios.ps1 [-Configuration Debug|Release]
# Run from the Phantom root or the PointCloud\PointCloudView directory.
#
# Note: unlike some sibling scenario runners, this one runs the exe with the
# Phantom root as the working directory, because these scenario JSON files
# reference sample data with Phantom-root-relative paths (e.g.
# "PointCloud/samples/sphere.ply"), and PointCloudFileLoader resolves paths
# as given without any root lookup of its own.
#
# The Phantom C++ modules (CGLib/Physics/PointCloud/RayTracer) now live under
# the "Phantom/" submodule with their own CMakePresets.json, so this locates
# the Phantom root by its build markers rather than the parent "Phantom2026.sln".

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
$exe       = Join-Path $repoRoot "build\$preset\PointCloud\PointCloudView.exe"
$scenDir   = Join-Path $scriptDir "scenarios"

if (-not (Test-Path $exe)) {
    Write-Host "ERROR: Executable not found: $exe"
    exit 1
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
        -PassThru -Wait -WorkingDirectory $repoRoot
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
