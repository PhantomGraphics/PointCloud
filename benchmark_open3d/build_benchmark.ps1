<#
.SYNOPSIS
  Builds benchmark_pointcloud.cpp in-place (a one-off benchmark, not part of the main .sln).
.DESCRIPTION
  Assumes the whole repo has already been built for the given configuration, and links
  against PointCloud.lib / Space.lib / Numerics.lib / Math.lib under x64\<Configuration>\.
  Rebuild the whole solution first if those libs are missing or stale:
    msbuild Phantom2026.sln /p:Configuration=Release /p:Platform=x64
  (Rebuilding only individual sub-projects can leave mismatched compiler flags/versions
  between libs, which fails to link with LNK2038 "RuntimeLibrary mismatch" or similar.)
.PARAMETER Configuration
  Release (default) or Debug. Uses the libs under x64\<Configuration>\.
#>
param(
    [string]$Configuration = "Release"
)
$ErrorActionPreference = 'Stop'

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$libDir = Join-Path $repoRoot "x64\$Configuration"
$glmDir = Join-Path $repoRoot "CGLib\ThirdParty\glm-0.9.9.8"

foreach ($lib in @('PointCloud.lib', 'Space.lib', 'Numerics.lib', 'Math.lib')) {
    if (-not (Test-Path (Join-Path $libDir $lib))) {
        throw "$libDir\$lib not found. Build the whole solution first: msbuild Phantom2026.sln /p:Configuration=$Configuration /p:Platform=x64"
    }
}

$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path $vswhere)) {
    throw "vswhere.exe not found (is Visual Studio installed?): $vswhere"
}
$vsInstallPath = & $vswhere -latest -products * -property installationPath
$vcvarsall = Join-Path $vsInstallPath 'VC\Auxiliary\Build\vcvarsall.bat'
if (-not (Test-Path $vcvarsall)) {
    throw "vcvarsall.bat not found: $vcvarsall"
}

$src = Join-Path $PSScriptRoot 'benchmark_pointcloud.cpp'
$exe = Join-Path $PSScriptRoot 'benchmark_pointcloud.exe'

$cl = "cl.exe /nologo /EHsc /std:c++17 /O2 /MD /I `"$repoRoot`" /I `"$glmDir`" `"$src`" /Fe:`"$exe`" /Fo:`"$PSScriptRoot\\`" /link /LTCG /LIBPATH:`"$libDir`" PointCloud.lib Space.lib Numerics.lib Math.lib"

$prevEap = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
cmd /c "`"$vcvarsall`" x64 && $cl"
$exitCode = $LASTEXITCODE
$ErrorActionPreference = $prevEap
if ($exitCode -ne 0) {
    throw "Build failed (exit $exitCode)"
}

Write-Host "`nBuild succeeded: $exe"
Write-Host "Run it like:  .\benchmark_pointcloud.exe .\datasets"
