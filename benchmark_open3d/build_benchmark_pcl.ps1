<#
.SYNOPSIS
  Builds benchmark_pcl.cpp in-place (a one-off benchmark, not part of the main .sln).
.DESCRIPTION
  Links against the official PCL 1.15.1 AllInOne prebuilt binaries
  (https://github.com/PointCloudLibrary/pcl/releases, installed to
  "C:\Program Files\PCL 1.15.1" via the msvc2022-win64 installer). Does NOT depend on
  this repo's own PointCloud/Space/Numerics/Math libs (unlike build_benchmark.ps1) -
  benchmark_pcl.cpp has its own tiny PLY reader, to avoid mixing PCL's VS2022(vc143)
  prebuilt static libs with this repo's VS2026(v145) libs at link time.
.PARAMETER PclRoot
  Install root of the PCL AllInOne package. Default: "C:\Program Files\PCL 1.15.1".
#>
param(
    [string]$PclRoot = "C:\Program Files\PCL 1.15.1"
)
$ErrorActionPreference = 'Stop'

if (-not (Test-Path $PclRoot)) {
    throw "PCL install not found at $PclRoot. Install the AllInOne Windows installer from https://github.com/PointCloudLibrary/pcl/releases first."
}

$pclInclude = Join-Path $PclRoot 'include\pcl-1.15'
$eigenInclude = Join-Path $PclRoot '3rdParty\Eigen3\include\eigen3'
$boostInclude = Join-Path $PclRoot '3rdParty\Boost\include\boost-1_87'
$flannInclude = Join-Path $PclRoot '3rdParty\FLANN\include'

$pclLib = Join-Path $PclRoot 'lib'
$boostLib = Join-Path $PclRoot '3rdParty\Boost\lib'
$flannLib = Join-Path $PclRoot '3rdParty\FLANN\lib'

$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path $vswhere)) {
    throw "vswhere.exe not found (is Visual Studio installed?): $vswhere"
}
$vsInstallPath = & $vswhere -latest -products * -property installationPath
$vcvarsall = Join-Path $vsInstallPath 'VC\Auxiliary\Build\vcvarsall.bat'
if (-not (Test-Path $vcvarsall)) {
    throw "vcvarsall.bat not found: $vcvarsall"
}

$src = Join-Path $PSScriptRoot 'benchmark_pcl.cpp'
$exe = Join-Path $PSScriptRoot 'benchmark_pcl.exe'

$pclLibs = @(
    'pcl_registration.lib', 'pcl_segmentation.lib', 'pcl_sample_consensus.lib',
    'pcl_features.lib', 'pcl_filters.lib', 'pcl_search.lib', 'pcl_kdtree.lib',
    'pcl_octree.lib', 'pcl_common.lib', 'flann_cpp_s.lib', 'flann_s.lib'
) -join ' '

$cl = "cl.exe /nologo /EHsc /std:c++17 /O2 /MD /arch:AVX2 /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS /DEIGEN_MAX_ALIGN_BYTES=32 " +
      "/I `"$pclInclude`" /I `"$eigenInclude`" /I `"$boostInclude`" /I `"$flannInclude`" " +
      "`"$src`" /Fe:`"$exe`" /Fo:`"$PSScriptRoot\\`" " +
      "/link /LIBPATH:`"$pclLib`" /LIBPATH:`"$boostLib`" /LIBPATH:`"$flannLib`" $pclLibs"

$prevEap = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
cmd /c "`"$vcvarsall`" x64 && $cl"
$exitCode = $LASTEXITCODE
$ErrorActionPreference = $prevEap
if ($exitCode -ne 0) {
    throw "Build failed (exit $exitCode)"
}

Write-Host "`nBuild succeeded: $exe"
Write-Host "Run it like:  `$env:PATH += `";$PclRoot\bin`"; .\benchmark_pcl.exe .\datasets"
