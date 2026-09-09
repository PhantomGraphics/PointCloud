@echo off
setlocal

if "%VULKAN_SDK%"=="" (
  echo [ERROR] VULKAN_SDK is not set.
  exit /b 1
)

cd /d "%~dp0"

"%VULKAN_SDK%\Bin\glslc.exe" gps_splat.comp     -o gps_splat.comp.spv
"%VULKAN_SDK%\Bin\glslc.exe" gps_pbvr3d.comp    -o gps_pbvr3d.comp.spv
"%VULKAN_SDK%\Bin\glslc.exe" gps_resolve.comp   -o gps_resolve.comp.spv
"%VULKAN_SDK%\Bin\glslc.exe" gps_composite.vert -o gps_composite.vert.spv
"%VULKAN_SDK%\Bin\glslc.exe" gps_composite.frag -o gps_composite.frag.spv

echo Done.
