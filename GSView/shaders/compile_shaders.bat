@echo off
setlocal

if "%VULKAN_SDK%"=="" (
  echo [ERROR] VULKAN_SDK is not set.
  exit /b 1
)

cd /d "%~dp0"

"%VULKAN_SDK%\Bin\glslc.exe" gs_pbvr.vert -o gs_pbvr.vert.spv
"%VULKAN_SDK%\Bin\glslc.exe" gs_pbvr.frag -o gs_pbvr.frag.spv
"%VULKAN_SDK%\Bin\glslc.exe" gs_pbvr_gen.comp -o gs_pbvr_gen.comp.spv

echo Done.
