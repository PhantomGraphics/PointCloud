@echo off
setlocal

set GLSLC="%VULKAN_SDK%\Bin\glslc.exe"
if not exist %GLSLC% (
    echo [ERROR] glslc.exe not found. VULKAN_SDK=%VULKAN_SDK%
    exit /b 1
)

set OUTDIR=%~dp0

%GLSLC% -fshader-stage=vert "%OUTDIR%point.vert" -o "%OUTDIR%point.vert.spv"
if errorlevel 1 ( echo FAILED: point.vert & exit /b 1 )

%GLSLC% -fshader-stage=frag "%OUTDIR%point.frag" -o "%OUTDIR%point.frag.spv"
if errorlevel 1 ( echo FAILED: point.frag & exit /b 1 )

%GLSLC% -fshader-stage=vert "%OUTDIR%line.vert" -o "%OUTDIR%line.vert.spv"
if errorlevel 1 ( echo FAILED: line.vert & exit /b 1 )

%GLSLC% -fshader-stage=frag "%OUTDIR%line.frag" -o "%OUTDIR%line.frag.spv"
if errorlevel 1 ( echo FAILED: line.frag & exit /b 1 )

%GLSLC% -fshader-stage=vert "%OUTDIR%gs_splat.vert" -o "%OUTDIR%gs_splat.vert.spv"
if errorlevel 1 ( echo FAILED: gs_splat.vert & exit /b 1 )

%GLSLC% -fshader-stage=frag "%OUTDIR%gs_splat.frag" -o "%OUTDIR%gs_splat.frag.spv"
if errorlevel 1 ( echo FAILED: gs_splat.frag & exit /b 1 )

%GLSLC% -fshader-stage=comp "%OUTDIR%gs_sort.comp" -o "%OUTDIR%gs_sort.comp.spv"
if errorlevel 1 ( echo FAILED: gs_sort.comp & exit /b 1 )

%GLSLC% -fshader-stage=vert "%OUTDIR%triangle.vert" -o "%OUTDIR%triangle.vert.spv"
if errorlevel 1 ( echo FAILED: triangle.vert & exit /b 1 )

%GLSLC% -fshader-stage=frag "%OUTDIR%triangle.frag" -o "%OUTDIR%triangle.frag.spv"
if errorlevel 1 ( echo FAILED: triangle.frag & exit /b 1 )

echo Shader compile succeeded.
endlocal
