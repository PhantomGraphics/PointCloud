# Cross-platform CMake build for the PointCloud core library (Phantom::PC /
# Phantom::PointCloud), its GoogleTest suite (PointCloudTest), and
# PointCloudView/PointRenderer/GSView (Vulkan apps).
#
# CMake is the only build system for this module (Windows and Linux).
# MathCore/SpaceCore/NumericsCore/VolumeRenderer/VkAppBaseCore/etc. are now
# defined once in ../cmake/PhantomCoreLibs.cmake and
# ../cmake/PhantomVulkanApp.cmake -- this file just calls
# them (PointCloudCore itself links against MathCore/SpaceCore/NumericsCore
# instead of bundling their raw sources, same behavior, cleaner; GSView's
# volume-rendering path now reuses the shared VolumeRenderer/VolumeCore
# instead of duplicating them as GSViewVolumeCore/GSViewVolumeRenderer) and
# adds PointCloudTest/PointRenderer/PointCloudView/GSView on top, so it also
# works standalone (`cmake -S PointCloud -B ...`) exactly as before, and
# identically when add_subdirectory()'d from the root CMakeLists.txt.
#
# Usage (from WSL, in a Linux-native checkout -- avoid /mnt/c/... for speed):
#   cmake -S PointCloud -B PointCloud/build_linux \
#       -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_C_COMPILER=clang \
#       -DCMAKE_BUILD_TYPE=Debug
#   cmake --build PointCloud/build_linux -j
#   ctest --test-dir PointCloud/build_linux --output-on-failure
#
# GSView.vcxproj additionally references CGLib/Volume/VolumeRenderer/
# VolumeRenderer.vcxproj (for Phantom::Volume::PBVRPipeline/ParticleSet, used
# by the PBVR render-mode toggle in GSViewRenderer/GSComputePBVR). NOTE:
# GSView.vcxproj's AdditionalIncludeDirectories points at a stale
# "..\..\CGLib\Volume\PBVRLib" path that doesn't exist on disk (renamed to
# VolumeRenderer/ at some point, per its own vcxproj's leftover
# RootNamespace=PBVRLib) -- resolves in practice via the ProjectReference's
# public include dirs on Windows; this file just points directly at the real
# path via phantom_add_volumerenderer_core()'s PUBLIC include dirs.

cmake_minimum_required(VERSION 3.20)

project(PointCloudCoreLinux LANGUAGES CXX)

# C++20, not 17 -- PointCloud.vcxproj itself already overrides
# <LanguageStandard> to stdcpp20 on Windows (as do Graphics.vcxproj/
# VulkanGraphics.vcxproj/UIWidgets.vcxproj/PointCloudView.vcxproj/
# PointRenderer.vcxproj, all needed by the PointCloudView block below --
# VkAppBase.cpp uses std::string_view::starts_with, a C++20 feature).
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(POINTCLOUD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(CGLIB_ROOT ${REPO_ROOT}/CGLib)

if(MSVC)
    add_compile_options(/utf-8)
    set(PHANTOM_WARN_FLAGS /W3)
else()
    set(PHANTOM_WARN_FLAGS -Wall)
endif()

list(APPEND CMAKE_MODULE_PATH ${REPO_ROOT}/cmake)
include(PhantomGTest)
include(PhantomCoreLibs)
include(PhantomVulkanApp)

phantom_add_math_core()
phantom_add_space_core()
phantom_add_numerics_core()

file(GLOB POINTCLOUD_SOURCES ${POINTCLOUD_ROOT}/PointCloud/*.cpp)
list(FILTER POINTCLOUD_SOURCES EXCLUDE REGEX "pch\\.cpp$")

add_library(PointCloudCore STATIC ${POINTCLOUD_SOURCES})
target_include_directories(PointCloudCore PRIVATE ${REPO_ROOT})
target_link_libraries(PointCloudCore PUBLIC MathCore SpaceCore NumericsCore)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(PointCloudCore PUBLIC OpenMP::OpenMP_CXX)
endif()

target_compile_options(PointCloudCore PRIVATE ${PHANTOM_WARN_FLAGS})

# --- PointCloudTest (GoogleTest) ---

phantom_find_gtest()

if(PHANTOM_GTEST_FOUND)
    enable_testing()

    file(GLOB POINTCLOUDTEST_SOURCES ${POINTCLOUD_ROOT}/PointCloudTest/*.cpp)
    list(FILTER POINTCLOUDTEST_SOURCES EXCLUDE REGEX "pch\\.cpp$")

    add_executable(PointCloudTest
        ${POINTCLOUDTEST_SOURCES}
        # CPU reference for the PBVR3DExperimental particle generation (not on any
        # render path -- see the file header). Pinned by GSParticleGeneratorTest.cpp.
        ${POINTCLOUD_ROOT}/GSView/reference/GSParticleGenerator.cpp
    )

    target_include_directories(PointCloudTest PRIVATE
        ${REPO_ROOT}
        ${POINTCLOUD_ROOT}/PointCloudTest
        ${POINTCLOUD_ROOT}/GSView/reference
        ${CGLIB_ROOT}/ThirdParty/glm-0.9.9.8
    )

    target_link_libraries(PointCloudTest PRIVATE
        PointCloudCore
        GTest::gtest
        GTest::gtest_main
    )

    target_compile_options(PointCloudTest PRIVATE ${PHANTOM_WARN_FLAGS})

    include(GoogleTest)
    gtest_discover_tests(PointCloudTest)
else()
    message(WARNING "GTest not found -- skipping PointCloudTest target")
endif()

# ---------------------------------------------------------------------------
# PointCloudView (VPC::PointCloudApp) + PointRenderer (shared with GSView) +
# GSView (GSView::GSViewApp).
# ---------------------------------------------------------------------------

phantom_find_vulkan_headers()

if(PHANTOM_VULKAN_INCLUDE_DIR)

phantom_find_vulkan_loader()
phantom_find_glfw_loader()

if(PHANTOM_VULKAN_LIBRARY AND PHANTOM_GLFW_LIBRARY)

phantom_add_vkappbase_core()
phantom_add_vkrenderer_core()
phantom_add_volumerenderer_core()

# --- PointRenderer (Phantom::VKR point/GS-splat pipelines, shared with GSView) ---

add_library(PointRendererCore STATIC
    ${POINTCLOUD_ROOT}/PointRenderer/src/VkGSPointRenderer.cpp
    ${POINTCLOUD_ROOT}/PointRenderer/src/VkPointCloudPipeline.cpp
    ${POINTCLOUD_ROOT}/PointRenderer/src/VkPointRenderer.cpp
    ${POINTCLOUD_ROOT}/PointRenderer/src/VkPointScene.cpp
)
target_include_directories(PointRendererCore PRIVATE
    ${CGLIB_ROOT}/ThirdParty/glm-0.9.9.8
)
# VkPointRenderer.cpp (PointRenderer/src/VkPointRenderer.cpp) uses
# Phantom::VKG::VkLineRenderer for normal-line overlays (via
# Renderer/VkRenderer/VkLineRenderer.h) -- must declare this so CMake orders
# VkRendererCore correctly on PointCloudView's final link line (undeclared,
# this static-archive link failed with "undefined reference to
# VkLineRenderer::..." since ld.bfd processes archives in a single left-to-right
# pass and VkRendererCore's needed members weren't re-pulled after
# PointRendererCore introduced the reference).
target_link_libraries(PointRendererCore PRIVATE VulkanGraphicsCore VkRendererCore)
target_compile_options(PointRendererCore PRIVATE ${PHANTOM_WARN_FLAGS})

# --- PointCloudView (VPC::PointCloudApp) -- standalone ImGui + Vulkan viewer ---
# Excludes VkGSPointRenderer.cpp/VulkanPointCloudPipeline.cpp still sitting in
# this directory -- dead leftovers from before the PointRenderer extraction,
# not referenced by PointCloudView.vcxproj's <ClCompile> list.

file(GLOB POINTCLOUDVIEW_SOURCES ${POINTCLOUD_ROOT}/PointCloudView/*.cpp)
list(FILTER POINTCLOUDVIEW_SOURCES EXCLUDE REGEX "VkGSPointRenderer\\.cpp$")
list(FILTER POINTCLOUDVIEW_SOURCES EXCLUDE REGEX "VulkanPointCloudPipeline\\.cpp$")

add_executable(PointCloudView ${POINTCLOUDVIEW_SOURCES})
target_include_directories(PointCloudView PRIVATE
    ${CGLIB_ROOT}/UIWidgets
    ${POINTCLOUD_ROOT}/PointCloud
)
target_link_libraries(PointCloudView PRIVATE
    PointCloudCore
    VkAppBaseCore
    VkRendererCore
    PointRendererCore
    ${PHANTOM_VULKAN_LIBRARY}
    ${PHANTOM_GLFW_LIBRARY}
)
if(OpenMP_CXX_FOUND)
    target_link_libraries(PointCloudView PRIVATE OpenMP::OpenMP_CXX)
endif()
target_compile_options(PointCloudView PRIVATE ${PHANTOM_WARN_FLAGS})

# loadSPVRepo() (VulkanSPVResolver.h) resolves "shaders/*.spv" relative to the
# running binary's directory on Windows, and to the current working directory
# on other platforms. The .spv files are Git-ignored, so compile them from GLSL
# with glslc and drop them beside the binary (phantom_add_runtime_shaders,
# cmake/PhantomVulkanApp.cmake). PointRenderer/shaders is listed first so its
# point.vert/gs_splat.frag (what VkPointRenderer's pipeline expects) win over
# PointCloudView/shaders' older single-purpose copies; PointCloudView/shaders
# still supplies the triangle.* pair for the plane/cylinder polygon overlay.
phantom_add_runtime_shaders(PointCloudView
    ${POINTCLOUD_ROOT}/PointRenderer/shaders
    ${POINTCLOUD_ROOT}/PointCloudView/shaders
)

# ---------------------------------------------------------------------------
# GSView (GSView::GSViewApp) -- standalone Gaussian Splatting ImGui + Vulkan
# viewer. Reuses PointCloudCore/PointRendererCore/VkAppBaseCore/VkRendererCore
# above, plus the shared VolumeRenderer (phantom_add_volumerenderer_core(),
# called above) for its PBVR render-mode toggle.
# ---------------------------------------------------------------------------

add_executable(GSView
    ${POINTCLOUD_ROOT}/GSView/main.cpp
    ${POINTCLOUD_ROOT}/GSView/GSViewApp.cpp
    ${POINTCLOUD_ROOT}/GSView/GSViewCommandDispatcher.cpp
    ${POINTCLOUD_ROOT}/GSView/GSViewRenderer.cpp
    ${POINTCLOUD_ROOT}/GSView/GSComputePBVR.cpp
    ${POINTCLOUD_ROOT}/GSView/ui/GSViewPanel.cpp
    ${POINTCLOUD_ROOT}/GSView/ui/GSViewMenuBar.cpp
)
target_include_directories(GSView PRIVATE
    ${CGLIB_ROOT}/UIWidgets
    ${CGLIB_ROOT}/Volume/VolumeRenderer
    ${POINTCLOUD_ROOT}/PointRenderer/include
    ${POINTCLOUD_ROOT}/PointCloud
)
target_link_libraries(GSView PRIVATE
    PointCloudCore
    PointRendererCore
    VolumeRenderer
    VkAppBaseCore
    VkRendererCore
    ${PHANTOM_VULKAN_LIBRARY}
    ${PHANTOM_GLFW_LIBRARY}
)
if(OpenMP_CXX_FOUND)
    target_link_libraries(GSView PRIVATE OpenMP::OpenMP_CXX)
endif()
target_compile_options(GSView PRIVATE ${PHANTOM_WARN_FLAGS})

# GSView compiles+copies BOTH PointRenderer/shaders (point/gs_splat/gs_sort
# pipelines, shared with PointCloudView) and GSView's own shaders/ (gs_pbvr*)
# into the output shaders/ dir -- glslc-compiled from GLSL, same as above.
phantom_add_runtime_shaders(GSView
    ${POINTCLOUD_ROOT}/PointRenderer/shaders
    ${POINTCLOUD_ROOT}/GSView/shaders
)

else()
    message(WARNING "PointCloud: Vulkan and/or GLFW loader not found -- PointCloudView/GSView are skipped (PointCloud/PointCloudTest still build). Pass -DVULKAN_LIBRARY=... -DGLFW_LIBRARY=... to enable it.")
endif()

else()
    message(WARNING "PointCloud: vulkan/vulkan.h not found -- PointCloudView is skipped. Pass -DVULKAN_INCLUDE_DIR=/path/to/include to enable it (on Windows, check $ENV{VULKAN_SDK} is set).")
endif()
