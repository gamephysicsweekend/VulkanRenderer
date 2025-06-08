cmake_minimum_required(VERSION 3.10)

project(VulkanRenderer
    LANGUAGES CXX
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (MSVC)
    message(FATAL_ERROR
        "Use the provided Visual Studio solution file to build this project with visual studio.\n"
        "This cmake file is meant for non-windows platforms.")
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Werror")
    # Display unused parameters as warnings
    # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=unused-parameter")
    # Suppress warnings about unused parameters
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")
endif ()

if (NOT WIN32)
    include(FetchContent)

    set(GLFW_LIBRARY_TYPE STATIC CACHE STRING "" FORCE)
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        glfw
        GIT_REPOSITORY  https://github.com/glfw/glfw
        GIT_TAG         3.4
        EXCLUDE_FROM_ALL)
    FetchContent_MakeAvailable(glfw)

    find_package(Vulkan REQUIRED)
endif ()

add_executable(VulkanRenderer)
target_sources(VulkanRenderer
    PRIVATE
        code/Fileio.cpp
        code/Fileio.h
        code/Scene.cpp
        code/Scene.h
        code/application.cpp
        code/application.h
        code/main.cpp
        code/Math/Bounds.cpp
        code/Math/Bounds.h
        code/Math/LCP.cpp
        code/Math/LCP.h
        code/Math/Matrix.h
        code/Math/Quat.h
        code/Math/Vector.h
        code/Physics/Body.cpp
        code/Physics/Body.h
        code/Physics/Broadphase.cpp
        code/Physics/Broadphase.h
        code/Physics/Constraints.cpp
        code/Physics/Constraints.h
        code/Physics/Contact.cpp
        code/Physics/Contact.h
        code/Physics/GJK.cpp
        code/Physics/GJK.h
        code/Physics/Intersections.cpp
        code/Physics/Intersections.h
        code/Physics/Manifold.cpp
        code/Physics/Manifold.h
        code/Physics/Shapes.cpp
        code/Physics/Shapes.h
        code/Physics/Constraints/ConstraintBase.h
        code/Physics/Constraints/ConstraintConstantVelocity.cpp
        code/Physics/Constraints/ConstraintConstantVelocity.h
        code/Physics/Constraints/ConstraintDistance.cpp
        code/Physics/Constraints/ConstraintDistance.h
        code/Physics/Constraints/ConstraintHinge.cpp
        code/Physics/Constraints/ConstraintHinge.h
        code/Physics/Constraints/ConstraintMotor.cpp
        code/Physics/Constraints/ConstraintMotor.h
        code/Physics/Constraints/ConstraintMover.cpp
        code/Physics/Constraints/ConstraintMover.h
        code/Physics/Constraints/ConstraintOrientation.cpp
        code/Physics/Constraints/ConstraintOrientation.h
        code/Physics/Constraints/ConstraintPenetration.cpp
        code/Physics/Constraints/ConstraintPenetration.h
        code/Physics/Shapes/ShapeBase.h
        code/Physics/Shapes/ShapeBox.cpp
        code/Physics/Shapes/ShapeBox.h
        code/Physics/Shapes/ShapeConvex.cpp
        code/Physics/Shapes/ShapeConvex.h
        code/Physics/Shapes/ShapeSphere.cpp
        code/Physics/Shapes/ShapeSphere.h
        code/Renderer/Buffer.cpp
        code/Renderer/Buffer.h
        code/Renderer/Descriptor.cpp
        code/Renderer/Descriptor.h
        code/Renderer/DeviceContext.cpp
        code/Renderer/DeviceContext.h
        code/Renderer/Fence.cpp
        code/Renderer/Fence.h
        code/Renderer/FrameBuffer.cpp
        code/Renderer/FrameBuffer.h
        code/Renderer/Image.cpp
        code/Renderer/Image.h
        code/Renderer/OffscreenRenderer.cpp
        code/Renderer/OffscreenRenderer.h
        code/Renderer/Pipeline.cpp
        code/Renderer/Pipeline.h
        code/Renderer/Samplers.cpp
        code/Renderer/Samplers.h
        code/Renderer/SwapChain.cpp
        code/Renderer/SwapChain.h
        code/Renderer/model.cpp
        code/Renderer/model.h
        code/Renderer/shader.cpp
        code/Renderer/shader.h
)

target_link_libraries(VulkanRenderer
    PRIVATE
        Vulkan::Vulkan
        glfw
)

