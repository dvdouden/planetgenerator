cmake_minimum_required(VERSION 3.14.2)
project(PlanetGenerator)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")
find_package(OpenGL REQUIRED)

#set( USE_GLUT )
set(USE_SDL ON)


if (MINGW)

elseif (USE_GLUT)
    find_package(GLUT REQUIRED)
elseif (USE_SDL)
    find_package(SDL REQUIRED)
endif ()

set(VL_BASE_COMPONENTS VLMain VLX VLGraphics VLCore VLVG VLVolume VLMolecule FreeType JPG PNG TIFF ZLib)


if (MINGW)
    find_package(VL COMPONENTS ${VL_BASE_COMPONENTS} VLWin32 REQUIRED)
elseif (USE_GLUT)
    find_package(VL COMPONENTS ${VL_BASE_COMPONENTS} VLGLUT REQUIRED)
elseif (USE_SDL)
    find_package(VL COMPONENTS ${VL_BASE_COMPONENTS} VLSDL REQUIRED)
endif ()
add_definitions(-DVL_STATIC_LINKING)


# add VL include directories
include_directories(${VL_INCLUDE_DIRS})
if (MINGW)
    set(PLATFORM_ENTRY win32_main.cpp)
elseif (USE_GLUT)
    include_directories(${GLUT_INCLUDE_DIR})
    set(PLATFORM_ENTRY glut_main.cpp)
elseif (USE_SDL)
    include_directories(${SDL_INCLUDE_DIR})
    set(PLATFORM_ENTRY sdl_main.cpp)
endif ()

add_executable(
        PlanetGenerator
        ${PLATFORM_ENTRY}
        MainWindow.cpp
        MainWindow.h
        util/fmath.cpp
        util/fmath.h
        delaunator.h
        IndexedTessellator.cpp
        IndexedTessellator.h
        Planet.cpp
        Planet.h
        util/Profiler.cpp
        util/Profiler.h
        Graph.cpp
        Graph.h
        geom/DrawArrayGeometry.cpp
        geom/DrawArrayGeometry.h
        geom/BorderLineGeometry.cpp
        geom/BorderLineGeometry.h
        geom/CellLineGeometry.cpp
        geom/CellLineGeometry.cpp
        geom/CellLineGeometry.h
        geom/CellVectorGeometry.cpp
        geom/CellVectorGeometry.h
        geom/PlanetGeometry.cpp
        geom/PlanetGeometry.h
        geom/DrawElementsGeometry.cpp
        geom/DrawElementsGeometry.h
        geom/RingGeometry.cpp
        geom/RingGeometry.h
        geom/PointsGeometry.cpp
        geom/PointsGeometry.h
        geom/TriangleGeometry.cpp
        geom/TriangleGeometry.h
        geom/CentroidsGeometry.cpp
        geom/CentroidsGeometry.h
        geom/WorldAxis.cpp
        geom/WorldAxis.h
        geom/GradientRingGeometry.cpp
        geom/GradientRingGeometry.h
        util/parameter.h
        util/tree.h
        PlanetIntersector.cpp
        PlanetIntersector.h)

if (MINGW)
    target_link_libraries(PlanetGenerator ${VL_LIBRARIES} ${OPENGL_LIBRARIES} -static)
elseif (USE_GLUT)
    target_link_libraries(PlanetGenerator ${VL_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
elseif (USE_SDL)
    target_link_libraries(PlanetGenerator ${VL_LIBRARIES} ${OPENGL_LIBRARIES} ${SDL_LIBRARY})
endif ()