cmake_minimum_required(VERSION 3.15)
project(IcePhysicsEngine VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/W4 /permissive-)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

set(ICEPHYSICS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(ICEPHYSICS_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(ICEPHYSICS_HEADERS
    ${ICEPHYSICS_INCLUDE_DIR}/IcePhysicsAPI.h
    ${ICEPHYSICS_INCLUDE_DIR}/MathUtils.h
    ${ICEPHYSICS_INCLUDE_DIR}/RigidBody.h
    ${ICEPHYSICS_INCLUDE_DIR}/Collider.h
    ${ICEPHYSICS_INCLUDE_DIR}/CollisionDetection.h
    ${ICEPHYSICS_INCLUDE_DIR}/Hydrodynamics.h
    ${ICEPHYSICS_INCLUDE_DIR}/Voronoi.h
    ${ICEPHYSICS_INCLUDE_DIR}/PhysicsWorld.h
)

set(ICEPHYSICS_SOURCES
    ${ICEPHYSICS_SOURCE_DIR}/RigidBody.cpp
    ${ICEPHYSICS_SOURCE_DIR}/Collider.cpp
    ${ICEPHYSICS_SOURCE_DIR}/CollisionDetection.cpp
    ${ICEPHYSICS_SOURCE_DIR}/Hydrodynamics.cpp
    ${ICEPHYSICS_SOURCE_DIR}/Voronoi.cpp
    ${ICEPHYSICS_SOURCE_DIR}/PhysicsWorld.cpp
    ${ICEPHYSICS_SOURCE_DIR}/IcePhysicsAPI.cpp
)

add_library(IcePhysicsEngine SHARED
    ${ICEPHYSICS_HEADERS}
    ${ICEPHYSICS_SOURCES}
)

target_include_directories(IcePhysicsEngine
    PUBLIC
        $<BUILD_INTERFACE:${ICEPHYSICS_INCLUDE_DIR}>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${ICEPHYSICS_SOURCE_DIR}
)

target_compile_definitions(IcePhysicsEngine
    PRIVATE
        ICEPHYSICS_EXPORTS
)

set_target_properties(IcePhysicsEngine PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    DEBUG_POSTFIX "d"
)

if(WIN32)
    set_target_properties(IcePhysicsEngine PROPERTIES
        OUTPUT_NAME "IcePhysicsEngine"
        PDB_NAME "IcePhysicsEngine"
    )
endif()

install(TARGETS IcePhysicsEngine
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(FILES ${ICEPHYSICS_HEADERS}
    DESTINATION include/IcePhysicsEngine
)

set(UNITY_PLUGINS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../UnityProject/Assets/Plugins)

add_custom_command(TARGET IcePhysicsEngine POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:IcePhysicsEngine>
        ${UNITY_PLUGINS_DIR}/$<TARGET_FILE_NAME:IcePhysicsEngine>
    COMMENT "Copying DLL to Unity Plugins folder"
)

if(MSVC)
    add_custom_command(TARGET IcePhysicsEngine POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_PDB_FILE:IcePhysicsEngine>
            ${UNITY_PLUGINS_DIR}/$<TARGET_PDB_FILE_NAME:IcePhysicsEngine>
        COMMENT "Copying PDB to Unity Plugins folder"
    )
endif()
