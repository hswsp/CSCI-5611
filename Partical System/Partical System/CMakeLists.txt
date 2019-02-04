project(hw3)

cmake_minimum_required(VERSION 3.12)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

file(GLOB SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/SourceCode/Cube3D.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/common/*.cpp
)

add_executable(${PROJECT_NAME} ${SRC})
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/Particle.fragmentshader
	${CMAKE_CURRENT_SOURCE_DIR}/Particle.vertexshader
)