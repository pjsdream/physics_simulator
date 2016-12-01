cmake_minimum_required(VERSION 2.8.12.2)

PROJECT(physics_simulator)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

## show all files (http://wiki.ros.org/IEDs#QtCreator)
# Add all files in subdirectories of the project in a dummy_target so qtcreator have access to all files
FILE(GLOB children ${CMAKE_SOURCE_DIR})
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Find the Qt library
set(CMAKE_PREFIX_PATH "/home/jaesungp/Qt/5.7/gcc_64")
find_package(Qt5Widgets)
find_package(Qt5OpenGL)
add_definitions(${Qt5Widgets_DEFINITIONS})
add_definitions(${Qt5OpenGL_DEFINITIONS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${Qt5OpenGL_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

# OpenGL
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
  message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

# copy shaders to target directory
file(COPY ${CMAKE_SOURCE_DIR}/shader DESTINATION ${CMAKE_BINARY_DIR}/bin)

include_directories(
  include
)

link_directories(
  lib
)

# hw2
qt5_wrap_cpp(visualizer_moc
  include/visualizer/visualizer.h
  include/visualizer/simulator_interface.h
)

add_library(physics_simulator
  src/physics/simulator.cpp
  src/visualizer/camera.cpp
  src/visualizer/visualizer.cpp
  src/visualizer/simulator_interface.cpp
  ${visualizer_moc}
)

target_link_libraries(physics_simulator
  ${OpenGL_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${Qt5OpenGL_LIBRARIES}
)


# hw2a executable
add_executable(test_interface
  src/test/test_interface.cpp
)

target_link_libraries(test_interface
  physics_simulator
)
