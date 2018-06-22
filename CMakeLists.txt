cmake_minimum_required(VERSION 2.8)

# Please insert your OpenCV path
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} [path to folder where you put FindOpenCV.cmake])

# This needs at least OpenCV 2.x, but it should also work with OpenCV 3.x
find_package(OpenCV QUIET)

if(NOT OpenCV_FOUND)
  message("OpenCV example will not be build as no OpenCV was found!")
  return()
endif()

set(CMAKE_PREFIX_PATH "../../../share")

find_package(royale REQUIRED)

link_directories(${royale_LIB_DIR})

project (OpenLicht_3d)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../inc
)

add_executable(OpenLicht_3d
   	OpenLicht_3d.cpp
	BeepSound.cpp
	Bluetooth.cpp
   )

if(WIN32)   
    if(OpenCV_STATIC)
        target_compile_options(OpenLicht_3d PRIVATE "/MT$<$<CONFIG:Debug>:d>")
    endif()
endif()
   
COPY_ROYALE_LIBS(OpenLicht_3d)

target_link_libraries(OpenLicht_3d "${royale_LIBS}" "${OpenCV_LIBS}")

ADD_LIBRARY(LibsModule
	OpenLicht_3d.cpp
)

target_link_libraries(LibsModule -lwiringPi)
target_link_libraries(LibsModule -L/usr/)
target_link_libraries(LibsModule -lbluetooth)
target_link_libraries(OpenLicht_3d LibsModule)
