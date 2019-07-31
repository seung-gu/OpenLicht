cmake_minimum_required(VERSION 2.8)

# Please insert your OpenCV path
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} [path to folder where you put FindOpenCV.cmake])

# This needs at least OpenCV 2.x, but it should also work with OpenCV 3.x
find_package(OpenCV QUIET)

if(NOT OpenCV_FOUND)
  message("OpenCV example will not be build as no OpenCV was found!")
  return()
endif()


set(CMAKE_PREFIX_PATH "../../../../share")

find_package(royale REQUIRED)

link_directories(${royale_LIB_DIR})

### include OpenMP ###
if(NOT HAVE_TBB AND NOT HAVE_CSTRIPES)
  include(FindOpenMP)
    if(OPENMP_FOUND)
      set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    else()
      set(HAVE_OPENMP FALSE)
    endif()
  else()
  set(HAVE_OPENMP 0)
endif()

project (KeyMotion_v1c)


### Include all directories ###

set(SSD1306_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/display/ssd1306)
set(SH1106_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/display/sh1106)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../inc
  ${SH1106_DIRECTORIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/bluetooth
  ${CMAKE_CURRENT_SOURCE_DIR}/sound
)


### Add execuatable source files ###

set(SRC_FILES_SSD1306 
	${CMAKE_CURRENT_SOURCE_DIR}/display/ssd1306/ssd1306_i2c.cpp
)
set(SRC_FILES_SH1106 
	${CMAKE_CURRENT_SOURCE_DIR}/display/sh1106/ArduiPi_OLED.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/display/sh1106/Adafruit_GFX.cpp	
)

add_executable(${PROJECT_NAME}
	main.cpp
   	MyListener.cpp
	${SRC_FILES_SH1106}
	${CMAKE_CURRENT_SOURCE_DIR}/bluetooth/Bluetooth.cpp
	#${CMAKE_CURRENT_SOURCE_DIR}/sound/BeepSound.cpp
)

if(WIN32)   
    if(OpenCV_STATIC)	#Debug, Release, RelWithDebInfo, MinSizeRel
        target_compile_options(${PROJECT_NAME} PRIVATE "/MT$<$<CONFIG:Release>:d>")
    endif()
endif()
   
COPY_ROYALE_LIBS(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME} "${royale_LIBS}" "${OpenCV_LIBS}")

ADD_LIBRARY(Libs${PROJECT_NAME}
	main.cpp	
)

target_link_libraries(Libs${PROJECT_NAME} -lwiringPi)
target_link_libraries(Libs${PROJECT_NAME} -lbcm2835)
target_link_libraries(Libs${PROJECT_NAME} -lX11)
target_link_libraries(Libs${PROJECT_NAME} -lXtst)
target_link_libraries(Libs${PROJECT_NAME} -lbluetooth)
target_link_libraries(Libs${PROJECT_NAME} -L/usr/)
target_link_libraries(${PROJECT_NAME} Libs${PROJECT_NAME})




