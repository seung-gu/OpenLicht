# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples

# Include any dependencies generated for this target.
include cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/depend.make

# Include the progress variables for this target.
include cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/progress.make

# Include the compile flags for this target's objects.
include cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/flags.make

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/OpenLicht_3d.cpp.o: cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/flags.make
cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/OpenLicht_3d.cpp.o: cpp/OpenLicht/OpenLicht_3d.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/OpenLicht_3d.cpp.o"
	cd /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OpenLicht_3d.dir/OpenLicht_3d.cpp.o -c /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/OpenLicht_3d.cpp

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/OpenLicht_3d.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OpenLicht_3d.dir/OpenLicht_3d.cpp.i"
	cd /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/OpenLicht_3d.cpp > CMakeFiles/OpenLicht_3d.dir/OpenLicht_3d.cpp.i

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/OpenLicht_3d.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OpenLicht_3d.dir/OpenLicht_3d.cpp.s"
	cd /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/OpenLicht_3d.cpp -o CMakeFiles/OpenLicht_3d.dir/OpenLicht_3d.cpp.s

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/OpenLicht_3d.cpp.o.requires:

.PHONY : cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/OpenLicht_3d.cpp.o.requires

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/OpenLicht_3d.cpp.o.provides: cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/OpenLicht_3d.cpp.o.requires
	$(MAKE) -f cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/build.make cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/OpenLicht_3d.cpp.o.provides.build
.PHONY : cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/OpenLicht_3d.cpp.o.provides

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/OpenLicht_3d.cpp.o.provides.build: cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/OpenLicht_3d.cpp.o


cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/BeepSound.cpp.o: cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/flags.make
cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/BeepSound.cpp.o: cpp/OpenLicht/BeepSound.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/BeepSound.cpp.o"
	cd /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OpenLicht_3d.dir/BeepSound.cpp.o -c /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/BeepSound.cpp

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/BeepSound.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OpenLicht_3d.dir/BeepSound.cpp.i"
	cd /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/BeepSound.cpp > CMakeFiles/OpenLicht_3d.dir/BeepSound.cpp.i

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/BeepSound.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OpenLicht_3d.dir/BeepSound.cpp.s"
	cd /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/BeepSound.cpp -o CMakeFiles/OpenLicht_3d.dir/BeepSound.cpp.s

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/BeepSound.cpp.o.requires:

.PHONY : cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/BeepSound.cpp.o.requires

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/BeepSound.cpp.o.provides: cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/BeepSound.cpp.o.requires
	$(MAKE) -f cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/build.make cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/BeepSound.cpp.o.provides.build
.PHONY : cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/BeepSound.cpp.o.provides

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/BeepSound.cpp.o.provides.build: cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/BeepSound.cpp.o


cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth.cpp.o: cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/flags.make
cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth.cpp.o: cpp/OpenLicht/Bluetooth.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth.cpp.o"
	cd /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OpenLicht_3d.dir/Bluetooth.cpp.o -c /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/Bluetooth.cpp

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OpenLicht_3d.dir/Bluetooth.cpp.i"
	cd /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/Bluetooth.cpp > CMakeFiles/OpenLicht_3d.dir/Bluetooth.cpp.i

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OpenLicht_3d.dir/Bluetooth.cpp.s"
	cd /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/Bluetooth.cpp -o CMakeFiles/OpenLicht_3d.dir/Bluetooth.cpp.s

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth.cpp.o.requires:

.PHONY : cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth.cpp.o.requires

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth.cpp.o.provides: cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth.cpp.o.requires
	$(MAKE) -f cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/build.make cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth.cpp.o.provides.build
.PHONY : cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth.cpp.o.provides

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth.cpp.o.provides.build: cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth.cpp.o


cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth_server.cpp.o: cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/flags.make
cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth_server.cpp.o: cpp/OpenLicht/Bluetooth_server.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth_server.cpp.o"
	cd /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OpenLicht_3d.dir/Bluetooth_server.cpp.o -c /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/Bluetooth_server.cpp

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth_server.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OpenLicht_3d.dir/Bluetooth_server.cpp.i"
	cd /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/Bluetooth_server.cpp > CMakeFiles/OpenLicht_3d.dir/Bluetooth_server.cpp.i

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth_server.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OpenLicht_3d.dir/Bluetooth_server.cpp.s"
	cd /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/Bluetooth_server.cpp -o CMakeFiles/OpenLicht_3d.dir/Bluetooth_server.cpp.s

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth_server.cpp.o.requires:

.PHONY : cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth_server.cpp.o.requires

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth_server.cpp.o.provides: cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth_server.cpp.o.requires
	$(MAKE) -f cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/build.make cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth_server.cpp.o.provides.build
.PHONY : cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth_server.cpp.o.provides

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth_server.cpp.o.provides.build: cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth_server.cpp.o


# Object files for target OpenLicht_3d
OpenLicht_3d_OBJECTS = \
"CMakeFiles/OpenLicht_3d.dir/OpenLicht_3d.cpp.o" \
"CMakeFiles/OpenLicht_3d.dir/BeepSound.cpp.o" \
"CMakeFiles/OpenLicht_3d.dir/Bluetooth.cpp.o" \
"CMakeFiles/OpenLicht_3d.dir/Bluetooth_server.cpp.o"

# External object files for target OpenLicht_3d
OpenLicht_3d_EXTERNAL_OBJECTS =

cpp/OpenLicht/OpenLicht_3d: cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/OpenLicht_3d.cpp.o
cpp/OpenLicht/OpenLicht_3d: cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/BeepSound.cpp.o
cpp/OpenLicht/OpenLicht_3d: cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth.cpp.o
cpp/OpenLicht/OpenLicht_3d: cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth_server.cpp.o
cpp/OpenLicht/OpenLicht_3d: cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/build.make
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_stitching.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_superres.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_videostab.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_aruco.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_bgsegm.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_bioinspired.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_ccalib.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_dpm.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_face.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_freetype.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_fuzzy.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_img_hash.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_line_descriptor.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_optflow.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_reg.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_rgbd.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_saliency.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_stereo.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_structured_light.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_surface_matching.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_tracking.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_xfeatures2d.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_ximgproc.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_xobjdetect.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_xphoto.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: cpp/OpenLicht/libLibsModule.a
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_shape.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_photo.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_datasets.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_plot.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_text.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_dnn.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_ml.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_video.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_calib3d.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_features2d.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_highgui.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_videoio.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_phase_unwrapping.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_flann.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_imgcodecs.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_objdetect.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_imgproc.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: /usr/local/lib/libopencv_core.so.3.4.0
cpp/OpenLicht/OpenLicht_3d: cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable OpenLicht_3d"
	cd /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/OpenLicht_3d.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/build: cpp/OpenLicht/OpenLicht_3d

.PHONY : cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/build

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/requires: cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/OpenLicht_3d.cpp.o.requires
cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/requires: cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/BeepSound.cpp.o.requires
cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/requires: cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth.cpp.o.requires
cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/requires: cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/Bluetooth_server.cpp.o.requires

.PHONY : cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/requires

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/clean:
	cd /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht && $(CMAKE_COMMAND) -P CMakeFiles/OpenLicht_3d.dir/cmake_clean.cmake
.PHONY : cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/clean

cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/depend:
	cd /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : cpp/OpenLicht/CMakeFiles/OpenLicht_3d.dir/depend

