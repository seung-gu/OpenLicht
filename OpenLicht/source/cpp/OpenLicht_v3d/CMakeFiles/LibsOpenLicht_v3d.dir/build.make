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
include cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/depend.make

# Include the progress variables for this target.
include cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/progress.make

# Include the compile flags for this target's objects.
include cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/flags.make

cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/main.cpp.o: cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/flags.make
cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/main.cpp.o: cpp/OpenLicht/OpenLicht_v3d/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/main.cpp.o"
	cd /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/OpenLicht_v3d && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LibsOpenLicht_v3d.dir/main.cpp.o -c /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/OpenLicht_v3d/main.cpp

cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LibsOpenLicht_v3d.dir/main.cpp.i"
	cd /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/OpenLicht_v3d && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/OpenLicht_v3d/main.cpp > CMakeFiles/LibsOpenLicht_v3d.dir/main.cpp.i

cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LibsOpenLicht_v3d.dir/main.cpp.s"
	cd /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/OpenLicht_v3d && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/OpenLicht_v3d/main.cpp -o CMakeFiles/LibsOpenLicht_v3d.dir/main.cpp.s

cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/main.cpp.o.requires:

.PHONY : cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/main.cpp.o.requires

cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/main.cpp.o.provides: cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/main.cpp.o.requires
	$(MAKE) -f cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/build.make cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/main.cpp.o.provides.build
.PHONY : cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/main.cpp.o.provides

cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/main.cpp.o.provides.build: cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/main.cpp.o


# Object files for target LibsOpenLicht_v3d
LibsOpenLicht_v3d_OBJECTS = \
"CMakeFiles/LibsOpenLicht_v3d.dir/main.cpp.o"

# External object files for target LibsOpenLicht_v3d
LibsOpenLicht_v3d_EXTERNAL_OBJECTS =

cpp/OpenLicht/OpenLicht_v3d/libLibsOpenLicht_v3d.a: cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/main.cpp.o
cpp/OpenLicht/OpenLicht_v3d/libLibsOpenLicht_v3d.a: cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/build.make
cpp/OpenLicht/OpenLicht_v3d/libLibsOpenLicht_v3d.a: cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libLibsOpenLicht_v3d.a"
	cd /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/OpenLicht_v3d && $(CMAKE_COMMAND) -P CMakeFiles/LibsOpenLicht_v3d.dir/cmake_clean_target.cmake
	cd /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/OpenLicht_v3d && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/LibsOpenLicht_v3d.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/build: cpp/OpenLicht/OpenLicht_v3d/libLibsOpenLicht_v3d.a

.PHONY : cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/build

cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/requires: cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/main.cpp.o.requires

.PHONY : cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/requires

cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/clean:
	cd /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/OpenLicht_v3d && $(CMAKE_COMMAND) -P CMakeFiles/LibsOpenLicht_v3d.dir/cmake_clean.cmake
.PHONY : cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/clean

cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/depend:
	cd /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/OpenLicht_v3d /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/OpenLicht_v3d /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : cpp/OpenLicht/OpenLicht_v3d/CMakeFiles/LibsOpenLicht_v3d.dir/depend

