# CMAKE generated file: DO NOT EDIT!
# Generated by "MSYS Makefiles" Generator, CMake Version 3.19

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /D/msys64/mingw64/bin/cmake.exe

# The command to remove a file.
RM = /D/msys64/mingw64/bin/cmake.exe -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /D/msys64/home/Olivier/webots/projects/robots/mobsya/thymio/libraries/dashel-src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /D/msys64/home/Olivier/webots/projects/robots/mobsya/thymio/libraries/dashel

# Include any dependencies generated for this target.
include examples/CMakeFiles/dws.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/dws.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/dws.dir/flags.make

examples/CMakeFiles/dws.dir/dws.cpp.obj: examples/CMakeFiles/dws.dir/flags.make
examples/CMakeFiles/dws.dir/dws.cpp.obj: D:/msys64/home/Olivier/webots/projects/robots/mobsya/thymio/libraries/dashel-src/examples/dws.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/D/msys64/home/Olivier/webots/projects/robots/mobsya/thymio/libraries/dashel/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/dws.dir/dws.cpp.obj"
	cd /D/msys64/home/Olivier/webots/projects/robots/mobsya/thymio/libraries/dashel/examples && /D/msys64/mingw64/bin/g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dws.dir/dws.cpp.obj -c /D/msys64/home/Olivier/webots/projects/robots/mobsya/thymio/libraries/dashel-src/examples/dws.cpp

examples/CMakeFiles/dws.dir/dws.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dws.dir/dws.cpp.i"
	cd /D/msys64/home/Olivier/webots/projects/robots/mobsya/thymio/libraries/dashel/examples && /D/msys64/mingw64/bin/g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /D/msys64/home/Olivier/webots/projects/robots/mobsya/thymio/libraries/dashel-src/examples/dws.cpp > CMakeFiles/dws.dir/dws.cpp.i

examples/CMakeFiles/dws.dir/dws.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dws.dir/dws.cpp.s"
	cd /D/msys64/home/Olivier/webots/projects/robots/mobsya/thymio/libraries/dashel/examples && /D/msys64/mingw64/bin/g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /D/msys64/home/Olivier/webots/projects/robots/mobsya/thymio/libraries/dashel-src/examples/dws.cpp -o CMakeFiles/dws.dir/dws.cpp.s

# Object files for target dws
dws_OBJECTS = \
"CMakeFiles/dws.dir/dws.cpp.obj"

# External object files for target dws
dws_EXTERNAL_OBJECTS =

examples/dws.exe: examples/CMakeFiles/dws.dir/dws.cpp.obj
examples/dws.exe: examples/CMakeFiles/dws.dir/build.make
examples/dws.exe: libdashel.dll.a
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/D/msys64/home/Olivier/webots/projects/robots/mobsya/thymio/libraries/dashel/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable dws.exe"
	cd /D/msys64/home/Olivier/webots/projects/robots/mobsya/thymio/libraries/dashel/examples && /D/msys64/mingw64/bin/cmake.exe -E rm -f CMakeFiles/dws.dir/objects.a
	cd /D/msys64/home/Olivier/webots/projects/robots/mobsya/thymio/libraries/dashel/examples && /D/msys64/mingw64/bin/ar.exe cr CMakeFiles/dws.dir/objects.a $(dws_OBJECTS) $(dws_EXTERNAL_OBJECTS)
	cd /D/msys64/home/Olivier/webots/projects/robots/mobsya/thymio/libraries/dashel/examples && /D/msys64/mingw64/bin/g++.exe -Wl,--whole-archive CMakeFiles/dws.dir/objects.a -Wl,--no-whole-archive -o dws.exe -Wl,--major-image-version,0,--minor-image-version,0  ../libdashel.dll.a -lwinspool -lws2_32 -lsetupapi -lkernel32 -luser32 -lgdi32 -lwinspool -lshell32 -lole32 -loleaut32 -luuid -lcomdlg32 -ladvapi32 

# Rule to build all files generated by this target.
examples/CMakeFiles/dws.dir/build: examples/dws.exe

.PHONY : examples/CMakeFiles/dws.dir/build

examples/CMakeFiles/dws.dir/clean:
	cd /D/msys64/home/Olivier/webots/projects/robots/mobsya/thymio/libraries/dashel/examples && $(CMAKE_COMMAND) -P CMakeFiles/dws.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/dws.dir/clean

examples/CMakeFiles/dws.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MSYS Makefiles" /D/msys64/home/Olivier/webots/projects/robots/mobsya/thymio/libraries/dashel-src /D/msys64/home/Olivier/webots/projects/robots/mobsya/thymio/libraries/dashel-src/examples /D/msys64/home/Olivier/webots/projects/robots/mobsya/thymio/libraries/dashel /D/msys64/home/Olivier/webots/projects/robots/mobsya/thymio/libraries/dashel/examples /D/msys64/home/Olivier/webots/projects/robots/mobsya/thymio/libraries/dashel/examples/CMakeFiles/dws.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/dws.dir/depend

