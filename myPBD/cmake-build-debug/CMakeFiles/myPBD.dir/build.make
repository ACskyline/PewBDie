# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.9

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
CMAKE_COMMAND = /home/sean/Program/clion-2017.3.2/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/sean/Program/clion-2017.3.2/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/sean/File/Project/clion_projects/myPBD

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sean/File/Project/clion_projects/myPBD/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/myPBD.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/myPBD.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/myPBD.dir/flags.make

CMakeFiles/myPBD.dir/main.cpp.o: CMakeFiles/myPBD.dir/flags.make
CMakeFiles/myPBD.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sean/File/Project/clion_projects/myPBD/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/myPBD.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myPBD.dir/main.cpp.o -c /home/sean/File/Project/clion_projects/myPBD/main.cpp

CMakeFiles/myPBD.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myPBD.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sean/File/Project/clion_projects/myPBD/main.cpp > CMakeFiles/myPBD.dir/main.cpp.i

CMakeFiles/myPBD.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myPBD.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sean/File/Project/clion_projects/myPBD/main.cpp -o CMakeFiles/myPBD.dir/main.cpp.s

CMakeFiles/myPBD.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/myPBD.dir/main.cpp.o.requires

CMakeFiles/myPBD.dir/main.cpp.o.provides: CMakeFiles/myPBD.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/myPBD.dir/build.make CMakeFiles/myPBD.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/myPBD.dir/main.cpp.o.provides

CMakeFiles/myPBD.dir/main.cpp.o.provides.build: CMakeFiles/myPBD.dir/main.cpp.o


CMakeFiles/myPBD.dir/PBD.cpp.o: CMakeFiles/myPBD.dir/flags.make
CMakeFiles/myPBD.dir/PBD.cpp.o: ../PBD.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sean/File/Project/clion_projects/myPBD/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/myPBD.dir/PBD.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myPBD.dir/PBD.cpp.o -c /home/sean/File/Project/clion_projects/myPBD/PBD.cpp

CMakeFiles/myPBD.dir/PBD.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myPBD.dir/PBD.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sean/File/Project/clion_projects/myPBD/PBD.cpp > CMakeFiles/myPBD.dir/PBD.cpp.i

CMakeFiles/myPBD.dir/PBD.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myPBD.dir/PBD.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sean/File/Project/clion_projects/myPBD/PBD.cpp -o CMakeFiles/myPBD.dir/PBD.cpp.s

CMakeFiles/myPBD.dir/PBD.cpp.o.requires:

.PHONY : CMakeFiles/myPBD.dir/PBD.cpp.o.requires

CMakeFiles/myPBD.dir/PBD.cpp.o.provides: CMakeFiles/myPBD.dir/PBD.cpp.o.requires
	$(MAKE) -f CMakeFiles/myPBD.dir/build.make CMakeFiles/myPBD.dir/PBD.cpp.o.provides.build
.PHONY : CMakeFiles/myPBD.dir/PBD.cpp.o.provides

CMakeFiles/myPBD.dir/PBD.cpp.o.provides.build: CMakeFiles/myPBD.dir/PBD.cpp.o


CMakeFiles/myPBD.dir/Points.cpp.o: CMakeFiles/myPBD.dir/flags.make
CMakeFiles/myPBD.dir/Points.cpp.o: ../Points.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sean/File/Project/clion_projects/myPBD/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/myPBD.dir/Points.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myPBD.dir/Points.cpp.o -c /home/sean/File/Project/clion_projects/myPBD/Points.cpp

CMakeFiles/myPBD.dir/Points.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myPBD.dir/Points.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sean/File/Project/clion_projects/myPBD/Points.cpp > CMakeFiles/myPBD.dir/Points.cpp.i

CMakeFiles/myPBD.dir/Points.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myPBD.dir/Points.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sean/File/Project/clion_projects/myPBD/Points.cpp -o CMakeFiles/myPBD.dir/Points.cpp.s

CMakeFiles/myPBD.dir/Points.cpp.o.requires:

.PHONY : CMakeFiles/myPBD.dir/Points.cpp.o.requires

CMakeFiles/myPBD.dir/Points.cpp.o.provides: CMakeFiles/myPBD.dir/Points.cpp.o.requires
	$(MAKE) -f CMakeFiles/myPBD.dir/build.make CMakeFiles/myPBD.dir/Points.cpp.o.provides.build
.PHONY : CMakeFiles/myPBD.dir/Points.cpp.o.provides

CMakeFiles/myPBD.dir/Points.cpp.o.provides.build: CMakeFiles/myPBD.dir/Points.cpp.o


CMakeFiles/myPBD.dir/Constraint.cpp.o: CMakeFiles/myPBD.dir/flags.make
CMakeFiles/myPBD.dir/Constraint.cpp.o: ../Constraint.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sean/File/Project/clion_projects/myPBD/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/myPBD.dir/Constraint.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myPBD.dir/Constraint.cpp.o -c /home/sean/File/Project/clion_projects/myPBD/Constraint.cpp

CMakeFiles/myPBD.dir/Constraint.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myPBD.dir/Constraint.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sean/File/Project/clion_projects/myPBD/Constraint.cpp > CMakeFiles/myPBD.dir/Constraint.cpp.i

CMakeFiles/myPBD.dir/Constraint.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myPBD.dir/Constraint.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sean/File/Project/clion_projects/myPBD/Constraint.cpp -o CMakeFiles/myPBD.dir/Constraint.cpp.s

CMakeFiles/myPBD.dir/Constraint.cpp.o.requires:

.PHONY : CMakeFiles/myPBD.dir/Constraint.cpp.o.requires

CMakeFiles/myPBD.dir/Constraint.cpp.o.provides: CMakeFiles/myPBD.dir/Constraint.cpp.o.requires
	$(MAKE) -f CMakeFiles/myPBD.dir/build.make CMakeFiles/myPBD.dir/Constraint.cpp.o.provides.build
.PHONY : CMakeFiles/myPBD.dir/Constraint.cpp.o.provides

CMakeFiles/myPBD.dir/Constraint.cpp.o.provides.build: CMakeFiles/myPBD.dir/Constraint.cpp.o


CMakeFiles/myPBD.dir/Scene.cpp.o: CMakeFiles/myPBD.dir/flags.make
CMakeFiles/myPBD.dir/Scene.cpp.o: ../Scene.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sean/File/Project/clion_projects/myPBD/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/myPBD.dir/Scene.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myPBD.dir/Scene.cpp.o -c /home/sean/File/Project/clion_projects/myPBD/Scene.cpp

CMakeFiles/myPBD.dir/Scene.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myPBD.dir/Scene.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sean/File/Project/clion_projects/myPBD/Scene.cpp > CMakeFiles/myPBD.dir/Scene.cpp.i

CMakeFiles/myPBD.dir/Scene.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myPBD.dir/Scene.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sean/File/Project/clion_projects/myPBD/Scene.cpp -o CMakeFiles/myPBD.dir/Scene.cpp.s

CMakeFiles/myPBD.dir/Scene.cpp.o.requires:

.PHONY : CMakeFiles/myPBD.dir/Scene.cpp.o.requires

CMakeFiles/myPBD.dir/Scene.cpp.o.provides: CMakeFiles/myPBD.dir/Scene.cpp.o.requires
	$(MAKE) -f CMakeFiles/myPBD.dir/build.make CMakeFiles/myPBD.dir/Scene.cpp.o.provides.build
.PHONY : CMakeFiles/myPBD.dir/Scene.cpp.o.provides

CMakeFiles/myPBD.dir/Scene.cpp.o.provides.build: CMakeFiles/myPBD.dir/Scene.cpp.o


# Object files for target myPBD
myPBD_OBJECTS = \
"CMakeFiles/myPBD.dir/main.cpp.o" \
"CMakeFiles/myPBD.dir/PBD.cpp.o" \
"CMakeFiles/myPBD.dir/Points.cpp.o" \
"CMakeFiles/myPBD.dir/Constraint.cpp.o" \
"CMakeFiles/myPBD.dir/Scene.cpp.o"

# External object files for target myPBD
myPBD_EXTERNAL_OBJECTS =

myPBD: CMakeFiles/myPBD.dir/main.cpp.o
myPBD: CMakeFiles/myPBD.dir/PBD.cpp.o
myPBD: CMakeFiles/myPBD.dir/Points.cpp.o
myPBD: CMakeFiles/myPBD.dir/Constraint.cpp.o
myPBD: CMakeFiles/myPBD.dir/Scene.cpp.o
myPBD: CMakeFiles/myPBD.dir/build.make
myPBD: CMakeFiles/myPBD.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sean/File/Project/clion_projects/myPBD/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable myPBD"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/myPBD.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/myPBD.dir/build: myPBD

.PHONY : CMakeFiles/myPBD.dir/build

CMakeFiles/myPBD.dir/requires: CMakeFiles/myPBD.dir/main.cpp.o.requires
CMakeFiles/myPBD.dir/requires: CMakeFiles/myPBD.dir/PBD.cpp.o.requires
CMakeFiles/myPBD.dir/requires: CMakeFiles/myPBD.dir/Points.cpp.o.requires
CMakeFiles/myPBD.dir/requires: CMakeFiles/myPBD.dir/Constraint.cpp.o.requires
CMakeFiles/myPBD.dir/requires: CMakeFiles/myPBD.dir/Scene.cpp.o.requires

.PHONY : CMakeFiles/myPBD.dir/requires

CMakeFiles/myPBD.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/myPBD.dir/cmake_clean.cmake
.PHONY : CMakeFiles/myPBD.dir/clean

CMakeFiles/myPBD.dir/depend:
	cd /home/sean/File/Project/clion_projects/myPBD/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sean/File/Project/clion_projects/myPBD /home/sean/File/Project/clion_projects/myPBD /home/sean/File/Project/clion_projects/myPBD/cmake-build-debug /home/sean/File/Project/clion_projects/myPBD/cmake-build-debug /home/sean/File/Project/clion_projects/myPBD/cmake-build-debug/CMakeFiles/myPBD.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/myPBD.dir/depend

