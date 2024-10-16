# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/hustlyrm/AIM24qmy

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hustlyrm/AIM24qmy/cmake-build-debug-hostss

# Include any dependencies generated for this target.
include modules/solver/CMakeFiles/solver.dir/depend.make

# Include the progress variables for this target.
include modules/solver/CMakeFiles/solver.dir/progress.make

# Include the compile flags for this target's objects.
include modules/solver/CMakeFiles/solver.dir/flags.make

modules/solver/CMakeFiles/solver.dir/Sources/AccuratePNPSolver.cpp.o: modules/solver/CMakeFiles/solver.dir/flags.make
modules/solver/CMakeFiles/solver.dir/Sources/AccuratePNPSolver.cpp.o: ../modules/solver/Sources/AccuratePNPSolver.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hustlyrm/AIM24qmy/cmake-build-debug-hostss/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object modules/solver/CMakeFiles/solver.dir/Sources/AccuratePNPSolver.cpp.o"
	cd /home/hustlyrm/AIM24qmy/cmake-build-debug-hostss/modules/solver && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/solver.dir/Sources/AccuratePNPSolver.cpp.o -c /home/hustlyrm/AIM24qmy/modules/solver/Sources/AccuratePNPSolver.cpp

modules/solver/CMakeFiles/solver.dir/Sources/AccuratePNPSolver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/solver.dir/Sources/AccuratePNPSolver.cpp.i"
	cd /home/hustlyrm/AIM24qmy/cmake-build-debug-hostss/modules/solver && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hustlyrm/AIM24qmy/modules/solver/Sources/AccuratePNPSolver.cpp > CMakeFiles/solver.dir/Sources/AccuratePNPSolver.cpp.i

modules/solver/CMakeFiles/solver.dir/Sources/AccuratePNPSolver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/solver.dir/Sources/AccuratePNPSolver.cpp.s"
	cd /home/hustlyrm/AIM24qmy/cmake-build-debug-hostss/modules/solver && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hustlyrm/AIM24qmy/modules/solver/Sources/AccuratePNPSolver.cpp -o CMakeFiles/solver.dir/Sources/AccuratePNPSolver.cpp.s

modules/solver/CMakeFiles/solver.dir/Sources/NormalPNPSolver.cpp.o: modules/solver/CMakeFiles/solver.dir/flags.make
modules/solver/CMakeFiles/solver.dir/Sources/NormalPNPSolver.cpp.o: ../modules/solver/Sources/NormalPNPSolver.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hustlyrm/AIM24qmy/cmake-build-debug-hostss/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object modules/solver/CMakeFiles/solver.dir/Sources/NormalPNPSolver.cpp.o"
	cd /home/hustlyrm/AIM24qmy/cmake-build-debug-hostss/modules/solver && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/solver.dir/Sources/NormalPNPSolver.cpp.o -c /home/hustlyrm/AIM24qmy/modules/solver/Sources/NormalPNPSolver.cpp

modules/solver/CMakeFiles/solver.dir/Sources/NormalPNPSolver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/solver.dir/Sources/NormalPNPSolver.cpp.i"
	cd /home/hustlyrm/AIM24qmy/cmake-build-debug-hostss/modules/solver && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hustlyrm/AIM24qmy/modules/solver/Sources/NormalPNPSolver.cpp > CMakeFiles/solver.dir/Sources/NormalPNPSolver.cpp.i

modules/solver/CMakeFiles/solver.dir/Sources/NormalPNPSolver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/solver.dir/Sources/NormalPNPSolver.cpp.s"
	cd /home/hustlyrm/AIM24qmy/cmake-build-debug-hostss/modules/solver && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hustlyrm/AIM24qmy/modules/solver/Sources/NormalPNPSolver.cpp -o CMakeFiles/solver.dir/Sources/NormalPNPSolver.cpp.s

# Object files for target solver
solver_OBJECTS = \
"CMakeFiles/solver.dir/Sources/AccuratePNPSolver.cpp.o" \
"CMakeFiles/solver.dir/Sources/NormalPNPSolver.cpp.o"

# External object files for target solver
solver_EXTERNAL_OBJECTS =

modules/solver/libsolver.a: modules/solver/CMakeFiles/solver.dir/Sources/AccuratePNPSolver.cpp.o
modules/solver/libsolver.a: modules/solver/CMakeFiles/solver.dir/Sources/NormalPNPSolver.cpp.o
modules/solver/libsolver.a: modules/solver/CMakeFiles/solver.dir/build.make
modules/solver/libsolver.a: modules/solver/CMakeFiles/solver.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hustlyrm/AIM24qmy/cmake-build-debug-hostss/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libsolver.a"
	cd /home/hustlyrm/AIM24qmy/cmake-build-debug-hostss/modules/solver && $(CMAKE_COMMAND) -P CMakeFiles/solver.dir/cmake_clean_target.cmake
	cd /home/hustlyrm/AIM24qmy/cmake-build-debug-hostss/modules/solver && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/solver.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
modules/solver/CMakeFiles/solver.dir/build: modules/solver/libsolver.a

.PHONY : modules/solver/CMakeFiles/solver.dir/build

modules/solver/CMakeFiles/solver.dir/clean:
	cd /home/hustlyrm/AIM24qmy/cmake-build-debug-hostss/modules/solver && $(CMAKE_COMMAND) -P CMakeFiles/solver.dir/cmake_clean.cmake
.PHONY : modules/solver/CMakeFiles/solver.dir/clean

modules/solver/CMakeFiles/solver.dir/depend:
	cd /home/hustlyrm/AIM24qmy/cmake-build-debug-hostss && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hustlyrm/AIM24qmy /home/hustlyrm/AIM24qmy/modules/solver /home/hustlyrm/AIM24qmy/cmake-build-debug-hostss /home/hustlyrm/AIM24qmy/cmake-build-debug-hostss/modules/solver /home/hustlyrm/AIM24qmy/cmake-build-debug-hostss/modules/solver/CMakeFiles/solver.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : modules/solver/CMakeFiles/solver.dir/depend

