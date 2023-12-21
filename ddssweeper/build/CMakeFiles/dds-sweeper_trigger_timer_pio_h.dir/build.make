# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/atripathi/Desktop/9th_Semester_Project/RPI_PICO/ddssweeper

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/atripathi/Desktop/9th_Semester_Project/RPI_PICO/ddssweeper/build

# Utility rule file for dds-sweeper_trigger_timer_pio_h.

# Include any custom commands dependencies for this target.
include CMakeFiles/dds-sweeper_trigger_timer_pio_h.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/dds-sweeper_trigger_timer_pio_h.dir/progress.make

CMakeFiles/dds-sweeper_trigger_timer_pio_h: trigger_timer.pio.h

trigger_timer.pio.h: /home/atripathi/Desktop/9th_Semester_Project/RPI_PICO/ddssweeper/trigger_timer.pio
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/atripathi/Desktop/9th_Semester_Project/RPI_PICO/ddssweeper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating trigger_timer.pio.h"
	pioasm/pioasm -o c-sdk /home/atripathi/Desktop/9th_Semester_Project/RPI_PICO/ddssweeper/trigger_timer.pio /home/atripathi/Desktop/9th_Semester_Project/RPI_PICO/ddssweeper/build/trigger_timer.pio.h

dds-sweeper_trigger_timer_pio_h: CMakeFiles/dds-sweeper_trigger_timer_pio_h
dds-sweeper_trigger_timer_pio_h: trigger_timer.pio.h
dds-sweeper_trigger_timer_pio_h: CMakeFiles/dds-sweeper_trigger_timer_pio_h.dir/build.make
.PHONY : dds-sweeper_trigger_timer_pio_h

# Rule to build all files generated by this target.
CMakeFiles/dds-sweeper_trigger_timer_pio_h.dir/build: dds-sweeper_trigger_timer_pio_h
.PHONY : CMakeFiles/dds-sweeper_trigger_timer_pio_h.dir/build

CMakeFiles/dds-sweeper_trigger_timer_pio_h.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/dds-sweeper_trigger_timer_pio_h.dir/cmake_clean.cmake
.PHONY : CMakeFiles/dds-sweeper_trigger_timer_pio_h.dir/clean

CMakeFiles/dds-sweeper_trigger_timer_pio_h.dir/depend:
	cd /home/atripathi/Desktop/9th_Semester_Project/RPI_PICO/ddssweeper/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/atripathi/Desktop/9th_Semester_Project/RPI_PICO/ddssweeper /home/atripathi/Desktop/9th_Semester_Project/RPI_PICO/ddssweeper /home/atripathi/Desktop/9th_Semester_Project/RPI_PICO/ddssweeper/build /home/atripathi/Desktop/9th_Semester_Project/RPI_PICO/ddssweeper/build /home/atripathi/Desktop/9th_Semester_Project/RPI_PICO/ddssweeper/build/CMakeFiles/dds-sweeper_trigger_timer_pio_h.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/dds-sweeper_trigger_timer_pio_h.dir/depend
