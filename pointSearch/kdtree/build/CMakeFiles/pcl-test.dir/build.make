# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/charbel/Projects/PCLtest/PCLtutorial/pointSearch/kdtree

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/charbel/Projects/PCLtest/PCLtutorial/pointSearch/kdtree/build

# Include any dependencies generated for this target.
include CMakeFiles/pcl-test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pcl-test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pcl-test.dir/flags.make

CMakeFiles/pcl-test.dir/main.cpp.o: CMakeFiles/pcl-test.dir/flags.make
CMakeFiles/pcl-test.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/charbel/Projects/PCLtest/PCLtutorial/pointSearch/kdtree/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pcl-test.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pcl-test.dir/main.cpp.o -c /home/charbel/Projects/PCLtest/PCLtutorial/pointSearch/kdtree/main.cpp

CMakeFiles/pcl-test.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pcl-test.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/charbel/Projects/PCLtest/PCLtutorial/pointSearch/kdtree/main.cpp > CMakeFiles/pcl-test.dir/main.cpp.i

CMakeFiles/pcl-test.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pcl-test.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/charbel/Projects/PCLtest/PCLtutorial/pointSearch/kdtree/main.cpp -o CMakeFiles/pcl-test.dir/main.cpp.s

CMakeFiles/pcl-test.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/pcl-test.dir/main.cpp.o.requires

CMakeFiles/pcl-test.dir/main.cpp.o.provides: CMakeFiles/pcl-test.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/pcl-test.dir/build.make CMakeFiles/pcl-test.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/pcl-test.dir/main.cpp.o.provides

CMakeFiles/pcl-test.dir/main.cpp.o.provides.build: CMakeFiles/pcl-test.dir/main.cpp.o


# Object files for target pcl-test
pcl__test_OBJECTS = \
"CMakeFiles/pcl-test.dir/main.cpp.o"

# External object files for target pcl-test
pcl__test_EXTERNAL_OBJECTS =

pcl-test: CMakeFiles/pcl-test.dir/main.cpp.o
pcl-test: CMakeFiles/pcl-test.dir/build.make
pcl-test: /usr/lib/x86_64-linux-gnu/libboost_system.so
pcl-test: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
pcl-test: /usr/lib/x86_64-linux-gnu/libboost_thread.so
pcl-test: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
pcl-test: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
pcl-test: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
pcl-test: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
pcl-test: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
pcl-test: /usr/lib/x86_64-linux-gnu/libboost_regex.so
pcl-test: /usr/lib/x86_64-linux-gnu/libpthread.so
pcl-test: /usr/local/lib/libpcl_common.so
pcl-test: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
pcl-test: /usr/local/lib/libpcl_kdtree.so
pcl-test: /usr/local/lib/libpcl_octree.so
pcl-test: /usr/local/lib/libpcl_search.so
pcl-test: /usr/lib/x86_64-linux-gnu/libqhull.so
pcl-test: /usr/local/lib/libpcl_surface.so
pcl-test: /usr/local/lib/libpcl_io.so
pcl-test: /usr/local/lib/libpcl_sample_consensus.so
pcl-test: /usr/local/lib/libpcl_filters.so
pcl-test: /usr/local/lib/libpcl_features.so
pcl-test: /usr/local/lib/libpcl_keypoints.so
pcl-test: /usr/local/lib/libpcl_ml.so
pcl-test: /usr/local/lib/libpcl_segmentation.so
pcl-test: /usr/local/lib/libpcl_visualization.so
pcl-test: /usr/local/lib/libpcl_outofcore.so
pcl-test: /usr/local/lib/libpcl_stereo.so
pcl-test: /usr/local/lib/libpcl_registration.so
pcl-test: /usr/local/lib/libpcl_recognition.so
pcl-test: /usr/local/lib/libpcl_people.so
pcl-test: /usr/local/lib/libpcl_tracking.so
pcl-test: /usr/lib/x86_64-linux-gnu/libboost_system.so
pcl-test: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
pcl-test: /usr/lib/x86_64-linux-gnu/libboost_thread.so
pcl-test: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
pcl-test: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
pcl-test: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
pcl-test: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
pcl-test: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
pcl-test: /usr/lib/x86_64-linux-gnu/libboost_regex.so
pcl-test: /usr/lib/x86_64-linux-gnu/libpthread.so
pcl-test: /usr/lib/x86_64-linux-gnu/libqhull.so
pcl-test: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
pcl-test: /usr/local/lib/libvtkIOExodus-7.1.so.1
pcl-test: /usr/local/lib/libvtkIOVideo-7.1.so.1
pcl-test: /usr/local/lib/libvtkViewsContext2D-7.1.so.1
pcl-test: /usr/local/lib/libvtkViewsInfovis-7.1.so.1
pcl-test: /usr/local/lib/libvtkChartsCore-7.1.so.1
pcl-test: /usr/local/lib/libvtkRenderingLabel-7.1.so.1
pcl-test: /usr/local/lib/libvtkIOImport-7.1.so.1
pcl-test: /usr/local/lib/libvtkFiltersGeneric-7.1.so.1
pcl-test: /usr/local/lib/libvtkFiltersParallelImaging-7.1.so.1
pcl-test: /usr/local/lib/libvtkIOPLY-7.1.so.1
pcl-test: /usr/local/lib/libvtkRenderingImage-7.1.so.1
pcl-test: /usr/local/lib/libvtkFiltersTexture-7.1.so.1
pcl-test: /usr/local/lib/libvtkFiltersVerdict-7.1.so.1
pcl-test: /usr/local/lib/libvtkFiltersPoints-7.1.so.1
pcl-test: /usr/local/lib/libvtkRenderingLOD-7.1.so.1
pcl-test: /usr/local/lib/libvtkFiltersFlowPaths-7.1.so.1
pcl-test: /usr/local/lib/libvtkImagingStatistics-7.1.so.1
pcl-test: /usr/local/lib/libvtkRenderingVolumeOpenGL2-7.1.so.1
pcl-test: /usr/local/lib/libvtkImagingMath-7.1.so.1
pcl-test: /usr/local/lib/libvtkIOMINC-7.1.so.1
pcl-test: /usr/local/lib/libvtkImagingMorphological-7.1.so.1
pcl-test: /usr/local/lib/libvtkIOInfovis-7.1.so.1
pcl-test: /usr/local/lib/libvtklibxml2-7.1.so.1
pcl-test: /usr/local/lib/libvtkFiltersSelection-7.1.so.1
pcl-test: /usr/local/lib/libvtkIOEnSight-7.1.so.1
pcl-test: /usr/local/lib/libvtkIOParallel-7.1.so.1
pcl-test: /usr/local/lib/libvtkIONetCDF-7.1.so.1
pcl-test: /usr/local/lib/libvtkIOParallelXML-7.1.so.1
pcl-test: /usr/local/lib/libvtkFiltersHyperTree-7.1.so.1
pcl-test: /usr/local/lib/libvtkIOMovie-7.1.so.1
pcl-test: /usr/local/lib/libvtkIOSQL-7.1.so.1
pcl-test: /usr/local/lib/libvtksqlite-7.1.so.1
pcl-test: /usr/local/lib/libvtkDomainsChemistryOpenGL2-7.1.so.1
pcl-test: /usr/local/lib/libvtkIOAMR-7.1.so.1
pcl-test: /usr/local/lib/libvtkRenderingContextOpenGL2-7.1.so.1
pcl-test: /usr/local/lib/libvtkFiltersProgrammable-7.1.so.1
pcl-test: /usr/local/lib/libvtkIOExport-7.1.so.1
pcl-test: /usr/local/lib/libvtkRenderingGL2PSOpenGL2-7.1.so.1
pcl-test: /usr/local/lib/libvtkgl2ps-7.1.so.1
pcl-test: /usr/local/lib/libvtkFiltersSMP-7.1.so.1
pcl-test: /usr/local/lib/libvtkGeovisCore-7.1.so.1
pcl-test: /usr/local/lib/libvtkImagingStencil-7.1.so.1
pcl-test: /usr/local/lib/libvtkIOLSDyna-7.1.so.1
pcl-test: /usr/local/lib/libvtkIOTecplotTable-7.1.so.1
pcl-test: /usr/local/lib/libvtkInteractionImage-7.1.so.1
pcl-test: /usr/local/lib/libpcl_common.so
pcl-test: /usr/local/lib/libpcl_kdtree.so
pcl-test: /usr/local/lib/libpcl_octree.so
pcl-test: /usr/local/lib/libpcl_search.so
pcl-test: /usr/local/lib/libpcl_surface.so
pcl-test: /usr/local/lib/libpcl_io.so
pcl-test: /usr/local/lib/libpcl_sample_consensus.so
pcl-test: /usr/local/lib/libpcl_filters.so
pcl-test: /usr/local/lib/libpcl_features.so
pcl-test: /usr/local/lib/libpcl_keypoints.so
pcl-test: /usr/local/lib/libpcl_ml.so
pcl-test: /usr/local/lib/libpcl_segmentation.so
pcl-test: /usr/local/lib/libpcl_visualization.so
pcl-test: /usr/local/lib/libpcl_outofcore.so
pcl-test: /usr/local/lib/libpcl_stereo.so
pcl-test: /usr/local/lib/libpcl_registration.so
pcl-test: /usr/local/lib/libpcl_recognition.so
pcl-test: /usr/local/lib/libpcl_people.so
pcl-test: /usr/local/lib/libpcl_tracking.so
pcl-test: /usr/local/lib/libvtkFiltersImaging-7.1.so.1
pcl-test: /usr/local/lib/libvtkverdict-7.1.so.1
pcl-test: /usr/local/lib/libvtkexoIIc-7.1.so.1
pcl-test: /usr/local/lib/libvtkFiltersParallel-7.1.so.1
pcl-test: /usr/local/lib/libvtkIOGeometry-7.1.so.1
pcl-test: /usr/local/lib/libvtkjsoncpp-7.1.so.1
pcl-test: /usr/local/lib/libvtkNetCDF_cxx-7.1.so.1
pcl-test: /usr/local/lib/libvtkNetCDF-7.1.so.1
pcl-test: /usr/local/lib/libvtkoggtheora-7.1.so.1
pcl-test: /usr/local/lib/libvtkDomainsChemistry-7.1.so.1
pcl-test: /usr/local/lib/libvtkhdf5_hl-7.1.so.1
pcl-test: /usr/local/lib/libvtkhdf5-7.1.so.1
pcl-test: /usr/local/lib/libvtkFiltersAMR-7.1.so.1
pcl-test: /usr/local/lib/libvtkParallelCore-7.1.so.1
pcl-test: /usr/local/lib/libvtkIOLegacy-7.1.so.1
pcl-test: /usr/local/lib/libvtkRenderingContext2D-7.1.so.1
pcl-test: /usr/local/lib/libvtkRenderingOpenGL2-7.1.so.1
pcl-test: /usr/local/lib/libvtkglew-7.1.so.1
pcl-test: /usr/lib/x86_64-linux-gnu/libSM.so
pcl-test: /usr/lib/x86_64-linux-gnu/libICE.so
pcl-test: /usr/lib/x86_64-linux-gnu/libX11.so
pcl-test: /usr/lib/x86_64-linux-gnu/libXext.so
pcl-test: /usr/lib/x86_64-linux-gnu/libXt.so
pcl-test: /usr/local/lib/libvtkViewsCore-7.1.so.1
pcl-test: /usr/local/lib/libvtkInfovisLayout-7.1.so.1
pcl-test: /usr/local/lib/libvtkInfovisCore-7.1.so.1
pcl-test: /usr/local/lib/libvtkproj4-7.1.so.1
pcl-test: /usr/local/lib/libvtkInteractionWidgets-7.1.so.1
pcl-test: /usr/local/lib/libvtkImagingGeneral-7.1.so.1
pcl-test: /usr/local/lib/libvtkFiltersHybrid-7.1.so.1
pcl-test: /usr/local/lib/libvtkImagingSources-7.1.so.1
pcl-test: /usr/local/lib/libvtkFiltersModeling-7.1.so.1
pcl-test: /usr/local/lib/libvtkImagingHybrid-7.1.so.1
pcl-test: /usr/local/lib/libvtkIOImage-7.1.so.1
pcl-test: /usr/local/lib/libvtkDICOMParser-7.1.so.1
pcl-test: /usr/local/lib/libvtkmetaio-7.1.so.1
pcl-test: /usr/local/lib/libvtkpng-7.1.so.1
pcl-test: /usr/local/lib/libvtktiff-7.1.so.1
pcl-test: /usr/local/lib/libvtkjpeg-7.1.so.1
pcl-test: /usr/lib/x86_64-linux-gnu/libm.so
pcl-test: /usr/local/lib/libvtkRenderingAnnotation-7.1.so.1
pcl-test: /usr/local/lib/libvtkRenderingFreeType-7.1.so.1
pcl-test: /usr/local/lib/libvtkfreetype-7.1.so.1
pcl-test: /usr/local/lib/libvtkRenderingVolume-7.1.so.1
pcl-test: /usr/local/lib/libvtkIOXML-7.1.so.1
pcl-test: /usr/local/lib/libvtkIOXMLParser-7.1.so.1
pcl-test: /usr/local/lib/libvtkIOCore-7.1.so.1
pcl-test: /usr/local/lib/libvtkzlib-7.1.so.1
pcl-test: /usr/local/lib/libvtkexpat-7.1.so.1
pcl-test: /usr/local/lib/libvtkImagingColor-7.1.so.1
pcl-test: /usr/local/lib/libvtkInteractionStyle-7.1.so.1
pcl-test: /usr/local/lib/libvtkRenderingCore-7.1.so.1
pcl-test: /usr/local/lib/libvtkCommonColor-7.1.so.1
pcl-test: /usr/local/lib/libvtkFiltersGeometry-7.1.so.1
pcl-test: /usr/local/lib/libvtkFiltersSources-7.1.so.1
pcl-test: /usr/local/lib/libvtkFiltersExtraction-7.1.so.1
pcl-test: /usr/local/lib/libvtkFiltersGeneral-7.1.so.1
pcl-test: /usr/local/lib/libvtkFiltersCore-7.1.so.1
pcl-test: /usr/local/lib/libvtkCommonComputationalGeometry-7.1.so.1
pcl-test: /usr/local/lib/libvtkFiltersStatistics-7.1.so.1
pcl-test: /usr/local/lib/libvtkImagingFourier-7.1.so.1
pcl-test: /usr/local/lib/libvtkImagingCore-7.1.so.1
pcl-test: /usr/local/lib/libvtkCommonExecutionModel-7.1.so.1
pcl-test: /usr/local/lib/libvtkCommonDataModel-7.1.so.1
pcl-test: /usr/local/lib/libvtkCommonTransforms-7.1.so.1
pcl-test: /usr/local/lib/libvtkCommonMisc-7.1.so.1
pcl-test: /usr/local/lib/libvtkCommonMath-7.1.so.1
pcl-test: /usr/local/lib/libvtkCommonSystem-7.1.so.1
pcl-test: /usr/local/lib/libvtkCommonCore-7.1.so.1
pcl-test: /usr/local/lib/libvtksys-7.1.so.1
pcl-test: /usr/local/lib/libvtkalglib-7.1.so.1
pcl-test: CMakeFiles/pcl-test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/charbel/Projects/PCLtest/PCLtutorial/pointSearch/kdtree/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable pcl-test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pcl-test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pcl-test.dir/build: pcl-test

.PHONY : CMakeFiles/pcl-test.dir/build

CMakeFiles/pcl-test.dir/requires: CMakeFiles/pcl-test.dir/main.cpp.o.requires

.PHONY : CMakeFiles/pcl-test.dir/requires

CMakeFiles/pcl-test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pcl-test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pcl-test.dir/clean

CMakeFiles/pcl-test.dir/depend:
	cd /home/charbel/Projects/PCLtest/PCLtutorial/pointSearch/kdtree/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/charbel/Projects/PCLtest/PCLtutorial/pointSearch/kdtree /home/charbel/Projects/PCLtest/PCLtutorial/pointSearch/kdtree /home/charbel/Projects/PCLtest/PCLtutorial/pointSearch/kdtree/build /home/charbel/Projects/PCLtest/PCLtutorial/pointSearch/kdtree/build /home/charbel/Projects/PCLtest/PCLtutorial/pointSearch/kdtree/build/CMakeFiles/pcl-test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pcl-test.dir/depend

