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
CMAKE_SOURCE_DIR = /home/teeramoo/Desktop/ORB-slam-script/Point_cloud_segmentation/cloudVisualizer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/teeramoo/Desktop/ORB-slam-script/Point_cloud_segmentation/cloudVisualizer/build

# Include any dependencies generated for this target.
include CMakeFiles/cloudScaling.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cloudScaling.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cloudScaling.dir/flags.make

CMakeFiles/cloudScaling.dir/cloudScaling.cpp.o: CMakeFiles/cloudScaling.dir/flags.make
CMakeFiles/cloudScaling.dir/cloudScaling.cpp.o: ../cloudScaling.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/teeramoo/Desktop/ORB-slam-script/Point_cloud_segmentation/cloudVisualizer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cloudScaling.dir/cloudScaling.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cloudScaling.dir/cloudScaling.cpp.o -c /home/teeramoo/Desktop/ORB-slam-script/Point_cloud_segmentation/cloudVisualizer/cloudScaling.cpp

CMakeFiles/cloudScaling.dir/cloudScaling.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cloudScaling.dir/cloudScaling.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/teeramoo/Desktop/ORB-slam-script/Point_cloud_segmentation/cloudVisualizer/cloudScaling.cpp > CMakeFiles/cloudScaling.dir/cloudScaling.cpp.i

CMakeFiles/cloudScaling.dir/cloudScaling.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cloudScaling.dir/cloudScaling.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/teeramoo/Desktop/ORB-slam-script/Point_cloud_segmentation/cloudVisualizer/cloudScaling.cpp -o CMakeFiles/cloudScaling.dir/cloudScaling.cpp.s

CMakeFiles/cloudScaling.dir/cloudScaling.cpp.o.requires:

.PHONY : CMakeFiles/cloudScaling.dir/cloudScaling.cpp.o.requires

CMakeFiles/cloudScaling.dir/cloudScaling.cpp.o.provides: CMakeFiles/cloudScaling.dir/cloudScaling.cpp.o.requires
	$(MAKE) -f CMakeFiles/cloudScaling.dir/build.make CMakeFiles/cloudScaling.dir/cloudScaling.cpp.o.provides.build
.PHONY : CMakeFiles/cloudScaling.dir/cloudScaling.cpp.o.provides

CMakeFiles/cloudScaling.dir/cloudScaling.cpp.o.provides.build: CMakeFiles/cloudScaling.dir/cloudScaling.cpp.o


# Object files for target cloudScaling
cloudScaling_OBJECTS = \
"CMakeFiles/cloudScaling.dir/cloudScaling.cpp.o"

# External object files for target cloudScaling
cloudScaling_EXTERNAL_OBJECTS =

cloudScaling: CMakeFiles/cloudScaling.dir/cloudScaling.cpp.o
cloudScaling: CMakeFiles/cloudScaling.dir/build.make
cloudScaling: /usr/lib/x86_64-linux-gnu/libboost_system.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libboost_thread.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libboost_regex.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libpthread.so
cloudScaling: /usr/local/lib/libpcl_common.so
cloudScaling: /usr/local/lib/libpcl_octree.so
cloudScaling: /usr/lib/libOpenNI.so
cloudScaling: /usr/lib/libOpenNI2.so
cloudScaling: /usr/local/lib/libpcl_io.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
cloudScaling: /usr/local/lib/libpcl_kdtree.so
cloudScaling: /usr/local/lib/libpcl_search.so
cloudScaling: /usr/local/lib/libpcl_sample_consensus.so
cloudScaling: /usr/local/lib/libpcl_filters.so
cloudScaling: /usr/local/lib/libpcl_features.so
cloudScaling: /usr/local/lib/libpcl_ml.so
cloudScaling: /usr/local/lib/libpcl_segmentation.so
cloudScaling: /usr/local/lib/libpcl_visualization.so
cloudScaling: /usr/local/lib/libpcl_surface.so
cloudScaling: /usr/local/lib/libpcl_registration.so
cloudScaling: /usr/local/lib/libpcl_keypoints.so
cloudScaling: /usr/local/lib/libpcl_tracking.so
cloudScaling: /usr/local/lib/libpcl_recognition.so
cloudScaling: /usr/local/lib/libpcl_stereo.so
cloudScaling: /usr/local/lib/libpcl_outofcore.so
cloudScaling: /usr/local/lib/libpcl_people.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libboost_system.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libboost_thread.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libboost_regex.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libpthread.so
cloudScaling: /usr/lib/libOpenNI.so
cloudScaling: /usr/lib/libOpenNI2.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
cloudScaling: /usr/local/lib/libvtkIOExport-7.1.so.1
cloudScaling: /usr/local/lib/libvtkRenderingGL2PSOpenGL2-7.1.so.1
cloudScaling: /usr/local/lib/libvtkgl2ps-7.1.so.1
cloudScaling: /usr/local/lib/libvtkIOAMR-7.1.so.1
cloudScaling: /usr/local/lib/libvtkFiltersParallelImaging-7.1.so.1
cloudScaling: /usr/local/lib/libvtkFiltersPoints-7.1.so.1
cloudScaling: /usr/local/lib/libvtkIOParallelXML-7.1.so.1
cloudScaling: /usr/local/lib/libvtkIOTecplotTable-7.1.so.1
cloudScaling: /usr/local/lib/libvtkIOSQL-7.1.so.1
cloudScaling: /usr/local/lib/libvtksqlite-7.1.so.1
cloudScaling: /usr/local/lib/libvtkViewsInfovis-7.1.so.1
cloudScaling: /usr/local/lib/libvtkRenderingLabel-7.1.so.1
cloudScaling: /usr/local/lib/libvtkFiltersProgrammable-7.1.so.1
cloudScaling: /usr/local/lib/libvtkRenderingContextOpenGL2-7.1.so.1
cloudScaling: /usr/local/lib/libvtkFiltersTexture-7.1.so.1
cloudScaling: /usr/local/lib/libvtkIOPLY-7.1.so.1
cloudScaling: /usr/local/lib/libvtkDomainsChemistryOpenGL2-7.1.so.1
cloudScaling: /usr/local/lib/libvtkRenderingImage-7.1.so.1
cloudScaling: /usr/local/lib/libvtkInteractionImage-7.1.so.1
cloudScaling: /usr/local/lib/libvtkImagingMorphological-7.1.so.1
cloudScaling: /usr/local/lib/libvtkFiltersGeneric-7.1.so.1
cloudScaling: /usr/local/lib/libvtkIOEnSight-7.1.so.1
cloudScaling: /usr/local/lib/libvtkViewsContext2D-7.1.so.1
cloudScaling: /usr/local/lib/libvtkIOImport-7.1.so.1
cloudScaling: /usr/local/lib/libvtkIOInfovis-7.1.so.1
cloudScaling: /usr/local/lib/libvtklibxml2-7.1.so.1
cloudScaling: /usr/local/lib/libvtkFiltersVerdict-7.1.so.1
cloudScaling: /usr/local/lib/libvtkverdict-7.1.so.1
cloudScaling: /usr/local/lib/libvtkRenderingVolumeOpenGL2-7.1.so.1
cloudScaling: /usr/local/lib/libvtkFiltersSMP-7.1.so.1
cloudScaling: /usr/local/lib/libvtkIOVideo-7.1.so.1
cloudScaling: /usr/local/lib/libvtkFiltersHyperTree-7.1.so.1
cloudScaling: /usr/local/lib/libvtkIOParallel-7.1.so.1
cloudScaling: /usr/local/lib/libvtkRenderingLOD-7.1.so.1
cloudScaling: /usr/local/lib/libvtkIOMINC-7.1.so.1
cloudScaling: /usr/local/lib/libvtkFiltersSelection-7.1.so.1
cloudScaling: /usr/local/lib/libvtkIOMovie-7.1.so.1
cloudScaling: /usr/local/lib/libvtkoggtheora-7.1.so.1
cloudScaling: /usr/local/lib/libvtkImagingStencil-7.1.so.1
cloudScaling: /usr/local/lib/libvtkGeovisCore-7.1.so.1
cloudScaling: /usr/local/lib/libvtkImagingStatistics-7.1.so.1
cloudScaling: /usr/local/lib/libvtkIOLSDyna-7.1.so.1
cloudScaling: /usr/local/lib/libvtkFiltersFlowPaths-7.1.so.1
cloudScaling: /usr/local/lib/libvtkIOExodus-7.1.so.1
cloudScaling: /usr/local/lib/libpcl_common.so
cloudScaling: /usr/local/lib/libpcl_octree.so
cloudScaling: /usr/local/lib/libpcl_io.so
cloudScaling: /usr/local/lib/libpcl_kdtree.so
cloudScaling: /usr/local/lib/libpcl_search.so
cloudScaling: /usr/local/lib/libpcl_sample_consensus.so
cloudScaling: /usr/local/lib/libpcl_filters.so
cloudScaling: /usr/local/lib/libpcl_features.so
cloudScaling: /usr/local/lib/libpcl_ml.so
cloudScaling: /usr/local/lib/libpcl_segmentation.so
cloudScaling: /usr/local/lib/libpcl_visualization.so
cloudScaling: /usr/local/lib/libpcl_surface.so
cloudScaling: /usr/local/lib/libpcl_registration.so
cloudScaling: /usr/local/lib/libpcl_keypoints.so
cloudScaling: /usr/local/lib/libpcl_tracking.so
cloudScaling: /usr/local/lib/libpcl_recognition.so
cloudScaling: /usr/local/lib/libpcl_stereo.so
cloudScaling: /usr/local/lib/libpcl_outofcore.so
cloudScaling: /usr/local/lib/libpcl_people.so
cloudScaling: /usr/local/lib/libvtkFiltersAMR-7.1.so.1
cloudScaling: /usr/local/lib/libvtkFiltersImaging-7.1.so.1
cloudScaling: /usr/local/lib/libvtkChartsCore-7.1.so.1
cloudScaling: /usr/local/lib/libvtkDomainsChemistry-7.1.so.1
cloudScaling: /usr/local/lib/libvtkRenderingContext2D-7.1.so.1
cloudScaling: /usr/local/lib/libvtkRenderingOpenGL2-7.1.so.1
cloudScaling: /usr/lib/x86_64-linux-gnu/libSM.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libICE.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libX11.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libXext.so
cloudScaling: /usr/lib/x86_64-linux-gnu/libXt.so
cloudScaling: /usr/local/lib/libvtkglew-7.1.so.1
cloudScaling: /usr/local/lib/libvtkImagingMath-7.1.so.1
cloudScaling: /usr/local/lib/libvtkFiltersParallel-7.1.so.1
cloudScaling: /usr/local/lib/libvtkParallelCore-7.1.so.1
cloudScaling: /usr/local/lib/libvtkIONetCDF-7.1.so.1
cloudScaling: /usr/local/lib/libvtkIOGeometry-7.1.so.1
cloudScaling: /usr/local/lib/libvtkIOLegacy-7.1.so.1
cloudScaling: /usr/local/lib/libvtkjsoncpp-7.1.so.1
cloudScaling: /usr/local/lib/libvtkViewsCore-7.1.so.1
cloudScaling: /usr/local/lib/libvtkInteractionWidgets-7.1.so.1
cloudScaling: /usr/local/lib/libvtkImagingGeneral-7.1.so.1
cloudScaling: /usr/local/lib/libvtkFiltersHybrid-7.1.so.1
cloudScaling: /usr/local/lib/libvtkImagingSources-7.1.so.1
cloudScaling: /usr/local/lib/libvtkRenderingAnnotation-7.1.so.1
cloudScaling: /usr/local/lib/libvtkImagingColor-7.1.so.1
cloudScaling: /usr/local/lib/libvtkRenderingFreeType-7.1.so.1
cloudScaling: /usr/local/lib/libvtkfreetype-7.1.so.1
cloudScaling: /usr/local/lib/libvtkRenderingVolume-7.1.so.1
cloudScaling: /usr/local/lib/libvtkIOXML-7.1.so.1
cloudScaling: /usr/local/lib/libvtkInteractionStyle-7.1.so.1
cloudScaling: /usr/local/lib/libvtkRenderingCore-7.1.so.1
cloudScaling: /usr/local/lib/libvtkFiltersGeometry-7.1.so.1
cloudScaling: /usr/local/lib/libvtkCommonColor-7.1.so.1
cloudScaling: /usr/local/lib/libvtkInfovisLayout-7.1.so.1
cloudScaling: /usr/local/lib/libvtkImagingHybrid-7.1.so.1
cloudScaling: /usr/local/lib/libvtkIOImage-7.1.so.1
cloudScaling: /usr/local/lib/libvtkDICOMParser-7.1.so.1
cloudScaling: /usr/local/lib/libvtkmetaio-7.1.so.1
cloudScaling: /usr/local/lib/libvtkpng-7.1.so.1
cloudScaling: /usr/local/lib/libvtktiff-7.1.so.1
cloudScaling: /usr/local/lib/libvtkjpeg-7.1.so.1
cloudScaling: /usr/lib/x86_64-linux-gnu/libm.so
cloudScaling: /usr/local/lib/libvtkFiltersModeling-7.1.so.1
cloudScaling: /usr/local/lib/libvtkInfovisCore-7.1.so.1
cloudScaling: /usr/local/lib/libvtkFiltersExtraction-7.1.so.1
cloudScaling: /usr/local/lib/libvtkFiltersStatistics-7.1.so.1
cloudScaling: /usr/local/lib/libvtkImagingFourier-7.1.so.1
cloudScaling: /usr/local/lib/libvtkalglib-7.1.so.1
cloudScaling: /usr/local/lib/libvtkproj4-7.1.so.1
cloudScaling: /usr/local/lib/libvtkImagingCore-7.1.so.1
cloudScaling: /usr/local/lib/libvtkFiltersSources-7.1.so.1
cloudScaling: /usr/local/lib/libvtkFiltersGeneral-7.1.so.1
cloudScaling: /usr/local/lib/libvtkCommonComputationalGeometry-7.1.so.1
cloudScaling: /usr/local/lib/libvtkFiltersCore-7.1.so.1
cloudScaling: /usr/local/lib/libvtkIOXMLParser-7.1.so.1
cloudScaling: /usr/local/lib/libvtkIOCore-7.1.so.1
cloudScaling: /usr/local/lib/libvtkCommonExecutionModel-7.1.so.1
cloudScaling: /usr/local/lib/libvtkCommonDataModel-7.1.so.1
cloudScaling: /usr/local/lib/libvtkCommonTransforms-7.1.so.1
cloudScaling: /usr/local/lib/libvtkCommonMisc-7.1.so.1
cloudScaling: /usr/local/lib/libvtkCommonMath-7.1.so.1
cloudScaling: /usr/local/lib/libvtkCommonSystem-7.1.so.1
cloudScaling: /usr/local/lib/libvtkCommonCore-7.1.so.1
cloudScaling: /usr/local/lib/libvtksys-7.1.so.1
cloudScaling: /usr/local/lib/libvtkexpat-7.1.so.1
cloudScaling: /usr/local/lib/libvtkexoIIc-7.1.so.1
cloudScaling: /usr/local/lib/libvtkNetCDF_cxx-7.1.so.1
cloudScaling: /usr/local/lib/libvtkNetCDF-7.1.so.1
cloudScaling: /usr/local/lib/libvtkhdf5_hl-7.1.so.1
cloudScaling: /usr/local/lib/libvtkhdf5-7.1.so.1
cloudScaling: /usr/local/lib/libvtkzlib-7.1.so.1
cloudScaling: CMakeFiles/cloudScaling.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/teeramoo/Desktop/ORB-slam-script/Point_cloud_segmentation/cloudVisualizer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable cloudScaling"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cloudScaling.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cloudScaling.dir/build: cloudScaling

.PHONY : CMakeFiles/cloudScaling.dir/build

CMakeFiles/cloudScaling.dir/requires: CMakeFiles/cloudScaling.dir/cloudScaling.cpp.o.requires

.PHONY : CMakeFiles/cloudScaling.dir/requires

CMakeFiles/cloudScaling.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cloudScaling.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cloudScaling.dir/clean

CMakeFiles/cloudScaling.dir/depend:
	cd /home/teeramoo/Desktop/ORB-slam-script/Point_cloud_segmentation/cloudVisualizer/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/teeramoo/Desktop/ORB-slam-script/Point_cloud_segmentation/cloudVisualizer /home/teeramoo/Desktop/ORB-slam-script/Point_cloud_segmentation/cloudVisualizer /home/teeramoo/Desktop/ORB-slam-script/Point_cloud_segmentation/cloudVisualizer/build /home/teeramoo/Desktop/ORB-slam-script/Point_cloud_segmentation/cloudVisualizer/build /home/teeramoo/Desktop/ORB-slam-script/Point_cloud_segmentation/cloudVisualizer/build/CMakeFiles/cloudScaling.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cloudScaling.dir/depend

