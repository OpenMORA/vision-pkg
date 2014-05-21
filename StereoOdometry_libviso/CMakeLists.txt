# File: CMakeListst.txt for module StereoOdometry
PROJECT(StereoOdometry_libviso)

# Use MRPT:
FIND_PACKAGE( MRPT REQUIRED obs gui ) # hmtslam hwdrivers ... any other sub-library

INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS}  )
LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS} )

INCLUDE_DIRECTORIES(.) # Required to include <../*.h>

# libviso2 src directory:
SET(VISO_SRC ${${PROJECT_NAME}_SOURCE_DIR}/libviso/src)

INCLUDE_DIRECTORIES(${VISO_SRC})

# use sse3 instruction set
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX)
	SET(CMAKE_CXX_FLAGS "-msse3")
endif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX)

# Add executable
ADD_EXECUTABLE( ${PROJECT_NAME}
	main.cpp
	CStereoOdometryApp.h CStereoOdometryApp.cpp
	../StereoOdometry_common/CBaseStereoOdometryApp.cpp
	../StereoOdometry_common/CBaseStereoOdometryApp.h
	# LibVISO:
	${VISO_SRC}/filter.cpp 
	${VISO_SRC}/filter.h
	${VISO_SRC}/matcher.cpp
	${VISO_SRC}/matcher.h
	${VISO_SRC}/matrix.cpp
	${VISO_SRC}/matrix.h
	${VISO_SRC}/reconstruction.cpp
	${VISO_SRC}/reconstruction.h
	${VISO_SRC}/triangle.cpp
	${VISO_SRC}/triangle.h
	${VISO_SRC}/viso.cpp
	${VISO_SRC}/viso.h
	${VISO_SRC}/viso_mono.cpp
	${VISO_SRC}/viso_mono.h
	${VISO_SRC}/viso_stereo.cpp
	${VISO_SRC}/viso_stereo.h
	)

# Link the executable
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS} )
