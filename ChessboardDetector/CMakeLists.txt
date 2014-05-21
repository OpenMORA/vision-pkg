# File: CMakeListst.txt for module ChessboardDetector
PROJECT(ChessboardDetector)

# Use MRPT:
FIND_PACKAGE( MRPT REQUIRED vision gui ) # hmtslam hwdrivers ... any other sub-library

INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS}  )
LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS} )

# Add executable
ADD_EXECUTABLE( ${PROJECT_NAME}
	main.cpp
	CChessboardDetectorApp.h CChessboardDetectorApp.cpp
	)

# Link the executable
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS} )
