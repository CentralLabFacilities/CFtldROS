
link_directories(${OpenCV_LIB_DIR})

include_directories(main
    ../libopentld/imacq
    ../libopentld/mftracker
    ../libopentld/tld
    ${CF_HEADER_DIRS}
    ${OpenCV_INCLUDE_DIRS})

if(NOT USE_SYSTEM_LIBS)
    include_directories(../3rdparty/libconfig)
endif(NOT USE_SYSTEM_LIBS)

#-------------------------------------------------------------------------------
# main
add_library(main
    main/ros/ros_grabber.cpp
    main/ros/ros_grabber.hpp
    main/Config.cpp
    main/Gui.cpp
    main/Main.cpp
    main/Settings.cpp
    main/Trajectory.cpp
    main/Config.h
    main/Gui.h
    main/Main.h
    main/Settings.h
    main/Trajectory.h
    ../3rdparty/cf_tracking/src/cf_libs/common/cf_tracker.hpp)

target_link_libraries(main libopentld config++ ${OpenCV_LIBS})

#-------------------------------------------------------------------------------
# opentld
add_executable(cftld
    OpenTLD.cpp)

target_link_libraries(cftld main libopentld config++ ${OpenCV_LIBS} ${catkin_LIBRARIES})

# install(TARGETS cftld DESTINATION bin)

INSTALL(TARGETS cftld RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
INSTALL(TARGETS main
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# Remember to add launch files
# INSTALL(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/data" DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#-------------------------------------------------------------------------------


