# --------------------------------------------------------------------------- #
# Common deps
# --------------------------------------------------------------------------- #
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# --------------------------------------------------------------------------- #
# Vicon DataStream SDK (built from source under ${PROJECT_SOURCE_DIR}/vicon_sdk)
# Mirrors the build pattern used by ethz-asl/vicon_bridge.
# --------------------------------------------------------------------------- #
set(VICON_SDK_ROOT ${PROJECT_SOURCE_DIR}/vicon_sdk)
if(EXISTS ${VICON_SDK_ROOT}/DataStream/ViconDataStreamSDK_CPP/DataStreamClient.h)
    find_package(Boost REQUIRED COMPONENTS thread date_time system)

    file(GLOB_RECURSE VICON_SDK_SOURCES "${VICON_SDK_ROOT}/**/**.cpp")
    add_library(vicon_sdk STATIC ${VICON_SDK_SOURCES})
    target_include_directories(vicon_sdk PUBLIC
        ${VICON_SDK_ROOT}
        ${VICON_SDK_ROOT}/DataStream
        ${Boost_INCLUDE_DIRS}
    )
    target_link_libraries(vicon_sdk PUBLIC ${Boost_LIBRARIES} pthread)
    set_target_properties(vicon_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)

    # Reusable wrapper around the SDK.
    add_library(vicon_client STATIC vicon_client.cpp)
    target_link_libraries(vicon_client PUBLIC vicon_sdk ${EIGEN3_LIBRARIES})
    set_target_properties(vicon_client PROPERTIES POSITION_INDEPENDENT_CODE ON)

    # Standalone test program: connects to Vicon Tracker and prints poses.
    add_executable(vicon_test vicon_test.cpp)
    target_link_libraries(vicon_test vicon_client)

    set(VICON_AVAILABLE TRUE)
else()
    message(WARNING "vicon_sdk not found under ${VICON_SDK_ROOT}; skipping vicon targets.")
    set(VICON_AVAILABLE FALSE)
endif()

# --------------------------------------------------------------------------- #
# Existing executables
# --------------------------------------------------------------------------- #

# go2_pid requires libcurl, OpenCV, and the RealSense SDK (librealsense2).
# Re-enable this block once those dependencies are installed.
# add_executable(go2_pid go2_pid.cpp)
#
# # sudo apt-get install libcurl4-openssl-dev
# find_package(CURL REQUIRED)
# include_directories(${CURL_INCLUDE_DIRS})
#
# find_package(OpenCV REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS})
#
# find_package(realsense2 REQUIRED)
# include_directories(${realsense2_INCLUDE_DIRS})
#
# target_link_libraries(go2_pid unitree_sdk2 zmq ${EIGEN3_LIBRARIES} ${CURL_LIBRARIES} ${OpenCV_LIBS} realsense2::realsense2)

add_executable(lunarlab_go2_waypoint_follow lunarlab_go2_waypoint_follow.cpp)
target_link_libraries(lunarlab_go2_waypoint_follow unitree_sdk2 vicon_client ${EIGEN3_LIBRARIES})
