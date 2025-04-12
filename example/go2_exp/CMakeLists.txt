# add_executable(go2_pure_pursuit_client go2_pure_pursuit_client.cpp)
# find_package(Eigen3 REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIR})
# target_link_libraries(go2_pure_pursuit_client unitree_sdk2 zmq ${EIGEN3_LIBRARIES})


# add_executable(go2_track_traj go2_track_traj.cpp)
# find_package(Eigen3 REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIR})
# target_link_libraries(go2_track_traj unitree_sdk2 zmq ${EIGEN3_LIBRARIES})

add_executable(go2_pid go2_pid.cpp)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
target_link_libraries(go2_pid unitree_sdk2 zmq ${EIGEN3_LIBRARIES})