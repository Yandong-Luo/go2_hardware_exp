// #include <iostream>
// #include <vector>
// #include <cmath>
// #include <Eigen/Dense>

// #define DEBUG_MODE true

// struct Pose {
//     Eigen::Vector3d position;
//     Eigen::Quaterniond orientation;
    
//     Pose() : position(0, 0, 0), orientation(Eigen::Quaterniond::Identity()) {}
    
//     Pose(const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient) 
//             : position(pos), orientation(orient) {}
    
//     Pose(double x, double y, double z, const Eigen::Quaterniond& orient) 
//             : position(x, y, z), orientation(orient) {}
// };

// class Waypoints {
// private:
//     struct Point2D {
//             double x, y, yaw;
            
//             Point2D() : x(0.0), y(0.0), yaw(0.0) {}
//             Point2D(double _x, double _y, double _yaw = 0.0) : x(_x), y(_y), yaw(_yaw) {}
            
//             Eigen::Vector2d position() const {
//                     return Eigen::Vector2d(x, y);
//             }
//     };
    
//     std::vector<Point2D> points;
    
// public:
//     Waypoints() {}
    
//     void addPoint(double x, double y, double yaw = 0.0) {
//             points.emplace_back(x, y, yaw);
//     }
    
//     const Point2D& getPoint(size_t index) const {
//             if (index < points.size()) {
//                     return points[index];
//             }
//             static Point2D defaultPoint;
//             return defaultPoint;
//     }
    
//     Eigen::Vector2d getPosition(size_t index) const {
//             return getPoint(index).position();
//     }
    
//     double getYaw(size_t index) const {
//             return getPoint(index).yaw;
//     }
    
//     size_t size() const {
//             return points.size();
//     }
    
//     bool empty() const {
//             return points.empty();
//     }
    
//     void clear() {
//             points.clear();
//     }
    
//     const Point2D& operator[](size_t index) const {
//             return points[index];
//     }
    
//     const Point2D& back() const {
//             return points.back();
//     }
    
//     typename std::vector<Point2D>::iterator begin() { return points.begin(); }
//     typename std::vector<Point2D>::iterator end() { return points.end(); }
//     typename std::vector<Point2D>::const_iterator begin() const { return points.begin(); }
//     typename std::vector<Point2D>::const_iterator end() const { return points.end(); }
// };

// class PIDController {
//     public:
//         // Constructor
//         PIDController(
//             double pos_p = 1.0, double pos_i = 0.0, double pos_d = 0.0,  // Position PID gains
//             double ang_p = 1.0, double ang_i = 0.0, double ang_d = 0.0,  // Angular PID gains
//             double max_linear_vel = 0.3,                                 // Max linear velocity
//             double max_angular_vel = 0.3,                                // Max angular velocity
//             double position_tolerance = 0.1,                             // Position tolerance
//             double angle_tolerance = 0.1                                 // Angle tolerance
//         )
//             : pos_p_(pos_p), pos_i_(pos_i), pos_d_(pos_d),
//               ang_p_(ang_p), ang_i_(ang_i), ang_d_(ang_d),
//               max_linear_vel_(max_linear_vel),
//               max_angular_vel_(max_angular_vel),
//               pos_tol_(position_tolerance),
//               ang_tol_(angle_tolerance),
//               current_waypoint_idx_(0),
//               goal_reached_(true),
//               prev_pos_error_(0.0),
//               prev_ang_error_(0.0),
//               pos_error_integral_(0.0),
//               ang_error_integral_(0.0),
//               dt_(0.01),  // Time step, adjust as needed
//               cmd_linear_(0.0),
//               cmd_angular_(0.0),
//               current_pose_()  // Initialize current_pose_ member
//         {}
        
//         // Set new path
//         void setPath(const Waypoints& new_path) {
//             path_ = new_path;
//             current_waypoint_idx_ = 0;
            
//             // Reset PID values
//             prev_pos_error_ = 0.0;
//             prev_ang_error_ = 0.0;
//             pos_error_integral_ = 0.0;
//             ang_error_integral_ = 0.0;
            
//             if (!new_path.empty()) {
//                 goal_reached_ = false;
//             } else {
//                 goal_reached_ = true;
//                 std::cout << "Received empty path!" << std::endl;
//             }
//         }
        
//         // Helper function to convert world point to robot frame
//         Eigen::Vector3d getPositionInRobotFrame(const Eigen::Vector3d& goal_position, const Pose& robot_pose) {
//             // Calculate global offset vector
//             Eigen::Vector3d global_offset = goal_position - robot_pose.position;
            
//             // Get the robot's rotation matrix
//             Eigen::Matrix3d rotation_matrix = robot_pose.orientation.toRotationMatrix();
            
//             // Apply rotation to local coordinates
//             Eigen::Vector3d local_position = rotation_matrix.transpose() * global_offset;
            
//             return local_position;
//         }
        
//         // Compute velocities based on current pose - Working in robot frame
//         void computeVelocities(const Pose& current_pose) {
//             // Store current pose for use in current_yaw() method
//             current_pose_ = current_pose;
            
//             // Check if we have a path and haven't reached the goal
//             if (path_.empty() || goal_reached_) {
//                 cmd_linear_ = 0.0;
//                 cmd_angular_ = 0.0;
//                 return;
//             }
            
//             // Get current waypoint in world frame
//             const auto& target = path_[current_waypoint_idx_];
            
//             // Create 3D point for target (z=0 for 2D navigation)
//             Eigen::Vector3d target_position(target.x, target.y, 0.0);
            
//             // Transform target point to robot frame using Eigen
//             Eigen::Vector3d target_in_robot_frame = getPositionInRobotFrame(target_position, current_pose);
            
//             // Extract x and y components in robot frame
//             double dx_robot = target_in_robot_frame.x();
//             double dy_robot = target_in_robot_frame.y();
            
//             // Distance to target (same in both frames)
//             double distance = std::sqrt(dx_robot*dx_robot + dy_robot*dy_robot);
            
//             // Check if we've reached the current waypoint
//             if (distance <= pos_tol_) {
//                 // Move to the next waypoint
//                 current_waypoint_idx_++;
                
//                 // Reset integral terms when switching waypoints
//                 pos_error_integral_ = 0.0;
//                 ang_error_integral_ = 0.0;
                
//                 // Check if we've reached the end of the path
//                 if (current_waypoint_idx_ >= path_.size()) {
//                     goal_reached_ = true;
//                     cmd_linear_ = 0.0;
//                     cmd_angular_ = 0.0;
//                     std::cout << "Goal reached!" << std::endl;
//                     return;
//                 }
                
//                 // Get new target waypoint
//                 const auto& new_target = path_[current_waypoint_idx_];
                
//                 // Recalculate for new target
//                 Eigen::Vector3d new_target_position(new_target.x, new_target.y, 0.0);
//                 Eigen::Vector3d new_target_in_robot_frame = getPositionInRobotFrame(new_target_position, current_pose);
                
//                 dx_robot = new_target_in_robot_frame.x();
//                 dy_robot = new_target_in_robot_frame.y();
//                 distance = std::sqrt(dx_robot*dx_robot + dy_robot*dy_robot);
//             }
            
//             // In robot frame, the error in heading is just atan2(dy_robot, dx_robot)
//             // since the robot's heading is along the x-axis in its own frame
//             double yaw_error = std::atan2(dy_robot, dx_robot);
            
//             // Calculate PID terms for position control
//             double pos_error = distance;
//             pos_error_integral_ += pos_error * dt_;
//             // Apply anti-windup to prevent integral term from growing too large
//             pos_error_integral_ = std::clamp(pos_error_integral_, -10.0, 10.0);
//             double pos_derivative = (pos_error - prev_pos_error_) / dt_;
            
//             // Calculate PID terms for heading control
//             ang_error_integral_ += yaw_error * dt_;
//             // Apply anti-windup to prevent integral term from growing too large
//             ang_error_integral_ = std::clamp(ang_error_integral_, -10.0, 10.0);
//             double ang_derivative = (yaw_error - prev_ang_error_) / dt_;
            
//             // Save current errors for next iteration
//             prev_pos_error_ = pos_error;
//             prev_ang_error_ = yaw_error;
            
//             // Calculate control signals
//             // For position, we use a cosine factor to reduce speed when off-heading
//             double heading_factor = std::cos(yaw_error);
//             double pos_control = pos_p_ * pos_error + 
//                                  pos_i_ * pos_error_integral_ + 
//                                  pos_d_ * pos_derivative;
            
//             // Linear velocity depends on position error and heading alignment
//             cmd_linear_ = std::min(pos_control * heading_factor, max_linear_vel_);
            
//             // If we're significantly off-heading, prioritize turning over moving
//             if (std::abs(yaw_error) > 0.5) {  // ~30 degrees
//                 cmd_linear_ *= 0.5;  // Reduce forward speed to focus on turning
//             }
            
//             // Angular velocity control
//             double ang_control = ang_p_ * yaw_error + 
//                                 ang_i_ * ang_error_integral_ + 
//                                 ang_d_ * ang_derivative;
            
//             cmd_angular_ = std::clamp(ang_control, -max_angular_vel_, max_angular_vel_);
            
//             if (DEBUG_MODE) {
//                 std::cout << "Target world: (" << target.x << ", " << target.y << ")" << std::endl;
//                 std::cout << "Target robot: (" << dx_robot << ", " << dy_robot << ")" << std::endl;
//                 std::cout << "Current: (" << current_pose.position.x() << ", " 
//                           << current_pose.position.y() << "), yaw: " << current_yaw() << std::endl;
//                 std::cout << "Distance: " << distance << ", Yaw error: " << yaw_error << " rad" << std::endl;
//                 std::cout << "Cmd: linear=" << cmd_linear_ << ", angular=" << cmd_angular_ << std::endl;
//             }
//         }
        
//         // Generate a path by interpolating waypoints
//         Waypoints interpolateWaypoints(const Eigen::Vector3d& start_position, const Eigen::Vector3d& goal_position, 
//                                     double max_segment_length = 1.0, bool include_start = false) {
//             Waypoints result;
            
//             // Calculate total distance
//             double dx = goal_position.x() - start_position.x();
//             double dy = goal_position.y() - start_position.y();
//             double dz = goal_position.z() - start_position.z();
//             double total_distance = std::sqrt(dx*dx + dy*dy + dz*dz);
            
//             // Calculate how many segments we need
//             int num_segments = std::ceil(total_distance / max_segment_length);
            
//             // Make sure we have at least one segment
//             num_segments = std::max(num_segments, 1);
            
//             // Calculate the step size for each segment
//             double step_x = dx / num_segments;
//             double step_y = dy / num_segments;
//             double step_z = dz / num_segments;
            
//             // Add start point if requested
//             if (include_start) {
//                 result.addPoint(start_position.x(), start_position.y());
//             }
            
//             // Add intermediate points
//             for (int i = 1; i < num_segments; i++) {
//                 double x = start_position.x() + step_x * i;
//                 double y = start_position.y() + step_y * i;
                
//                 // Calculate heading towards the next point
//                 double next_x = (i == num_segments - 1) ? goal_position.x() : start_position.x() + step_x * (i + 1);
//                 double next_y = (i == num_segments - 1) ? goal_position.y() : start_position.y() + step_y * (i + 1);
//                 double heading = std::atan2(next_y - y, next_x - x);
                
//                 result.addPoint(x, y, heading);
//             }
            
//             // Add goal point
//             // Calculate final heading (from the previous-to-last point to the goal)
//             double prev_x = start_position.x() + step_x * (num_segments - 1);
//             double prev_y = start_position.y() + step_y * (num_segments - 1);
//             double final_heading = std::atan2(goal_position.y() - prev_y, goal_position.x() - prev_x);
            
//             result.addPoint(goal_position.x(), goal_position.y(), final_heading);
            
//             return result;
//         }
        
//         // Helper method to set a path with interpolation when needed
//         void setTargetWithInterpolation(double x, double y, double yaw = 0.0, double max_segment_length = 1.0) {
//             // Create target position as 3D point
//             Eigen::Vector3d target_position(x, y, 0.0);
            
//             // Get current robot position
//             Eigen::Vector3d start_position(current_pose_.position);
            
//             // Calculate direct distance
//             double dx = target_position.x() - start_position.x();
//             double dy = target_position.y() - start_position.y();
//             double distance = std::sqrt(dx*dx + dy*dy);
            
//             // If distance is greater than the segment length, create interpolated waypoints
//             if (distance > max_segment_length) {
//                 Waypoints interpolated_path = interpolateWaypoints(start_position, target_position, max_segment_length);
//                 setPath(interpolated_path);
                
//                 if(DEBUG_MODE) {
//                     std::cout << "Generated interpolated path with " << interpolated_path.size() 
//                               << " waypoints over " << distance << " meters" << std::endl;
//                     for (size_t i = 0; i < interpolated_path.size(); i++) {
//                         std::cout << "  Waypoint " << i << ": (" << interpolated_path[i].x 
//                                   << ", " << interpolated_path[i].y << ")" << std::endl;
//                     }
//                 }
//             } else {
//                 // Distance is small, just set a single waypoint
//                 Waypoints path;
//                 path.addPoint(x, y, yaw);
//                 setPath(path);
                
//                 if(DEBUG_MODE) {
//                     std::cout << "Set direct target point: (" << x << ", " << y << ", " << yaw << ")" << std::endl;
//                 }
//             }
//         }
        
//         // Helper to extract yaw angle from pose
//         double current_yaw() const {
//             Eigen::Matrix3d rot_matrix = current_pose_.orientation.toRotationMatrix();
//             return std::atan2(rot_matrix(1, 0), rot_matrix(0, 0));
//         }
        
//         // Get current linear velocity command
//         double getLinearVelocity() const {
//             return cmd_linear_;
//         }
        
//         // Get current angular velocity command
//         double getAngularVelocity() const {
//             return cmd_angular_;
//         }
        
//         // Check if goal has been reached
//         bool isGoalReached() const {
//             return goal_reached_;
//         }
        
//         // Set the position tolerance
//         void setPositionTolerance(double tolerance) {
//             pos_tol_ = tolerance;
//         }
        
//         // Set the angle tolerance
//         void setAngleTolerance(double tolerance) {
//             ang_tol_ = tolerance;
//         }
        
//         // Set the time step
//         void setTimeStep(double dt) {
//             dt_ = dt;
//         }
        
//         // Public access to current pose (needed for Custom class)
//         Pose current_pose_;
        
//     private:
//         // Path
//         Waypoints path_;
        
//         // PID gains
//         double pos_p_, pos_i_, pos_d_;  // Position control gains
//         double ang_p_, ang_i_, ang_d_;  // Angular control gains
        
//         // Control limits
//         double max_linear_vel_;
//         double max_angular_vel_;
        
//         // Tolerances
//         double pos_tol_;
//         double ang_tol_;
        
//         // PID state variables
//         double prev_pos_error_;
//         double prev_ang_error_;
//         double pos_error_integral_;
//         double ang_error_integral_;
//         double dt_;
        
//         // Path following state
//         size_t current_waypoint_idx_;
//         bool goal_reached_;
        
//         // Command outputs
//         double cmd_linear_;
//         double cmd_angular_;
// };

// class Custom
// {
// public:
// 	Custom()
// 	{
// 		// sport_client.SetTimeout(10.0f);
// 		// sport_client.Init();

// 		// suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
// 		// suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);

// 		// try {
// 		// 	zmq_context = new zmq::context_t();
// 		// 	zmq_socket = new zmq::socket_t(*zmq_context, zmq::socket_type::sub);
			
// 		// 	zmq_socket->connect("tcp://128.61.21.22:5555");
// 		// 	zmq_socket->set(zmq::sockopt::subscribe, "");
// 		// 	std::cout << "Connected to MoCap server" << std::endl;
// 		// } catch (const std::exception& e) {
// 		// 	std::cerr << "Failed to connect to MoCap server: " << e.what() << std::endl;
// 		// }
		
// 		// robot_pose = Pose();

//         robot_pose.position.x() = 0;
//         robot_pose.position.y() = 0;
//         robot_pose.position.z() = 0;

//         robot_pose.orientation.x() = 0;
//         robot_pose.orientation.y() = 0;
//         robot_pose.orientation.z() = 0;
//         robot_pose.orientation.w() = 0;

// 		// Create PID controller with tuned parameters
// 		// Position PID: P=1.0, I=0.05, D=0.1
// 		// Angular PID: P=1.5, I=0.0, D=0.3
// 		controller = PIDController(1.0, 0.05, 0.1, 1.5, 0.0, 0.3, 0.3, 0.3, 0.1, 0.1);
		
// 		// Set the sampling time for the controller
// 		// controller.setTimeStep(dt);

// 		// Wait a moment to get initial pose
// 		// sleep(1);
// 		// updatePoseFromMocap();

// 		// Set initial waypoint with interpolation
// 		// setTargetWithInterpolation(4.0, 0.0, 0.0);
// 	};

// 	~Custom(){
// 		// delete zmq_socket;
// 		// delete zmq_context;
// 	}

//     void wayppointInterpolation() {
//         // Clear existing path
//         path.clear();
        
//         // Get current position from robot_pose
//         Eigen::Vector2d start(robot_pose.position.x(), robot_pose.position.y());
        
//         // Get target position
//         Eigen::Vector2d end(goal_position.x(), goal_position.y());
        
//         // Calculate distance between current position and target
//         double distance = (end - start).norm();
        
//         // If distance is below threshold, just add the target point
//         const double DISTANCE_THRESHOLD = 0.5; // meters
//         if (distance <= DISTANCE_THRESHOLD) {
//             path.addPoint(goal_position.x(), goal_position.y(), goal_position.z()); // 使用目标点的yaw值
//             return;
//         }
        
//         // For longer distances, create interpolated waypoints
//         // Calculate number of points based on distance
//         const double POINT_SPACING = 0.5; // meters between points
//         std::cout<<static_cast<int>(distance / POINT_SPACING)<<std::endl;
//         int num_points = std::min(30, static_cast<int>(distance / POINT_SPACING));
        
//         // Direction vector
//         Eigen::Vector2d direction = (end - start).normalized();
        
//         // Calculate heading/yaw from direction vector
//         double path_yaw = std::atan2(direction.y(), direction.x());
        
//         // 获取当前机器人的yaw角
//         Eigen::Matrix3d rot_matrix = robot_pose.orientation.toRotationMatrix();
//         double current_yaw = std::atan2(rot_matrix(1, 0), rot_matrix(0, 0));
        
//         // 目标yaw角值(使用goal_position.z()存储的yaw值)
//         double target_yaw = goal_position.z();
        
//         // 添加中间点
//         for (int i = 0; i < num_points; i++) {
//             double ratio = static_cast<double>(i) / (num_points - 1);
            
//             // 位置线性插值
//             Eigen::Vector2d point = start + ratio * (end - start);
            
//             // Yaw角线性插值
//             // 有两种可能的插值方式:
//             // 1. 从当前朝向到目标朝向的插值
//             // 2. 使用路径朝向(path_yaw)作为每个点的朝向
            
//             // 这里使用第一种方式: 从当前朝向到目标朝向的插值
//             // 但首先需要处理角度差，确保选择最短路径旋转
//             double yaw_diff = target_yaw - current_yaw;
            
//             // 标准化角度差到[-π, π]范围
//             while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
//             while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
            
//             // 计算插值后的yaw
//             double interpolated_yaw = current_yaw + ratio * yaw_diff;
            
//             // 添加点到路径
//             path.addPoint(point.x(), point.y(), interpolated_yaw);
//         }
        
//         if (DEBUG_MODE) {
//             std::cout << "Generated " << path.size() << " waypoints for a " 
//                     << distance << "m path" << std::endl;
//             std::cout << "Yaw interpolation: start=" << current_yaw * 180/M_PI 
//                     << "°, end=" << target_yaw * 180/M_PI << "°" << std::endl;
//         }
//     }

//     void setTargetPoint(double x, double y, double yaw = 0.0) {
//         // Add the target point
//         // goal_position(x, y, yaw);
//         goal_position.x() = x;
//         goal_position.y() = y;
//         goal_position.z() = yaw;
//         // target_position.addPoint(x, y, yaw);
        
//         // Pass the path to the controller
//         // controller.receivePath(path);
        
//         if(DEBUG_MODE) {
//             std::cout << "Set new target point: (" << x << ", " << y << ", " << yaw << ")" << std::endl;
//         }
//     }

//     void control() {
//         setTargetPoint(0.0, 8.0, 0.0);
        
//         // Create Vector3d objects from robot and goal positions
//         Eigen::Vector3d start_pos = robot_pose.position;
//         Eigen::Vector3d goal_pos = goal_position;
        
//         // Call interpolateWaypoints with correct types
//         wayppointInterpolation();
        
//         // Print out the waypoints for verification
//         std::cout << "Generated " << path.size() << " waypoints:" << std::endl;
//         for (size_t i = 0; i < path.size(); i++) {
//             std::cout << "Waypoint " << i << ": (" 
//                       << path[i].x << ", " 
//                       << path[i].y << "), heading: " 
//                       << path[i].yaw << " rad" << std::endl;
//         }

//         controller.setPath(path);

//         controller.computeVelocities(robot_pose);
			
//         // If the goal hasn't been reached, send control commands
//         if (!controller.isGoalReached()) {
//             // Get computed linear and angular velocities
//             double linear_vel = controller.getLinearVelocity();
//             double angular_vel = controller.getAngularVelocity();
            
//             // Send control commands to the robot
//             // sport_client.Move(linear_vel, 0, angular_vel);
            
//             if(DEBUG_MODE){
//                 std::cout << "PID Control: v=" << linear_vel << ", w=" << angular_vel << std::endl;
//                 std::cout << "Position: " << robot_pose.position.x() << ", " 
//                                 << robot_pose.position.y() << ", " 
//                                 << robot_pose.position.z() << std::endl;
//             }
//         } else {
//             // Goal reached, stop moving
//             // sport_client.StopMove();
//             std::cout << "PID Controller: Goal reached!" << std::endl;
//         }
        
//         // Store the path if needed
//         // path = interpolated_path;
//     }

//     private:
//         Pose last_mocap_pose;
//         double last_timestamp = 0.0; 
//         double last_print_time = 0.0;
//         Pose robot_pose;
//         std::vector<Pose> obstacles_pose;
//         // Create a new Waypoints object
//         Eigen::Vector3d goal_position;
//         Waypoints path;
//         PIDController controller;
// };

// int main() {
//     Custom m_custom;
//     m_custom.control(); 
//     return 0;
// }



#include <iostream>
#include <cassert>
#include <cmath>
#include <Eigen/Dense>

// Function to test
Eigen::Vector3d getPositionInRobotFrame2(const Eigen::Vector3d& world_point, 
                                         double robot_x, double robot_y, double robot_yaw) 
{
    double dx = world_point.x() - robot_x;
    double dy = world_point.y() - robot_y;

    double cos_theta = std::cos(robot_yaw);
    double sin_theta = std::sin(robot_yaw);

    double x_r =  cos_theta * dx + sin_theta * dy;
    double y_r = -sin_theta * dx + cos_theta * dy;
    double z_r = world_point.z();  // Assume no rotation in Z

    return Eigen::Vector3d(x_r, y_r, z_r);
}

// Helper to check near-equality of vectors
bool isClose(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double tol = 1e-6) {
    return (a - b).norm() < tol;
}

// Test case
void test_getPositionInRobotFrame2() {
    // World point at (4, 2, 0)
    Eigen::Vector3d world_point(1.8, 1.3, 0.0);
    
    // Robot at (2, 1) with 90 degrees (pi/2) rotation
    // 1.79028, 0.277671
    double robot_x = 1.8;
    double robot_y = 0.3;
    double robot_yaw = M_PI / 2;  // 90 degrees

    // Eigen::Vector3d expected(1.0, -2.0, 0.0);  // After rotating world_point into robot frame
    Eigen::Vector3d result = getPositionInRobotFrame2(world_point, robot_x, robot_y, robot_yaw);

    std::cout << "Result: " << result.transpose() << std::endl;

    // assert(isClose(result, expected));
    std::cout << "Test passed!" << std::endl;
}

int main() {
    test_getPositionInRobotFrame2();
    return 0;
}
