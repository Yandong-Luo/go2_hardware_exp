#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/common/thread/recurrent_thread.hpp>
#include <math.h>

#include <Eigen/Dense>
#include <vector>
#include <zmq.hpp>
#include <nlohmann/json.hpp> // 需要添加JSON解析库

using json = nlohmann::json;

#define DEBUG_MODE true

struct Pose {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    
    Pose() : position(0, 0, 0), orientation(Eigen::Quaterniond::Identity()) {}
    
    Pose(const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient) 
            : position(pos), orientation(orient) {}
    
    Pose(double x, double y, double z, const Eigen::Quaterniond& orient) 
            : position(x, y, z), orientation(orient) {}
};

class Waypoints {
private:
    struct Point2D {
            double x, y, yaw;
            
            Point2D() : x(0.0), y(0.0), yaw(0.0) {}
            Point2D(double _x, double _y, double _yaw = 0.0) : x(_x), y(_y), yaw(_yaw) {}
            
            Eigen::Vector2d position() const {
                    return Eigen::Vector2d(x, y);
            }
    };
    
    std::vector<Point2D> points;
    
public:
    Waypoints() {}
    
    void addPoint(double x, double y, double yaw = 0.0) {
            points.emplace_back(x, y, yaw);
    }
    
    const Point2D& getPoint(size_t index) const {
            if (index < points.size()) {
                    return points[index];
            }
            static Point2D defaultPoint;
            return defaultPoint;
    }
    
    Eigen::Vector2d getPosition(size_t index) const {
            return getPoint(index).position();
    }
    
    double getYaw(size_t index) const {
            return getPoint(index).yaw;
    }
    
    size_t size() const {
            return points.size();
    }
    
    bool empty() const {
            return points.empty();
    }
    
    void clear() {
            points.clear();
    }
    
    const Point2D& operator[](size_t index) const {
            return points[index];
    }
    
    const Point2D& back() const {
            return points.back();
    }
    
    typename std::vector<Point2D>::iterator begin() { return points.begin(); }
    typename std::vector<Point2D>::iterator end() { return points.end(); }
    typename std::vector<Point2D>::const_iterator begin() const { return points.begin(); }
    typename std::vector<Point2D>::const_iterator end() const { return points.end(); }
};

class Custom
{
    public:
        Custom() {
            try {
				zmq_context = new zmq::context_t();
				zmq_socket = new zmq::socket_t(*zmq_context, zmq::socket_type::sub);
				
				zmq_socket->connect("tcp://128.61.21.22:5555");
				std::cout << "hihihihi2" << std::endl;
				zmq_socket->set(zmq::sockopt::subscribe, "");
				std::cout << "Connected to MoCap server" << std::endl;
			} catch (const std::exception& e) {
				std::cerr << "Failed to connect to MoCap server: " << e.what() << std::endl;
				// Provide a fallback behavior here
			}
        }
        void control();

        unitree::robot::go2::SportClient tc;

        int c = 0;
        float dt = 0.002; // 0.001~0.01

        void updatePoseFromMocap() {
			try {
				zmq::message_t message;
				auto result = zmq_socket->recv(message, zmq::recv_flags::dontwait);
				
				if (result) {
					std::cout << "receive the msg" << std::endl;
					std::string msg_str = message.to_string();
					std::cout << msg_str << std::endl;
		
					json msg_json = json::parse(msg_str);
					
					// Clear obstacles vector to refresh with new data
					obstacles_pose.clear();
					
					if (msg_json.contains("robots")) {
						auto& robots = msg_json["robots"];
						
						// Process robot pose
						if (robots.contains("Go2")) {
							auto& robot = robots["Go2"];
							
							if (robot.contains("position") && robot.contains("rotation")) {
								auto& pos = robot["position"];
								auto& rot = robot["rotation"];
								
								// Update robot pose
								robot_pose.position = Eigen::Vector3d(
									pos[0].get<double>(),
									pos[1].get<double>(),
									pos[2].get<double>()
								);
								
								robot_pose.orientation = Eigen::Quaterniond(
									rot[3].get<double>(),  // w    
									rot[0].get<double>(),  // x
									rot[1].get<double>(),  // y
									rot[2].get<double>()   // z
								);
							}
						}
						
						// Process all obstacles
						for (auto& [name, data] : robots.items()) {
							// Skip our robot
							if (name == "Go2") continue;
							
							// Process any object with "obstacle" in the name
							if (name.find("obstacle") != std::string::npos) {
								if (data.contains("position") && data.contains("rotation")) {
									auto& pos = data["position"];
									auto& rot = data["rotation"];
									
									// Create new Pose for this obstacle
									Pose obstacle_pose;
									obstacle_pose.position = Eigen::Vector3d(
										pos[0].get<double>(),
										pos[1].get<double>(),
										pos[2].get<double>()
									);
									
									obstacle_pose.orientation = Eigen::Quaterniond(
										rot[3].get<double>(),  // w    
										rot[0].get<double>(),  // x
										rot[1].get<double>(),  // y
										rot[2].get<double>()   // z
									);
									
									// Add to obstacles vector
									obstacles_pose.push_back(obstacle_pose);
								}
							}
						}
						
						// Record timestamp
						if (msg_json.contains("timestamp")) {
							last_timestamp = msg_json["timestamp"].get<double>();
						}
					}
					
					if(DEBUG_MODE){
						// Print robot position (you already have this)
						std::cout << "robots postion x:" << robot_pose.position.x() << " " 
						<< robot_pose.position.y() << "orientation " 
						<< robot_pose.orientation.x() << " " << robot_pose.orientation.y() << " " 
						<< robot_pose.orientation.z() << " " << robot_pose.orientation.w() << std::endl;
			
						// Print obstacle information
						std::cout << "Found " << obstacles_pose.size() << " obstacles:" << std::endl;
						for (size_t i = 0; i < obstacles_pose.size(); i++) {
							std::cout << "  Obstacle " << i << ": position(" 
										<< obstacles_pose[i].position.x() << ", " 
										<< obstacles_pose[i].position.y() << ", " 
										<< obstacles_pose[i].position.z() << "), orientation(" 
										<< obstacles_pose[i].orientation.x() << ", " 
										<< obstacles_pose[i].orientation.y() << ", " 
										<< obstacles_pose[i].orientation.z() << ", " 
										<< obstacles_pose[i].orientation.w() << ")" << std::endl;
						}
					}
					
				}
			} catch (const std::exception& e) {
				std::cerr << "Error getting MoCap data: " << e.what() << std::endl;
			}
		}

        ~Custom(){
			delete zmq_socket;
			delete zmq_context;
		}

        Eigen::Affine3d transformToBaseLink(const Pose& target_pose, const Eigen::Affine3d& world_to_robot) {
            Eigen::Affine3d target_transform;
            target_transform.translation() = target_pose.position;
            target_transform.linear() = target_pose.orientation.toRotationMatrix();
            
            // 计算相对变换: robot^-1 * target
            return world_to_robot.inverse() * target_transform;
        }

        // Eigen::Vector3d getTargetPositionInRobotFrame(const Eigen::Vector3d& goal_position, const Pose& robot_pose) {
        //     // Step 1: Calculate the vector from robot to target in global frame
        //     Eigen::Vector3d global_offset = goal_position - robot_pose.position;
            
        //     // Step 2: Get the rotation matrix from the robot's orientation
        //     Eigen::Matrix3d rotation_matrix = robot_pose.orientation.toRotationMatrix();
            
        //     // Step 3: Apply the inverse rotation to transform to local coordinates
        //     // The inverse of a rotation matrix is its transpose
        //     Eigen::Vector3d local_position = rotation_matrix.transpose() * global_offset;
            
        //     return local_position;
        // }

        // Eigen::Vector3d getTargetPositionInRobotFrame(const Eigen::Vector3d& goal_position, const Pose& robot_pose) {
        //     // 计算全局偏移向量
        //     Eigen::Vector3d global_offset = goal_position - robot_pose.position;
            
        //     // 获取机器人朝向的旋转矩阵
        //     Eigen::Matrix3d rotation_matrix = robot_pose.orientation.toRotationMatrix();
            
        //     // 创建绕Z轴顺时针旋转90度的补偿矩阵
        //     // 注意：顺时针旋转90度相当于逆时针旋转270度或-90度
        //     double angle = M_PI/2; // 顺时针旋转90度（弧度制）
        //     Eigen::Matrix3d compensation;
        //     compensation << cos(angle), -sin(angle), 0,
        //                     sin(angle),  cos(angle), 0,
        //                          0,           0,     1;
            
        //     // 应用补偿旋转矩阵和原始旋转矩阵
        //     // 先应用原始旋转，再应用补偿旋转
        //     Eigen::Matrix3d combined_rotation = compensation * rotation_matrix;
            
        //     // 应用组合旋转矩阵到局部坐标
        //     Eigen::Vector3d local_position = combined_rotation.transpose() * global_offset;
            
        //     if(DEBUG_MODE) {
        //         std::cout << "Transform debug with compensation:" << std::endl;
        //         std::cout << "  Goal global: (" << goal_position.x() << ", " << goal_position.y() << ")" << std::endl;
        //         std::cout << "  Robot global: (" << robot_pose.position.x() << ", " << robot_pose.position.y() << ")" << std::endl;
        //         std::cout << "  Global offset: (" << global_offset.x() << ", " << global_offset.y() << ")" << std::endl;
        //         std::cout << "  Original rotation matrix:" << std::endl;
        //         std::cout << "    " << rotation_matrix(0,0) << " " << rotation_matrix(0,1) << " " << rotation_matrix(0,2) << std::endl;
        //         std::cout << "    " << rotation_matrix(1,0) << " " << rotation_matrix(1,1) << " " << rotation_matrix(1,2) << std::endl;
        //         std::cout << "    " << rotation_matrix(2,0) << " " << rotation_matrix(2,1) << " " << rotation_matrix(2,2) << std::endl;
        //         std::cout << "  Combined rotation matrix (with compensation):" << std::endl;
        //         std::cout << "    " << combined_rotation(0,0) << " " << combined_rotation(0,1) << " " << combined_rotation(0,2) << std::endl;
        //         std::cout << "    " << combined_rotation(1,0) << " " << combined_rotation(1,1) << " " << combined_rotation(1,2) << std::endl;
        //         std::cout << "    " << combined_rotation(2,0) << " " << combined_rotation(2,1) << " " << combined_rotation(2,2) << std::endl;
        //         std::cout << "  Local position (with compensation): (" << local_position.x() << ", " << local_position.y() << ")" << std::endl;
        //     }
            
        //     return local_position;
        // }

        Eigen::Vector3d getTargetPositionInRobotFrame(const Eigen::Vector3d& goal_position, const Pose& robot_pose) {
            // 打印调试信息
            // if(DEBUG_MODE) {
            //     std::cout << "Transform debug:" << std::endl;
            //     std::cout << "  Goal global: (" << goal_position.x() << ", " << goal_position.y() << ")" << std::endl;
            //     std::cout << "  Robot global: (" << robot_pose.position.x() << ", " << robot_pose.position.y() << ")" << std::endl;
            //     std::cout << "  Robot quaternion: (" 
            //               << robot_pose.orientation.w() << ", " 
            //               << robot_pose.orientation.x() << ", " 
            //               << robot_pose.orientation.y() << ", " 
            //               << robot_pose.orientation.z() << ")" << std::endl;
            // }
            
            // 计算全局偏移向量
            Eigen::Vector3d global_offset = goal_position - robot_pose.position;
            
            // 获取机器人朝向的旋转矩阵
            Eigen::Matrix3d rotation_matrix = robot_pose.orientation.toRotationMatrix();
            
            // 应用旋转到局部坐标
            Eigen::Vector3d local_position = rotation_matrix.transpose() * global_offset;
            
            // if(DEBUG_MODE) {
            //     std::cout << "  Global offset: (" << global_offset.x() << ", " << global_offset.y() << ")" << std::endl;
            //     std::cout << "  Local position: (" << local_position.x() << ", " << local_position.y() << ")" << std::endl;
                
            //     // 打印旋转矩阵
            //     std::cout << "  Rotation matrix:" << std::endl;
            //     std::cout << "    " << rotation_matrix(0,0) << " " << rotation_matrix(0,1) << " " << rotation_matrix(0,2) << std::endl;
            //     std::cout << "    " << rotation_matrix(1,0) << " " << rotation_matrix(1,1) << " " << rotation_matrix(1,2) << std::endl;
            //     std::cout << "    " << rotation_matrix(2,0) << " " << rotation_matrix(2,1) << " " << rotation_matrix(2,2) << std::endl;
            // }
            
            return local_position;
        }

        void setTargetPoint(double x, double y, double yaw = 0.0) {
			// Add the target point
            // goal_position(x, y, yaw);
            goal_position.x() = x;
            goal_position.y() = y;
            goal_position.z() = yaw;
			// target_position.addPoint(x, y, yaw);
			
			// Pass the path to the controller
			// controller.receivePath(path);
			
			if(DEBUG_MODE) {
				std::cout << "Set new target point: (" << x << ", " << y << ", " << yaw << ")" << std::endl;
			}
		}

        // void wayppointInterpolation(){
        //     // Clear existing path
        //     path.clear();
            
        //     // Get current position from robot_pose
        //     Eigen::Vector2d start(robot_pose.position.x(), robot_pose.position.y());
            
        //     // Get target position
        //     Eigen::Vector2d end(goal_position.x(), goal_position.y());
            
        //     // Calculate distance between current position and target
        //     double distance = (end - start).norm();
            
        //     // If distance is below threshold, just add the target point
        //     const double DISTANCE_THRESHOLD = 0.5; // meters
        //     if (distance <= DISTANCE_THRESHOLD) {
        //         path.addPoint(goal_position.x(), goal_position.y(), 0.0); // Using default yaw
        //         return;
        //     }
            
        //     // For longer distances, create interpolated waypoints
        //     // Calculate number of points based on distance
        //     const double POINT_SPACING = 0.3; // meters between points
        //     int num_points = std::max(5, static_cast<int>(distance / POINT_SPACING));
            
        //     // Direction vector
        //     Eigen::Vector2d direction = (end - start).normalized();
            
        //     // Calculate heading/yaw from direction vector
        //     double target_yaw = std::atan2(direction.y(), direction.x());
            
        //     // Add intermediate points along the path
        //     for (int i = 0; i < num_points; i++) {
        //         double ratio = static_cast<double>(i) / (num_points - 1);
        //         Eigen::Vector2d point = start + ratio * (end - start);
                
        //         // Add the interpolated point to the path
        //         path.addPoint(point.x(), point.y(), target_yaw);
        //     }
            
        //     if (DEBUG_MODE) {
        //         std::cout << "Generated " << path.size() << " waypoints for a " 
        //                 << distance << "m path" << std::endl;
        //     }
        // }

        void wayppointInterpolation() {
            // Clear existing path
            path.clear();
            
            // Get current position from robot_pose
            Eigen::Vector2d start(robot_pose.position.x(), robot_pose.position.y());
            
            // Get target position
            Eigen::Vector2d end(goal_position.x(), goal_position.y());
            
            // Calculate distance between current position and target
            double distance = (end - start).norm();
            
            // If distance is below threshold, just add the target point
            const double DISTANCE_THRESHOLD = 0.5; // meters
            if (distance <= DISTANCE_THRESHOLD) {
                path.addPoint(goal_position.x(), goal_position.y(), goal_position.z()); // 使用目标点的yaw值
                return;
            }
            
            // For longer distances, create interpolated waypoints
            // Calculate number of points based on distance
            const double POINT_SPACING = 0.5; // meters between points

            int num_points = std::max(30, static_cast<int>(distance / POINT_SPACING));
            
            // Direction vector
            Eigen::Vector2d direction = (end - start).normalized();
            
            // Calculate heading/yaw from direction vector
            double path_yaw = std::atan2(direction.y(), direction.x());
            
            // 获取当前机器人的yaw角
            Eigen::Matrix3d rot_matrix = robot_pose.orientation.toRotationMatrix();
            double current_yaw = std::atan2(rot_matrix(1, 0), rot_matrix(0, 0));
            
            // 目标yaw角值(使用goal_position.z()存储的yaw值)
            double target_yaw = goal_position.z();
            
            // 添加中间点
            for (int i = 0; i < num_points; i++) {
                double ratio = static_cast<double>(i) / (num_points - 1);
                
                // 位置线性插值
                Eigen::Vector2d point = start + ratio * (end - start);
                
                // Yaw角线性插值
                // 有两种可能的插值方式:
                // 1. 从当前朝向到目标朝向的插值
                // 2. 使用路径朝向(path_yaw)作为每个点的朝向
                
                // 这里使用第一种方式: 从当前朝向到目标朝向的插值
                // 但首先需要处理角度差，确保选择最短路径旋转
                double yaw_diff = target_yaw - current_yaw;
                
                // 标准化角度差到[-π, π]范围
                while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
                while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
                
                // 计算插值后的yaw
                double interpolated_yaw = current_yaw + ratio * yaw_diff;
                
                // 添加点到路径
                path.addPoint(point.x(), point.y(), interpolated_yaw);
            }
            
            if (DEBUG_MODE) {
                std::cout << "Generated " << path.size() << " waypoints for a " 
                        << distance << "m path" << std::endl;
                std::cout << "Yaw interpolation: start=" << current_yaw * 180/M_PI 
                        << "°, end=" << target_yaw * 180/M_PI << "°" << std::endl;
            }
        }

    private:
        zmq::context_t* zmq_context;
        zmq::socket_t* zmq_socket;
        Pose last_mocap_pose;
        double last_timestamp = 0.0; 
        double last_print_time = 0.0;
        Pose robot_pose;
        std::vector<Pose> obstacles_pose;
        // Create a new Waypoints object
        Eigen::Vector3d goal_position;
        Waypoints path;
};



void Custom::control()
{
    c++;
    
    // int32_t ret;

    // float vx = 0.3;
    // float delta = 0.06;
    // static float count = 0;
    // count += dt;
    // std::vector<unitree::robot::go2::PathPoint> path;
    // for (int i=0; i<30; i++) {
    //   unitree::robot::go2::PathPoint p;
    //   float var = (count + i * delta);
    //   p.timeFromStart = i * delta;
    //   p.x = vx * var;
    //   p.y = 0.6 * sin(M_PI * vx * var);
    //   p.yaw = 2*0.6 * vx * M_PI * cos(M_PI * vx * var);
    //   p.vx = vx;
    //   p.vy = M_PI * vx * (0.6 * cos(M_PI * vx * var));
    //   p.vyaw = - M_PI * vx*2*0.6 * vx * M_PI * sin(M_PI * vx * var);
    //   path.push_back(p);
    // }

    // ret = tc.TrajectoryFollow(path);
    // if(ret != 0){
    //   std::cout << "Call TrajectoryFollow: " << ret << std::endl;
    // }

    // Update robot pose from motion capture system
    updatePoseFromMocap();
    
    // Set a target point (you need to define where to go)
    // For example, to set a target 2 meters ahead:
    setTargetPoint(8.0, 0.0, 0.0);
    
    // Generate waypoints for path following
    wayppointInterpolation();
    
    // Convert target to robot frame
    Eigen::Vector3d target_pos = getTargetPositionInRobotFrame(goal_position, robot_pose);
    
    // Create path for robot to follow
    std::vector<unitree::robot::go2::PathPoint> unitree_path;
    float delta = 0.06; // Time interval between points


    unitree::robot::go2::PathPoint p;

    // p.timeFromStart = 0;
    // p.x = 0.5;
    // p.y = 0.0;
    // p.vx = 0.3;
    // p.vy= 0.0;
    // p.yaw = 0.0;
    // p.vyaw = 0.0;

    // unitree_path.push_back(p);
    std::cout<<path.size()<<std::endl;
    // Convert our waypoints to Unitree path format
    for (size_t i = 0; i < path.size(); i++) {
        unitree::robot::go2::PathPoint p;
        p.timeFromStart = i * delta;
        
        // Get waypoint in robot's local frame
        Eigen::Vector3d local_pos = getTargetPositionInRobotFrame(
            Eigen::Vector3d(path[i].x, path[i].y, path[i].yaw),
            robot_pose
        );
        
        p.x = local_pos.x();
        p.y = local_pos.y();
        // p.yaw = path[i].yaw;
        p.yaw = local_pos.z();
        
        // Set velocities (simple approximation)
        // For a more sophisticated approach, you could calculate these based on path curvature
        if (i < path.size() - 1) {
            double dt = delta;
            Eigen::Vector3d next_pos = getTargetPositionInRobotFrame(
                Eigen::Vector3d(path[i+1].x, path[i+1].y, path[i+1].yaw),
                robot_pose
            );
            
            
            p.vx = (next_pos.x() - local_pos.x()) / dt;
            p.vy = (next_pos.y() - local_pos.y()) / dt;
            p.vyaw = (next_pos.z() - local_pos.z()) / dt;
            if(p.vx > 0.15)  p.vx = 0.15;
            if(p.vy > 0.15)  p.vy = 0.15;
            if(p.vy > 0.15)  p.vyaw = 0.15;

            if(p.vx < -0.15)  p.vx = -0.15;
            if(p.vy < -0.15)  p.vy = -0.15;
            if(p.vy < -0.15)  p.vyaw = -0.15;
            // p.vyaw = ; // Simplified - could calculate angular velocity if needed
        } else {
            // Last point - zero velocity to stop
            p.vx = 0.0;
            p.vy = 0.0;
            p.vyaw = 0.0;
        }
        
        unitree_path.push_back(p);
    }

    // Print trajectory details
    std::cout << "==== Generated Trajectory ====" << std::endl;
    std::cout << "Total waypoints: " << unitree_path.size() << std::endl;
    std::cout << "Original target: (" << goal_position.x() << ", " << goal_position.y() << ")" << std::endl;
    std::cout << "Robot position: (" << robot_pose.position.x() << ", " << robot_pose.position.y() << ")" << std::endl;
    std::cout << std::endl;

    // Print waypoints in global frame
    std::cout << "Waypoints (global frame):" << std::endl;
    for (size_t i = 0; i < path.size(); i++) {
        std::cout << "  Point " << i << ": (" 
                << path[i].x << ", " 
                << path[i].y << "), yaw: " 
                << path[i].yaw << std::endl;
    }
    std::cout << std::endl;

    // Print waypoints in robot frame
    std::cout << "Waypoints (robot frame):" << std::endl;
    for (size_t i = 0; i < unitree_path.size(); i++) {
        std::cout << "  Point " << i << ": (" 
                << unitree_path[i].x << ", " 
                << unitree_path[i].y << "), yaw: " 
                << unitree_path[i].yaw 
                << ", time: " << unitree_path[i].timeFromStart
                << ", vx: " << unitree_path[i].vx 
                << ", vy: " << unitree_path[i].vy 
                << ", vyaw: " << unitree_path[i].vyaw << std::endl;
    }
    
    // Send path to robot
    int32_t ret = tc.TrajectoryFollow(unitree_path);
    if (ret != 0) {
        std::cout << "Call TrajectoryFollow: " << ret << std::endl;
    }

    std::cout << c << std::endl;
}

int main(int argc, char** argv)
{
  unitree::robot::ChannelFactory::Instance()->Init(0);

  Custom custom;
  custom.tc.SetTimeout(10.0f);
  custom.tc.Init();

  unitree::common::ThreadPtr threadPtr = unitree::common::CreateRecurrentThread(custom.dt * 1000000, std::bind(&Custom::control, &custom));

  while (1)
  {
    sleep(10);
  }

  return 0;
}