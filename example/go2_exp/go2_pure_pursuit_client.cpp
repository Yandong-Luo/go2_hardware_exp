/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <cmath>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#include <Eigen/Dense>
#include <vector>
#include <zmq.hpp>
#include <nlohmann/json.hpp> // 需要添加JSON解析库

using json = nlohmann::json;

#define TOPIC_HIGHSTATE "rt/sportmodestate"

#define DEBUG_MODE true

using namespace unitree::common;

enum test_mode
{
	/*---Basic motion---*/
	pure_pursuit,
	normal_stand,
	balance_stand,
	velocity_move,
	trajectory_follow,
	stand_down,
	stand_up,
	damp,
	recovery_stand,
	/*---Special motion ---*/
	sit,
	rise_sit,
	stretch,
	wallow,
	//content,
	pose,
	scrape,
	front_flip,
	front_jump,
	front_pounce,
	stop_move = 99
};

const int TEST_MODE = pure_pursuit;

// ##########################################################################################
// ####################################### Pure Pursuit #####################################
// ##########################################################################################
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

class PurePursuit {
public:
		// 构造函数
		PurePursuit(double lookahead_distance = 1.0,
								double max_velocity = 0.3,
								double max_angular_velocity = 0.3,
								double position_tolerance = 0.1,
								double wheelbase = 1.0,
								double max_steering_angle = 0.3) 
				: ld_(lookahead_distance),
					v_max_(max_velocity),
					v_(max_velocity),
					w_max_(max_angular_velocity),
					pos_tol_(position_tolerance),
					L_(wheelbase),
					delta_max_(max_steering_angle),
					idx_(0),
					goal_reached_(true) {
		}
		
		// compute the volocities
		void computeVelocities(const Pose& current_pose) {
			if (path_.empty() || goal_reached_) {
					// sport_client.StopMove();
					stopMoving();
					return;
			}
			
			Eigen::Affine3d world_to_robot;
			world_to_robot.translation() = current_pose.position;
			world_to_robot.linear() = current_pose.orientation.toRotationMatrix();
			
			bool found_target = false;
			Pose target_pose;
			for (; idx_ < path_.size(); idx_++) {
				// current distance between current position and target
				double dx = path_[idx_].x - current_pose.position.x();
				double dy = path_[idx_].y - current_pose.position.y();
				double distance = std::sqrt(dx*dx + dy*dy);
				
				if (distance > ld_) {
					target_pose.position = Eigen::Vector3d(path_[idx_].x, path_[idx_].y, 0);
					target_pose.orientation = Eigen::Quaterniond(
							Eigen::AngleAxisd(path_[idx_].yaw, Eigen::Vector3d::UnitZ()));
					
					Eigen::Affine3d target_in_robot = transformToBaseLink(target_pose, world_to_robot);
					lookahead_point_ = target_in_robot.translation();
					
					found_target = true;
					break;
				}
			}
			
			if (!found_target && !path_.empty()) {
				// 接近目标点，目标比预瞄距离近
				target_pose.position = Eigen::Vector3d(path_.back().x, path_.back().y, 0);
				target_pose.orientation = Eigen::Quaterniond(
						Eigen::AngleAxisd(path_.back().yaw, Eigen::Vector3d::UnitZ()));
				
				// 变换到机器人坐标系
				Eigen::Affine3d target_in_robot = transformToBaseLink(target_pose, world_to_robot);
				
				if (std::fabs(target_in_robot.translation().x()) <= pos_tol_) {
						// 到达目标
						goal_reached_ = true;
						path_.clear();
						// sport_client.StopMove();
						stopMoving();
						return;
				} else {
					// 计算延长线与预瞄圆的交点
					double yaw = std::atan2(target_in_robot.translation().y(), 
																	target_in_robot.translation().x());
					double k_end = std::tan(yaw); // 直线斜率
					double l_end = target_in_robot.translation().y() - 
												k_end * target_in_robot.translation().x();
					double a = 1 + k_end * k_end;
					double b = 2 * l_end;
					double c = l_end * l_end - ld_ * ld_;
					double D = std::sqrt(b*b - 4*a*c);
					double x_ld = (-b + std::copysign(D, v_)) / (2*a);
					double y_ld = k_end * x_ld + l_end;
					
					lookahead_point_ = Eigen::Vector3d(x_ld, y_ld, 0);
				}
			}
			
			if (!goal_reached_) {
				// 跟踪中
				// 设置线速度
				v_ = std::copysign(v_max_, v_);
				
				// 计算角速度 - Pure Pursuit核心公式
				double yt = lookahead_point_.y();
				double ld_2 = ld_ * ld_;
				cmd_angular_ = std::min(2*v_ / ld_2 * yt, w_max_);
				
				// 计算Ackermann转向角
				cmd_steering_ = std::min(std::atan2(2 * yt * L_, ld_2), delta_max_);
				
				// 设置线速度
				cmd_linear_ = v_;
			}
		}

		// Inside the PurePursuit class, modify the computeVelocities function:

		// void computeVelocities(const Pose& current_pose) {
		// 	if (path_.empty() || goal_reached_) {
		// 		stopMoving();
		// 		return;
		// 	}
			
		// 	// First, calculate the distance to the target point
		// 	double dx = path_.back().x - current_pose.position.x();
		// 	double dy = path_.back().y - current_pose.position.y();
		// 	double distance_to_goal = std::sqrt(dx*dx + dy*dy);
			
		// 	// Check if we've reached the goal
		// 	if (distance_to_goal <= pos_tol_) {
		// 		goal_reached_ = true;
		// 		path_.clear();
		// 		stopMoving();
		// 		std::cout << "Goal reached!" << std::endl;
		// 		return;
		// 	}
			
		// 	// Get the current yaw from quaternion
		// 	Eigen::Matrix3d rot_matrix = current_pose.orientation.toRotationMatrix();
		// 	double current_yaw = std::atan2(rot_matrix(1, 0), rot_matrix(0, 0));

		// 	std::cout<<"current distance to goal "<<distance_to_goal
		// 				<<"current yaw "<< current_yaw
		// 				<<"current robot pose "<< "("<<current_pose.position.x()<<" "<<current_pose.position.y()<<")"<<std::endl;
			
		// 	// Find the lookahead point
		// 	bool found_target = false;
		// 	Eigen::Vector2d target_point;
			
		// 	// Start from the current index and find the first point beyond the lookahead distance
		// 	for (; idx_ < path_.size(); idx_++) {
		// 		dx = path_[idx_].x - current_pose.position.x();
		// 		dy = path_[idx_].y - current_pose.position.y();
		// 		double dist = std::sqrt(dx*dx + dy*dy);
				
		// 		if (dist > ld_) {
		// 			target_point = Eigen::Vector2d(path_[idx_].x, path_[idx_].y);
		// 			found_target = true;
		// 			break;
		// 		}
		// 	}
			
		// 	// If no point is beyond lookahead distance, use the last point
		// 	if (!found_target && !path_.empty()) {
		// 		target_point = Eigen::Vector2d(path_.back().x, path_.back().y);
		// 	}
			
		// 	// Calculate the target point in the robot's local frame
		// 	// First rotate the vector by -current_yaw, then translate
		// 	double local_x = (target_point[0] - current_pose.position.x()) * std::cos(-current_yaw) - 
		// 					(target_point[1] - current_pose.position.y()) * std::sin(-current_yaw);
		// 	double local_y = (target_point[0] - current_pose.position.x()) * std::sin(-current_yaw) + 
		// 					(target_point[1] - current_pose.position.y()) * std::cos(-current_yaw);
			
		// 	// Calculate the angular velocity using the pure pursuit formula
		// 	cmd_angular_ = std::clamp(2.0 * v_max_ * local_y / (ld_ * ld_), -w_max_, w_max_);
			
		// 	// Use a constant forward velocity
		// 	cmd_linear_ = v_max_;
			
		// 	// Debug output
		// 	std::cout << "Target: (" << target_point[0] << ", " << target_point[1] << ")" << std::endl;
		// 	std::cout << "Robot: (" << current_pose.position.x() << ", " << current_pose.position.y() 
		// 			<< "), yaw: " << current_yaw << std::endl;
		// 	std::cout << "Local target: (" << local_x << ", " << local_y << ")" << std::endl;
		// 	std::cout << "Cmd: linear=" << cmd_linear_ << ", angular=" << cmd_angular_ << std::endl;
		// }
		
		// 接收新路径
		void receivePath(const Waypoints& new_path) {
			path_ = new_path;
			idx_ = 0;
			
			if (!new_path.empty()) {
					goal_reached_ = false;
			} else {
					goal_reached_ = true;
					std::cout << "Received empty path!" << std::endl;
			}
		}
		
		// 获取当前线速度命令
		double getLinearVelocity() const {
			return cmd_linear_;
		}
		
		// 获取当前角速度命令
		double getAngularVelocity() const {
			return cmd_angular_;
		}
		
		// 获取当前转向角命令
		double getSteeringAngle() const {
			return cmd_steering_;
		}
		
		// 是否到达目标
		bool isGoalReached() const {
			return goal_reached_;
		}
		
private:
		// 变换到机器人坐标系
		Eigen::Affine3d transformToBaseLink(const Pose& target_pose, const Eigen::Affine3d& world_to_robot) {
			Eigen::Affine3d target_transform;
			target_transform.translation() = target_pose.position;
			target_transform.linear() = target_pose.orientation.toRotationMatrix();
			
			// 计算相对变换: robot^-1 * target
			return world_to_robot.inverse() * target_transform;
		}
		
		// 停止移动
		void stopMoving() {
			cmd_linear_ = 0.0;
			cmd_angular_ = 0.0;
			cmd_steering_ = 0.0;
			lookahead_point_ = Eigen::Vector3d::Zero();
		}
		
		// 路径
		Waypoints path_;
		
		// 算法参数
		double ld_;         // 预瞄距离
		double pos_tol_;    // 位置容差
		double L_;          // 轴距
		
		// 控制变量
		double v_max_;      // 最大速度
		double v_;          // 当前速度
		double w_max_;      // 最大角速度
		double delta_max_;  // 最大转向角
		
		// 命令输出
		double cmd_linear_;   // 线速度命令
		double cmd_angular_;  // 角速度命令
		double cmd_steering_; // 转向角命令
		
		// 状态变量
		unsigned idx_;        // 当前路径点索引
		bool goal_reached_;   // 是否到达目标
		Eigen::Vector3d lookahead_point_; // 预瞄点在机器人坐标系中的位置
};

// ##########################################################################################
// ####################################### Pure Pursuit #####################################
// ##########################################################################################
class Custom
{
public:
	Custom()
	{
		sport_client.SetTimeout(10.0f);
		sport_client.Init();

		suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
		suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);

		// controller = PurePursuit(0.8, 0.3, 0.5, 0.1, 0.4, 0.7);

		// zmq_context = new zmq::context_t(1);
		// zmq_socket = new zmq::socket_t(*zmq_context, zmq::socket_type::req);
		// // zmq_socket->connect("tcp://10.136.158.185:5555");
		// zmq_socket->connect("tcp://128.61.21.22:5555");
		// zmq_socket->set(zmq::sockopt::subscribe, 0);
		
		// std::cout << "Connected to MoCap server" << std::endl;
		std::cout << "hihihihi" << std::endl;
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
		robot_pose = Pose();
		// obstacles_pose = std::vector<Pose>;

		controller = PurePursuit(0.8, 0.3, 0.3, 0.1, 0.4, 0.3);

		setTargetPoint(0.0, 4.0, 0.0);
	};

	~Custom(){
		delete zmq_socket;
		delete zmq_context;
	}

	// In your Custom class, add a method to set a target point
	void setTargetPoint(double x, double y, double yaw = 0.0) {
		// Create a new Waypoints object
		Waypoints path;
		
		// Add the target point
		path.addPoint(x, y, yaw);
		
		// Pass the path to the controller
		controller.receivePath(path);
		
		if(DEBUG_MODE) {
			std::cout << "Set new target point: (" << x << ", " << y << ", " << yaw << ")" << std::endl;
		}
	}

	// To set multiple waypoints
	void setPath(const std::vector<std::array<double, 3>>& points) {
		// Create a new Waypoints object
		Waypoints path;
		
		// Add all points
		for (const auto& point : points) {
			path.addPoint(point[0], point[1], point[2]);
		}
		
		// Pass the path to the controller
		controller.receivePath(path);
		
		if(DEBUG_MODE) {
			std::cout << "Set new path with " << points.size() << " waypoints" << std::endl;
		}
	}

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

	void RobotControl()
	{
		ct += dt;
		double px_local, py_local, yaw_local;
		double vx_local, vy_local, vyaw_local;
		double px_err, py_err, yaw_err;
		double time_seg, time_temp;

		unitree::robot::go2::PathPoint path_point_tmp;
		std::vector<unitree::robot::go2::PathPoint> path;

		switch (TEST_MODE)
		{
		case pure_pursuit:{
			updatePoseFromMocap();
			
			// 使用PurePursuit计算控制命令
			controller.computeVelocities(robot_pose);
			
			// 如果目标未达成，发送控制命令
			if (!controller.isGoalReached()) {
					// 获取计算出的线速度和角速度
					double linear_vel = controller.getLinearVelocity();
					double angular_vel = controller.getAngularVelocity();
					
					// 发送控制命令到机器人
					sport_client.Move(linear_vel, 0, angular_vel);
					
					if(DEBUG_MODE){
						std::cout << "Pure Pursuit: v=" << linear_vel << ", w=" << angular_vel << std::endl;
						std::cout << "Position: " << robot_pose.position.x() << ", " 
											<< robot_pose.position.y() << ", " 
											<< robot_pose.position.z() << std::endl;
					}
					// 打印调试信息
					// if (ct - last_print_time > 1.0) { // 每秒打印一次
					// 		std::cout << "Pure Pursuit: v=" << linear_vel << ", w=" << angular_vel << std::endl;
					// 		std::cout << "Position: " << robot_pose.position.x() << ", " 
					// 							<< robot_pose.position.y() << ", " 
					// 							<< robot_pose.position.z() << std::endl;
					// 		last_print_time = ct;
					// }
			} else {
				// 目标已达成，停止移动
				sport_client.StopMove();
				// StopMoving();
				std::cout << "Pure Pursuit: Goal reached!" << std::endl;
				
				// 可选：重置标志，以便下次重新创建路径
				// flag = 0;
			}
			break;
		}

		case normal_stand:            // 0. idle, default stand
			sport_client.SwitchGait(0); // 0:idle; 1:tort; 2:tort running; 3:climb stair; 4:tort obstacle
			sport_client.StandUp();
			break;

		case balance_stand:                  // 1. Balance stand (controlled by dBodyHeight + rpy)
			sport_client.Euler(0.1, 0.2, 0.3); // roll, pitch, yaw
			sport_client.BodyHeight(0.0);      // relative height [-0.18~0.03]
			sport_client.BalanceStand();
			break;

		case velocity_move: // 2. target velocity walking (controlled by velocity + yawSpeed)
			sport_client.Move(0.3, 0, 0.3);
			break;

		case trajectory_follow: // 3. path mode walking
			time_seg = 0.2;
			time_temp = ct - time_seg;
			for (int i = 0; i < 30; i++)
			{
				time_temp += time_seg;

				px_local = 0.5 * sin(0.5 * time_temp);
				py_local = 0;
				yaw_local = 0;
				vx_local = 0.5 * cos(0.5 * time_temp);
				vy_local = 0;
				vyaw_local = 0;

				path_point_tmp.timeFromStart = i * time_seg;
				path_point_tmp.x = px_local * cos(yaw0) - py_local * sin(yaw0) + px0;
				path_point_tmp.y = px_local * sin(yaw0) + py_local * cos(yaw0) + py0;
				path_point_tmp.yaw = yaw_local + yaw0;
				path_point_tmp.vx = vx_local * cos(yaw0) - vy_local * sin(yaw0);
				path_point_tmp.vy = vx_local * sin(yaw0) + vy_local * cos(yaw0);
				path_point_tmp.vyaw = vyaw_local;
				path.push_back(path_point_tmp);
			}
			sport_client.TrajectoryFollow(path);
			break;

		case stand_down: // 4. position stand down.
			updatePoseFromMocap();
			sport_client.StandDown();
			break;

		case stand_up: // 5. position stand up
			sport_client.StandUp();
			break;

		case damp: // 6. damping mode
			sport_client.Damp();
			break;

		case recovery_stand: // 7. recovery stand
			sport_client.RecoveryStand();
			break;

		case sit:
			if (flag == 0)
			{
				sport_client.Sit();
				flag = 1;
			}
			break;

		case rise_sit:
			if (flag == 0)
			{
				sport_client.RiseSit();
				flag = 1;
			}
			break;

		case stretch:
			if (flag == 0)
			{
				sport_client.Stretch();
				flag = 1;
			}
			break;

		case wallow:
			if (flag == 0)
			{
				sport_client.Wallow();
				flag = 1;
			}
			break;
		/*
		case content:
			if (flag == 0)
			{
				sport_client.Content();
				flag = 1;
			}
			break;
		*/
		case pose:
			if (flag == 0)
			{
				sport_client.Pose(true);
				flag = 1;
			}
			break;

		case scrape:
			if (flag == 0)
			{
				sport_client.Scrape();
				flag = 1;
			}
			break;

		case front_flip:
			if (flag == 0)
			{
				sport_client.FrontFlip();
				flag = 1;
			}
			break;

		case front_jump:
			if (flag == 0)
			{
				sport_client.FrontJump();
				flag = 1;
			}
			break;
		case front_pounce:
			if (flag == 0)
			{
				sport_client.FrontPounce();
				flag = 1;
			}
			break;

		case stop_move: // stop move
			sport_client.StopMove();
			break;

		default:
			sport_client.StopMove();
		}
	};

	// Get initial position
	void GetInitState()
	{
		px0 = state.position()[0];
		py0 = state.position()[1];
		yaw0 = state.imu_state().rpy()[2];
		std::cout << "initial position: x0: " << px0 << ", y0: " << py0 << ", yaw0: " << yaw0 << std::endl;
	};

	void HighStateHandler(const void *message)
	{
		state = *(unitree_go::msg::dds_::SportModeState_ *)message;

		// std::cout << "Position: " << state.position()[0] << ", " << state.position()[1] << ", " << state.position()[2] << std::endl;
		// std::cout << "IMU rpy: " << state.imu_state().rpy()[0] << ", " << state.imu_state().rpy()[1] << ", " << state.imu_state().rpy()[2] << std::endl;
	};

	unitree_go::msg::dds_::SportModeState_ state;
	unitree::robot::go2::SportClient sport_client;
	unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;

	PurePursuit controller;
	zmq::context_t* zmq_context;
	zmq::socket_t* zmq_socket;
	Pose last_mocap_pose;
	double last_timestamp = 0.0; 
	double last_print_time = 0.0;
	Pose robot_pose;
	std::vector<Pose> obstacles_pose;

	double px0, py0, yaw0; // 初始状态的位置和偏航
	double ct = 0;         // 运行时间
	int flag = 0;          // 特殊动作执行标志
	float dt = 0.005;      // 控制步长0.001~0.01
};

int main(int argc, char **argv)
{
	if (argc < 2)
	{
		std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
		exit(-1);
	}

	unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
	Custom custom;
	PurePursuit controller;

	sleep(1); // Wait for 1 second to obtain a stable state

	custom.GetInitState(); // Get initial position
	unitree::common::ThreadPtr threadPtr = unitree::common::CreateRecurrentThread(custom.dt * 1000000, std::bind(&Custom::RobotControl, &custom));

	while (1)
	{
		sleep(10);
	}
	return 0;
}
