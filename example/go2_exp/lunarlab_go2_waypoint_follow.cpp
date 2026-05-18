
/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <cmath>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#include <Eigen/Dense>
#include <vector>
#include <nlohmann/json.hpp>
#include <chrono>
#include <iostream>
#include <fstream>
#include <ctime>
#include <memory>

#include "vicon_client.h"

using json = nlohmann::json;

#define TOPIC_HIGHSTATE "rt/sportmodestate"

#define DEBUG_MODE true

#define MAX_LINEAR_VEL 0.6
#define MAX_ANGULAR_VEL 1.0

using namespace unitree::common;

enum test_mode
{
	/*---Basic motion---*/
	waypoint_tracking,
	normal_stand,
	balance_stand,
	velocity_move,
	stand_down,
	stand_up,
	damp,
	recovery_stand,
	stop_move = 99
};

const int TEST_MODE = waypoint_tracking;

// ##########################################################################################
// ####################################### PID Controller ###################################
// ##########################################################################################
struct Pose {
	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;

	double yaw;

	Pose() : position(0, 0, 0), orientation(Eigen::Quaterniond::Identity()) {}

	Pose(const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient)
			: position(pos), orientation(orient) {}

	Pose(double x, double y, double z, const Eigen::Quaterniond& orient)
			: position(x, y, z), orientation(orient) {}
};

class Waypoints {
private:
	struct Point2D {
		double x, y, yaw, wait_time;

		Point2D() : x(0.0), y(0.0), yaw(0.0), wait_time(0.0){}
		Point2D(double _x, double _y, double _yaw = 0.0, double _wait_time=0.0) : x(_x), y(_y), yaw(_yaw), wait_time(_wait_time) {}

		Eigen::Vector2d position() const {
			return Eigen::Vector2d(x, y);
		}
	};

	std::vector<Point2D> points;

public:
	Waypoints() {}

	void addPoint(double x, double y, double yaw = 0.0, double wait_time=0.0) {
		points.emplace_back(x, y, yaw, wait_time);
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

double getCurrentTime() {
    using namespace std::chrono;
    return duration_cast<duration<double>>(steady_clock::now().time_since_epoch()).count();
}

// Single-dimension PID controller for x, y, or yaw control
class SingleDimensionPID {
private:
    double kp_, ki_, kd_;
    double min_output_;
    double max_output_;
    double error_sum_;
    double prev_error_;
    double dt_;

public:
    SingleDimensionPID(
        double kp = 1.0, double ki = 0.0, double kd = 0.0,
        double min_output = -1.0, double max_output = 1.0,
        double dt = 0.01
    ) : kp_(kp), ki_(ki), kd_(kd),
        min_output_(min_output), max_output_(max_output),
        error_sum_(0.0), prev_error_(0.0), dt_(dt) {}

    void reset() {
        error_sum_ = 0.0;
        prev_error_ = 0.0;
    }

    void setTimeStep(double dt) {
        dt_ = dt;
    }

    void setOutputLimits(double min_output, double max_output) {
        min_output_ = min_output;
        max_output_ = max_output;
    }

    double calculate(double error) {
        double p_term = kp_ * error;

        error_sum_ += error * dt_;
        error_sum_ = std::clamp(error_sum_, -10.0, 10.0);
        double i_term = ki_ * error_sum_;

        double error_rate = (error - prev_error_) / dt_;
        double d_term = kd_ * error_rate;

        prev_error_ = error;

        double output = p_term + i_term + d_term;
        return std::clamp(output, min_output_, max_output_);
    }
};

// Main controller that uses three separate PID controllers
class ThreeDimensionPIDController {
public:
    ThreeDimensionPIDController(
        double x_p = 1.0, double x_i = 0.0, double x_d = 0.0,
        double y_p = 1.0, double y_i = 0.0, double y_d = 0.0,
        double yaw_p = 1.0, double yaw_i = 0.0, double yaw_d = 0.0,
        double max_linear_vel = 0.3,
        double max_angular_vel = 0.3,
        double position_tolerance = 0.3,
        double angle_tolerance = 0.2
    ) : max_linear_vel_(max_linear_vel),
        max_angular_vel_(max_angular_vel),
        pos_tol_(position_tolerance),
        ang_tol_(angle_tolerance),
        current_waypoint_idx_(0),
        goal_reached_(true),
        dt_(0.01),
        cmd_linear_x_(0.0),
        cmd_linear_y_(0.0),
        cmd_angular_(0.0),
		waiting_(false)
    {
        x_controller_ = SingleDimensionPID(x_p, x_i, x_d, -max_linear_vel, max_linear_vel, dt_);
        y_controller_ = SingleDimensionPID(y_p, y_i, y_d, -max_linear_vel, max_linear_vel, dt_);
        yaw_controller_ = SingleDimensionPID(yaw_p, yaw_i, yaw_d, -max_angular_vel, max_angular_vel, dt_);
    }

    void setPath(const Waypoints& new_path) {
        path_ = new_path;
        current_waypoint_idx_ = 0;

        x_controller_.reset();
        y_controller_.reset();
        yaw_controller_.reset();

        if (!new_path.empty()) {
            goal_reached_ = false;
        } else {
            goal_reached_ = true;
            std::cout << "Received empty path!" << std::endl;
        }
    }

    // Transform a target position from world frame to robot frame
    Eigen::Vector3d getPositionInRobotFrame(const Eigen::Vector3d& world_point,
        double robot_x, double robot_y, double robot_yaw) {
        double cos_yaw = cos(robot_yaw);
        double sin_yaw = sin(robot_yaw);

        double translated_x = world_point.x() - robot_x;
        double translated_y = world_point.y() - robot_y;

        double robot_frame_x = translated_x * cos_yaw + translated_y * sin_yaw;
        double robot_frame_y = -translated_x * sin_yaw + translated_y * cos_yaw;

        return Eigen::Vector3d(
            robot_frame_x,
            robot_frame_y,
            world_point.z()
        );
    }

    double normalize_angle(double angle) {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

	// Compute velocities based on current pose
	void computeVelocities(const Pose& current_pose) {
		current_pose_ = current_pose;

		if (path_.empty() || goal_reached_) {
			cmd_linear_x_ = 0.0;
			cmd_linear_y_ = 0.0;
			cmd_angular_ = 0.0;
			return;
		}

		const auto& target = path_[current_waypoint_idx_];

		Eigen::Vector3d target_position(target.x, target.y, 0.0);

		Eigen::Vector3d target_in_robot_frame = getPositionInRobotFrame(
			target_position,
			current_pose.position.x(),
			current_pose.position.y(),
			current_pose.yaw
		);

		double dx_robot = target_in_robot_frame.x();
		double dy_robot = target_in_robot_frame.y();

		double distance = std::sqrt(dx_robot*dx_robot + dy_robot*dy_robot);

		double yaw_error = normalize_angle(target.yaw - current_pose_.yaw);

		// Check if we've reached the current waypoint (both position AND orientation)
		if (distance <= pos_tol_ && std::abs(yaw_error) <= ang_tol_) {
			const double wait_duration = target.wait_time;
			std::cout<<"current wait time:"<<wait_duration<<std::endl;
			if (!waiting_ && wait_duration > 0.0) {
				waiting_ = true;
				wait_start_time_ = getCurrentTime();

				cmd_linear_x_ = 0.0;
				cmd_linear_y_ = 0.0;
				cmd_angular_ = 0.0;

				if (DEBUG_MODE) {
					std::cout << "Waiting for " << wait_duration << " seconds at waypoint " << current_waypoint_idx_ << std::endl;
				}
				return;
			}

			if (waiting_) {
				double elapsed = getCurrentTime() - wait_start_time_;
				if (elapsed < wait_duration) {
					cmd_linear_x_ = 0.0;
					cmd_linear_y_ = 0.0;
					cmd_angular_ = 0.0;
					return;
				} else {
					waiting_ = false;
				}
			}

			current_waypoint_idx_++;

			x_controller_.reset();
			y_controller_.reset();
			yaw_controller_.reset();

			if (current_waypoint_idx_ >= path_.size()) {
				goal_reached_ = true;
				cmd_linear_x_ = 0.0;
				cmd_linear_y_ = 0.0;
				cmd_angular_ = 0.0;
				std::cout << "Goal reached!" << std::endl;
				return;
			}

			const auto& new_target = path_[current_waypoint_idx_];

			Eigen::Vector3d new_target_position(new_target.x, new_target.y, 0.0);
			Eigen::Vector3d new_target_in_robot_frame = getPositionInRobotFrame(
				new_target_position,
				current_pose.position.x(),
				current_pose.position.y(),
				current_pose.yaw
			);

			dx_robot = new_target_in_robot_frame.x();
			dy_robot = new_target_in_robot_frame.y();
			distance = std::sqrt(dx_robot*dx_robot + dy_robot*dy_robot);

			yaw_error = normalize_angle(new_target.yaw - current_pose_.yaw);
		}

		double x_error = dx_robot;
		double y_error = dy_robot;

		cmd_linear_x_ = x_controller_.calculate(x_error);
		cmd_linear_y_ = y_controller_.calculate(y_error);
		cmd_angular_ = yaw_controller_.calculate(yaw_error);

		// If we're significantly off-heading, prioritize turning over moving
		if (std::abs(yaw_error) > 0.5) {
			cmd_linear_x_ *= 0.5;
			cmd_linear_y_ *= 0.5;
		}

		// When close to the waypoint, prioritize orientation
		if (distance < pos_tol_ * 2 && std::abs(yaw_error) > ang_tol_) {
			cmd_linear_x_ *= 0.3;
			cmd_linear_y_ *= 0.3;
		}

		if (DEBUG_MODE) {
			std::cout << "=========================================================" << std::endl;
			std::cout << "Target world: (" << target.x << ", " << target.y << "), yaw: " << target.yaw << std::endl;
			std::cout << "Target robot: (" << dx_robot << ", " << dy_robot << ")" << std::endl;
			std::cout << "Current: (" << current_pose.position.x() << ", "
					<< current_pose.position.y() << "), yaw: " << current_yaw() << std::endl;
			std::cout << "Distance: " << distance << ", Yaw error: " << yaw_error << " rad" << std::endl;
			std::cout << "Position tolerance: " << pos_tol_ << ", Angle tolerance: " << ang_tol_ << std::endl;
			std::cout << "Cmd: x=" << cmd_linear_x_ << ", y=" << cmd_linear_y_ << ", angular=" << cmd_angular_ << std::endl;
			std::cout << "=========================================================" << std::endl;
		}
	}

    double current_yaw() const {
        return current_pose_.yaw;
    }

    double getLinearVelocityX() const {
        return cmd_linear_x_;
    }

    double getLinearVelocityY() const {
        return cmd_linear_y_;
    }

    double getAngularVelocity() const {
        return cmd_angular_;
    }

    bool isGoalReached() const {
        return goal_reached_;
    }

	bool isWaitState() const {
		return waiting_;
	}

    void setPositionTolerance(double tolerance) {
        pos_tol_ = tolerance;
    }

    void setAngleTolerance(double tolerance) {
        ang_tol_ = tolerance;
    }

    void setTimeStep(double dt) {
        dt_ = dt;
        x_controller_.setTimeStep(dt);
        y_controller_.setTimeStep(dt);
        yaw_controller_.setTimeStep(dt);
    }

    Pose current_pose_;

private:
    SingleDimensionPID x_controller_;
    SingleDimensionPID y_controller_;
    SingleDimensionPID yaw_controller_;

    Waypoints path_;

    double max_linear_vel_;
    double max_angular_vel_;

    double pos_tol_;
    double ang_tol_;

    size_t current_waypoint_idx_;
    bool goal_reached_;
    double dt_;

	bool waiting_;
	double wait_start_time_ = 0.0;

    double cmd_linear_x_;
    double cmd_linear_y_;
    double cmd_angular_;
};

// ##########################################################################################
// ####################################### Main Class #######################################
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

		vicon_client = std::make_unique<ViconClient>("169.254.170.240:801");
		if (vicon_client->isConnected()) {
			vicon_client->setRobotSubjects({"go2_body"});
			std::cout << "Connected to Vicon" << std::endl;
		} else {
			std::cerr << "Failed to connect to Vicon" << std::endl;
		}

		robot_pose = Pose();

		// Open pose log file (JSON Lines: one JSON object per line).
		char log_filename[128];
		std::time_t now = std::time(nullptr);
		std::strftime(log_filename, sizeof(log_filename), "pose_log_%Y%m%d_%H%M%S.jsonl", std::localtime(&now));
		pose_log.open(log_filename, std::ios::out | std::ios::app);
		if (pose_log.is_open()) {
			std::cout << "Logging pose to " << log_filename << std::endl;
		} else {
			std::cerr << "Failed to open pose log file: " << log_filename << std::endl;
		}

		// Create three-dimension PID controller with tuned parameters
		controller = ThreeDimensionPIDController(
		    2.0, 0.05, 0.1,   // X PID gains
		    2.0, 0.05, 0.1,   // Y PID gains
		    10.0, 0.0, 0.3,   // Yaw PID gains
		    MAX_LINEAR_VEL, MAX_ANGULAR_VEL, 0.2, 0.2);

		controller.setTimeStep(dt);

		sleep(1);

		setWaypoints();
	};

	~Custom(){
		if (pose_log.is_open()) {
			pose_log.close();
		}
	}

	void setWaypoints(){
		std::vector<Eigen::Vector3d> waypoint_pos;
		std::vector<double> wait_time;

		// Define the waypoints the robot should follow.
		// Format: Vector3d(x, y, yaw), wait_time at each waypoint (seconds).
		// waypoint_pos.push_back(Eigen::Vector3d(0.500, 2.643, -1.571)); wait_time.push_back(0.0);
		// waypoint_pos.push_back(Eigen::Vector3d(0.500, 2.114, -1.571)); wait_time.push_back(0.0);
		// waypoint_pos.push_back(Eigen::Vector3d(0.500, 1.586, -1.571)); wait_time.push_back(0.0);
		// waypoint_pos.push_back(Eigen::Vector3d(0.500, 1.057, -1.571)); wait_time.push_back(0.0);
		// waypoint_pos.push_back(Eigen::Vector3d(0.500, 0.529,  0.000)); wait_time.push_back(2.0);
		// waypoint_pos.push_back(Eigen::Vector3d(0.500, 1.057,  1.571)); wait_time.push_back(0.0);
		// waypoint_pos.push_back(Eigen::Vector3d(0.500, 1.586,  1.571)); wait_time.push_back(0.0);
		// waypoint_pos.push_back(Eigen::Vector3d(0.500, 2.114,  1.571)); wait_time.push_back(0.0);
		// waypoint_pos.push_back(Eigen::Vector3d(0.500, 2.643,  0.000)); wait_time.push_back(0.0);

		// face to kuka arm is x positive, face to white wall is y positive
		waypoint_pos.push_back(Eigen::Vector3d(1.5, 0.0, 0.0)); wait_time.push_back(0.0);

		for(size_t i = 0; i < waypoint_pos.size(); ++i){
			path.addPoint(waypoint_pos[i].x(), waypoint_pos[i].y(), waypoint_pos[i].z(), wait_time[i]);
		}

		controller.setPath(path);

		if(DEBUG_MODE) {
			std::cout << "Loaded " << path.size() << " waypoints into the controller" << std::endl;
		}
	}

	void updatePoseFromMocap() {
		if (!vicon_client || !vicon_client->isConnected()) return;

		ViconClient::Frame frame;
		if (!vicon_client->tryGetFrame(frame)) return;

		auto it = frame.robots.find("go2_body");
		if (it == frame.robots.end()) return;

		const auto& pose = it->second;
		// Convert from Vicon world frame to the EKF/control world frame
		// (EKF +X = Vicon +Y, EKF +Y = Vicon -X), matching the Python pipeline
		// in mocap_server.py / EKF logs. In this frame raw vicon_yaw already
		// equals the robot's physical-forward angle, so no yaw offset is needed.
		robot_pose.position = Eigen::Vector3d(pose.position.y(),
		                                     -pose.position.x(),
		                                     pose.position.z());
		robot_pose.orientation = pose.orientation;
		robot_pose.yaw = pose.yaw;
		last_timestamp = frame.timestamp;

		if (pose_log.is_open()) {
			json log_entry;
			log_entry["t"] = getCurrentTime();
			log_entry["mocap_timestamp"] = last_timestamp;
			log_entry["position"] = {
				robot_pose.position.x(),
				robot_pose.position.y(),
				robot_pose.position.z()
			};
			log_entry["orientation"] = {
				robot_pose.orientation.x(),
				robot_pose.orientation.y(),
				robot_pose.orientation.z(),
				robot_pose.orientation.w()
			};
			log_entry["yaw"] = robot_pose.yaw;
			pose_log << log_entry.dump() << "\n";
			pose_log.flush();
		}

		if (DEBUG_MODE) {
			std::cout << "Robot position: x:" << robot_pose.position.x() << " y:"
			          << robot_pose.position.y() << " yaw: " << robot_pose.yaw << std::endl;
		}
	}

	void RobotControl()
	{
		ct += dt;

		switch (TEST_MODE)
		{
		case waypoint_tracking:{
			updatePoseFromMocap();
			controller.computeVelocities(robot_pose);

			if (!controller.isGoalReached()) {

				if (controller.isWaitState()){
					sport_client.StopMove();
				}
				else{
					double angular_vel = controller.getAngularVelocity();
					double vx = controller.getLinearVelocityX();
					double vy = controller.getLinearVelocityY();

					std::cout<<"vx: "<<vx<<" vy: "<<vy<<" angular vel: "<<angular_vel<<std::endl;

					if(vx > MAX_LINEAR_VEL)	vx = MAX_LINEAR_VEL;
					if(vx < -MAX_LINEAR_VEL)	vx = -MAX_LINEAR_VEL;

					if(vy > MAX_LINEAR_VEL)	vy = MAX_LINEAR_VEL;
					if(vy < -MAX_LINEAR_VEL)	vy = -MAX_LINEAR_VEL;

					if(angular_vel > MAX_LINEAR_VEL)	angular_vel = MAX_LINEAR_VEL;
					if(angular_vel < -MAX_LINEAR_VEL)	angular_vel = -MAX_LINEAR_VEL;

					sport_client.Move(vx, vy, angular_vel);

					if(DEBUG_MODE){
						std::cout << "PID Control: vx=" << vx << " vy=" << vy <<", w=" << angular_vel << std::endl;
						std::cout << "Position: " << robot_pose.position.x() << ", "
												<< robot_pose.position.y() << ", "
												<< robot_pose.position.z() << std::endl;
					}
				}
			} else {
				sport_client.StopMove();
				std::cout << "PID Controller: Goal reached!" << std::endl;
			}
			break;
		}

		case normal_stand:
			sport_client.SwitchGait(0);
			sport_client.StandUp();
			break;

		case balance_stand:
			sport_client.Euler(0.1, 0.2, 0.3);
			sport_client.BodyHeight(0.0);
			sport_client.BalanceStand();
			break;

		case velocity_move:
			sport_client.Move(0.3, 0, 0.3);
			break;

		case stand_down:
			sport_client.StandDown();
			break;

		case stand_up:
			sport_client.StandUp();
			break;

		case damp:
			sport_client.Damp();
			break;

		case recovery_stand:
			sport_client.RecoveryStand();
			break;

		case stop_move:
			updatePoseFromMocap();
			sport_client.StopMove();
			break;

		default:
			sport_client.StopMove();
		}
	};

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
	};

	unitree_go::msg::dds_::SportModeState_ state;
	unitree::robot::go2::SportClient sport_client;
	unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;

	ThreeDimensionPIDController controller;
	std::unique_ptr<ViconClient> vicon_client;

	double last_timestamp = 0.0;
	Pose robot_pose;
    Waypoints path;

	std::ofstream pose_log;

	double px0, py0, yaw0;
	double ct = 0;
	int flag = 0;
	float dt = 0.005;
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

	sleep(1);

	custom.GetInitState();
	unitree::common::ThreadPtr threadPtr = unitree::common::CreateRecurrentThread(custom.dt * 1000000, std::bind(&Custom::RobotControl, &custom));

	while (1)
	{
		sleep(1);
	}

	return 0;
}
