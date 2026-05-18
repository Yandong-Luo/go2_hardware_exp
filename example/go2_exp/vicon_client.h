#pragma once

#include <Eigen/Dense>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ViconDataStreamSDK {
namespace CPP {
class Client;
}
}

// Thin C++ wrapper around the Vicon DataStream SDK.
// Connects directly to Vicon Tracker (default port 801) and exposes a frame
// API matching the JSON shape that mocap_server.py used to publish over ZMQ:
//   robots: subject_name -> { position(m), quaternion(x,y,z,w), yaw }
//   points: subject_name -> position(m)
class ViconClient {
public:
    struct RobotPose {
        Eigen::Vector3d position{0.0, 0.0, 0.0};         // meters, world frame
        Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0}; // (w, x, y, z)
        double yaw = 0.0;                                 // radians, about Z
        bool occluded = false;
    };

    struct Frame {
        double timestamp = 0.0;
        std::map<std::string, RobotPose> robots;
        std::map<std::string, Eigen::Vector3d> points;
    };

    // host examples: "169.254.170.240:801", "localhost:801"
    explicit ViconClient(const std::string& host);
    ~ViconClient();

    ViconClient(const ViconClient&) = delete;
    ViconClient& operator=(const ViconClient&) = delete;

    bool isConnected() const;

    // 6-DoF subjects: returned in Frame::robots.
    void setRobotSubjects(const std::vector<std::string>& names);
    // Single-point subjects (only translation): returned in Frame::points.
    void setPointSubjects(const std::vector<std::string>& names);

    // Pull one frame from Vicon and fill `out`. Returns false on failure or
    // if no frame is currently available (ClientPull mode is used internally).
    bool tryGetFrame(Frame& out);

    // Discovery helper: return all subject names currently published by Tracker.
    std::vector<std::string> listAllSubjects();

private:
    std::unique_ptr<ViconDataStreamSDK::CPP::Client> client_;
    std::vector<std::string> robot_subjects_;
    std::vector<std::string> point_subjects_;
    bool connected_ = false;
};
