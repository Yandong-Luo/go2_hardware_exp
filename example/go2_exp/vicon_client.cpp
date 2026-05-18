#include "vicon_client.h"

#include "ViconDataStreamSDK_CPP/DataStreamClient.h"

#include <chrono>
#include <cmath>
#include <iostream>

namespace VDS = ViconDataStreamSDK::CPP;

namespace {
double yawFromQuat(double qx, double qy, double qz, double qw) {
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace

ViconClient::ViconClient(const std::string& host)
    : client_(std::make_unique<VDS::Client>()) {
    std::cout << "[vicon] connecting to " << host << " ..." << std::endl;

    auto out = client_->Connect(host);
    if (out.Result != VDS::Result::Success) {
        std::cerr << "[vicon] Connect() failed, code=" << static_cast<int>(out.Result) << std::endl;
        return;
    }

    client_->EnableSegmentData();
    client_->EnableMarkerData();
    client_->SetStreamMode(VDS::StreamMode::ClientPull);
    // make sure this align with the mocap system's streaming rate to avoid unnecessary frame drops.
    client_->SetAxisMapping(VDS::Direction::Forward,
                            VDS::Direction::Left,
                            VDS::Direction::Up);

    auto version = client_->GetVersion();
    std::cout << "[vicon] connected. SDK version: "
              << version.Major << "." << version.Minor << "."
              << version.Point << "." << version.Revision << std::endl;
    connected_ = true;
}

ViconClient::~ViconClient() {
    if (client_ && connected_) {
        client_->Disconnect();
    }
}

bool ViconClient::isConnected() const {
    return connected_;
}

void ViconClient::setRobotSubjects(const std::vector<std::string>& names) {
    robot_subjects_ = names;
}

void ViconClient::setPointSubjects(const std::vector<std::string>& names) {
    point_subjects_ = names;
}

std::vector<std::string> ViconClient::listAllSubjects() {
    std::vector<std::string> names;
    if (!connected_) return names;

    if (client_->GetFrame().Result != VDS::Result::Success) {
        return names;
    }
    auto count = client_->GetSubjectCount();
    if (count.Result != VDS::Result::Success) return names;

    for (unsigned int i = 0; i < count.SubjectCount; ++i) {
        auto name = client_->GetSubjectName(i);
        if (name.Result == VDS::Result::Success) {
            names.emplace_back(name.SubjectName);
        }
    }
    return names;
}

bool ViconClient::tryGetFrame(Frame& out) {
    if (!connected_) return false;

    if (client_->GetFrame().Result != VDS::Result::Success) {
        return false;
    }

    out.robots.clear();
    out.points.clear();
    out.timestamp = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    for (const auto& name : robot_subjects_) {
        auto trans = client_->GetSegmentGlobalTranslation(name, name);
        auto rot = client_->GetSegmentGlobalRotationQuaternion(name, name);
        if (trans.Result != VDS::Result::Success ||
            rot.Result != VDS::Result::Success) {
            continue;
        }

        RobotPose pose;
        pose.occluded = trans.Occluded || rot.Occluded;
        // Vicon translations are in millimetres.
        pose.position = Eigen::Vector3d(trans.Translation[0] / 1000.0,
                                        trans.Translation[1] / 1000.0,
                                        trans.Translation[2] / 1000.0);
        // SDK returns quaternion as [qx, qy, qz, qw].
        const double qx = rot.Rotation[0];
        const double qy = rot.Rotation[1];
        const double qz = rot.Rotation[2];
        const double qw = rot.Rotation[3];
        pose.orientation = Eigen::Quaterniond(qw, qx, qy, qz);
        pose.yaw = yawFromQuat(qx, qy, qz, qw);
        out.robots.emplace(name, pose);
    }

    for (const auto& name : point_subjects_) {
        auto trans = client_->GetSegmentGlobalTranslation(name, name);
        if (trans.Result != VDS::Result::Success || trans.Occluded) continue;
        out.points.emplace(name, Eigen::Vector3d(trans.Translation[0] / 1000.0,
                                                  trans.Translation[1] / 1000.0,
                                                  trans.Translation[2] / 1000.0));
    }

    return true;
}
