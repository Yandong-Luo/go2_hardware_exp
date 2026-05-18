// Minimal smoke test for ViconClient.
//
// Usage:
//   ./vicon_test                            # connects to 169.254.170.240:801
//   ./vicon_test 192.168.1.10:801           # custom host
//   ./vicon_test 169.254.170.240:801 go_1   # also subscribe to subject "go_1"
//
// First it lists every subject Vicon Tracker currently publishes, so you can
// confirm names without having to look at Tracker's GUI. Then, if you passed
// any subject names on the command line, it pulls frames and prints poses.

#include "vicon_client.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

int main(int argc, char** argv) {
    std::string host = (argc >= 2) ? argv[1] : "169.254.170.240:801";

    std::vector<std::string> subjects;
    for (int i = 2; i < argc; ++i) {
        subjects.emplace_back(argv[i]);
    }

    ViconClient vicon(host);
    if (!vicon.isConnected()) {
        std::cerr << "Failed to connect to " << host << std::endl;
        return 1;
    }

    auto all = vicon.listAllSubjects();
    std::cout << "[vicon] subjects published by Tracker (" << all.size() << "):\n";
    for (const auto& s : all) std::cout << "  - " << s << "\n";

    if (subjects.empty()) {
        if (all.empty()) {
            std::cout << "[vicon] no subjects in Tracker. Exiting.\n";
            return 0;
        }
        std::cout << "[vicon] no subjects specified on CLI — subscribing to all "
                     "discovered subjects as 6-DoF robots.\n";
        subjects = all;
    }

    vicon.setRobotSubjects(subjects);

    std::cout << "\n[vicon] streaming... Ctrl-C to stop.\n";
    int frame_idx = 0;
    while (true) {
        ViconClient::Frame frame;
        if (!vicon.tryGetFrame(frame)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        // Print roughly twice a second.
        if (++frame_idx % 50 == 0) {
            std::cout << std::fixed << std::setprecision(3);
            std::cout << "[t=" << frame.timestamp << "] ";
            for (const auto& [name, pose] : frame.robots) {
                std::cout << name
                          << " pos=(" << pose.position.x()
                          << ", " << pose.position.y()
                          << ", " << pose.position.z() << ") "
                          << "yaw=" << pose.yaw
                          << (pose.occluded ? " [occluded]" : "")
                          << "  ";
            }
            std::cout << std::endl;
        }
    }
    return 0;
}
