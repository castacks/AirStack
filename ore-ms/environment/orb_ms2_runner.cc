// Development adapter for pinned GPLv3 ORB-SLAM3; no estimator changes.
#include <System.h>
#include <MapPoint.h>
#include <Map.h>
#include <Tracking.h>
#include <Settings.h>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>

struct Input {
    long long left_ns, right_ns;
    std::string stem, left, right;
};

int main(int argc, char** argv) {
    if (argc != 7) {
        std::cerr << "usage: orb_ms2_runner vocabulary settings inputs.tsv output.jsonl seed threads\n";
        return 2;
    }
    try {
        std::ifstream file(argv[3]);
        if (!file) throw std::runtime_error("cannot open input list");
        std::vector<Input> inputs;
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream stream(line);
            std::vector<std::string> fields;
            std::string field;
            while (std::getline(stream, field, '\t')) fields.push_back(field);
            if (fields.size() != 5) throw std::runtime_error("invalid input row");
            Input row{std::stoll(fields[0]), std::stoll(fields[1]), fields[2], fields[3], fields[4]};
            if (!inputs.empty() && (row.left_ns <= inputs.back().left_ns ||
                                   row.right_ns <= inputs.back().right_ns))
                throw std::runtime_error("nonmonotonic input clock");
            inputs.push_back(row);
        }
        if (inputs.empty()) throw std::runtime_error("empty input list");
        int fd = open(argv[4], O_WRONLY | O_CREAT | O_EXCL, 0600);
        if (fd < 0) throw std::runtime_error("output already exists or cannot be created");
        close(fd);
        std::ofstream out(argv[4]);
        out << std::setprecision(10);
        cv::setRNGSeed(std::stoi(argv[5]));
        cv::setNumThreads(std::stoi(argv[6]));
        ORB_SLAM3::Settings configuration(argv[2], ORB_SLAM3::System::STEREO);
        out << "{\"configuration\":{\"baseline_m\":" << configuration.b()
            << ",\"bf\":" << configuration.bf()
            << ",\"fx\":" << configuration.camera1()->getParameter(0)
            << ",\"rerectifies\":" << (configuration.needToRectify() ? "true" : "false")
            << ",\"resizes\":" << (configuration.needToResize() ? "true" : "false") << "}}\n";
        out.flush();
        ORB_SLAM3::System slam(argv[1], argv[2], ORB_SLAM3::System::STEREO, false);
        const auto start = std::chrono::steady_clock::now();
        for (size_t i = 0; i < inputs.size(); ++i) {
            const auto& row = inputs[i];
            auto left = cv::imread(row.left, cv::IMREAD_UNCHANGED);
            auto right = cv::imread(row.right, cv::IMREAD_UNCHANGED);
            if (left.empty() || right.empty()) throw std::runtime_error("prepared image unreadable");
            if (left.size() != right.size() || left.depth() != CV_8U || right.depth() != CV_8U)
                throw std::runtime_error("invalid prepared stereo input");
            const auto before = std::chrono::steady_clock::now();
            // Integer subtraction precedes float conversion. Keep original ns in journal.
            auto tcw = slam.TrackStereo(left, right, (row.left_ns - inputs[0].left_ns) * 1e-9);
            const auto received = std::chrono::steady_clock::now();
            int state = slam.GetTrackingState();
            std::set<unsigned long> maps;
            if (state == ORB_SLAM3::Tracking::OK) {
                for (auto point : slam.GetTrackedMapPoints())
                    if (point && !point->isBad()) maps.insert(point->GetMap()->GetId());
            }
            bool valid = state == ORB_SLAM3::Tracking::OK && maps.size() == 1 && tcw.matrix().allFinite();
            out << "{\"index\":" << i << ",\"time_ns\":" << row.left_ns
                << ",\"right_time_ns\":" << row.right_ns << ",\"state\":" << state
                << ",\"map_id\":" << (maps.size() == 1 ? std::to_string(*maps.begin()) : "null")
                << ",\"processing_ns\":"
                << std::chrono::duration_cast<std::chrono::nanoseconds>(received-before).count()
                << ",\"receipt_elapsed_ns\":"
                << std::chrono::duration_cast<std::chrono::nanoseconds>(received-start).count()
                << ",\"T_camera_from_world\":";
            if (!valid) out << "null";
            else {
                auto m = tcw.matrix();
                out << '[';
                for (int r=0; r<4; ++r) {
                    if (r) out << ',';
                    out << '[';
                    for (int c=0; c<4; ++c) { if (c) out << ','; out << m(r,c); }
                    out << ']';
                }
                out << ']';
            }
            out << "}\n";
            out.flush();
        }
        slam.Shutdown();
        out << "{\"terminal_success\":true,\"processed_frames\":" << inputs.size() << "}\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "adapter failure: " << e.what() << '\n';
        return 1;
    }
}
