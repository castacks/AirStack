// Development adapter for pinned GPLv3 ORB-SLAM3; no estimator changes.
#include <System.h>
#include <MapPoint.h>
#include <Map.h>
#include <Tracking.h>
#include <Atlas.h>
#include <LocalMapping.h>
#include <LoopClosing.h>
#include <map>
#include <vector>
#include <algorithm>
#include <Eigen/LU>
#include <Settings.h>
#include <Frame.h>
#include <cmath>
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

template<class M> void matrix_json(std::ostream& out, const M& m) {
    out << "["; for(int r=0;r<4;++r) { if(r)out<<",";out<<"[";for(int c=0;c<4;++c){if(c)out<<",";out<<m(r,c);}out<<"]";}out<<"]";
}
std::ofstream exclusive(const std::string& path) {
    int fd=open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL,0600);
    if(fd<0)throw std::runtime_error("export already exists or inaccessible");close(fd);return std::ofstream(path);
}

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
        auto stereo=exclusive(std::string(argv[4])+".stereo.jsonl");stereo<<std::setprecision(17);
        std::vector<bool> received_valid;
        std::vector<long long> received_maps;
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
            received_valid.push_back(valid);
            received_maps.push_back(maps.size()==1 ? static_cast<long long>(*maps.begin()) : -1);
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
            // mCurrentFrame is owned by this synchronous tracking thread. Do not
            // dereference shared MapPoint geometry; pointer presence is enough.
            const auto& frame=slam.ExportTracker()->mCurrentFrame;
            // A map reset assigns a default Frame; its scalar fields are
            // uninitialized. Inspect constructed vectors before any scalars.
            if(frame.mvKeys.empty()) {
                stereo<<"{\"index\":"<<i<<",\"source_id\":\""<<row.stem<<"\",\"time_ns\":"<<row.left_ns
                      <<",\"right_time_ns\":"<<row.right_ns<<",\"state\":"<<state
                      <<",\"available\":false,\"reason\":\"native_frame_empty_after_return\",\"features\":[]}\n";
                stereo.flush();continue;
            }
            if(frame.Nleft!=-1 || frame.N<0 || frame.mvKeys.size()!=static_cast<size_t>(frame.N)
                || frame.mvuRight.size()!=static_cast<size_t>(frame.N) || frame.mvDepth.size()!=static_cast<size_t>(frame.N))
                throw std::runtime_error("nonparallel native rectified stereo arrays");
            stereo<<"{\"index\":"<<i<<",\"source_id\":\""<<row.stem<<"\",\"time_ns\":"<<row.left_ns
                  <<",\"right_time_ns\":"<<row.right_ns<<",\"native_time\":"<<frame.mTimeStamp
                  <<",\"available\":true,\"native_frame_id\":"<<frame.mnId<<",\"state\":"<<state<<",\"N\":"<<frame.N
                  <<",\"right_keypoints\":"<<frame.mvKeysRight.size()<<",\"bf\":"<<frame.mbf
                  <<",\"baseline_m\":"<<frame.mb<<",\"close_depth_m\":"<<frame.mThDepth
                  <<",\"map_array_size\":"<<frame.mvpMapPoints.size()<<",\"outlier_array_size\":"<<frame.mvbOutlier.size()
                  <<",\"features\":[";
            for(int j=0;j<frame.N;++j) {
                if(j)stereo<<",";const auto& k=frame.mvKeys[j];
                if(!std::isfinite(k.pt.x)||!std::isfinite(k.pt.y)||!std::isfinite(frame.mvuRight[j])||!std::isfinite(frame.mvDepth[j]))
                    throw std::runtime_error("nonfinite native stereo measurement");
                stereo<<"["<<k.pt.x<<","<<k.pt.y<<","<<k.octave<<","<<frame.mvuRight[j]<<","<<frame.mvDepth[j]<<",";
                if(static_cast<size_t>(j)<frame.mvpMapPoints.size())stereo<<(frame.mvpMapPoints[j]?"true":"false");else stereo<<"null";
                stereo<<",";if(static_cast<size_t>(j)<frame.mvbOutlier.size())stereo<<(frame.mvbOutlier[j]?"true":"false");else stereo<<"null";
                stereo<<"]";
            }
            stereo<<"]}\n";stereo.flush();
        }
        slam.Shutdown();
        auto mapper=slam.ExportLocalMapper();auto loop=slam.ExportLoopCloser();
        const auto shutdown_start=std::chrono::steady_clock::now();
        while(!mapper->isFinished() || !loop->isFinished() || loop->isRunningGBA()) {
            if(std::chrono::duration<double>(std::chrono::steady_clock::now()-shutdown_start).count()>60.)
                throw std::runtime_error("export worker-quiescence timeout");
            usleep(1000);
        }
        auto tracker=slam.ExportTracker();auto atlas=slam.ExportAtlas();
        size_t count=tracker->mlRelativeFramePoses.size();
        if(count!=tracker->mlpReferences.size() || count!=tracker->mlFrameTimes.size() || count!=tracker->mlbLost.size())
            throw std::runtime_error("nonparallel native final history");
        struct H { EIGEN_MAKE_ALIGNED_OPERATOR_NEW Sophus::SE3f relative; ORB_SLAM3::KeyFrame* ref; double time; bool lost; };
        std::vector<H,Eigen::aligned_allocator<H>> history;
        auto refs=tracker->mlpReferences.begin();auto stamps=tracker->mlFrameTimes.begin();auto lost=tracker->mlbLost.begin();
        for(auto it=tracker->mlRelativeFramePoses.begin();it!=tracker->mlRelativeFramePoses.end();++it,++refs,++stamps,++lost)
            history.push_back(H{*it,*refs,*stamps,*lost});
        std::map<double,size_t> exact_source;
        for(size_t i=0;i<inputs.size();++i) {
            double t=(inputs[i].left_ns-inputs[0].left_ns)*1e-9;
            if(!exact_source.emplace(t,i).second)throw std::runtime_error("native clock collision");
        }
        std::vector<std::vector<size_t>> candidates(inputs.size());
        for(size_t h=0;h<history.size();++h) {
            auto it=exact_source.find(history[h].time);
            if(it==exact_source.end())throw std::runtime_error("history has unknown exact source clock");
            if(!history[h].lost)candidates[it->second].push_back(h);
        }
        auto maps=atlas->GetAllMaps();
        std::sort(maps.begin(),maps.end(),[](ORB_SLAM3::Map* a, ORB_SLAM3::Map* b){return a->GetId()<b->GetId();});
        std::map<unsigned long,Eigen::Matrix4d> gauges;
        auto metadata=exclusive(std::string(argv[4])+".maps.json");metadata<<std::setprecision(17);
        metadata<<"{\"local_mapper_finished\":true,\"loop_closer_finished\":true,\"gba_running\":false,\"history_entries\":"<<history.size()<<",\"maps\":[";
        bool first_map=true;
        for(auto map:maps) {
            auto keyframes=map->GetAllKeyFrames();
            std::sort(keyframes.begin(),keyframes.end(),ORB_SLAM3::KeyFrame::lId);
            if(keyframes.empty())continue;
            auto origin=keyframes.front();
            gauges[map->GetId()]=origin->GetPoseInverse().matrix().cast<double>();
            if(!first_map)metadata<<",";first_map=false;
            metadata<<"{\"map_id\":"<<map->GetId()<<",\"keyframes\":"<<keyframes.size()<<",\"origin_keyframe_id\":"<<origin->mnId<<",\"T_map_from_origin\":";
            matrix_json(metadata,gauges[map->GetId()]);metadata<<"}";
        }
        metadata<<"]}\n";metadata.close();
        auto final=exclusive(std::string(argv[4])+".final.jsonl");final<<std::setprecision(17);
        for(size_t i=0;i<inputs.size();++i) {
            final<<"{\"index\":"<<i<<",\"source_id\":\""<<inputs[i].stem<<"\",\"time_ns\":"<<inputs[i].left_ns
                 <<",\"right_time_ns\":"<<inputs[i].right_ns<<",\"received_valid\":"<<(received_valid[i]?"true":"false")
                 <<",\"received_map_id\":"<<received_maps[i]<<",\"candidate_count\":"<<candidates[i].size();
            std::string reason;
            if(!received_valid[i])reason="received_missing";
            else if(candidates[i].empty())reason="native_lost_or_missing_history";
            else if(candidates[i].size()!=1)reason="ambiguous_native_history";
            if(!reason.empty()) {final<<",\"reason\":\""<<reason<<"\",\"T_world_from_camera\":null}\n";continue;}
            size_t h=candidates[i].front();auto kf=history[h].ref;
            final<<",\"history_index\":"<<h<<",\"native_time\":"<<history[h].time<<",\"native_lost\":false";
            if(!kf) {final<<",\"reason\":\"null_reference\",\"T_world_from_camera\":null}\n";continue;}
            Eigen::Matrix4d relative=history[h].relative.matrix().cast<double>();
            Eigen::Matrix4d chain=Eigen::Matrix4d::Identity();
            std::set<unsigned long> visited;
            final<<",\"T_frame_from_reference\":";matrix_json(final,relative);final<<",\"reference_chain\":[";
            bool first_chain=true;
            while(kf && kf->isBad()) {
                if(!visited.insert(kf->mnId).second || visited.size()>10000)throw std::runtime_error("culled-keyframe cycle");
                auto parent=kf->GetParent();Eigen::Matrix4d link=kf->mTcp.matrix().cast<double>();
                if(!first_chain)final<<",";first_chain=false;
                final<<"{\"keyframe_id\":"<<kf->mnId<<",\"parent_id\":"<<(parent?std::to_string(parent->mnId):"null")<<",\"T_child_from_parent\":";
                matrix_json(final,link);final<<"}";chain=chain*link;kf=parent;
            }
            final<<"]";
            if(!kf || !kf->GetMap() || !gauges.count(kf->GetMap()->GetId()) || static_cast<long long>(kf->GetMap()->GetId())!=received_maps[i]) {
                final<<",\"reason\":\"final_map_missing_or_changed\",\"T_world_from_camera\":null}\n";continue;
            }
            auto id=kf->GetMap()->GetId();Eigen::Matrix4d reference=kf->GetPose().matrix().cast<double>();
            Eigen::Matrix4d result=(relative*chain*reference*gauges[id]).inverse();
            if(!result.allFinite())throw std::runtime_error("nonfinite finalized reconstruction");
            final<<",\"final_map_id\":"<<id<<",\"resolved_keyframe_id\":"<<kf->mnId<<",\"T_reference_from_map\":";
            matrix_json(final,reference);final<<",\"T_map_from_origin\":";matrix_json(final,gauges[id]);
            final<<",\"reason\":null,\"T_world_from_camera\":";matrix_json(final,result);final<<"}\n";
        }
        final.close();
        for(auto map:maps) {
            if(map->GetAllKeyFrames().empty())continue;
            std::string path=std::string(argv[4])+".map-"+std::to_string(map->GetId())+".euroc.txt";
            if(access(path.c_str(),F_OK)==0)throw std::runtime_error("official map export already exists");
            slam.SaveTrajectoryEuRoC(path,map);
        }
        out << "{\"terminal_success\":true,\"processed_frames\":" << inputs.size() << "}\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "adapter failure: " << e.what() << '\n';
        return 1;
    }
}
