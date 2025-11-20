#ifndef _ASTAR_VDB_H
#define _ASTAR_VDB_H

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <chrono>
#include <unordered_set>
#include <vector>
#include <memory>

#include <openvdb/openvdb.h>
#include <openvdb/Grid.h>

#include <vdb_edt/vdbmap.h>

struct CoordHash
{
    std::size_t operator()(const openvdb::Coord &c) const noexcept
    {
        std::uint64_t h = 0xCBF29CE484222325ull;
        auto mix = [&](int v)
        {
            std::uint64_t x = static_cast<std::uint32_t>(v);
            h ^= x + 0x9E3779B97F4A7C15ull + (h << 6) + (h >> 2);
        };
        mix(c.x());
        mix(c.y());
        mix(c.z());
        return static_cast<std::size_t>(h);
    }
};

class AstarNode
{
public:
    openvdb::Coord ijk;
    double f_score, g_score;
    AstarNode *parent;

    AstarNode()
    {
        parent = NULL;
    }
    ~AstarNode() {};
};

class NodeComparator0
{
public:
    bool operator()(AstarNode *node1, AstarNode *node2)
    {
        return node1->f_score > node2->f_score;
    }
};

class Astar
{
public:
    Astar();
    ~Astar();
    enum
    {
        REACH_END = 1,
        NO_PATH = 2
    };

    void initialize(std::shared_ptr<VDBMap> &map_manager);
    void reset();

    int search(const openvdb::Coord &start_pt, const openvdb::Coord &end_pt);
    static double pathLength(const std::vector<openvdb::Coord> &path);

    std::vector<openvdb::Vec3d> getPathAstar();
    std::vector<openvdb::Coord> getVisited();
    double getEarlyTerminateCost();

    double lambda_heu_;
    double max_search_time_;

    void pathSmooth(std::vector<openvdb::Vec3d> &sparse_path_out);
    void pathShorten(std::vector<openvdb::Vec3d> &sparse_path_out);


private:
    void backtrack(const AstarNode *end_node);

    double getDiagHeu(const openvdb::Coord &x1, const openvdb::Coord &x2);
    double getManhHeu(const openvdb::Coord &x1, const openvdb::Coord &x2);
    double getEuclHeu(const openvdb::Coord &x1, const openvdb::Coord &x2);

    // main data structure
    std::vector<AstarNode *> path_node_pool_;
    int use_node_num_, iter_num_;

    std::priority_queue<AstarNode *, std::vector<AstarNode *>, NodeComparator0> open_set_;
    std::unordered_map<openvdb::Coord, AstarNode *, CoordHash> open_set_map_;
    std::unordered_set<openvdb::Coord, CoordHash> close_set_;

    std::vector<openvdb::Coord> path_nodes_;
    double early_terminate_cost_;

    std::shared_ptr<VDBMap> map_manager_;

    // parameter
    double margin_;
    int allocate_num_;
    double tie_breaker_;

    double safe_sq_index_dist_;
    double safe_index_dist_;

    // path shorten
    double L_THRESH;
    int num_inflate_check_normal_;
    double normal_step_delta_;

    bool estimateSurfaceNormalAtHit(const openvdb::Coord &hit_coord,
                                    openvdb::Vec3d &normal_out);
};

#endif