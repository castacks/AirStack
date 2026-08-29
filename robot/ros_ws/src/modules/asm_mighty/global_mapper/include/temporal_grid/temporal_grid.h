// Temporal grid class for dynamic obstacle segmentation

#pragma once

#include <array>
#include <vector>
#include <Eigen/Core>
#include <cmath>

#include "voxel_grid/voxel_grid.h"
#include "occupancy_grid/occupancy_grid.h"

namespace temporal_grid {

class TemporalGrid: public voxel_grid::VoxelGrid<std::array<double, 7>> // Might want the elements to be arrays instead of vectors
{
public:
// Constructor and destructor go here along with any other public methods
    TemporalGrid(const double origin[3], const double world_dimensions[3], const float resolution, float occupied_threshold, float unoccupied_threshold, int neigh_thresh = 5, double dynamic_persistence = 1.0, double maturity_time = 0.0);
    void UpdateTemporalInfo(const int ixyz[3], const bool is_occupied, const bool is_unknown, const double timestamp);
    void UpdateTemporalInfo(const double xyz[3], const bool is_occupied, const bool is_unknown, const double timestamp);
    bool AreNeighborsStatic(const int ixyz[3], int neigh_thresh);
    bool IsDynamic(const int ixyz[3], bool is_occupied) const;
    bool IsDynamic(const double xyz[3], bool is_occupied) const;
    std::array<double, 7> GetTemporalInfo(const int ixyz[3]) const;
    std::array<double, 7> GetTemporalInfo(const double xyz[3]) const;
    void SetFree(const double xyz[3], float is_free);
    void SetFree(const int ixyz[3], float is_free);
    void PostShiftOrigin(const std::vector<int>& slice_indexes);
    void PreShiftOrigin(const std::vector<int>& slice_indexes);

    void UpdateTemporalInfo(const int ind, const bool is_occupied, const bool is_unknown, const double timestamp); // TODO: Move back to private!
    bool IsDynamic(const int ind, bool is_occupied) const;  //TODO: Move back to private!
    std::array<double, 7> GetTemporalInfo(const int ind) const; //TODO: Move back to private!
    void SetFree(const int ind, float is_free); //TODO: Move back to private!

private:
// Private methods go here
    float occupied_threshold_;
    float unoccupied_threshold_;
    float resolution_;
    double timestamp_;
    int neigh_thresh_;
    double dynamic_persistence_;
    double maturity_time_;
    std::vector<std::array<int, 3>> offsets_;

    inline static std::vector<std::array<int, 3>> generateOffsets(int infla = 1)
    {
        std::vector<std::array<int, 3>> offsets;
        offsets.reserve((2 * infla + 1) * (2 * infla + 1) * (2 * infla + 1));
        for (int dx = -infla; dx <= infla; ++dx) {
            for (int dy = -infla; dy <= infla; ++dy) {
                for (int dz = -infla; dz <= infla; ++dz) {
                    offsets.push_back({dx, dy, dz});
                }
            }
        }
        return offsets;
    }
};

} // Namespace