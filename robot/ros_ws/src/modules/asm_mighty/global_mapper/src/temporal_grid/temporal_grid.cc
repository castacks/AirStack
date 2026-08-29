// Temporal grid class for dynamic obstacle segmentation
#include "temporal_grid/temporal_grid.h"

// Fields: {is_free, occupied_duration, unoccupied_duration, last_occupied_time, last_unoccupied_time, start_time, last_dynamic_time}
// Zero-initialized arrays represent uninitialized voxels (start_time == 0.0).

namespace temporal_grid
{
TemporalGrid::TemporalGrid(const double origin[3], const double world_dimensions[3], const float resolution, float occupied_threshold, float unoccupied_threshold, int neigh_thresh, double dynamic_persistence, double maturity_time)
: voxel_grid::VoxelGrid<std::array<double, 7>>(origin, world_dimensions, resolution), occupied_threshold_(occupied_threshold), unoccupied_threshold_(unoccupied_threshold), resolution_(resolution), neigh_thresh_(neigh_thresh), dynamic_persistence_(dynamic_persistence), maturity_time_(maturity_time)
{
    offsets_ = this->generateOffsets();
}

void TemporalGrid::UpdateTemporalInfo(const int ind, const bool is_occupied, const bool is_unknown, const double timestamp)
{
    timestamp_ = timestamp;

    std::array<double, 7>& info = this->GetReference(ind);

    if (is_unknown)
    {
        info = {0.0, 0.0, 0.0, timestamp, timestamp, timestamp, 0.0};
        return;
    }

    // Initialize if this voxel has never been touched (start_time == 0.0)
    if (info[5] == 0.0)
    {
        info = {0.0, 0.0, 0.0, timestamp, timestamp, timestamp, 0.0};
    }

    double is_free = info[0];
    double occupied_duration, unoccupied_duration;
    double last_occupied_time = info[3];
    double last_unoccupied_time = info[4];
    double start_time = info[5];

    if (is_occupied)
    {
        occupied_duration = timestamp - last_unoccupied_time;
        unoccupied_duration = 0.0;
        last_occupied_time = timestamp;
    }
    else
    {
        unoccupied_duration = timestamp - last_occupied_time;
        occupied_duration = 0.0;
        last_unoccupied_time = timestamp;
    }

    if (unoccupied_duration > unoccupied_threshold_)
    {
        is_free = 1.0;
    }
    if (occupied_duration > occupied_threshold_)
    {
        is_free = 0.0;
    }

    info[0] = is_free;
    info[1] = occupied_duration;
    info[2] = unoccupied_duration;
    info[3] = last_occupied_time;
    info[4] = last_unoccupied_time;
    info[5] = start_time;

    // Pre-compute dynamic classification here (in the Spin thread) so that
    // IsDynamic() becomes a cheap read-only check during Publish.
    if (is_free == 1.0 && is_occupied)
    {
        int ixyz[3];
        this->IndexToGrid(ind, ixyz);
        if (this->AreNeighborsStatic(ixyz, neigh_thresh_))
        {
            // Neighbors are static → this is likely a static surface, clear dynamic label
            info[6] = 0.0;
        }
        else
        {
            // Only mark dynamic if voxel has been observed long enough (maturity guard)
            double age = timestamp - start_time;
            if (maturity_time_ <= 0.0 || age >= maturity_time_)
            {
                info[6] = timestamp;  // last_dynamic_time
            }
        }
    }
}

void TemporalGrid::UpdateTemporalInfo(const int ixyz[3], const bool is_occupied, const bool is_unknown, const double timestamp)
{
    int ind = GridToIndex(ixyz);
    this->UpdateTemporalInfo(ind, is_occupied, is_unknown, timestamp);
}

void TemporalGrid::UpdateTemporalInfo(const double xyz[3], const bool is_occupied, const bool is_unknown, const double timestamp)
{
    int ixyz[3];
    this->WorldToGrid(xyz, ixyz);
    if (this->IsInMap(ixyz))
    {
        this->UpdateTemporalInfo(ixyz, is_occupied, is_unknown, timestamp);
    }
}

bool TemporalGrid::AreNeighborsStatic(const int ixyz[3], int neigh_thresh)
{
    int not_free_neigh_counter = 0;
    int neigh_xyz[3];
    for (const auto& offset : offsets_)
    {
        neigh_xyz[0] = ixyz[0] + offset[0];
        neigh_xyz[1] = ixyz[1] + offset[1];
        neigh_xyz[2] = ixyz[2] + offset[2];

        if (!this->IsInMap(neigh_xyz)) continue;

        const std::array<double, 7>& temporal_info = this->ReadValue(neigh_xyz);
        // Voxel is initialized (start_time != 0) and is static (is_free == 0.0)
        if (temporal_info[5] != 0.0 && temporal_info[0] == 0.0)
        {
            not_free_neigh_counter++;
        }

        if (not_free_neigh_counter >= neigh_thresh)
        {
            return true;
        }
    }
    return false;
}

bool TemporalGrid::IsDynamic(const int ind, bool is_occupied) const
{
    // Read-only O(1) check — heavy work (neighbor check + persistence write)
    // was already done in UpdateTemporalInfo during the Spin thread.
    const std::array<double, 7>& voxel = this->ReadValue(ind);
    if (voxel[5] == 0.0)
        return false;
    return voxel[6] > 0.0 && (timestamp_ - voxel[6]) < dynamic_persistence_;
}

bool TemporalGrid::IsDynamic(const int ixyz[3], bool is_occupied) const
{
    int ind = this->GridToIndex(ixyz);
    return this->IsDynamic(ind, is_occupied);
}

bool TemporalGrid::IsDynamic(const double xyz[3], bool is_occupied) const
{
    int ixyz[3];
    this->WorldToGrid(xyz, ixyz);
    if (this->IsInMap(ixyz))
    {
        return this->IsDynamic(ixyz, is_occupied);
    }
    return false;
}

std::array<double, 7> TemporalGrid::GetTemporalInfo(const int ixyz[3]) const
{
    int ind = this->GridToIndex(ixyz);
    return this->GetTemporalInfo(ind);
}

std::array<double, 7> TemporalGrid::GetTemporalInfo(const double xyz[3]) const
{
    int ixyz[3];
    this->WorldToGrid(xyz, ixyz);
    if (this->IsInMap(ixyz))
    {
        return this->GetTemporalInfo(ixyz);
    }
    return std::array<double, 7>{}; // zero-initialized = uninitialized sentinel
}

std::array<double, 7> TemporalGrid::GetTemporalInfo(const int ind) const
{
    return this->ReadValue(ind);
}

void TemporalGrid::SetFree(const double xyz[3], float is_free)
{
    int ixyz[3];
    this->WorldToGrid(xyz, ixyz);
    if (this->IsInMap(ixyz))
    {
        this->SetFree(ixyz, is_free);
    }
}

void TemporalGrid::SetFree(const int ixyz[3], float is_free)
{
    int ind = this->GridToIndex(ixyz);
    this->SetFree(ind, is_free);
}

void TemporalGrid::SetFree(const int ind, float is_free)
{
    std::array<double, 7>& info = this->GetReference(ind);
    info = {static_cast<double>(is_free), 0.0, 0.0, timestamp_, timestamp_, timestamp_, 0.0};
}

void TemporalGrid::PostShiftOrigin(const std::vector<int>& slice_indexes)
{
    for (const int index : slice_indexes)
    {
        std::array<double, 7>& info = this->GetReference(index);
        info = {0.0, 0.0, 0.0, timestamp_, timestamp_, timestamp_, 0.0};
    }
}

void TemporalGrid::PreShiftOrigin(const std::vector<int>& slice_indexes)
{
    // Do nothing
}
} // namespace
