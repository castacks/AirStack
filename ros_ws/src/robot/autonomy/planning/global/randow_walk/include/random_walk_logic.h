
#include <string>
#include <tuple>
#include <vector>

typedef std::vector<std::tuple<float, float, float, float>> Path;  // x, y, z, yaw

struct random_walk_init_params {
    float max_start_to_goal_dist_m;
    float max_angle_change_deg;
    float checking_dist_m;
    float waypoint_dist_m;
    float max_z_m;
    float collision_padding_m;
    std::tuple<float, float, float> voxel_size_m;
};

class RandomWalkPlanner {
   public:
    std::vector<std::tuple<float, float, float>> voxel_points;

    RandomWalkPlanner(random_walk_init_params init_params);

    std::vector<std::tuple<float, float, float>> generate_path(
        std::tuple<float, float, float> start_point);

   private:
    // Numerical constants
    float max_start_to_goal_dist_m_;
    float max_angle_change_deg_;
    float checking_dist_m_;
    float waypoint_dist_m_;
    float max_z_m_;
    float collision_padding_m;

    // Variables
    // convetion is x, y, z
    Path generated_path;
    std::tuple<float, float, float> voxel_size_m;

    // Functions
    bool check_if_collided_single_voxel(std::tuple<float, float, float> point,
                              std::tuple<float, float, float> voxel_center);

    bool check_if_collided(std::tuple<float, float, float> point);

    std::tuple<float, float, float> generate_goal_point(std::tuple<float, float, float> start_point);



};
