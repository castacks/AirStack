#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include <random>

// Simple RRT node structure for testing
struct RRTNode {
    double x, y, z;
    int parent_id;
    
    RRTNode(double x = 0, double y = 0, double z = 0, int parent = -1) 
        : x(x), y(y), z(z), parent_id(parent) {}
    
    double distance(const RRTNode& other) const {
        return std::sqrt(std::pow(x - other.x, 2) + 
                        std::pow(y - other.y, 2) + 
                        std::pow(z - other.z, 2));
    }
};

// Simple RRT implementation for testing
class SimpleRRT {
public:
    std::vector<RRTNode> nodes;
    std::mt19937 rng;
    std::uniform_real_distribution<double> dist;
    
    SimpleRRT(double min_val = -10.0, double max_val = 10.0) 
        : rng(std::random_device{}()), dist(min_val, max_val) {}
    
    void addNode(const RRTNode& node) {
        nodes.push_back(node);
    }
    
    int findNearestNode(const RRTNode& target) {
        if (nodes.empty()) return -1;
        
        int nearest_id = 0;
        double min_dist = nodes[0].distance(target);
        
        for (size_t i = 1; i < nodes.size(); ++i) {
            double dist = nodes[i].distance(target);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_id = i;
            }
        }
        return nearest_id;
    }
    
    RRTNode generateRandomNode() {
        return RRTNode(dist(rng), dist(rng), dist(rng));
    }
    
    RRTNode steerTowards(const RRTNode& from, const RRTNode& to, double step_size) {
        double dx = to.x - from.x;
        double dy = to.y - from.y;
        double dz = to.z - from.z;
        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        if (distance <= step_size) {
            return to;
        }
        
        double ratio = step_size / distance;
        return RRTNode(from.x + dx * ratio, 
                      from.y + dy * ratio, 
                      from.z + dz * ratio);
    }
    
    std::vector<RRTNode> getPath(int goal_node_id) {
        std::vector<RRTNode> path;
        int current_id = goal_node_id;
        
        while (current_id != -1) {
            path.push_back(nodes[current_id]);
            current_id = nodes[current_id].parent_id;
        }
        
        std::reverse(path.begin(), path.end());
        return path;
    }
};

// Test fixture
class RRTTest : public ::testing::Test {
protected:
    SimpleRRT rrt;
    
    void SetUp() override {
        // Add root node
        rrt.addNode(RRTNode(0, 0, 0, -1));
    }
};

// Test basic node operations
TEST_F(RRTTest, NodeDistance) {
    RRTNode node1(0, 0, 0);
    RRTNode node2(3, 4, 0);
    
    EXPECT_DOUBLE_EQ(node1.distance(node2), 5.0);
}

TEST_F(RRTTest, AddNode) {
    EXPECT_EQ(rrt.nodes.size(), 1);
    
    rrt.addNode(RRTNode(1, 1, 1, 0));
    EXPECT_EQ(rrt.nodes.size(), 2);
    
    EXPECT_DOUBLE_EQ(rrt.nodes[1].x, 1.0);
    EXPECT_DOUBLE_EQ(rrt.nodes[1].y, 1.0);
    EXPECT_DOUBLE_EQ(rrt.nodes[1].z, 1.0);
    EXPECT_EQ(rrt.nodes[1].parent_id, 0);
}

TEST_F(RRTTest, FindNearestNode) {
    rrt.addNode(RRTNode(1, 0, 0, 0));
    rrt.addNode(RRTNode(0, 1, 0, 0));
    rrt.addNode(RRTNode(2, 2, 0, 0));
    
    RRTNode target(1.1, 0.1, 0);
    int nearest = rrt.findNearestNode(target);
    
    EXPECT_EQ(nearest, 1); // Should be node at (1, 0, 0)
}

TEST_F(RRTTest, SteerTowards) {
    RRTNode from(0, 0, 0);
    RRTNode to(10, 0, 0);
    double step_size = 2.0;
    
    RRTNode steered = rrt.steerTowards(from, to, step_size);
    
    EXPECT_DOUBLE_EQ(steered.x, 2.0);
    EXPECT_DOUBLE_EQ(steered.y, 0.0);
    EXPECT_DOUBLE_EQ(steered.z, 0.0);
}

TEST_F(RRTTest, SteerTowardsCloseTarget) {
    RRTNode from(0, 0, 0);
    RRTNode to(1, 0, 0);
    double step_size = 2.0;
    
    RRTNode steered = rrt.steerTowards(from, to, step_size);
    
    // Should return the target since it's within step size
    EXPECT_DOUBLE_EQ(steered.x, 1.0);
    EXPECT_DOUBLE_EQ(steered.y, 0.0);
    EXPECT_DOUBLE_EQ(steered.z, 0.0);
}

TEST_F(RRTTest, PathGeneration) {
    // Build a simple tree: 0 -> 1 -> 2
    rrt.addNode(RRTNode(1, 0, 0, 0));  // Node 1, parent 0
    rrt.addNode(RRTNode(2, 0, 0, 1));  // Node 2, parent 1
    
    std::vector<RRTNode> path = rrt.getPath(2);
    
    EXPECT_EQ(path.size(), 3);
    EXPECT_DOUBLE_EQ(path[0].x, 0.0); // Root
    EXPECT_DOUBLE_EQ(path[1].x, 1.0); // Node 1
    EXPECT_DOUBLE_EQ(path[2].x, 2.0); // Node 2
}

TEST_F(RRTTest, RandomNodeGeneration) {
    RRTNode random1 = rrt.generateRandomNode();
    RRTNode random2 = rrt.generateRandomNode();
    
    // Check that random nodes are within bounds
    EXPECT_GE(random1.x, -10.0);
    EXPECT_LE(random1.x, 10.0);
    EXPECT_GE(random1.y, -10.0);
    EXPECT_LE(random1.y, 10.0);
    EXPECT_GE(random1.z, -10.0);
    EXPECT_LE(random1.z, 10.0);
    
    // Check that we get different random nodes (very high probability)
    EXPECT_FALSE(random1.x == random2.x && random1.y == random2.y && random1.z == random2.z);
}

// Test RRT algorithm behavior
TEST_F(RRTTest, BasicRRTGrowth) {
    RRTNode goal(5, 5, 0);
    double step_size = 1.0;
    int max_iterations = 100;
    
    for (int i = 0; i < max_iterations; ++i) {
        RRTNode random_node = rrt.generateRandomNode();
        int nearest_id = rrt.findNearestNode(random_node);
        
        RRTNode new_node = rrt.steerTowards(rrt.nodes[nearest_id], random_node, step_size);
        new_node.parent_id = nearest_id;
        
        rrt.addNode(new_node);
        
        // Check if we're close to goal
        if (new_node.distance(goal) < 1.0) {
            break;
        }
    }
    
    // Should have grown the tree
    EXPECT_GT(rrt.nodes.size(), 1);
    
    // All nodes should have valid parent relationships
    for (size_t i = 1; i < rrt.nodes.size(); ++i) {
        EXPECT_GE(rrt.nodes[i].parent_id, -1);
        EXPECT_LT(rrt.nodes[i].parent_id, static_cast<int>(i));
    }
}

// Test cost map functionality (simplified)
class CostMapTest : public ::testing::Test {
protected:
    struct SimpleCostMap {
        std::vector<std::vector<double>> costs;
        double resolution;
        double origin_x, origin_y;
        
        SimpleCostMap(int width, int height, double res = 1.0) 
            : resolution(res), origin_x(0), origin_y(0) {
            costs.resize(height, std::vector<double>(width, 0.0));
        }
        
        double getCost(double x, double y) {
            int grid_x = static_cast<int>((x - origin_x) / resolution);
            int grid_y = static_cast<int>((y - origin_y) / resolution);
            
            if (grid_x < 0 || grid_x >= static_cast<int>(costs[0].size()) ||
                grid_y < 0 || grid_y >= static_cast<int>(costs.size())) {
                return 1.0; // High cost for out of bounds
            }
            
            return costs[grid_y][grid_x];
        }
        
        void setCost(int x, int y, double cost) {
            if (x >= 0 && x < static_cast<int>(costs[0].size()) &&
                y >= 0 && y < static_cast<int>(costs.size())) {
                costs[y][x] = cost;
            }
        }
    };
};

TEST_F(CostMapTest, BasicCostMapOperations) {
    SimpleCostMap cost_map(10, 10, 1.0);
    
    // Test default cost
    EXPECT_DOUBLE_EQ(cost_map.getCost(5, 5), 0.0);
    
    // Test setting and getting cost
    cost_map.setCost(5, 5, 0.8);
    EXPECT_DOUBLE_EQ(cost_map.getCost(5.0, 5.0), 0.8);
    
    // Test out of bounds
    EXPECT_DOUBLE_EQ(cost_map.getCost(-1, -1), 1.0);
    EXPECT_DOUBLE_EQ(cost_map.getCost(15, 15), 1.0);
}

TEST_F(CostMapTest, CollisionChecking) {
    SimpleCostMap cost_map(10, 10, 1.0);
    
    // Add obstacle
    cost_map.setCost(5, 5, 1.0);
    
    auto isCollisionFree = [&](const RRTNode& node) {
        return cost_map.getCost(node.x, node.y) < 0.5;
    };
    
    EXPECT_TRUE(isCollisionFree(RRTNode(0, 0, 0)));
    EXPECT_FALSE(isCollisionFree(RRTNode(5, 5, 0)));
}

// Main function for running tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}