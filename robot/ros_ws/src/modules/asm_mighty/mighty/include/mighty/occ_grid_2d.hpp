#pragma once

#include <cmath>
#include <cstdint>
#include <memory>
#include <queue>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

/** @brief 2D tristate occupancy grid with precomputed distance-based heat map.
 *
 *  Built from the mapper's occ_2d_topic (nav_msgs::OccupancyGrid). Each cell is
 *  one of three states: UNKNOWN (-1), FREE (0), or OCCUPIED (>=100). The
 *  unknown bit is required for frontier detection — a frontier is, by
 *  definition, a known-free cell adjacent to an unknown cell.
 *
 *  HGP A* only consumes `occupiedData()`, which contains true-occupied cells
 *  only. Unknown cells are therefore implicitly traversable for global
 *  planning (the "plan through unknown" semantics every modern explorer uses).
 *
 *  Computes a BFS distance transform from occupied cells, then converts to a
 *  linear heat ramp for A* cost.
 *
 *  Immutable after construction — safe to share across threads via
 *  shared_ptr<const>.
 */
class OccGrid2D {
 public:
  static std::shared_ptr<const OccGrid2D> fromOccupancyGrid(
      const nav_msgs::msg::OccupancyGrid& msg) {
    auto grid = std::make_shared<OccGrid2D>();
    grid->origin_x_ = msg.info.origin.position.x;
    grid->origin_y_ = msg.info.origin.position.y;
    grid->resolution_ = msg.info.resolution;
    grid->inv_resolution_ = 1.0 / grid->resolution_;
    grid->width_ = static_cast<int>(msg.info.width);
    grid->height_ = static_cast<int>(msg.info.height);

    const size_t n = msg.data.size();
    grid->occupied_.resize(n);
    grid->unknown_.resize(n);
    for (size_t i = 0; i < n; ++i) {
      grid->occupied_[i] = (msg.data[i] >= 100);
      grid->unknown_[i]  = (msg.data[i] < 0);
    }
    return grid;
  }

  /** @brief Build from a raw tristate buffer (test helper). */
  static std::shared_ptr<const OccGrid2D> fromTristate(
      int width, int height, double resolution,
      double origin_x, double origin_y,
      const std::vector<int8_t>& data) {
    auto grid = std::make_shared<OccGrid2D>();
    grid->origin_x_ = origin_x;
    grid->origin_y_ = origin_y;
    grid->resolution_ = resolution;
    grid->inv_resolution_ = 1.0 / resolution;
    grid->width_ = width;
    grid->height_ = height;
    const size_t n = static_cast<size_t>(width) * height;
    grid->occupied_.assign(n, false);
    grid->unknown_.assign(n, false);
    for (size_t i = 0; i < n && i < data.size(); ++i) {
      grid->occupied_[i] = (data[i] >= 100);
      grid->unknown_[i]  = (data[i] < 0);
    }
    return grid;
  }

  bool isOccupied(int ix, int iy) const {
    if (ix < 0 || ix >= width_ || iy < 0 || iy >= height_) return true;
    return occupied_[iy * width_ + ix];
  }

  bool isUnknown(int ix, int iy) const {
    // Out-of-bounds is conceptually unknown (the mapper window doesn't cover
    // it). This is what frontier detectors and global memory expect.
    if (ix < 0 || ix >= width_ || iy < 0 || iy >= height_) return true;
    return unknown_[iy * width_ + ix];
  }

  bool isFree(int ix, int iy) const {
    if (ix < 0 || ix >= width_ || iy < 0 || iy >= height_) return false;
    const size_t idx = static_cast<size_t>(iy) * width_ + ix;
    return !occupied_[idx] && !unknown_[idx];
  }

  /** @brief Query occupancy at world coordinates. */
  bool isOccupiedWorld(double x, double y) const {
    int ix = static_cast<int>(std::floor((x - origin_x_) * inv_resolution_));
    int iy = static_cast<int>(std::floor((y - origin_y_) * inv_resolution_));
    return isOccupied(ix, iy);
  }

  bool isUnknownWorld(double x, double y) const {
    int ix = static_cast<int>(std::floor((x - origin_x_) * inv_resolution_));
    int iy = static_cast<int>(std::floor((y - origin_y_) * inv_resolution_));
    return isUnknown(ix, iy);
  }

  bool isFreeWorld(double x, double y) const {
    int ix = static_cast<int>(std::floor((x - origin_x_) * inv_resolution_));
    int iy = static_cast<int>(std::floor((y - origin_y_) * inv_resolution_));
    return isFree(ix, iy);
  }

  void worldToGrid(double x, double y, int& ix, int& iy) const {
    ix = static_cast<int>(std::floor((x - origin_x_) * inv_resolution_));
    iy = static_cast<int>(std::floor((y - origin_y_) * inv_resolution_));
  }

  void gridToWorld(int ix, int iy, double& wx, double& wy) const {
    wx = origin_x_ + (ix + 0.5) * resolution_;
    wy = origin_y_ + (iy + 0.5) * resolution_;
  }

  bool inBounds(int ix, int iy) const {
    return ix >= 0 && ix < width_ && iy >= 0 && iy < height_;
  }

  /** @brief Compute BFS distance transform (in cells) from all occupied cells.
   *  Returns a float vector of distances in meters, truncated at max_dist_m. */
  std::vector<float> computeDistanceField(double max_dist_m) const {
    const size_t n = static_cast<size_t>(width_) * height_;
    const float INF = static_cast<float>(max_dist_m);
    std::vector<float> dist(n, INF);

    // BFS from all occupied cells
    struct Cell { int x, y; };
    std::queue<Cell> q;
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        if (occupied_[y * width_ + x]) {
          dist[y * width_ + x] = 0.0f;
          q.push({x, y});
        }
      }
    }

    // 8-connected BFS with Euclidean distance
    const int dx8[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy8[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const float dd8[] = {1.414f, 1.0f, 1.414f, 1.0f, 1.0f, 1.414f, 1.0f, 1.414f};

    while (!q.empty()) {
      auto [cx, cy] = q.front();
      q.pop();
      float cd = dist[cy * width_ + cx];

      for (int i = 0; i < 8; ++i) {
        int nx = cx + dx8[i], ny = cy + dy8[i];
        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;
        float nd = cd + dd8[i] * static_cast<float>(resolution_);
        if (nd >= max_dist_m) continue;
        size_t nidx = ny * width_ + nx;
        if (nd < dist[nidx]) {
          dist[nidx] = nd;
          q.push({nx, ny});
        }
      }
    }
    return dist;
  }

  int width() const { return width_; }
  int height() const { return height_; }
  double originX() const { return origin_x_; }
  double originY() const { return origin_y_; }
  double resolution() const { return resolution_; }
  double invResolution() const { return inv_resolution_; }
  const std::vector<bool>& occupiedData() const { return occupied_; }
  const std::vector<bool>& unknownData() const { return unknown_; }

 private:
  double origin_x_ = 0.0, origin_y_ = 0.0;
  double resolution_ = 0.2;
  double inv_resolution_ = 5.0;
  int width_ = 0, height_ = 0;
  std::vector<bool> occupied_;
  std::vector<bool> unknown_;
};
