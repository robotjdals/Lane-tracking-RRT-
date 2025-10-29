#ifndef min_22_pkg_RRT_STAR_HPP_
#define min_22_pkg_RRT_STAR_HPP_

#include "qnode.hpp"

#include <vector>
#include <random>
#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>


class RRT_star {
 public:
  struct Node {
    double x, y;
    int parent_idx;
    double cost;

    Node(double x_ = 0, double y_ = 0, int parent_idx_ = -1, double cost_ = 0)
      : x(x_), y(y_), parent_idx(parent_idx_), cost(cost_) {}
  };

  enum SamplingBias {
    CENTER,    // 중앙
    LEFT,      // 왼쪽 편향
    RIGHT      // 오른쪽 편향
  };

  void setSamplingBias(SamplingBias bias) {
    sampling_bias = bias;
  }

  RRT_star();
  ~RRT_star()  = default;

  std::vector<int> planPath(cv::Point2f start, cv::Point2f goal, int max_iterations = 1000);
  //std::vector<cv::Point2f> getWaypoint_Path(const std::vector<cv::Point2f>& path, double step_size = 0.1);

  void notifyVisualizationUpdate() {
    if (visualization_callback) visualization_callback();
  }

  void updateMap(const sensor_msgs::msg::LaserScan::SharedPtr scan, double robot_x = 0, double robot_y = 0, double robot_theta = 0);
  bool isCollisionFree(int grid_x, int grid_y) const;
  cv::Mat getgridmap() const;
  cv::Mat getVisualizationMap() const;
/*
  void setVisualizationCallback(std::function<void()> callback) {
    visualization_callback = callback;
  }
*/
  cv::Point2i gridToPixel(int grid_x, int grid_y) const;
  cv::Point2i pixelToGrid(int pixel_x, int pixel_y) const;
  cv::Point2i worldToGrid(double world_x, double world_y)const;
  std::vector<int> worldToPixel(const std::vector<cv::Point2f>& world_path);
  cv::Point2f gridToWorld(int grid_x, int grid_y) const;

  void setGoal(cv::Point2f goal) {
    saved_goal = goal;
    goal_set = true;
  }

  cv::Point2f getGoal() const { return saved_goal; }
  bool hasGoal() const { return goal_set; }

  void setCurrentPixelPath(const std::vector<int>& path) {
    current_pixel_path = path;
  }

  std::vector<int> getCurrentPixelPath() const {
    return current_pixel_path;
  }

  static constexpr double pixel_to_meter = -0.0018;
  static constexpr int Raw_X = 640;
  static constexpr int Raw_Y = 360;

  void setLanePoints(const std::vector<cv::Point2f>& left_points, const std::vector<cv::Point2f>& right_points) {
    left_lane_points = left_points;
    right_lane_points = right_points;

    //left_lane_available = !left_points.empty();
    //right_lane_available = !right_points.empty();
    lane_data_available = !left_points.empty() && !right_points.empty();
  }

  void clearLanePoints() {

    left_lane_points.clear();
    right_lane_points.clear();

    //left_lane_available = false;
    //right_lane_available = false;
    lane_data_available = false;
  }

 private:
  int addNode(double x, double y, int parent_idx = -1, double cost = 0);
  int findNearestNode(double x, double y) const;
  std::vector<int> findNearNodes(double x, double y, double radius) const;
  bool isPathCollisionFree(double x1, double y1, double x2, double y2) const;
  double distance(double x1, double y1, double x2, double y2) const;
  std::vector<cv::Point2f> reconstructPath(int goal_idx) const;
  void rewireTree(int new_idx, const std::vector<int>& near_indices);
  void clearTree();
  bool isWithinLane(double world_x, double world_y) const;

  std::vector<Node> nodes;
  cv::Mat gridmap;
  mutable std::mutex map_mutex;
  std::function<void()> visualization_callback;
  std::mt19937 rng_;
  cv::Point2f saved_goal;
  bool goal_set = false;
  std::vector<int> current_pixel_path;

  SamplingBias sampling_bias = CENTER;

  std::uniform_real_distribution<double> x_dist;
  std::uniform_real_distribution<double> y_dist;
  std::uniform_real_distribution<double> x_dist_left;
  std::uniform_real_distribution<double> x_dist_right;

  // 차선 정보 저장
  //std::atomic<bool> left_lane_available{false};
  //std::atomic<bool> right_lane_available{false};
  std::atomic<bool> lane_data_available{false};
  std::vector<cv::Point2f> left_lane_points;   // 왼쪽 차선 포인트들
  std::vector<cv::Point2f> right_lane_points;  // 오른쪽 차선 포인트들
  //bool lane_data_available = false;
  //bool left_lane_available = false;
  //bool right_lane_available = false;

  static constexpr double step_size = 0.09;
  static constexpr double goal_threshold = 0.15;
  static constexpr double resolution = 0.003;
  static constexpr int width = 300;
  static constexpr int height = 300;
  static constexpr int robot_radius_pixels = 10;

};
#endif
