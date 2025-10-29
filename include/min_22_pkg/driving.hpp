#ifndef DRIVING_HPP
#define DRIVING_HPP

#include <QObject>
#include "qnode.hpp"

class MainWindow;
class RRT_star;

class Driving : public QObject {
  Q_OBJECT

 public:
  Driving(QObject* parent = nullptr);
  ~Driving() = default;

  enum Current_state{
    LANE_TRACKING,
    AVOIDANCE,
    RETURN_LANE
  };

  void setQNode(QNode* qnode_ptr);
  void setMainWindow(MainWindow* main_window_ptr);
  void setRRTPlanner(RRT_star* planner);

  static constexpr double pixel_to_meter = -0.0018;
  static constexpr double w_lim = 1.8;
  static constexpr double b = 0.16;
  double current_speed;

  std::vector<int>rrt_waypoints;
  cv::Point2f start, goal;
  int current_path_index = 0;
  Current_state state = LANE_TRACKING;
  cv::Point2f saved_avoidance_goal;
  bool has_avoidance_goal;

 public Q_SLOTS:
  void go(const std::vector<int>& waypoints);
  void tracking(const std::vector<int>& waypoints);
  void rrt_tracking(const std::vector<int>& waypoints);
  double angular_velocity(double R, double v);
  double R_track(double L, int x);
  double Look_aheadDistance(double v);
  void drive(double linear_x, double angular_z);
  void avoidanceMode();
  bool followRRTPath();

 private:
  QNode* qnode;
  MainWindow* main_window;
  RRT_star* rrt_planner = nullptr;
  std::vector<cv::Point2f> rrt_path;

};

#endif
