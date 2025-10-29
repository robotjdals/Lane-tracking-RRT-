#include "../include/min_22_pkg/driving.hpp"
#include "../include/min_22_pkg/main_window.hpp"
#include "../include/min_22_pkg/rrt_star.hpp"

#include <cmath>

Driving::Driving(QObject* parent) : QObject(parent) {

  qnode = nullptr;
  current_speed = 0.0;
  has_avoidance_goal = false;
}

void Driving::setQNode(QNode* qnode_ptr) {
    qnode = qnode_ptr;
}

void Driving::setMainWindow(MainWindow* main_window_ptr) {
  main_window = main_window_ptr;
}
void Driving::setRRTPlanner(RRT_star* planner) {
  rrt_planner = planner;
}

void Driving::go(const std::vector<int>& waypoints){
  if(qnode && qnode->speed_received){
    current_speed = qnode->current_linear_x;
  }

  switch(state){
    case LANE_TRACKING:
    {
      static int obstacle_check_counter = 0;
      obstacle_check_counter = (obstacle_check_counter + 1) % 40;  // 오버플로우 방지

      if(obstacle_check_counter == 0 && qnode && qnode->detectObstacle(0.2, 1.8)){
        std::cout << "Obstacle detected! Planning static RRT path..." << std::endl;
        state = AVOIDANCE;
        has_avoidance_goal = false;  // 새 회피 시작 시 리셋
        avoidanceMode();
      }
      else{
        tracking(waypoints);
      }
    }
    break;

    case AVOIDANCE:
    {
      static int replan_counter = 0;
      replan_counter++;

      // 목표에 도달했는지 확인
      bool goal_reached = false;
      if(has_avoidance_goal) {
        // 로봇은 (0,0)에서 시작하므로 목표까지의 거리 계산
        double dist_to_goal = sqrt(saved_avoidance_goal.x * saved_avoidance_goal.x + saved_avoidance_goal.y * saved_avoidance_goal.y);

        // 목표에 0.3m 이내 도달하면 성공으로 간주
        if(dist_to_goal < 0.3) {
          goal_reached = true;
          std::cout << "Goal reached! Distance: " << dist_to_goal << "m" << std::endl;
        }
      }

      // 경로가 없거나 완료되었을 때만 재계획
      bool path_empty = rrt_waypoints.empty();
      bool path_completed = current_path_index >= rrt_waypoints.size();

      // 장애물이 사라졌거나 목표에 도달했으면 복귀
      bool obstacle_cleared = !(qnode && qnode->detectObstacle(0.2, 1.8));

      if(goal_reached || obstacle_cleared) {
        std::cout << (goal_reached ? "Goal reached!" : "Obstacle cleared!") << std::endl;
        state = RETURN_LANE;
        current_path_index = 0;
        rrt_waypoints.clear();
        has_avoidance_goal = false;  // 목표점 리셋
        break;
      }

      // 경로 재계획이 필요한 경우
      if(path_empty || path_completed || (replan_counter % 70 == 0)) {
        std::cout << "Re-planning path (goal=" << (has_avoidance_goal ? "KEEP" : "NEW") << ")..." << std::endl;
        avoidanceMode();  // 목표점 유지하면서 경로만 재계획
      }

      if(!rrt_waypoints.empty()) {
        bool completed = followRRTPath();
        if(completed) {
          std::cout << "Path segment completed, replanning..." << std::endl;
          current_path_index = 0;
          rrt_waypoints.clear();
        }
      }
      else {
        std::cout << "No path available in AVOIDANCE" << std::endl;
        drive(0.03, 0.0);  // 천천히 전진
      }
    }
    break;
/*
    case AVOIDANCE:
    {
      static int replan_counter = 0;
      replan_counter++;

      // 경로가 없거나, 완료되었거나, 30프레임(1.5초)마다 재계획
      bool need_replan = rrt_waypoints.empty() ||current_path_index >= rrt_waypoints.size() || (replan_counter % 30 == 0);

      if(need_replan) {
        std::cout << "Re-planning RRT path..." << std::endl;

        // 여전히 장애물이 있으면 재계획
        if(qnode && qnode->detectObstacle(0.2, 1.8)) {
          avoidanceMode();
        }
        else {
          // 장애물 사라짐 → 차선 복귀
          std::cout << "Obstacle cleared! Returning to lane" << std::endl;
          state = RETURN_LANE;
          current_path_index = 0;
          rrt_waypoints.clear();
          break;
        }
      }

      if(!rrt_waypoints.empty()) {
        bool completed = followRRTPath();

        // 짧은 경로가 완료되면 재계획 (상태는 유지)
        if(completed) {
          std::cout << "Short path completed, will replan next frame" << std::endl;
          current_path_index = 0;
          rrt_waypoints.clear();
        }
      }
      else {
        std::cout << "No path available in AVOIDANCE" << std::endl;
        state = RETURN_LANE;
      }
        }

    break;*/

      /*
      std::cout << "AVOIDANCE mode - RRT waypoints size: ";
      if(!rrt_waypoints.empty()) {
        std::cout << "Current waypoint index: " << current_path_index  << ", Target: " << rrt_waypoints[current_path_index] << std::endl;
        bool path_completed = followRRTPath();

        if(path_completed) {
          std::cout << "Static RRT path completed, returning to lane tracking" << std::endl;
          state = RETURN_LANE;
          current_path_index = 0;
          rrt_waypoints.clear();
        }
      }
      else {
        std::cout << "No RRT path available, emergency mode" << std::endl;
        drive(0.02, 0.0);

        state = RETURN_LANE;
      }*/


    case RETURN_LANE:
    {
      bool lanes_detected = (main_window->left_detected || main_window->right_detected);

      if(waypoints.size() > 5 && lanes_detected){
        std::cout << "Lane recovered, returning to normal tracking" << std::endl;
        state = LANE_TRACKING;
        current_path_index = 0;
        rrt_waypoints.clear();
        tracking(waypoints);
      }
      else{
        std::cout << "Waiting for lane recovery... waypoints: " << waypoints.size() << ", lanes: " << lanes_detected << std::endl;
        drive(0.05, 0.0);  // 천천히 직진하며 차선 찾기
      }
    }
    break;
  }

}

void Driving::tracking(const std::vector<int>& waypoints){

  int target_idx = std::min(4, (int)waypoints.size() - 1);
  int target_waypoint = waypoints[target_idx];

  if(waypoints.empty()) return;

  int deviation = abs(target_waypoint - 320);
  /*
  std::cout << "=== DEBUG INFO ===" << std::endl;
  std::cout << "Target waypoint: " << target_waypoint << std::endl;
  std::cout << "Center (320): " << 320 << std::endl;
  std::cout << "Deviation: " << deviation << std::endl;
  */

  if(deviation < 10) {

    double target_speed = std::min(0.10, current_speed+0.002);  // ìµœëŒ€ 15cm/s
    //std::cout << "Going STRAIGHT, speed: " << target_speed << std::endl;
    drive(target_speed, 0.0);

  }
  else {

    double normalized_deviation = std::min(deviation / 320.0, 1.0);

    double curve_factor = 1.0 - 0.3 * normalized_deviation * normalized_deviation;
    double target_speed = std::max(0.05, current_speed * curve_factor);
    double L = Look_aheadDistance(target_speed);
    double R = R_track(L, target_waypoint);
    double w = angular_velocity(R, target_speed);
    /*
    std::cout << "Look ahead distance L: " << L << std::endl;
    std::cout << "Radius R: " << R << std::endl;
    std::cout << "Angular velocity w: " << w << std::endl;
    std::cout << "Linear speed: " << target_speed << std::endl;
    */

    drive(target_speed, w);
  }
  //std::cout << "==================" << std::endl;
}

void Driving::rrt_tracking(const std::vector<int>& waypoints){
  std::cout << "=== RRT TRACKING DEBUG ===" << std::endl;
  std::cout << "RRT Waypoints size: " << waypoints.size() << std::endl;
  std::cout << "Current RRT path index: " << current_path_index << std::endl;
  std::cout << "Total RRT waypoints: " << rrt_waypoints.size() << std::endl;

  std::cout << "RRT Waypoint values: ";
  for(size_t i = 0; i < waypoints.size(); i++) {
    std::cout << waypoints[i] << " ";
  }
  std::cout << std::endl;

  int target_idx = std::min(2, (int)waypoints.size() - 1);
  int target_waypoint = waypoints[target_idx];

  if(waypoints.empty()) return;

  std::cout << "RRT Target index: " << target_idx << std::endl;
  std::cout << "RRT Target waypoint: " << target_waypoint << std::endl;

  int deviation = abs(target_waypoint - 320);
  std::cout << "RRT Deviation: " << deviation << std::endl;

  if(deviation < 15) {

    double target_speed = std::min(0.03, current_speed+0.001);
    std::cout << "Going STRAIGHT, speed: " << target_speed << std::endl;
    drive(target_speed, 0.0);

  }
  else {
    double normalized_deviation = std::min(deviation / 320.0, 1.0);

    double curve_factor = 1.0 - 0.5 * normalized_deviation * normalized_deviation;
    double target_speed = std::max(0.02, current_speed * curve_factor);

    double L = Look_aheadDistance(target_speed) * 0.7;
    double R = R_track(L, target_waypoint);
    double w = angular_velocity(R, target_speed);

    //w = std::max(-2.0, std::min(2.0, w));

    //w = std::max(-2.0, std::min(2.0, w));
    std::cout << "RRT Action: CURVE" << std::endl;
    std::cout << "RRT L: " << L << ", R: " << R << ", w: " << w << ", speed: " << target_speed << std::endl;
    drive(target_speed, w);
  }
  std::cout << "==================" << std::endl;
}


double Driving::angular_velocity(double R, double v){
  if(abs(R) > 100.0) return 0.0;

  double w = v / R;
  return std::max(-w_lim, std::min(w_lim, w));

}

double Driving::R_track(double L, int x){

  double y = (x - 320)* pixel_to_meter;
  /*
  std::cout << "R_track debug (reversed):" << std::endl;
  std::cout << "  x: " << x << std::endl;
  std::cout << "  (x-320): " << (x-320) << std::endl;
  std::cout << "  y: " << y << std::endl;
  */

  if(abs(y) < 0.001) {
    return 1000.0;
  }
  double R = (L * L) / (2 * y);
  //std::cout << "  Calculated R: " << R << std::endl;
  if(abs(R) > 100.0) {
    return (R > 0) ? 100.0 : -100.0;
  }

  return R;
}

double Driving::Look_aheadDistance(double v){
  const double MIN_L = 0.16;  // 0.1m
  const double MAX_L = 0.48;  // 0.5m

  double L = 2 * v / w_lim;
  return std::max(MIN_L, std::min(MAX_L, L));
}

void Driving::drive(double linear_x, double angular_z){
  if(qnode)
    qnode->drive(linear_x, angular_z);
}

void Driving::avoidanceMode() {
  if(!rrt_planner) {
    std::cout << "RRT planner is null!" << std::endl;
    return;
  }

  cv::Point2f goal;

  // 목표점이 이미 설정되어 있으면 재사용
  if(has_avoidance_goal) {
    goal = saved_avoidance_goal;
    std::cout << "=== REUSING GOAL: (" << goal.x << ", " << goal.y << ") ===" << std::endl;
  }
  else {
    // 새 목표점 결정
    std::cout << "=== SETTING NEW GOAL ===" << std::endl;

    bool obstacle_left = qnode->detectObstacleInSector(0.2, 1.0, 0.25);
    bool obstacle_right = qnode->detectObstacleInSector(0.2, -1.0, 0.25);
    bool obstacle_center = qnode->detectObstacleInSector(0.6, 0.0, 1.5);
    bool over_obstacle_left = qnode->detectObstacleInSector(0.1, 0.785, 0.2);
    bool over_obstacle_right = qnode->detectObstacleInSector(0.1, -0.785, 0.2);


    std::cout << "Obstacles - Left: " << obstacle_left
              << ", Right: " << obstacle_right
              << ", Center: " << obstacle_center
              << std::endl;

    if(obstacle_center) {
      // 정면 괜찮을 때
      if(!main_window->left_detected && main_window->right_detected) {
          goal = cv::Point2f(0.2, 0.4);
          std::cout << "-> LEFT" << std::endl;
        }
      else if(main_window->left_detected && !main_window->right_detected) {
        goal = cv::Point2f(-0.2, 0.4);
        std::cout << "<- RIGHT" << std::endl;
        }
      else {
        goal = cv::Point2f(0.0, 0.3);
      }
      std::cout << "^ CENTER (sides clear)" << std::endl;
    }
    else{

    }


    /*
    if(obstacle_center) {
    // 정면에 장애물 있을 때
      if(obstacle_left && !obstacle_right) {
        // 왼쪽도 막힘 -> 오른쪽으로
        goal = cv::Point2f(0.15, 0.6);
        std::cout << "-> LEFT" << std::endl;
      }
      else if(obstacle_right && !obstacle_left) {
        // 오른쪽도 막힘 -> 왼쪽으로
        goal = cv::Point2f(-0.15, 0.6);
        std::cout << "<- RIGHT" << std::endl;
      }
      else if(obstacle_left && obstacle_right) {
        // 양쪽 다 막힘 -> 천천히 직진
        goal = cv::Point2f(0.0, 0.2);
        std::cout << "^ SLOW FORWARD (all blocked)" << std::endl;
      }
      else {
        // 정면만 막힘, 양옆 괜찮음 -> 차선 정보 활용
        if(main_window->left_detected && !main_window->right_detected) {
          goal = cv::Point2f(0.0, 0.4);
        }
        else if(main_window->right_detected && !main_window->left_detected) {
          goal = cv::Point2f(0.0, 0.4);
        }
        else {
          goal = cv::Point2f(0.0, 0.6);
        }
        std::cout << "^ CENTER (sides clear)" << std::endl;
      }
    }
    else {

      // 정면 괜찮을 때
      if(over_obstacle_left && !obstacle_right) {
        goal = cv::Point2f(0.05, 0.15);
        std::cout << "-> SLIGHT RIGHT" << std::endl;
      }
      else if(over_obstacle_right && !obstacle_left) {
        goal = cv::Point2f(-0.05, 0.15);
        std::cout << "<- SLIGHT LEFT" << std::endl;
      }
      else {
        goal = cv::Point2f(0.0, 0.8);
        std::cout << "^ FORWARD" << std::endl;
      }
    }*/

    saved_avoidance_goal = goal;
    has_avoidance_goal = true;
    std::cout << "🎯 FINAL GOAL: (" << goal.x << ", " << goal.y << ")" << std::endl;
  }

  // 맵 업데이트
  if(qnode->lidar_received && !qnode->lidar_ranges.empty()) {
    auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    scan_msg->ranges = qnode->lidar_ranges;
    scan_msg->angle_min = qnode->lidar_angle_min;
    scan_msg->angle_max = qnode->lidar_angle_max;
    scan_msg->angle_increment = qnode->lidar_angle_increment;
    scan_msg->range_min = 0.1;
    scan_msg->range_max = 10.0;
    rrt_planner->updateMap(scan_msg, 0.0, 0.0, 0.0);
  }

  cv::Point2f start(0.0, 0.0);

  // ⚠️ 다시 한 번 검증
  if(goal.x == 0.0 && goal.y == 0.0) {
    std::cout << "CRITICAL ERROR: Goal is still (0,0)!" << std::endl;
    return;
  }

  std::cout << "Planning path from (0,0) to (" << goal.x << "," << goal.y << ")" << std::endl;

  std::vector<int> new_path = rrt_planner->planPath(start, goal, 500);

  if(!new_path.empty()) {
    rrt_waypoints = new_path;
    current_path_index = 0;
    std::cout << "✓ Path created: " << rrt_waypoints.size() << " waypoints" << std::endl;
  }
  else {
    std::cout << "✗ Path planning FAILED!" << std::endl;
    current_path_index = 0;
  }

  std::cout << "================================" << std::endl;
}
/*
bool Driving::followRRTPath() {
  if(rrt_waypoints.empty() || current_path_index >= rrt_waypoints.size()) {
    return true;
  }

  std::vector<int> current_waypoint = {rrt_waypoints[current_path_index]};
  rrt_tracking(current_waypoint);

  // 픽셀 편차와 물리적 거리 기반 판단
  int deviation = abs(rrt_waypoints[current_path_index] - 320);

  // Look-ahead: 다음 waypoint가 있으면 그쪽으로 향하는지 확인
  bool should_advance = false;

  if(deviation < 80) {  // 50픽셀 이내면 통과로 간주
    should_advance = true;
  }

  // 또는 이미 지나쳤는지 확인 (다음 waypoint가 더 가까우면)
  if(current_path_index < rrt_waypoints.size() - 1) {
    int next_deviation = abs(rrt_waypoints[current_path_index + 1] - 320);
    if(next_deviation < deviation) {
      should_advance = true;  // 이미 지나침
    }
  }

  if(should_advance) {
    current_path_index++;
  }

  return (current_path_index >= rrt_waypoints.size());
}*/
bool Driving::followRRTPath() {
  if(rrt_waypoints.empty() || current_path_index >= rrt_waypoints.size()) {
    return true;
  }

  // ⭐ 여러 waypoint의 평균 사용
  int look_ahead = std::min(3, (int)(rrt_waypoints.size() - current_path_index));
  int sum = 0;
  for(int i = 0; i < look_ahead; i++) {
    sum += rrt_waypoints[current_path_index + i];
  }
  int avg_waypoint = sum / look_ahead;

  std::vector<int> smoothed_waypoint = {avg_waypoint};
  rrt_tracking(smoothed_waypoint);

  // ⭐ 진행 조건 완화
  int deviation = abs(rrt_waypoints[current_path_index] - 320);
  if(deviation < 120) {  // 기존 80 → 120으로 완화
    current_path_index++;
  }

  return (current_path_index >= rrt_waypoints.size());
}
