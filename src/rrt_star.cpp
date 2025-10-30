#include "../include/min_22_pkg/rrt_star.hpp"

RRT_star::RRT_star() : rng_(std::random_device{}()) {

  x_dist = std::uniform_real_distribution<double>(-0.5, 0.5);  // ì¢Œìš° 1.5m
  y_dist = std::uniform_real_distribution<double>(0.05, 1.5);   // ì•žìª½ 3m

  // ì™¼ìª½ íŽ¸í–¥ ë¶„í¬
  //x_dist_left = std::uniform_real_distribution<double>(-1.0, -0.05);  // ì™¼ìª½ì— ë” ë§Žì€ ìƒ˜í”Œ

  // ì˜¤ë¥¸ìª½ íŽ¸í–¥ ë¶„í¬
  //x_dist_right = std::uniform_real_distribution<double>(0.05, 1.0);  // ì˜¤ë¥¸ìª½ì— ë” ë§Žì€ ìƒ˜í”Œ

  visualization_callback = nullptr;
  nodes.reserve(2000);
  sampling_bias = CENTER;
  lane_data_available = false;
  //std::cout << "RRT* grid mapper initialized" << std::endl;
}
//ê·¸ë¦¬ë“œ ë§µ
void RRT_star::updateMap(const sensor_msgs::msg::LaserScan::SharedPtr scan, double robot_x, double robot_y, double robot_theta){
  std::lock_guard<std::mutex> lock(map_mutex);


  gridmap = cv::Mat::zeros(height, width, CV_8UC1);

  if (!scan || scan->ranges.empty()) {
    //std::cout << "Invalid scan data, returning" << std::endl;
    return;
  }

  // ì—¬ëŸ¬ ë²ˆ ê²€ì¦ì„ í†µí•œ ì•ˆì •ì ì¸ ë°ì´í„°ë§Œ ì‚¬ìš©
  std::vector<bool> valid_points(scan->ranges.size(), false);

  for (size_t i = 1; i < scan->ranges.size()-1; i++) {
    double range = scan->ranges[i];

    if (range > 0.05 && range < 0.4 && !std::isinf(range) && !std::isnan(range)) {
      // 3ì  ì—°ì† ê²€ì¦
      float prev = scan->ranges[i-1];
      float next = scan->ranges[i+1];

      // ëª¨ë“  ê°’ì´ ìœ íš¨í•˜ê³  ì—°ì†ì„±ì´ ìžˆì„ ë•Œë§Œ ì‚¬ìš©
      if (prev > 0.05 && next > 0.05 && !std::isinf(prev) && !std::isinf(next)) {
        // ê¸‰ê²©í•œ ë³€í™”ê°€ ì—†ìœ¼ë©´ ìœ íš¨í•œ ì ìœ¼ë¡œ íŒë‹¨
        if (abs(range - prev) < 1.0 && abs(range - next) < 1.0) {
          valid_points[i] = true;
        }
      }
    }
  }
  // ê²€ì¦ëœ í¬ì¸íŠ¸ë§Œ ë§µì— í‘œì‹œ
  for (size_t i = 0; i < scan->ranges.size(); i++) {
    if (!valid_points[i]) continue;
    double range = scan->ranges[i];
    if (range < 0.1 || range > 2.0 || std::isinf(range) || std::isnan(range)) {
      continue;
    }

    double angle = scan->angle_min + i * scan->angle_increment + robot_theta - M_PI/2;

    //ì›”ë“œ ì¢Œí‘œë¡œ ë³€í™˜
    double world_x = robot_x + range * cos(angle);
    double world_y = robot_y + range * sin(angle);
    //ê·¸ë¦¬ë“œ ì¢Œí‘œë¡œ ë³€í™˜
   // worldToGridì˜ ë°˜í™˜ê°’ì„ grid_pos ë³€ìˆ˜ì— ì €ìž¥
    cv::Point2i grid_pos = worldToGrid(world_x, -(world_y));

    // grid_posë¥¼ ì‚¬ìš©
    if (grid_pos.x >= 0 && grid_pos.x < width &&
        grid_pos.y >= 0 && grid_pos.y < height) {
        cv::circle(gridmap, grid_pos, 15, 255, -1);
    }

    if(lane_data_available) {
      for(size_t i = 0; i < left_lane_points.size() - 1; i++) {
        cv::Point2i p1 = worldToGrid((left_lane_points[i].x*0.8), (left_lane_points[i].y)+0.5);
        cv::Point2i p2 = worldToGrid((left_lane_points[i+1].x*0.8), (left_lane_points[i+1].y)+0.5);

        if(p1.x >= 0 && p1.x < width && p1.y >= 0 && p1.y < height &&
          p2.x >= 0 && p2.x < width && p2.y >= 0 && p2.y < height) {
          cv::line(gridmap, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), 255, 3);
          cv::circle(gridmap, p1, 3, 255, -1);
        }
      }

      for(size_t i = 0; i < right_lane_points.size() - 1; i++) {
        cv::Point2i p1 = worldToGrid((right_lane_points[i].x*0.8), (right_lane_points[i].y)+0.5);
        cv::Point2i p2 = worldToGrid((right_lane_points[i+1].x*0.8), (right_lane_points[i+1].y)+0.5);

        if(p1.x >= 0 && p1.x < width && p1.y >= 0 && p1.y < height &&
          p2.x >= 0 && p2.x < width && p2.y >= 0 && p2.y < height) {
          cv::line(gridmap, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), 255, 3);
          cv::circle(gridmap, p1, 3, 255, -1);
        }
      }

      //cv::putText(gridmap, "Yellow: Left Lane", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, 255, 1);
      //cv::putText(gridmap, "Cyan: Right Lane", cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, 255, 1);
    }
  }

  notifyVisualizationUpdate();
}

// ì¶©ëŒê²€ì‚¬
bool RRT_star::isCollisionFree(int grid_x, int grid_y) const {
  std::lock_guard<std::mutex> lock(map_mutex);

  // ê²½ê³„ ì²´í¬
  if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height) {
    return false;
  }

  if (gridmap.empty()) {
    return true;
  }

  // 1. ë¼ì´ë‹¤ ìž¥ì• ë¬¼ ì²´í¬
  if(gridmap.at<uint8_t>(grid_y, grid_x) != 0) {
    return false;  // ìž¥ì• ë¬¼ ìžˆìŒ
  }


  // 2. ì°¨ì„  ì²´í¬ (ì°¨ì„  ì •ë³´ê°€ ìžˆì„ ë•Œë§Œ)
  if(lane_data_available) {
    cv::Point2f world_pos = gridToWorld(grid_x, grid_y);

    // ì°¨ì„  ë°–ì´ë©´ ì¶©ëŒë¡œ ê°„ì£¼
    if(!isWithinLane(world_pos.x, world_pos.y)) {
      return false;
    }
  }

  return true;  // ì¶©ëŒ ì—†ìŒ
}

cv::Mat RRT_star::getgridmap() const {
  std::lock_guard<std::mutex> lock(map_mutex);
  return gridmap.clone();
}

//ê·¸ë¦¬ë“œë§µ ì‹œê°í™”
cv::Mat RRT_star::getVisualizationMap() const {
  std::lock_guard<std::mutex> lock(map_mutex);

  cv::Mat vis_map = gridmap.clone();

  // ì»¬ëŸ¬ ë§µìœ¼ë¡œ ë³€í™˜
  cv::Mat color_map;
  cv::cvtColor(vis_map, color_map, cv::COLOR_GRAY2BGR);

  // ë¡œë´‡ ìœ„ì¹˜ (ë§µ ì¤‘ì•™)
  int robot_x = width/2;
  int robot_y = height/2;

  // ë¡œë´‡ì„ íŒŒëž€ìƒ‰ ì›ìœ¼ë¡œ í‘œì‹œ
  cv::circle(color_map, cv::Point(robot_x, robot_y), 8, cv::Scalar(255, 0, 0), -1);

  // ë¡œë´‡ ë°©í–¥ì„ í™”ì‚´í‘œë¡œ í‘œì‹œ (ìœ„ìª½ì´ ì „ë°©)
  cv::Point arrow_end(robot_x, robot_y + 15);
  cv::arrowedLine(color_map, cv::Point(robot_x, robot_y), arrow_end, cv::Scalar(0, 255, 0), 3, 8, 0, 0.3);

  /*
  if(lane_data_available) {
    for(size_t i = 0; i < left_lane_points.size() - 1; i++) {
      cv::Point2i p1 = worldToGrid((left_lane_points[i].x), (left_lane_points[i].y)+0.5);
      cv::Point2i p2 = worldToGrid((left_lane_points[i+1].x), (left_lane_points[i+1].y)+0.5);


      if(p1.x >= 0 && p1.x < width && p1.y >= 0 && p1.y < height &&
         p2.x >= 0 && p2.x < width && p2.y >= 0 && p2.y < height) {
        cv::line(color_map, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), cv::Scalar(0, 255, 255), 3);  // Ã«â€¦Â¸Ã«Å¾â‚¬Ã¬Æ’â€° (BGR)
        cv::circle(color_map, p1, 3, cv::Scalar(0, 255, 255), -1);
      }
    }

    for(size_t i = 0; i < right_lane_points.size() - 1; i++) {
      cv::Point2i p1 = worldToGrid((right_lane_points[i].x), (right_lane_points[i].y)+0.5);
      cv::Point2i p2 = worldToGrid((right_lane_points[i+1].x), (right_lane_points[i+1].y)+0.5);

      if(p1.x >= 0 && p1.x < width && p1.y >= 0 && p1.y < height &&
         p2.x >= 0 && p2.x < width && p2.y >= 0 && p2.y < height) {
        cv::line(color_map, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), cv::Scalar(255, 255, 0), 3);  // Ã­â€¢ËœÃ«Å ËœÃ¬Æ’â€° (BGR)
        cv::circle(color_map, p1, 3, cv::Scalar(255, 255, 0), -1);
      }
    }

    cv::putText(color_map, "Yellow: Left Lane", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
    cv::putText(color_map, "Cyan: Right Lane", cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
  }*/

  if(!nodes.empty()) {
    // ì˜µì…˜ 1: ëª¨ë“  ë…¸ë“œ í‘œì‹œ (íšŒìƒ‰, ì•„ì£¼ ìž‘ê²Œ)
    for(const auto& node : nodes) {
      cv::Point2i grid_pos = worldToGrid(-node.x, node.y);
      if(grid_pos.x >= 0 && grid_pos.x < width && grid_pos.y >= 0 && grid_pos.y < height) {
        cv::circle(color_map, cv::Point(grid_pos.x, grid_pos.y), 2, cv::Scalar(128, 128, 128), -1); // íšŒìƒ‰, ìž‘ê²Œ
      }
    }

    // ìµœì¢… ê²½ë¡œë§Œ ë¹¨ê°„ìƒ‰ìœ¼ë¡œ ê°•ì¡° í‘œì‹œ
    if(nodes.size() > 1) {
      // ë§ˆì§€ë§‰ ë…¸ë“œë¶€í„° ì‹œìž‘í•´ì„œ ë¶€ëª¨ë¥¼ ë”°ë¼ê°€ë©° ê²½ë¡œ í‘œì‹œ
      int current_idx = nodes.size() - 1;
      std::vector<cv::Point2i> final_path;

      while(current_idx != -1 && nodes[current_idx].parent_idx != -1) {
        cv::Point2i current_pos = worldToGrid(nodes[current_idx].x, nodes[current_idx].y);
        final_path.push_back(current_pos);
        current_idx = nodes[current_idx].parent_idx;
      }

      // ì‹œìž‘ì ë„ ì¶”ê°€
      if(current_idx != -1) {
        cv::Point2i start_pos = worldToGrid(nodes[current_idx].x, nodes[current_idx].y);
        final_path.push_back(start_pos);
      }

      // ìµœì¢… ê²½ë¡œ ì„ ê³¼ ì  ê·¸ë¦¬ê¸°
      for(size_t i = 1; i < final_path.size(); i++) {
        cv::Point2i prev = final_path[i-1];
        cv::Point2i curr = final_path[i];

        if(prev.x >= 0 && prev.x < width && prev.y >= 0 && prev.y < height &&
           curr.x >= 0 && curr.x < width && curr.y >= 0 && curr.y < height) {
          // ê²½ë¡œ ì„  ê·¸ë¦¬ê¸° (ë¹¨ê°„ìƒ‰, êµµê²Œ)
          cv::line(color_map, cv::Point(curr.x, curr.y), cv::Point(prev.x, prev.y),
                  cv::Scalar(0, 0, 255), 3);
          // ì›¨ì´í¬ì¸íŠ¸ ì  ê·¸ë¦¬ê¸° (ë¹¨ê°„ìƒ‰, í¬ê²Œ)
          cv::circle(color_map, cv::Point(prev.x, prev.y), 5, cv::Scalar(0, 0, 255), -1);
        }
      }
    }
  }


  // ì¢Œí‘œê³„ í‘œì‹œ (ê²©ìžì„ )
  for(int i = 0; i < width; i += 40) {
      cv::line(color_map, cv::Point(i, 0), cv::Point(i, height), cv::Scalar(100, 100, 100), 1);
  }
  for(int j = 0; j < height; j += 40) {
      cv::line(color_map, cv::Point(0, j), cv::Point(width, j), cv::Scalar(100, 100, 100), 1);
  }

  return color_map;
}

//rrt*
std::vector<int> RRT_star::planPath(cv::Point2f start, cv::Point2f goal, int max_iterations) {
  std::cout << "\n========== RRT* PLANNING STARTED ==========" << std::endl;
  std::cout << "ðŸŽ¯ Target: (" << goal.x << ", " << goal.y << ")" << std::endl;

  // ê·¸ë¦¬ë“œ ë§µ í™•ì¸
  if(gridmap.empty()) {
    std::cout << "âŒ FATAL: Grid map is EMPTY!" << std::endl;
    return {};
  }
  std::cout << "âœ“ Grid map: " << gridmap.rows << "x" << gridmap.cols << std::endl;

  // ë…¸ë“œ ì´ˆê¸°í™”
  nodes.clear();

  // ì¢Œí‘œ ë³€í™˜
  cv::Point2i start_grid = worldToGrid(start.x, start.y);
  cv::Point2i goal_grid = worldToGrid(goal.x, goal.y);

  std::cout << "Start: world(" << start.x << "," << start.y << ") grid("<< start_grid.x << "," << start_grid.y << ")" << std::endl;
  std::cout << "Goal: world(" << goal.x << "," << goal.y << ") grid("<< goal_grid.x << "," << goal_grid.y << ")" << std::endl;

  // ê²½ê³„ ì²´í¬ (ë¶€ë“œëŸ½ê²Œ)
  if(goal_grid.x < 5) goal_grid.x = 5;
  if(goal_grid.x >= width-5) goal_grid.x = width-6;
  if(goal_grid.y < 5) goal_grid.y = 5;
  if(goal_grid.y >= height-5) goal_grid.y = height-6;

  // ì›”ë“œ ì¢Œí‘œ ìž¬ê³„ì‚°
  cv::Point2f adjusted_goal = gridToWorld(goal_grid.x, goal_grid.y);
  goal.x = adjusted_goal.x;
  goal.y = adjusted_goal.y;
  std::cout << "Adjusted goal: (" << goal.x << ", " << goal.y << ")" << std::endl;

  // ì¶©ëŒ ì²´í¬
  if(!isCollisionFree(goal_grid.x, goal_grid.y)) {
    std::cout << "âš ï¸  Goal in collision, searching..." << std::endl;
    bool found = false;

    for(int r = 3; r <= 20 && !found; r += 3) {
      for(int dx = -r; dx <= r && !found; dx++) {
        for(int dy = -r; dy <= r && !found; dy++) {
          int nx = goal_grid.x + dx;
          int ny = goal_grid.y + dy;
          if(nx >= 5 && nx < width-5 && ny >= 5 && ny < height-5) {
            if(isCollisionFree(nx, ny)) {
              cv::Point2f safe = gridToWorld(nx, ny);
              goal.x = safe.x;
              goal.y = safe.y;
              goal_grid.x = nx;
              goal_grid.y = ny;
              found = true;
              std::cout << "âœ“ Safe goal: (" << goal.x << "," << goal.y << ")" << std::endl;
            }
          }
        }
      }
    }

    if(!found) {
      std::cout << "âŒ No safe goal found!" << std::endl;
      return {};
    }
  }

  // ì‹œìž‘ ë…¸ë“œ ì¶”ê°€
  int start_idx = addNode(start.x, start.y);
  std::cout << "âœ“ Tree initialized with start node" << std::endl;
  std::cout << "Parameters: threshold=" << goal_threshold<< "m, step=" << step_size << "m" << std::endl;

  // âœ“ ì°¨ì„  ì •ë³´ í™•ì¸
  std::cout << "ðŸ›£ï¸  Lane data available: " << (lane_data_available ? "YES" : "NO") << std::endl;
  if(lane_data_available) {
    std::cout << "   Left points: " << left_lane_points.size() << std::endl;
    std::cout << "   Right points: " << right_lane_points.size() << std::endl;
  }

  // âœ“ ìƒ˜í”Œë§ ë²”ìœ„ í™•ì¸
  std::cout << "ðŸ“ Sampling ranges:" << std::endl;
  std::cout << "   x: [" << x_dist.min() << ", " << x_dist.max() << "]" << std::endl;
  std::cout << "   y: [" << y_dist.min() << ", " << y_dist.max() << "]" << std::endl;

  std::cout << "Starting " << max_iterations << " iterations...\n" << std::endl;

  // í†µê³„
  int valid = 0, collisions = 0, lane_reject = 0;
  int sample_debug_count = 0;

  // RRT* ë©”ì¸ ë£¨í”„
  for(int iter = 0; iter < max_iterations; iter++) {
    // ìƒ˜í”Œë§
    cv::Point2f sample;
    if((rng_() % 100) < 30) {
        sample = goal;  // 30% ëª©í‘œ ì§€í–¥
    } else {
      sample.x = x_dist(rng_);
      sample.y = y_dist(rng_);
    }

    if(sample_debug_count < 10) {
      cv::Point2i sample_grid = worldToGrid(sample.x, sample.y);
      std::cout << "  Sample " << sample_debug_count << ": world("
                << sample.x << "," << sample.y << ") -> grid("
                << sample_grid.x << "," << sample_grid.y << ")";

      // ê·¸ë¦¬ë“œê°€ ë§µ ì•ˆì¸ì§€ ì²´í¬
      if(sample_grid.x < 0 || sample_grid.x >= width || sample_grid.y < 0 || sample_grid.y >= height) {
        std::cout << " âŒ OUT OF BOUNDS!" << std::endl;
      }else {
       std::cout << " âœ“ in bounds" << std::endl;
      }
      sample_debug_count++;
    }

    // ê°€ìž¥ ê°€ê¹Œìš´ ë…¸ë“œ
    int nearest_idx = findNearestNode(sample.x, sample.y);

    if(nearest_idx == -1) {
      std::cout << "  âŒ No nearest node!" << std::endl;
      continue;
    }
    //if(nearest_idx == -1) continue;

    const Node& nearest = nodes[nearest_idx];

    // Steering
    double d = distance(nearest.x, nearest.y, sample.x, sample.y);
    cv::Point2f new_pt;
    if(d > step_size) {
      double ratio = step_size / d;
      new_pt.x = nearest.x + ratio * (sample.x - nearest.x);
      new_pt.y = nearest.y + ratio * (sample.y - nearest.y);
    } else {
      new_pt = sample;
    }
/*
    double dx = new_pt.x - nearest.x;
    const double max_dx = 0.07;  // xì¶• ìµœëŒ€ ë³€í™”ëŸ‰ 5cm
    if(std::abs(dx) > max_dx) {
      dx = (dx > 0) ? max_dx : -max_dx;
      new_pt.x = nearest.x + dx;
    }*/

    if(iter < 10) {
      cv::Point2i new_grid = worldToGrid(new_pt.x, new_pt.y);
      std::cout << "  New pt " << iter << ": world("
                << new_pt.x << "," << new_pt.y << ") -> grid("
                << new_grid.x << "," << new_grid.y << ")";
    }

    // ì°¨ì„  ì²´í¬
    if(lane_data_available && !isWithinLane(new_pt.x, new_pt.y)) {
      if(iter < 10) std::cout << " âŒ LANE REJECT" << std::endl;
      lane_reject++;
      continue;
    }

    // ì¶©ëŒ ì²´í¬
    if(!isPathCollisionFree(nearest.x, nearest.y, new_pt.x, new_pt.y)) {
      if(iter < 10) std::cout << " âŒ COLLISION" << std::endl;
      collisions++;
      continue;
    }

    /*
    // ì°¨ì„  ì²´í¬
    if(lane_data_available && !isWithinLane(new_pt.x, new_pt.y)) {
        lane_reject++;
        continue;
    }

    // ì¶©ëŒ ì²´í¬
    if(!isPathCollisionFree(nearest.x, nearest.y, new_pt.x, new_pt.y)) {
        collisions++;
        continue;
    }*/
    if(iter < 10) std::cout << " âœ… VALID!" << std::endl;
    valid++;

    // ìµœì  ë¶€ëª¨ ì„ íƒ
    double radius = std::min(0.3, step_size * sqrt(log(nodes.size()+1)/(nodes.size()+1)));
    std::vector<int> near_nodes = findNearNodes(new_pt.x, new_pt.y, radius);

    int best_parent = nearest_idx;
    double best_cost = nearest.cost + distance(nearest.x, nearest.y, new_pt.x, new_pt.y);

    for(int n_idx : near_nodes) {
      const Node& n = nodes[n_idx];
      if(isPathCollisionFree(n.x, n.y, new_pt.x, new_pt.y)) {
        double cost = n.cost + distance(n.x, n.y, new_pt.x, new_pt.y);
        if(cost < best_cost) {
            best_parent = n_idx;
            best_cost = cost;
        }
      }
    }

    // ë…¸ë“œ ì¶”ê°€
    int new_idx = addNode(new_pt.x, new_pt.y, best_parent, best_cost);

    // Rewiring
    rewireTree(new_idx, near_nodes);

    // ëª©í‘œ ë„ë‹¬ ì²´í¬
    double goal_dist = distance(new_pt.x, new_pt.y, goal.x, goal.y);
    if(goal_dist < goal_threshold) {
      std::cout << "\nðŸŽ‰ SUCCESS! Goal reached!" << std::endl;
      std::cout << "â””â”€ Iteration: " << (iter+1) << "/" << max_iterations << std::endl;
      std::cout << "â””â”€ Nodes: " << nodes.size() << std::endl;
      std::cout << "â””â”€ Valid: " << valid << " | Collisions: " << collisions
                << " | Lane: " << lane_reject << std::endl;

      // ëª©í‘œ ë…¸ë“œ ì¶”ê°€
      double final_cost = nodes[new_idx].cost + goal_dist;
      int goal_idx = addNode(goal.x, goal.y, new_idx, final_cost);

      // ê²½ë¡œ ìž¬êµ¬ì„±
      std::vector<cv::Point2f> path = reconstructPath(goal_idx);
      std::cout << "â””â”€ Waypoints: " << path.size() << std::endl;

      // í”½ì…€ë¡œ ë³€í™˜
      std::vector<int> pixels = worldToPixel(path);
      std::cout << "â””â”€ Pixel waypoints: " << pixels.size() << std::endl;
      std::cout << "========================================\n" << std::endl;

      return pixels;
    }

    // ì§„í–‰ ìƒí™©
    if((iter+1) % 100 == 0) {
      std::cout << "  [" << (iter+1) << "] Nodes:" << nodes.size()
                << " | Valid:" << valid << " | Coll:" << collisions
                << " | Lane:" << lane_reject << std::endl;
    }
  }

  // ì‹¤íŒ¨
  std::cout << "\nâŒ FAILED after " << max_iterations << " iterations" << std::endl;
  std::cout << "â””â”€ Final nodes: " << nodes.size() << std::endl;
  std::cout << "â””â”€ Valid: " << valid << " | Collisions: " << collisions
            << " | Lane: " << lane_reject << std::endl;
  std::cout << "========================================\n" << std::endl;

  return {};
}

//std::vector<cv::Point2f> RRT_star::getWaypoint_Path(const std::vector<cv::Point2f>& path, double step_size);

//ë…¸ë“œìƒì„±
int RRT_star::addNode(double x, double y, int parent_idx, double cost){
  nodes.emplace_back(x, y, parent_idx, cost);
  return nodes.size()-1;
}
//ê°€ìž¥ ê°€ê¹Œì€ ë…¸ë“œ ì°¾ê¸°
int RRT_star::findNearestNode(double x, double y) const{
  if(nodes.empty()) return -1;

  int nearest_idx =0;
  double min_dist = distance(nodes[0].x, nodes[0].y, x, y);

  for(size_t  i = 1; i < nodes.size(); i++){
    double dist = distance(nodes[i].x, nodes[i].y, x, y);
    if(dist < min_dist){
      min_dist = dist;
      nearest_idx = i;
    }
  }
  return nearest_idx;
}
//ì£¼ë³€ ë…¸ë“œ íƒìƒ‰
std::vector<int> RRT_star::findNearNodes(double x, double y, double radius) const{

  std::vector<int> near_find;

  for(size_t  i = 0; i < nodes.size(); i++){
    if(distance(nodes[i].x, nodes[i].y, x, y) < radius){
      near_find.push_back(i);
    }
  }
  return near_find;
}
//ì¶©ëŒ ê²€ì‚¬
bool RRT_star::isPathCollisionFree(double x1, double y1, double x2, double y2) const {
  static int debug_call_count = 0;
  bool should_debug = (debug_call_count < 5);

  if(should_debug) {
    std::cout << "\n    ðŸ” Checking path (" << x1 << "," << y1 << ") -> (" << x2 << "," << y2 << ")" << std::endl;
  }

  // ì°¨ì„  ì²´í¬
  if(lane_data_available) {
    if(!isWithinLane(x1, y1)) {
      if(should_debug) {
          std::cout << "    âŒ Start point outside lane" << std::endl;
          debug_call_count++;
      }
      return false;
    }
    if(!isWithinLane(x2, y2)) {
      if(should_debug) {
        std::cout << "    âŒ End point outside lane" << std::endl;
        debug_call_count++;
      }
      return false;
    }
  }

  // ê²½ë¡œ ì„¸ë¶„í™”
  double path_length = distance(x1, y1, x2, y2);
  int steps = static_cast<int>(path_length / 0.02);
  steps = std::max(steps, 5);

  if(should_debug) {
    std::cout << "    Checking " << steps << " intermediate points..." << std::endl;
  }

  for(int i = 1; i < steps; i++) {
    double ratio = (double)i / steps;
    double x = x1 + ratio * (x2 - x1);
    double y = y1 + ratio * (y2 - y1);

    // ì°¨ì„  ì²´í¬
    if(lane_data_available && !isWithinLane(x, y)) {
      if(should_debug) {
        std::cout << "    âŒ Mid point (" << x << "," << y << ") outside lane" << std::endl;
        debug_call_count++;
      }
      return false;
    }

    // ê·¸ë¦¬ë“œ ë³€í™˜
    cv::Point2i grid = worldToGrid(x, y);

    // ê²½ê³„ ì²´í¬
    if(grid.x < 0 || grid.x >= width || grid.y < 0 || grid.y >= height) {
      if(should_debug) {
        std::cout << "    âŒ Grid (" << grid.x << "," << grid.y << ") out of bounds!" << std::endl;
        debug_call_count++;
      }
      return false;
    }

    // ì¶©ëŒ ì²´í¬ (ì¤‘ì‹¬ì ë§Œ)
    if(!isCollisionFree(grid.x, grid.y)) {
      if(should_debug) {
        std::cout << "    âŒ Grid (" << grid.x << "," << grid.y << ") has obstacle" << std::endl;
        debug_call_count++;
      }
      return false;
    }
  }

  if(should_debug) {
    std::cout << "    âœ… Path is FREE!" << std::endl;
    debug_call_count++;
  }

  return true;
}
/*
bool RRT_star::isWithinLane(double world_x, double world_y) const {
  if(!lane_data_available) return true;  // ì°¨ì„  ì •ë³´ ì—†ìœ¼ë©´ í†µê³¼

  // í•´ë‹¹ y ìœ„ì¹˜ì—ì„œ ì°¨ì„ ì˜ x ë²”ìœ„ ì°¾ê¸°
  double left_boundary = -0.3;   // ê¸°ë³¸ê°’
  double right_boundary = 0.3;

  // ì„ í˜• ë³´ê°„ìœ¼ë¡œ í•´ë‹¹ y ìœ„ì¹˜ì˜ ì°¨ì„  x ì¢Œí‘œ ì¶”ì •
  for(size_t i = 0; i < left_lane_points.size() - 1; i++) {
    if(world_y >= left_lane_points[i].y && world_y <= left_lane_points[i+1].y) {
      // ì„ í˜• ë³´ê°„
      double ratio = (world_y - left_lane_points[i].y) /(left_lane_points[i+1].y - left_lane_points[i].y);
      left_boundary = left_lane_points[i].x + ratio * (left_lane_points[i+1].x - left_lane_points[i].x);
      break;
    }
  }


  for(size_t i = 0; i < right_lane_points.size() - 1; i++) {
    if(world_y >= right_lane_points[i].y && world_y <= right_lane_points[i+1].y) {
      double ratio = (world_y - right_lane_points[i].y) / (right_lane_points[i+1].y - right_lane_points[i].y);
      right_boundary = right_lane_points[i].x + ratio * (right_lane_points[i+1].x - right_lane_points[i].x);
      break;
    }
  }

  // ì•ˆì „ ë§ˆì§„ ì¶”ê°€ (5cm)
  const double safety_margin = 0.05;
  left_boundary += safety_margin;
  right_boundary -= safety_margin;

  // ì°¨ì„  ê²½ê³„ë¥¼ ë²—ì–´ë‚¬ëŠ”ì§€ í™•ì¸
  return (world_x >= left_boundary && world_x <= right_boundary);
}*/

bool RRT_star::isWithinLane(double world_x, double world_y) const {

  if(!lane_data_available) return true;


  if(left_lane_points.empty() || right_lane_points.empty()) {
    return true;
  }


  double min_y_left = std::min(left_lane_points.front().y, left_lane_points.back().y);
  double max_y_left = std::max(left_lane_points.front().y, left_lane_points.back().y);
  double min_y_right = std::min(right_lane_points.front().y, right_lane_points.back().y);
  double max_y_right = std::max(right_lane_points.front().y, right_lane_points.back().y);

  double min_y = std::min(min_y_left, min_y_right);
  double max_y = std::max(max_y_left, max_y_right);


  if(world_y < min_y || world_y > max_y) {

    if(std::abs(world_y) < 0.1) {
        return true;
    }

    const double default_lane_width = 0.7;
    return (std::abs(world_x) < default_lane_width / 2.0);
  }


  double left_boundary = -0.2;
  bool found_left = false;

  for(size_t i = 0; i < left_lane_points.size() - 1; i++) {
    double y1 = left_lane_points[i].y;
    double y2 = left_lane_points[i+1].y;

    double y_min = std::min(y1, y2);
    double y_max = std::max(y1, y2);

    if(world_y >= y_min && world_y <= y_max) {
      double dy = y2 - y1;
      if(std::abs(dy) < 1e-6) {
          left_boundary = left_lane_points[i].x;
      } else {
          double ratio = (world_y - y1) / dy;
          left_boundary = left_lane_points[i].x + ratio * (left_lane_points[i+1].x - left_lane_points[i].x);
      }
      found_left = true;
      break;
    }
  }

  double right_boundary = 0.2;
  bool found_right = false;

  for(size_t i = 0; i < right_lane_points.size() - 1; i++) {
    double y1 = right_lane_points[i].y;
    double y2 = right_lane_points[i+1].y;

    double y_min = std::min(y1, y2);
    double y_max = std::max(y1, y2);

    if(world_y >= y_min && world_y <= y_max) {
      double dy = y2 - y1;
      if(std::abs(dy) < 1e-6) {
          right_boundary = right_lane_points[i].x;
      } else {
          double ratio = (world_y - y1) / dy;
          right_boundary = right_lane_points[i].x + ratio * (right_lane_points[i+1].x - right_lane_points[i].x);
      }
      found_right = true;
      break;
    }
  }

  if(!found_left && !found_right) {
    return true;
  }


  const double safety_margin = 0.01;
  left_boundary += safety_margin;
  right_boundary -= safety_margin;


  if(left_boundary > right_boundary) {
      std::swap(left_boundary, right_boundary);
  }

  bool result = (world_x >= left_boundary && world_x <= right_boundary);

  static int debug_count = 0;
  if(debug_count < 10) {
    std::cout << "Lane check: (" << world_x << "," << world_y
              << ") boundaries [" << left_boundary << "," << right_boundary
              << "] -> " << (result ? "OK" : "OUT") << std::endl;
    debug_count++;
  }

  return result;

  //return (world_x >= left_boundary && world_x <= right_boundary);
}


//ê±°ë¦¬ ì¸¡ì •
double RRT_star::distance(double x1, double y1, double x2, double y2) const{
  return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
}
//ê²½ë¡œ ìž¬êµ¬ì„±
std::vector<cv::Point2f> RRT_star::reconstructPath(int goal_idx) const{
  std::vector<cv::Point2f> path;

  int current_idx = goal_idx;
  while (current_idx != -1){
    const Node& node = nodes[current_idx];
    path.emplace_back(node.x, node.y);
    current_idx = node.parent_idx;
  }

  std::reverse(path.begin(), path.end());
  return path;
}

//íŠ¸ë¦¬ ìž¬ì—°ê²°
void RRT_star::rewireTree(int new_idx, const std::vector<int>& near_find){
  const Node& new_node = nodes[new_idx];

  for(int near_idx : near_find){
    if(near_idx == new_idx || near_idx == new_node.parent_idx) continue;

    Node& near_node = nodes[near_idx];

    double new_cost = new_node.cost + distance(new_node.x, new_node.y, near_node.x, near_node.y);

    if(new_cost < near_node.cost && isPathCollisionFree(new_node.x, new_node.y, near_node.x, near_node.y)){
      near_node.parent_idx = new_idx;
      near_node.cost = new_cost;
    }
  }
}

cv::Point2i RRT_star::worldToGrid(double world_x, double world_y) const {
  int grid_x = static_cast<int>(world_x / resolution + width/2);
  int grid_y = static_cast<int>(world_y / resolution + height/2);

  /*
  // âœ“ ë””ë²„ê·¸ (ì²˜ìŒ 10ë²ˆë§Œ)
  static int debug_count = 0;
  if(debug_count < 10) {
    std::cout << "      worldToGrid: (" << world_x << "," << world_y
              << ") -> raw(" << grid_x << "," << grid_y << ")";
    debug_count++;
  }*/

  // í´ëž¨í•‘
  grid_x = std::max(0, std::min(width-1, grid_x));
  grid_y = std::max(0, std::min(height-1, grid_y));

  /*
  if(debug_count <= 10) {
    std::cout << " -> clamped(" << grid_x << "," << grid_y << ")" << std::endl;
  }*/

  return cv::Point2i(grid_x, grid_y);
}

std::vector<int> RRT_star::worldToPixel(const std::vector<cv::Point2f>& world_path) {
  std::vector<int> pixel_waypoints;

  for(const auto& point : world_path) {
    // ì›”ë“œ ì¢Œí‘œ(ë¯¸í„°) â†’ í”½ì…€ ì¢Œí‘œ ì§ì ‘ ë³€í™˜
    int pixel_x = static_cast<int>(point.x / pixel_to_meter + 320);
    pixel_waypoints.push_back(pixel_x);

  }
  return pixel_waypoints;
}

cv::Point2f RRT_star::gridToWorld(int grid_x, int grid_y) const {
  double world_x = (grid_x - width/2) * resolution;
  double world_y = (grid_y - height/2) * resolution;
  return cv::Point2f(world_x, world_y);
}
