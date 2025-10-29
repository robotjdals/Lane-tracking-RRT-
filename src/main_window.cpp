#include "../include/min_22_pkg/main_window.hpp"
#include "../include/min_22_pkg/driving.hpp"


MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign) {
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();
  driving = new Driving(this);
  rrt_planner = new RRT_star();

  driving->setQNode(qnode);
  driving->setMainWindow(this);
  driving->setRRTPlanner(rrt_planner);



  //rrt_planner->setVisualizationCallback(nullptr);
/*
  rrt_planner = new RRT_star();
  rrt_planner->setVisualizationCallback([this]() {
    cv::Mat grid_map = rrt_planner->getgridmap();
    if (!grid_map.empty()) {
      QImage qimg(grid_map.data, grid_map.cols, grid_map.rows, QImage::Format_Grayscale8);
      ui->display_5->setPixmap(QPixmap::fromImage(qimg));
    }
  });
  */

  prev_left_x = 0;
  prev_right_x = 0;
  first_frame = true;

  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
  QObject::connect(qnode, SIGNAL(sigRcvImg()), this, SLOT(slotUpdateImg()));
  QObject::connect(this, &MainWindow::waypointsReady, this, &MainWindow::processWaypoints);

}

void MainWindow::closeEvent(QCloseEvent* event) { QMainWindow::closeEvent(event); }

MainWindow::~MainWindow() {
  if (qnode) {
    qnode->running_ = false;  // 스레드 종료 신호
  }

  delete ui;
}
/*
void MainWindow::calibration(const Mat &input_img, Mat &output_img){
  Mat intrinsicRead;
  Mat distCoeffsRead;
  getMatrixVal(&input_img, &intrinsicRead, &distCoeffsRead);

  undistort(input_img,&input_img, intrinsicRead, distCoeffsRead);
}*/

void MainWindow::slotUpdateImg() {  //UI에 캠화면 출력
  std::lock_guard<std::mutex> lock(qnode->img_mutex);
  clone_mat = qnode->imgRaw->clone();                                           // 원본 이미지 복사
  cv::resize(clone_mat, clone_mat, cv::Size(640, 360), 0, 0, cv::INTER_CUBIC);  // 이미지 크기 조정

  cv::Mat display_img = clone_mat.clone();
  cv::Mat combine_img = clone_mat.clone();

  int left_start, right_start;

  //Find_Binary_img(display1_img);
  //yellow_imgs(display1_img);
  Gaussain_Filter(display_img);
  combine_img = sumImg(white_hsv(display_img), yellow_hsv(display_img));
  perspective_transform(combine_img, Perspective_img);

  QImage RGB_im2((const unsigned char*)(Perspective_img.data), Perspective_img.cols, Perspective_img.rows, QImage::Format_Grayscale8);
  ui->display_3->setPixmap(QPixmap::fromImage(RGB_im2));

  cv::Mat window_img = Perspective_img.clone();

  if(first_frame){
    left_start = left_line(Perspective_img);
    right_start = right_line(Perspective_img);
    first_frame = false;
  }
  else{
    left_start = prev_left_x;
    right_start = prev_right_x;
  }

  prev_left_x = left_start;
  prev_right_x = right_start;


  std::vector<int> waypoints = getWindowSearch(Perspective_img, left_start, right_start);

  current_waypoints = waypoints;
  emit waypointsReady();

  QImage RGB_im1((const unsigned char*)(display_img.data), display_img.cols, display_img.rows, QImage::Format_RGB888);
  ui->label->setPixmap(QPixmap::fromImage(RGB_im1));

  //QImage RGB_im_test((const unsigned char*)(clone_mat.data), clone_mat.cols, clone_mat.rows, QImage::Format_RGB888);
  //ui->display_3->setPixmap(QPixmap::fromImage(RGB_im_test));
  static int rrt_update_counter = 0;
  rrt_update_counter++;

  int update_frequency = (driving && driving->state == Driving::AVOIDANCE) ? 1 : 3;


  if (rrt_update_counter % update_frequency == 0 && qnode->lidar_received && !qnode->lidar_ranges.empty() &&rrt_planner) {
    //rrt_update_counter % 3 == 0 &&
    try {
      auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
      scan_msg->ranges = qnode->lidar_ranges;
      scan_msg->angle_min = qnode->lidar_angle_min;
      scan_msg->angle_max = qnode->lidar_angle_max;
      scan_msg->angle_increment = qnode->lidar_angle_increment;
      scan_msg->range_min = 0.1;
      scan_msg->range_max = 10.0;

      rrt_planner->updateMap(scan_msg, 0.0, 0.0, 0.0);

      cv::Mat test_map = rrt_planner->getgridmap();
      //std::cout << "Map after update - Empty: " << test_map.empty()<< ", Size: " << test_map.rows << "x" << test_map.cols << std::endl;
      // 직접 UI 업데이트 (콜백 대신)
      cv::Mat grid_map = rrt_planner->getVisualizationMap();
      QImage qimg(grid_map.data, grid_map.cols, grid_map.rows, grid_map.step, QImage::Format_BGR888);
      ui->display_5->setPixmap(QPixmap::fromImage(qimg));

    }catch (const std::exception& e) {
      //std::cout << "RRT update error: " << e.what() << std::endl;
    }
  }
  /*
  if (qnode->lidar_received && !qnode->lidar_ranges.empty()) {
    // LaserScan 메시지 생성
    auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    scan_msg->ranges = qnode->lidar_ranges;
    scan_msg->angle_min = qnode->lidar_angle_min;
    scan_msg->angle_max = qnode->lidar_angle_max;
    scan_msg->angle_increment = qnode->lidar_angle_increment;
    scan_msg->range_min = 0.1;
    scan_msg->range_max = 10.0;

    // 그리드맵 업데이트 (로봇 위치는 0,0,0으로 가정)
    rrt_planner->updateMap(scan_msg, 0.0, 0.0, 0.0);

  }*/
/*
  if(driving && !driving->rrt_waypoints.empty()) {
    // 빈 이미지 생성 (640x360, 차선 추적 이미지와 같은 크기)
    cv::Mat rrt_visual = cv::Mat::zeros(360, 640, CV_8UC3);

    // 중앙선 표시 (320픽셀)
    cv::line(rrt_visual, cv::Point(320, 0), cv::Point(320, 360), cv::Scalar(255, 255, 255), 2);

    // RRT 웨이포인트들을 원으로 표시
    for(size_t i = 0; i < driving->rrt_waypoints.size(); i++) {
      int x = driving->rrt_waypoints[i];
      int y = 350 - (i * 30);  // y 좌표는 아래부터 위로 배치

      // 웨이포인트 원 그리기
      cv::circle(rrt_visual, cv::Point(x, y), 8, cv::Scalar(0, 255, 0), -1);

      // 인덱스 번호 표시
      cv::putText(rrt_visual, std::to_string(i), cv::Point(x-5, y+3),
                  cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
    }

    // 현재 추적 중인 웨이포인트 강조 표시
    if(driving->current_path_index < driving->rrt_waypoints.size()) {
      int current_x = driving->rrt_waypoints[driving->current_path_index];
      int current_y = 350 - (driving->current_path_index * 30);
      cv::circle(rrt_visual, cv::Point(current_x, current_y), 12, cv::Scalar(0, 0, 255), 3);
    }

    // Qt에 표시
    QImage rrt_qimg(rrt_visual.data, rrt_visual.cols, rrt_visual.rows,
                    rrt_visual.step, QImage::Format_BGR888);
    ui->display_6->setPixmap(QPixmap::fromImage(rrt_qimg));
  }*/
  QImage RGB_im3((const unsigned char*)(Perspective_img.data), Perspective_img.cols, Perspective_img.rows, QImage::Format_Grayscale8);
  ui->display_4->setPixmap(QPixmap::fromImage(RGB_im3));
  if (qnode->imgRaw) {
    delete qnode->imgRaw;// 동적 할당된 원본 이미지 메모리 해제
    qnode->imgRaw = nullptr;
  }
  qnode->isreceived = false;  // 이미지 수신 플래그 재설정
}

void MainWindow::processWaypoints() {
  if (driving && !current_waypoints.empty()) {
      driving->go(current_waypoints);  // 직접 호출
  }
}

void MainWindow::perspective_transform(const cv::Mat& input_img, cv::Mat& output_img){
  std::array<cv::Point2f, 4> input_birdseyeview = {cv::Point2f(640 - 520, 360 - 80), cv::Point2f(640 - 120, 360 - 80), cv::Point2f(640 - 10, 360 - 20), cv::Point2f(640 - 630, 360 - 20)};
  //std::array<cv::Point2f, 4> input_birdseyeview = {cv::Point2f(640 - 540, 360 - 80), cv::Point2f(640 - 100, 360 - 80), cv::Point2f(640 - 10, 360 - 30), cv::Point2f(640 - 630, 360 - 30)};
  //std::array<cv::Point2f, 4> input_birdseyeview = {cv::Point2f(640 - 540 , 360 - 80), cv::Point2f(0, 360 - 80), cv::Point2f(640 - 10, 360 - 40), cv::Point2f(640 - 630, 360 - 40)};
  std::array<cv::Point2f, 4> output_birdseyeview = {cv::Point2f(0,0), cv::Point2f(Raw_X,0), cv::Point2f(Raw_X, Raw_Y), cv::Point2f(0,Raw_Y)};

  cv::Mat birdseyeview = cv::getPerspectiveTransform(input_birdseyeview.data(), output_birdseyeview.data());
//*invMatx = getPerspectiveTransform(output_birdseyeview.data(), input_birdseyeview.data());

  cv::warpPerspective(input_img, output_img, birdseyeview, cv::Size(Raw_X, Raw_Y));

  cv::line(const_cast<cv::Mat &>(input_img), input_birdseyeview[0], input_birdseyeview[1], cv::Scalar(0, 255, 255), 3, cv::LINE_AA);
  cv::line(const_cast<cv::Mat &>(input_img), input_birdseyeview[1], input_birdseyeview[2], cv::Scalar(0, 255, 255), 3, cv::LINE_AA);
  cv::line(const_cast<cv::Mat &>(input_img), input_birdseyeview[2], input_birdseyeview[3], cv::Scalar(0, 255, 255), 3, cv::LINE_AA);
  cv::line(const_cast<cv::Mat &>(input_img), input_birdseyeview[3], input_birdseyeview[0], cv::Scalar(0, 255, 255), 3, cv::LINE_AA);
}


cv::Mat MainWindow::white_hsv(cv::Mat& img) {
  // 복사 이미지 HSV로 변환
  cv::Mat hsvImg;
  cv::cvtColor(img, hsvImg, cv::COLOR_BGR2HSV);

  cv::Mat mask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
  cv::erode(hsvImg, hsvImg, mask, cv::Point(-1, -1), 1);

  // HSV 이미지를 사용하여 범위 내의 색상을 임계값으로 설정
  cv::Scalar lower(0, 0, 180);
  cv::Scalar upper(180, 30, 255);
  cv::Mat binaryImage;
  cv::inRange(hsvImg, lower, upper, binaryImage);

  //QImage RGB_im3((const unsigned char*)(binaryImage.data), binaryImage.cols, binaryImage.rows, QImage::Format_Grayscale8);
  //ui->display_4->setPixmap(QPixmap::fromImage(RGB_im3));

  return binaryImage;
}

// 이진화
cv::Mat MainWindow::yellow_hsv(cv::Mat& img) {
  // 복사 이미지 HSV로 변환
  cv::Mat hsvImg;
  cv::cvtColor(img, hsvImg, cv::COLOR_BGR2HSV);

  cv::Mat mask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
  cv::erode(hsvImg, hsvImg, mask, cv::Point(-1, -1), 1);

  // HSV 이미지를 사용하여 범위 내의 색상을 임계값으로 설정
  cv::Scalar lower(0, 90, 90);
  cv::Scalar upper(100, 255, 255);
  cv::Mat binaryImage;
  cv::inRange(hsvImg, lower, upper, binaryImage);

  //QImage RGB_im4((const unsigned char*)(binaryImage.data), binaryImage.cols, binaryImage.rows, QImage::Format_Grayscale8);
  //ui->display_5->setPixmap(QPixmap::fromImage(RGB_im4));

  return binaryImage;
}

void MainWindow::Gaussain_Filter(cv::Mat& img){

  cv::Mat blurredImage;
  cv::GaussianBlur(img, blurredImage, cv::Size(7,7), 0); // 가우시안 필터

  QImage binaryQImage(blurredImage.data, blurredImage.cols, blurredImage.rows, blurredImage.step, QImage::Format_RGB888); // 시각화
  ui->test->setPixmap(QPixmap::fromImage(binaryQImage));

}

int MainWindow::left_line(cv::Mat original)
{
  if (original.empty() || original.rows < 4 || original.cols < 4) {
    return 0;
  }

  double minVal, maxVal;
  cv::Point minLoc, maxLoc;
  cv::Mat left = original(cv::Range(original.rows* 3 / 4, original.rows), cv::Range(0, original.cols/2));
  cv::Mat left_float;
  left.convertTo(left_float, CV_32F);

  cv::Mat outimg;
  cv::reduce(left_float, outimg, 0, cv::REDUCE_SUM, CV_32F);
  cv::minMaxLoc(outimg, &minVal, &maxVal, &minLoc, &maxLoc);

  return maxLoc.x;
}

int MainWindow::right_line(cv::Mat original)
{
  if (original.empty() || original.rows < 4 || original.cols < 4) {
    return original.cols - 1;
  }

  double minVal, maxVal;
  cv::Point minLoc, maxLoc;
  cv::Mat right = original(cv::Range(original.rows * 3 / 4, original.rows), cv::Range(original.cols/2, original.cols));
  cv::Mat right_float;
  right.convertTo(right_float, CV_32F);

  cv::Mat outimg;
  cv::reduce(right_float, outimg, 0, cv::REDUCE_SUM, CV_32F);
  cv::minMaxLoc(outimg, &minVal, &maxVal, &minLoc, &maxLoc);

  return maxLoc.x + original.cols/2 + 20;
}

std::vector<int> MainWindow::getWindowSearch(cv::Mat& searchimg, int& left_x, int& right_x){
  //cv::Rect rect(x, y, width, height);

  int numwindow = 10;
  int window_width = 120;
  int window_height = searchimg.rows / numwindow;
  int margin = 60;


  std::vector<int> left_centers, right_centers, waypoint;

  int current_left_x = left_x;
  int current_right_x = right_x;

  std::vector<cv::Point2f> left_world_points;
  std::vector<cv::Point2f> right_world_points;

  bool left_detected = false;
  bool right_detected = false;
  int left_detection_count = 0;
  int right_detection_count = 0;

  for(int i = 0; i < numwindow; i++){
    int window_y = searchimg.rows - (i + 1) * window_height;
    if(window_y < 0) continue;


    int left_window_x = std::max(0, current_left_x - margin);
    int left_window_width = std::min(window_width, searchimg.cols - left_window_x);
    int left_window_height = std::min(window_height, searchimg.rows - window_y);

    if(left_window_width > 0 && left_window_height > 0){
      cv::Rect leftwindow_rect(left_window_x, window_y, left_window_width, left_window_height);
      cv::rectangle(searchimg, leftwindow_rect, cv::Scalar(255),2);

      cv::Mat Window = searchimg(leftwindow_rect);
      cv::Mat left_lane;
      cv::findNonZero (Window, left_lane);

      if(left_lane.rows > 3){
        int left_sum = 0;
        for(int j =0; j < left_lane.rows; j++){
          cv::Point pt = left_lane.at<cv::Point>(j);
          left_sum += pt.x;
        }
        int new_center = left_window_x + left_sum / left_lane.rows;
        current_left_x = new_center;
        left_centers.push_back(new_center);

        left_detected = true;
        //left_detection_count++;

        double world_x = (new_center - 320) * Driving::pixel_to_meter ;
        double world_y = ((numwindow - 1 - i) * window_height) * Driving::pixel_to_meter;
        left_world_points.push_back(cv::Point2f(world_x, world_y));

        cv::circle(searchimg, cv::Point(new_center, window_y + window_height/2), 5, cv::Scalar(128), -1);
      }
      else {
        /*
        current_left_x = 0;
        left_centers.push_back(-0);
        cv::circle(searchimg, cv::Point(0, window_y + window_height/2), 5, cv::Scalar(128), -1);
        */
        left_centers.push_back(current_left_x);
        cv::circle(searchimg, cv::Point(current_left_x, window_y + window_height/2), 5, cv::Scalar(128), -1);
      }
    }
    else{
      left_centers.push_back(current_left_x);
    }

    int right_window_x = std::max(0, std::min(current_right_x - margin, searchimg.cols - window_width));
    int right_window_width = std::min(window_width, searchimg.cols - right_window_x);
    int right_window_height = std::min(window_height, searchimg.rows - window_y);

    if(right_window_width > 0 && right_window_height >0) {

      cv::Rect rightwindow_rect(right_window_x, window_y, right_window_width, right_window_height);
      cv::rectangle(searchimg, rightwindow_rect, cv::Scalar(255),2);

      cv::Mat Window = searchimg(rightwindow_rect);
      cv::Mat right_lane;
      cv::findNonZero (Window, right_lane);

      if(right_lane.rows > 3) {
        int right_sum = 0;
        for(int j =0; j < right_lane.rows; j++){
          cv::Point pt = right_lane.at<cv::Point>(j);
          right_sum += pt.x;
        }
        int new_center = right_window_x + right_sum / right_lane.rows;
        current_right_x = new_center;
        right_centers.push_back(new_center);

        right_detected = true;
        //right_detection_count++;

        double world_x = (new_center - 320) * Driving::pixel_to_meter;
        double world_y = ((numwindow - 1 - i) * window_height) * Driving::pixel_to_meter;
        right_world_points.push_back(cv::Point2f(world_x, world_y));

        cv::circle(searchimg, cv::Point(new_center, window_y + window_height/2), 5, cv::Scalar(128), -1);
      }
      else {
        /*
        current_right_x = searchimg.cols -1;
        right_centers.push_back(searchimg.cols-1);
        cv::circle(searchimg, cv::Point(searchimg.cols-1, window_y + window_height/2), 5, cv::Scalar(128), -1);
        */
        right_centers.push_back(current_right_x);
        cv::circle(searchimg, cv::Point(current_right_x, window_y + window_height/2), 5, cv::Scalar(128), -1);
      }
    }
    else{
      right_centers.push_back(current_right_x);
    }
  }

  if(!left_centers.empty()) {

    int bottom_windows = std::min(3, (int)left_centers.size());
    int sum = 0;
    for(int i = 0; i < bottom_windows; i++) {
        sum += left_centers[i];
    }
    left_x = sum / bottom_windows;

  }

  if(!right_centers.empty()) {

    int bottom_windows = std::min(3, (int)right_centers.size());
    int sum = 0;
    for(int i = 0; i < bottom_windows; i++) {
        sum += right_centers[i];
    }
    right_x = sum / bottom_windows;

  }

  for(int i = 0; i < numwindow; i++){
    int window_y = searchimg.rows - (i + 1) * window_height;

    if(i < left_centers.size() && i < right_centers.size()) {
      int mean = (left_centers[i] + right_centers[i]) / 2;
      waypoint.push_back(mean);

      cv::circle(searchimg, cv::Point(mean, window_y + window_height/2), 4, cv::Scalar(200), -1);
    }
  }

  this -> left_detected = (left_world_points.size() >= numwindow / 2);
  this -> right_detected = (right_world_points.size() >= numwindow / 2);

  if(rrt_planner) {
    // 양쪽 모두 있어야만 차선 사용
    if(!left_world_points.empty() && !right_world_points.empty()) {
      rrt_planner->setLanePoints(left_world_points, right_world_points);
    } else {
      rrt_planner->clearLanePoints();
    }
  }


  return waypoint;

}

/*
cv::Mat MainWindow::HLS_L(cv::Mat output_img){
  cv::Mat imgHLS_L, imgNormal;
  double minVal, maxVal;
  cv::Point minLoc, maxLoc;
  int lowThres = 220; //200~255 사이 임계값


  imgHLS_L = filterImg(output_img, HLS_CHANNEL, L_FILTER);
  minMaxLoc(imgHLS_L, &minVal, &maxVal, &minLoc, &maxLoc);

  imgNormal = (255.0 / maxVal) * imgHLS_L;

  cv::Mat imgOut = cv::Mat::zeros(imgNormal.rows, imgNormal.cols, imgNormal.type());
  cv::threshold(imgNormal, imgOut, lowThres, 255, cv::THRESH_BINARY);

  QImage RGB_im3((const unsigned char*)(imgOut.data), imgOut.cols, imgOut.rows, QImage::Format_Grayscale8);
  ui->display_4->setPixmap(QPixmap::fromImage(RGB_im3));
  return imgOut;
}


cv::Mat MainWindow::LAB_B(cv::Mat output_img){
  cv::Mat imgLAB_B, imgNormal;
  double minVal, maxVal;
  cv::Point minLoc, maxLoc;
  int yellowCrit = 130;
  int lowThres = 110; //200~255 사이 임계값

  imgLAB_B = filterImg(output_img, LAB_CHANNEL, B_FILTER_);
  minMaxLoc(imgLAB_B, &minVal, &maxVal, &minLoc, &maxLoc);


  if (maxVal > yellowCrit)
    {
      imgNormal = (255.0 / (maxval)) * imgLAB_B;
    }
    else
    {
      imgNormal = imgLAB_B;
    }

  cv::Mat imgOut = cv::Mat::zeros(imgNormal.rows, imgNormal.cols, imgNormal.type());
  cv::threshold(imgNormal, imgOut, lowThres, 255, cv::THRESH_BINARY);

  QImage RGB_im4((const unsigned char*)(imgOut.data), imgOut.cols, imgOut.rows, QImage::Format_Grayscale8);
  ui->display_5->setPixmap(QPixmap::fromImage(RGB_im4));
  return imgOut;
}

cv::Mat MainWindow::filterImg(cv::Mat input, int colorspace, int channel)
{
  cv::Mat imgConverted;

  if(colorspace == HLS_CHANNEL)
    cvtColor(input, imgConverted, cv::COLOR_BGR2HLS);

  else if(colorspace == LAB_CHANNEL)
    cvtColor(input, imgConverted, cv::COLOR_BGR2Lab);

  std::vector<cv::Mat> channels;
  cv::split(imgConverted, channels);

  return channels[channel];
}*/

cv::Mat MainWindow::sumImg(cv::Mat img1, cv::Mat img2)
{
  cv::Mat imgout;
  cv::bitwise_or(img1, img2, imgout);

  return imgout;
}

/*
void getMatrixVal(Mat *input_img, Mat *intrinsicRead, Mat *distCoeffsRead){

  int m = 0, n = o;

  ifstream in(*input_img);

  in >> m >> n;

  Mat intrinsicRead_tmp = Mat(m,n,6);
  for (int i = 0; i < m; i++)
    for (int j = 0; j < n; j++)
      in >> intrinsicRead_tmp.ptr<double>(i)[j];

  in >> m >> n;

  Mat distCoeffsRead_tmp = Mat(m, n, 6);
  for (int i = 0; i < m; i++)
    for (int j = 0; j < n; j++)
      in >> distCoeffsRead_tmp.ptr<double>(i)[j];

  *intrinsicRead = intrinsicRead_tmp;
  *distCoeffsRead = distCoeffsRead_tmp;

  in.close();
}*/
