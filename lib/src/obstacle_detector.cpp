#include <ryusei/navi/obstacle_detector.hpp>
#include <opencv2/opencv.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
using namespace project_ryusei;
namespace prop = boost::property_tree;

#define DEBUG 0

/************************************
* Public methods
*************************************/
ObstacleDetector::ObstacleDetector()
{

}
ObstacleDetector::~ObstacleDetector()
{

}

/********************************************
* iniファイルからパラメータを読み込むメソッド
*********************************************/
bool ObstacleDetector::init(const std::string &conf_path)
{
  prop::ptree pt;
  prop::read_ini(conf_path, pt);
  if(auto v = pt.get_optional<double>("ObstacleDetector.RangeX")) RANGE_X_ = v.get();
  if(auto v = pt.get_optional<double>("ObstacleDetector.RangeY")) RANGE_Y_ = v.get();
  if(auto v = pt.get_optional<double>("ObstacleDetector.MaxZ")) MAX_RANGE_Z_ = v.get();
  if(auto v = pt.get_optional<double>("ObstacleDetector.MinZ")) MIN_RANGE_Z_ = v.get();
  if(auto v = pt.get_optional<double>("ObstacleDetector.TimesDelate")) TIMES_DELATE_ = v.get();
  if(auto v = pt.get_optional<double>("ObstacleDetector.TimesErode")) TIMES_ERODE_ = v.get();
  if(auto v = pt.get_optional<double>("ObstacleDetector.MinLineLength")) MIN_LINE_LENGTH_ = v.get();
  if(auto v = pt.get_optional<double>("ObstacleDetector.MaxLineGap")) MAX_LINE_GAP_ = v.get();
  if(auto v = pt.get_optional<double>("ObstacleDetector.HoughLineThoreshold")) THORESHOLD_ = v.get();
  if(auto v = pt.get_optional<double>("ObstacleDetector.Unit")) UNIT_ = v.get();
  IMG_SIZE_ = (RANGE_X_*2 / UNIT_) + 1;

  return true;
}

/********************************************
* 障害物検出状況を画像で確認するためのメソッド
*********************************************/
void ObstacleDetector::visualizeLocalMap(const LocalMap &map)
{
  #if !DEBUG
  int x, y;
  cv::Mat img(IMG_SIZE_, IMG_SIZE_, CV_8UC3, cv::Scalar(0));
  img_ = img;

  for(int i = 0; i < map.points.size(); i++){
    if(map.points[i].x < RANGE_X_ && map.points[i].x > (-1) * (RANGE_X_) && map.points[i].y < RANGE_Y_ && 
       map.points[i].y > (-1) * (RANGE_Y_) && map.points[i].z < MAX_RANGE_Z_ && map.points[i].z > MIN_RANGE_Z_){
      x = IMG_SIZE_/2 - (map.points[i].y * (1/UNIT_));
      y = IMG_SIZE_/2 - (map.points[i].x * (1/UNIT_));
      img_.at<cv::Vec3b>(x,y) = cv::Vec3b(255, 255, 255);
    }
  }

  #endif
  #if DEBUG
  cv::Mat img(IMG_SIZE_, IMG_SIZE_, CV_8UC3, cv::Scalar(0));
  img_ = img;
  /*** 左に縦線の障害物を描写 ***/
  cv::line(img_, cv::Point(100,300), cv::Point(100,100), cv::Scalar(255,255,255), 1);
  #endif
}

/************************************************************************
* 点群から障害物を検出し矩形か直線で表現して格納するメソッド.
* 全て矩形だと大きい矩形になったときに点群を含まない領域が大きくなってしまう.
*************************************************************************/
void ObstacleDetector::detect(const Pose2D &pose, std::vector<Obstacle> &obstacles)
{
  /*** ロボットの状態 ***/
  std::cout << "pose[(m)(m)(deg)]: " << pose << std::endl;
  /*** 入力された点群(point_cloud)から障害物を検出 ***/
  /*** Obstacleクラスのインスタンスを生成 ***/
  Obstacle result;
  /*** マップ画像をグレースケールに変換 ***/
  cv::cvtColor(img_, gray_img_, cv::COLOR_BGR2GRAY);
  /*** 点群の距離を縮めるため膨張処理 ***/
  cv::dilate(gray_img_, gray_img_, cv::Mat(), cv::Point(-1,-1), TIMES_DELATE_);
  cv::erode(gray_img_, gray_img_, cv::Mat(), cv::Point(-1,-1), TIMES_ERODE_);
  /*** MinLineLengthより大きい物体を直線で表現 ***/
  detectLineObjects(pose, obstacles, result);
  /*** MinLineLengthより小さい物体を矩形で表現 ***/
  detectRectObjects(pose, obstacles, result);
}

/*************************************
* Private methods
*************************************/

/**********************************************************************
* 画像処理された点群に対して直線検出を行い検出された全直線を画像上から消し、
* 直線障害物として格納していくメソッド
***********************************************************************/
void ObstacleDetector::detectLineObjects(const Pose2D &pose, std::vector<Obstacle> &obstacles, Obstacle &result)
{
  while(1){
    /*** 確率的Hough変換で直線を検出 ***/
    std::vector<cv::Vec4i> lines;
    HoughLinesP(gray_img_, lines, 1, CV_PI / 180.0, THORESHOLD_, MIN_LINE_LENGTH_, MAX_LINE_GAP_);
    cv::cvtColor(gray_img_, gray_img_, cv::COLOR_GRAY2BGR);
    /*** 直線の2点を絶対座標と相対座標で表す ***/
    double l_abs_x1, l_abs_y1, l_abs_x2, l_abs_y2, l_rel_x1, l_rel_y1, l_rel_x2, l_rel_y2;
    double cos_val = cos(pose.yaw);
    double sin_val = sin(pose.yaw);
    if(lines.size() != 0){
      line(img_, cv::Point(lines[0][0], lines[0][1]), cv::Point(lines[0][2], lines[0][3]), cv::Scalar(0,0,255), 5);
      /*** 検出された直線は黒で塗りつぶす ***/
      line(gray_img_, cv::Point(lines[0][0], lines[0][1]), cv::Point(lines[0][2], lines[0][3]), cv::Scalar(0,0,0), 10);
      cv::cvtColor(gray_img_, gray_img_, cv::COLOR_BGR2GRAY);
      /*** 元の距離[m]に戻す ***/
      /*** ロボットとの相対位置座標 ***/
      l_rel_x1 = ((double)lines[0][1] - (IMG_SIZE_ / 2)) * UNIT_ * (-1);
      l_rel_y1 = ((double)lines[0][0] - (IMG_SIZE_ / 2)) * UNIT_ * (-1);
      l_rel_x2 = ((double)lines[0][3] - (IMG_SIZE_ / 2)) * UNIT_ * (-1);
      l_rel_y2 = ((double)lines[0][2] - (IMG_SIZE_ / 2)) * UNIT_ * (-1);
      result.rel.push_back(cv::Point3f(l_rel_x1,l_rel_y1,0));
      result.rel.push_back(cv::Point3f(l_rel_x2,l_rel_y2,0));
      /*** 絶対位置座標 ***/
      l_abs_x1 = (l_rel_x1 * cos_val) - (l_rel_y1 * sin_val) + pose.x;
      l_abs_y1 = (l_rel_y1 * cos_val) + (l_rel_x1 * sin_val) + pose.y;
      l_abs_x2 = (l_rel_x2 * cos_val) - (l_rel_y2 * sin_val) + pose.x;
      l_abs_y2 = (l_rel_y2 * cos_val) + (l_rel_x2 * sin_val) + pose.y;
      result.abs.push_back(cv::Point3f(l_abs_x1,l_abs_y1,0));
      result.abs.push_back(cv::Point3f(l_abs_x2,l_abs_y2,0));
      obstacles.push_back(result);
      result.abs.clear();
      result.rel.clear();
    }else{
      break;
    }
  }
}

/***********************************************************************
* detectLineObjectsで検出されず残った点群を矩形障害物として格納するメソッド
************************************************************************/
void ObstacleDetector::detectRectObjects(const Pose2D &pose, std::vector<Obstacle> &obstacles, Obstacle &result)
{
  /*** 一定の大きさ未満の点群のまとまりをラベリング ***/
  cv::Mat label,stats,centroids;
  cv::cvtColor(gray_img_, gray_img_, cv::COLOR_BGR2GRAY);
  cv::Mat label_img = gray_img_.clone();
  int nLab = connectedComponentsWithStats(gray_img_, label, stats, centroids);
  
  for(int i = 1; i < nLab; ++i){
    /*** 矩形の4点を絶対座標と相対座標で表す ***/
    double r_abs_x1, r_abs_y1, r_abs_x2, r_abs_y2, r_abs_x3, r_abs_y3, r_abs_x4, r_abs_y4,
           r_rel_x1, r_rel_y1, r_rel_x2, r_rel_y2, r_rel_x3, r_rel_y3, r_rel_x4, r_rel_y4;
    double cos_val = cos(pose.yaw);
    double sin_val = sin(pose.yaw);
    int *param = stats.ptr<int>(i);
    int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
    int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
    int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
    int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
    cv::rectangle(label_img, cv::Rect(x, y, width, height), cv::Scalar(127), 2);
    cv::rectangle(img_, cv::Rect(x, y, width, height), cv::Scalar(127), 2);
    /*** 元の距離[m]に戻す ***/
    /*** ロボットとの相対位置座標 *1左上 *2右上 *3左下 *4右下 ***/
    r_rel_x1 = ((double)y - (IMG_SIZE_ / 2)) * UNIT_ * (-1);
    r_rel_y1 = ((double)x - (IMG_SIZE_ / 2)) * UNIT_ * (-1);
    r_rel_x2 = ((double)y - (IMG_SIZE_ / 2)) * UNIT_ * (-1);
    r_rel_y2 = ((double)x + (double)width - (IMG_SIZE_ / 2)) * UNIT_ * (-1);
    r_rel_x3 = ((double)y + (double)height - (IMG_SIZE_ / 2)) * UNIT_ * (-1);
    r_rel_y3 = ((double)x - (IMG_SIZE_ / 2)) * UNIT_ * (-1);
    r_rel_x4 = ((double)y + (double)height - (IMG_SIZE_ / 2)) * UNIT_ * (-1);
    r_rel_y4 = ((double)x + (double)width - (IMG_SIZE_ / 2)) * UNIT_ * (-1);
    result.rel.push_back(cv::Point3f(r_rel_x1,r_rel_y1,0));
    result.rel.push_back(cv::Point3f(r_rel_x2,r_rel_y2,0));
    result.rel.push_back(cv::Point3f(r_rel_x3,r_rel_y3,0));
    result.rel.push_back(cv::Point3f(r_rel_x4,r_rel_y4,0));
    /*** 絶対位置座標 *1左上 *2右上 *3左下 *4右下 ***/
    r_abs_x1 = (r_rel_x1 * cos_val) - (r_rel_y1 * sin_val) + pose.x; 
    r_abs_y1 = (r_rel_y1 * cos_val) + (r_rel_x1 * sin_val) + pose.y;
    r_abs_x2 = (r_rel_x2 * cos_val) - (r_rel_y2 * sin_val) + pose.x; 
    r_abs_y2 = (r_rel_y2 * cos_val) + (r_rel_x2 * sin_val) + pose.y;
    r_abs_x3 = (r_rel_x3 * cos_val) - (r_rel_y3 * sin_val) + pose.x; 
    r_abs_y3 = (r_rel_y3 * cos_val) + (r_rel_x3 * sin_val) + pose.y;
    r_abs_x4 = (r_rel_x4 * cos_val) - (r_rel_y4 * sin_val) + pose.x; 
    r_abs_y4 = (r_rel_y4 * cos_val) + (r_rel_x4 * sin_val) + pose.y;
    result.abs.push_back(cv::Point3f(r_abs_x1,r_abs_y1,0));
    result.abs.push_back(cv::Point3f(r_abs_x2,r_abs_y2,0));
    result.abs.push_back(cv::Point3f(r_abs_x3,r_abs_y3,0));
    result.abs.push_back(cv::Point3f(r_abs_x4,r_abs_y4,0));
    obstacles.push_back(result);
    result.rel.clear();
    result.abs.clear();
  }
}