#include <ryusei/navi/potential.hpp>
#include <iostream>
#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>

using std::cout;
using std::endl;
using std::vector;
using std::ofstream;
using namespace project_ryusei;
namespace prop = boost::property_tree;

#define DEBUG 0
#define SHOW_SCORE 0

/*************************************
* Public methods
*************************************/

Potential::Potential()
{

}

Potential::~Potential()
{

}

/**************************************
* iniファイルの読み込みを行うメソッド
***************************************/
bool Potential::init(const std::string &conf_path)
{
  prop::ptree pt;
  prop::read_ini(conf_path, pt);
  /*** iniファイルからパラメータを読み込み ***/
  if(auto v = pt.get_optional<double>("Potential.ObsWeight")) OBS_WEIGHT_ = v.get();
  if(auto v = pt.get_optional<double>("Potential.DstWeight")) DST_WEIGHT_ = v.get();

  return true;
}

/*************************************************************************************************
* 座標を受け取りその1点におけるポテンシャルスコア値を返すメソッド.
* ・斥力 = Σ(e ^ ((-x座標の障害物との相対距離^2)/(2*分散) - (-y座標の障害物との相対距離^2)/(2*分散)))
* ・引力 = e ^ ((-x座標の障害物との相対距離^2)/(2*分散) - (-y座標の障害物との相対距離^2)/(2*分散))
**************************************************************************************************/
double Potential::calcPotential(const Pose2D &pose, const Pose2D &dst_pose, const std::vector<cv::Point2f> &obstacles)
{
  double attractive_force = 0, repulsion = 0, power_temp1, power_temp2, potential;
  double var_attractive = 2.5, var_repulsion = 0.3;

  power_temp1 = -1 * ((pose.x - dst_pose.x) * (pose.x - dst_pose.x) / (2 * var_attractive * var_attractive));
  power_temp2 = -1 * ((pose.y - dst_pose.y) * (pose.y - dst_pose.y) / (2 * var_attractive * var_attractive));
  attractive_force = -1 * DST_WEIGHT_ * exp(power_temp1 + power_temp2);

  for(int i = 0; i < obstacles.size(); i++){
    power_temp1 = -1 * ((pose.x - obstacles[i].x) * (pose.x - obstacles[i].x) / (2 * var_repulsion * var_repulsion));
    power_temp2 = -1 * ((pose.y - obstacles[i].y) * (pose.y - obstacles[i].y) / (2 * var_repulsion * var_repulsion));
    repulsion += OBS_WEIGHT_ * exp(power_temp1 + power_temp2);
  }

  potential = attractive_force + repulsion;
  #if DEBUG
  cout << "potential: " << potential << endl;
  #endif
  
  return potential;
}

/*********************************************************************
* ロボットの座標、全障害物、目標地点を引数に取り、複数方向から一番
* ポテンシャル値の合計が低い最適な方向をradianで返すメソッド
**********************************************************************/
double Potential::findOptimalWay(const Pose2D &pose, const std::vector<Obstacle> &obstacles, const Pose2D &goal)
{
  /*** 直線の2端点または矩形の4頂点 ***/
  cv::Point2f point1, point2, point3, point4;

  /*** 直線または矩形で表された障害物をポテンシャル計算に使う点を決定する関数に入れる ***/
  for(int i = 0; i < obstacles.size(); i++){
    if(obstacles[i].abs.size() == 2){      // obstacles[i]が直線のとき
      point1.x = obstacles[i].abs[0].x;
      point1.y = obstacles[i].abs[0].y;
      point2.x = obstacles[i].abs[1].x;
      point2.y = obstacles[i].abs[1].y;
      findLinePotential(point1, point2);
    }else{      // obstacles[i]が矩形のとき
      point1.x = obstacles[i].abs[0].x;
      point1.y = obstacles[i].abs[0].y;
      point2.x = obstacles[i].abs[1].x;
      point2.y = obstacles[i].abs[1].y;
      point3.x = obstacles[i].abs[2].x;
      point3.y = obstacles[i].abs[2].y;
      point4.x = obstacles[i].abs[3].x;
      point4.y = obstacles[i].abs[3].y;
      findRectPotential(point1, point2, point3, point4);
    }
  }

  /*** 最適な進行方向を一定距離で分割した地点のポテンシャル値を合計してそれぞれの角度で比較 ***/
  double min_direction_score = 1000.0, temp_direction_score = 0.0, theta = -45.0, angle_intervals = 5.0;
  double direction, distance_observe, rad;
  Pose2D pose_observe, pose_observe_rel, pose_observe_abs, goal_abs;

  while(theta <= 45){
    #if SHOW_SCORE
    cout << "----------------------" << endl;
    cout << "theta: " << theta << endl;
    #endif
    rad = theta * CV_PI/180;
    double sin_val = sin(rad);
    double cos_val = cos(rad);
    for(int i = 0; i < 5; i++){
      distance_observe = i/4.0;    // 0m,0.25m,0.5m,0.75m,1m地点を計算
      pose_observe.x = (distance_observe * cos_val) + pose.x;
      pose_observe.y = (distance_observe * sin_val) + pose.y;
      temp_direction_score += calcPotential(pose_observe, goal, potential_points_);
    }
    if(temp_direction_score < min_direction_score){
      min_direction_score = temp_direction_score;
      direction = rad;
    }
    #if SHOW_SCORE
    cout << "score: " << temp_direction_score << endl;
    cout << "----------------------" << endl;
    #endif
    temp_direction_score = 0;
    theta += angle_intervals;
  }                                      
  double re_theta_rel = direction * 180/CV_PI;
  cout << "Target Direction(deg)(rel): " << re_theta_rel << endl;
  double re_theta_abs = (direction  + pose.yaw) * 180/CV_PI;
  cout << "Target Direction(deg)(abs): " << re_theta_abs << endl;
  direction = direction + pose.yaw;

  return direction;
}

/****************************************************************************
* ロボットの周囲(range_x[m] * 2, range_y[m] * 2)のポテンシャルを
* unit[m]ごとに計算しcv::Mat型(CV_32FC1)のpotential_mapに値を代入するメソッド
*****************************************************************************/
void Potential::createPotentialMap(const Pose2D &pose, const std::vector<Obstacle> &obstacles, const Pose2D &goal,
                                   const float &range_x, const float &range_y, const float &unit, cv::Mat &potential_map)
{
 /*** 直線の2端点または矩形の4頂点 ***/
  cv::Point2f point1, point2, point3, point4;

  /*** 直線または矩形で表された障害物をポテンシャル計算に使う点を決定する関数に入れる ***/
  for(int i = 0; i < obstacles.size(); i++){
    if(obstacles[i].abs.size() == 2){      // obstacles[i]が直線のとき
      point1.x = obstacles[i].abs[0].x;
      point1.y = obstacles[i].abs[0].y;
      point2.x = obstacles[i].abs[1].x;
      point2.y = obstacles[i].abs[1].y;
      findLinePotential(point1, point2);
    }else{           // obstacles[i]が矩形のとき
      point1.x = obstacles[i].abs[0].x;
      point1.y = obstacles[i].abs[0].y;
      point2.x = obstacles[i].abs[1].x;
      point2.y = obstacles[i].abs[1].y;
      point3.x = obstacles[i].abs[2].x;
      point3.y = obstacles[i].abs[2].y;
      point4.x = obstacles[i].abs[3].x;
      point4.y = obstacles[i].abs[3].y;
      findRectPotential(point1, point2, point3, point4);
    }
  }

  double cos_val = cos(pose.yaw);
  double sin_val = sin(pose.yaw);
  /*** potential_mapのサイズ設定 ***/
  int potential_map_cols = 2 * range_x / unit;
  int potential_map_rows = 2 * range_y / unit;
  potential_map = cv::Mat(potential_map_cols, potential_map_rows, CV_32FC1);

  /*** 座標変換しつつpotential_mapにポテンシャル値を代入 ***/
  Pose2D pose_observe_rel, pose_observe_abs;
  for(int y = 0; y < potential_map.rows; y++){
    for(int x = 0; x < potential_map.cols; x++){
      pose_observe_rel.x = (potential_map.rows / 2 - y) * unit;
      pose_observe_rel.y = (potential_map.cols / 2 - x) * unit;
      pose_observe_abs.x = (pose_observe_rel.x * cos_val) - (pose_observe_rel.y * sin_val) + pose.x;
      pose_observe_abs.y = (pose_observe_rel.y * cos_val) + (pose_observe_rel.x * sin_val) + pose.y;
      potential_map.at<float>(x,y) = (float)calcPotential(pose_observe_abs, goal, potential_points_);
    }
  }
  
}

/*************************************
* Private methods
*************************************/

/************************************************************************
* ロボット位置(絶対座標なので原点)から一番近い直線障害物上の点を返すメソッド.
* 最短距離の計算には内積を使用している.
*************************************************************************/
void Potential::findLinePotential(const cv::Point2f &point1, const cv::Point2f &point2)
{
  /*** point1からpoint2へのベクトルa, point1から原点へのベクトルb ***/
  cv::Point2f vector_a, vector_b;
  /*** 交点までの距離比率 ***/
  double ratio;
  /*** ベクトルaとベクトルb ***/
  vector_a.x = point2.x - point1.x;
  vector_a.y = point2.y - point1.y;
  vector_b.x = -1 * point1.x;
  vector_b.y = -1 * point1.y;
  /*** 交点までの距離比率 = 内積 ÷ |a|^2 ***/
  ratio = (vector_a.x * vector_b.x + vector_a.y * vector_b.y) / (vector_a.x * vector_a.x + vector_a.y * vector_a.y);
  /*** ratio(交点までの距離比率)が0以下ならpoint1を返す, 1以上ならpoint2を返す, それ以外ならpoint1 + r*aを返す ***/
  if(ratio <= 0){
    potential_points_.push_back(cv::Point2f(point1.x, point1.y));
  }else if(ratio >= 1){
    potential_points_.push_back(cv::Point2f(point2.x, point2.y));
  }else{
    cv::Point2f result;
    result.x = point1.x + ratio * vector_a.x;
    result.y = point1.y + ratio * vector_a.y;
    potential_points_.push_back(cv::Point2f(result.x, result.y));
  }
}

/*****************************************************************************
* 矩形障害物の４辺から原点に一番近い辺をfindLinePotentialメソッドに送るメソッド.
* findLinePotentialメソッドは直線の一番近い点を返すことから、これによって矩形上
* の一番近い点を取得することができる.
******************************************************************************/
void Potential::findRectPotential(const cv::Point2f &point1, const cv::Point2f &point2, const cv::Point2f &point3,
                                  const cv::Point2f &point4)
{
  /*** 矩形障害物の4つの頂点とロボットの距離 ***/
  double point_distance1, point_distance2, point_distance3, point_distance4;
  double intersection1_x, intersection1_y, intersection2_x, intersection2_y,intersection3_x, intersection3_y,
         intersection4_x, intersection4_y, theta;
  point_distance1 = sqrt(point1.x * point1.x + point1.y * point1.y);
  point_distance2 = sqrt(point2.x * point2.x + point2.y * point2.y);
  point_distance3 = sqrt(point3.x * point3.x + point3.y * point3.y);
  point_distance4 = sqrt(point4.x * point4.x + point4.y * point4.y);
  /*** 矩形の頂点で一番原点に近い点を探す ***/
  int closest_point = 1;
  double min_distance = point_distance1;
  if(point_distance2 < min_distance){
    closest_point = 2;
    min_distance = point_distance2;
  }
  if(point_distance3 < min_distance){
    closest_point = 3;
    min_distance = point_distance3;
  }
  if(point_distance4 < min_distance){
    closest_point = 4;
  }
  /*** 原点から一番近い、直線上の点から出てる2辺をfindLinePotential関数に入れる ***/
  switch (closest_point)
  {
    case 1:
      findLinePotential(point1, point2);
      findLinePotential(point1, point3);
      break;
    case 2:
      findLinePotential(point2, point1);
      findLinePotential(point2, point4);
      break;
    case 3:
      findLinePotential(point3, point1);
      findLinePotential(point3, point4);
      break;
    case 4:
      findLinePotential(point4, point2);
      findLinePotential(point4, point3);
      break;
    default:
      break;
  }
}