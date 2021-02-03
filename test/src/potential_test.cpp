#include <ryusei/navi/obstacle_detector.hpp>
#include <ryusei/navi/potential.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include "common.hpp"

using std::cout;
using std::endl;
using std::vector;
using std::stod;
namespace rs = project_ryusei;

#define OBSTACLE_INFO 0

int main(int argc, char **argv)
{
  if(argc < 7){
    cout << "Error : Usage is... " << argv[0] << " [config file] [pose file] [point file] [goal_x] [goal_y] [goal_yaw]" << endl;
    return -1;
  }

  /*** LocalMapの構築 ***/
  rs::LocalMap local_map;
  rs::loadLocalMap(argv[2], argv[3], local_map);

  /*** クラスのインスタンスを生成 ***/
  rs::ObstacleDetector detector;
  rs::Potential potential;
  
  /*** ObstacleDetectorの実行 ***/
  vector<rs::Obstacle> obstacles;
  if(!detector.init(argv[1]))
  {
    cout << "Failed to load obstacle config file" << endl;
    return -1;
  }
  detector.visualizeLocalMap(local_map);
  detector.detect(local_map.pose, obstacles);

  /*** 最適方向の計算 ***/
  rs::Pose2D goal(stod(argv[4]), stod(argv[5]), stod(argv[6]));
  if(!potential.init(argv[1]))
  {
    cout << "Failed to load potential config file" << endl;
    return -1;
  }
  double target_dir = potential.findOptimalWay(local_map.pose, obstacles, goal);

  /*** 描画ウィンドウを生成しマップ画像を表示 ***/
  cv::namedWindow("detected result");
  cv::imshow("detected result", detector.img_);
  /*** ウィンドウ上でキーが押されるまで待機 ***/
  cv::waitKey(0);

  /*** 障害物の位置座標を表示 ***/
  #if OBSTACLE_INFO
  for(int i = 0; i < obstacles.size(); i++){
    cout << "[" << i << "]" << endl;
    cout << obstacles[i] << endl;
  }
  #endif

  /*** 進むべき方向の表示 ***/
  cout << "Target Direction : " << target_dir << endl;
  
  return 0;
}