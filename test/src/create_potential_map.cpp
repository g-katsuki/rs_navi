#include <ryusei/navi/obstacle_detector.hpp>
#include <ryusei/navi/potential.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include "common.hpp"

using namespace std;
namespace rs = project_ryusei;

#define RANGE_X 2.0
#define RANGE_Y 2.0
#define UNIT 0.1

int main(int argc, char **argv)
{
  if(argc < 8){
    cout << "Error : Usage is... " << argv[0] << " [config file] [pose file] [point file] [goal_x] [goal_y] [goal_yaw] [output_path]" << endl;
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
  detector.init(argv[1]);
  detector.visualizeLocalMap(local_map);
  detector.detect(local_map.pose, obstacles);
  /*** 周囲のポテンシャル場を計算 ***/
  rs::Pose2D goal(stod(argv[4]), stod(argv[5]), stod(argv[6]));
  cv::Mat potential_map;
  potential.init(argv[1]);
  potential.createPotentialMap(local_map.pose, obstacles, goal, RANGE_X, RANGE_Y, UNIT, potential_map);
  /*** 計算したポテンシャルをファイルに保存 ***/
  ofstream ofs(argv[7]);
  /*** ヘッダ部の書き込み ***/
  ofs << "RangeX," << RANGE_X << endl;
  ofs << "RangeY," << RANGE_Y << endl;
  ofs << "Unit," << UNIT << endl << endl;
  /*** ポテンシャル場の書き込み ***/
  for(int i = 0; i < potential_map.rows; i++){
    for(int j = 0; j < potential_map.cols; j++){
      float val = potential_map.at<float>(i, j);
      if(j == potential_map.cols - 1) ofs << val;
      else ofs << val << ",";
    }
    if(i < potential_map.rows - 1) ofs << endl;
  }
  ofs.close();
  return 0;
}