#include <ryusei/navi/obstacle_detector.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include "common.hpp"

using std::cout;
using std::endl;
using std::vector;
namespace rs = project_ryusei;

#define SHOW_RESULT 0  // 矩形と直線で表された障害物を表示

int main(int argc, char **argv)
{
  if(argc < 4){
    cout << "Error : Usage is... " << argv[0] << " [config file] [pose file] [point file]" << endl;
    return -1;
  }
  /*** LocalMapの構築 ***/
  rs::LocalMap local_map;
  rs::loadLocalMap(argv[2], argv[3], local_map);
  /*** ObstacleDetectorの実行 ***/
  rs::ObstacleDetector detector;
  vector<rs::Obstacle> obstacles;
  if(!detector.init(argv[1]))
  {
    cout << "Failed to load config file" << endl;
    return -1;
  }
  detector.visualizeLocalMap(local_map);
  detector.detect(local_map.pose, obstacles);
  #if SHOW_RESULT
  /*** 描画ウィンドウを生成しマップ画像を表示 ***/
  cv::namedWindow("detected result");
  cv::imshow("detected result", detector.img_);
  #endif
  /*** ウィンドウ上でキーが押されるまで待機 ***/
  cv::waitKey(0);
  /*** 描画用のウィンドウを破棄 ***/
  cv::destroyWindow("detected result");
  /*** 障害物の位置座標を表示 ***/
  for(int i = 0; i < obstacles.size(); i++){
    cout << "[" << i << "]" << endl;
    cout << obstacles[i] << endl;
  }
  return 0;
}