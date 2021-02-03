#ifndef _RS_NAVI_OBSTACLE_DETECTOR_H_
#define _RS_NAVI_OBSTACLE_DETECTOR_H_

#include <ryusei/common/defs.hpp>
namespace project_ryusei
{
class ObstacleDetector{
public:
  ObstacleDetector();
  ~ObstacleDetector();
  bool init(const std::string &conf_path);
  void visualizeLocalMap(const LocalMap &map);
  void detect(const Pose2D &pose, std::vector<Obstacle> &obstacles);
  int IMG_SIZE_;
  double UNIT_, RANGE_X_, RANGE_Y_, MAX_RANGE_Z_, MIN_RANGE_Z_, MIN_LINE_LENGTH_, MAX_LINE_GAP_, THORESHOLD_,
         TIMES_DELATE_, TIMES_ERODE_;
  cv::Mat img_, gray_img_;
private:
  void detectRectObjects(const Pose2D &pose, std::vector<Obstacle> &obstacles, Obstacle &result);
  void detectLineObjects(const Pose2D &pose, std::vector<Obstacle> &obstacles, Obstacle &result);
};
}

#endif