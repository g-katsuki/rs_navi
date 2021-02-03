#ifndef _RS_NAVI_POTENTIAL_H_
#define _RS_NAVI_POTENTIAL_H_

#include <ryusei/common/defs.hpp>

namespace project_ryusei
{

class Potential{
public:
  Potential();
  ~Potential();
  bool init(const std::string &conf_path);
  double findOptimalWay(const Pose2D &pose, const std::vector<Obstacle> &obstacles, const Pose2D &goal);
  void createPotentialMap(const Pose2D &pose, const std::vector<Obstacle> &obstacles, const Pose2D &goal, 
                          const float &range_x, const float &range_y, const float &unit, cv::Mat &potential_map);
private:
  void findLinePotential(const cv::Point2f &point1, const cv::Point2f &point2);
  void findRectPotential(const cv::Point2f &point1, const cv::Point2f &point2, const cv::Point2f &point3,
                         const cv::Point2f &point4);
  double calcPotential(const Pose2D &pose, const Pose2D &dst_pose, const std::vector<cv::Point2f> &obstacles);
  double OBS_WEIGHT_, DST_WEIGHT_;
  Pose2D dst_pose_; 
  std::vector<cv::Point2f> potential_points_;
};

}

#endif