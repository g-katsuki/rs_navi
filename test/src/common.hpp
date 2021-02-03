#include <boost/algorithm/string.hpp>

namespace project_ryusei
{

static bool loadLocalMap(const std::string &pose_path, const std::string &point_path, project_ryusei::LocalMap &map)
{
  /*** ログファイルからLocalMapを構築 ***/
  std::string read_str, separator = ",";
  std::vector<std::string> split_str;
  std::ifstream pose_file(pose_path), point_file(point_path);
  if(pose_file.fail() || point_file.fail())
  {
    std::cout << "failed" << std::endl;
    exit(-1);
  }
  /*** x, y, z, roll, pitch, yaw ***/
  while(getline(pose_file, read_str))
  {
    boost::algorithm::split(split_str, read_str, boost::is_any_of(separator));
    if(split_str[0] == "pos")
    {
      map.pose = project_ryusei::Pose2D(stof(split_str[1]), stof(split_str[2]), stof(split_str[6]));
      break;
    }
  }
  while(getline(point_file, read_str))
  {
    boost::algorithm::split(split_str, read_str, boost::is_any_of(separator));
    if(1 < split_str.size()) map.points.emplace_back(cv::Point3f(stof(split_str[0]), stof(split_str[1]), stof(split_str[2])));
  }
  return true;
}

} 