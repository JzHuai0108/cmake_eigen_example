#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <filesystem>
#include <regex>

ros::Publisher pub_pose_;

void odometryCallback_(const nav_msgs::Odometry::ConstPtr msg) {
    geometry_msgs::Pose2D pose2d;
    pose2d.x = msg->pose.pose.position.x;
    pose2d.y = msg->pose.pose.position.y;
    
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    pose2d.theta = yaw;
    pub_pose_.publish(pose2d);
}

void convert() {
    tf::Matrix3x3 m(-1, 0, 0, 0, -1, 0, 0, 0, 1);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    std::cout << "RPY: " << roll << " " << pitch << " " << yaw << "\n";

    // equivalently in python
    // import numpy as np
    // m =  np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    // rpy = tf.transformations.euler_from_matrix(m)
}

std::vector<std::string> find_and_sort_pcd_files(const std::string &pcd_folder) {
  std::vector<std::string> pcd_files;
  for (const auto &entry : std::filesystem::directory_iterator(pcd_folder)) {
      if (entry.path().extension() == ".pcd") {
          pcd_files.push_back(entry.path().string());
      }
  }

  // Sorting function: extracts numeric part of the filename
  auto extract_number = [](const std::string &filename) -> int {
      std::regex re(R"((\d+))"); // Match numeric sequences
      std::smatch match;
      if (std::regex_search(filename, match, re)) {
          return std::stoi(match.str(0)); // Convert first matched number to int
      }
      return -1; // If no number is found, return -1 (sorts it to the beginning)
  };

  std::sort(pcd_files.begin(), pcd_files.end(), [&](const std::string &a, const std::string &b) {
      return extract_number(a) < extract_number(b);
  });
  return pcd_files;
}


int main(int argc, char **argv)
{
    std::string folder = "/media/jhuai/docker/lidarslam/BALM_ws/src/BALM/datas/temp";
    std::vector<std::string> pcds = find_and_sort_pcd_files(folder);
    int i = 0;
    for (const auto &p : pcds) {
        std::cout << i << " " << p << std::endl;
        i++;
    }
    return 0;

    convert();
    ros::init(argc, argv, "conversion_node");
    
    ros::NodeHandle nh_;
    
    ros::Subscriber sub_odom_ = nh_.subscribe("odom", 1, odometryCallback_);
    pub_pose_ = nh_.advertise<geometry_msgs::Pose2D>("pose2d", 1);
    
    ros::spin();
    
    return 0;
}
