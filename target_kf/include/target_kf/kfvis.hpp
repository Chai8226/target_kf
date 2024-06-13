#pragma once
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Geometry>
#include <iostream>
#include <unordered_map>

namespace visualization {
using PublisherMap = std::unordered_map<std::string, ros::Publisher>;
enum Color { white,
             red,
             green,
             blue,
             yellow,
             greenblue,
             orange
            };

class Visualization {
 private:
  ros::NodeHandle nh_;
  PublisherMap publisher_map_;

  void setMarkerColor(visualization_msgs::Marker& marker,
                      Color color = blue,
                      double a = 1) {
    marker.color.a = a;
    switch (color) {
      case white:
        marker.color.r = 1;
        marker.color.g = 1;
        marker.color.b = 1;
        break;
      case red:
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        break;
      case green:
        marker.color.r = 0.196;
        marker.color.g = 0.8039;
        marker.color.b = 0;
        break;
      case blue:
        marker.color.r = 0.26275;
        marker.color.g = 0.46275;
        marker.color.b = 1;
        break;
      case yellow:
        marker.color.r = 1;
        marker.color.g = 0.84314;
        marker.color.b = 0;
        break;
      case greenblue:
        marker.color.r = 0;
        marker.color.g = 1;
        marker.color.b = 1;
        break;
      case orange:
        marker.color.r = 1;
        marker.color.g = 0.2706;
        marker.color.b = 0;
        break;
    }
  }

  void setMarkerColor(visualization_msgs::Marker& marker,
                      double a,
                      double r,
                      double g,
                      double b) {
    marker.color.a = a;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
  }

  void setMarkerScale(visualization_msgs::Marker& marker,
                      const double& x,
                      const double& y,
                      const double& z) {
    marker.scale.x = x;
    marker.scale.y = y;
    marker.scale.z = z;
  }

  void setMarkerPose(visualization_msgs::Marker& marker,
                     const double& x,
                     const double& y,
                     const double& z) {
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
  }
  
  template <class ROTATION>
  void setMarkerPose(visualization_msgs::Marker& marker,
                     const double& x,
                     const double& y,
                     const double& z,
                     const ROTATION& R) {
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    Eigen::Quaterniond r(R);
    marker.pose.orientation.w = r.w();
    marker.pose.orientation.x = r.x();
    marker.pose.orientation.y = r.y();
    marker.pose.orientation.z = r.z();
  }

 public:
  Visualization(ros::NodeHandle& nh) : nh_(nh) {}

  template <class CENTER, class TOPIC>
  void visualize_a_ball(const CENTER& c,
                        const double& r,
                        const TOPIC& topic,
                        const Color color = blue,
                        const double a = 1) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10);
      publisher_map_[topic] = pub;
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    setMarkerColor(marker, color, a);
    setMarkerScale(marker, 2 * r, 2 * r, 2 * r);
    setMarkerPose(marker, c[0], c[1], c[2]);
    marker.header.stamp = ros::Time::now();
    publisher_map_[topic].publish(marker);
  }
  
  template <class PATH, class TOPIC>
  void visualize_path(const PATH& path, const TOPIC& topic) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub = nh_.advertise<nav_msgs::Path>(topic, 10);
      publisher_map_[topic] = pub;
    }
    nav_msgs::Path path_msg;
    geometry_msgs::PoseStamped tmpPose;
    tmpPose.header.frame_id = "world";
    for (const auto& pt : path) {
      tmpPose.pose.position.x = pt[0];
      tmpPose.pose.position.y = pt[1];
      tmpPose.pose.position.z = pt[2];
      path_msg.poses.push_back(tmpPose);
    }
    path_msg.header.frame_id = "world";
    path_msg.header.stamp = ros::Time::now();
    publisher_map_[topic].publish(path_msg);
  }

  template <class PATH, class TOPIC>
  void visualize_dashed_path(const PATH& path, const TOPIC& topic, const Color color = blue, const double a = 1) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10);
      publisher_map_[topic] = pub;
    }
    
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "world";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "dashed_lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    
    // Dashed line properties
    line_list.scale.x = 0.15; // Line width
    setMarkerColor(line_list, color, a);

    for (size_t i = 0; i < path.size() - 1;) {
      geometry_msgs::Point p_start;
      p_start.x = path[i][0];
      p_start.y = path[i][1];
      p_start.z = path[i][2];

      geometry_msgs::Point p_end;
      p_end.x = path[i + 3][0];
      p_end.y = path[i + 3][1];
      p_end.z = path[i + 3][2];

      line_list.points.push_back(p_start);
      line_list.points.push_back(p_end);

      i += 5;
    }
    publisher_map_[topic].publish(line_list);
  } 

  template <class TOPIC>
  void visualize_dashed_paths(const Eigen::MatrixXd& matrix, const TOPIC& topic, const Color color = blue, const double a = 1) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
        ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10);
        publisher_map_[topic] = pub;
    }
    
    visualization_msgs::Marker line_list, points;
    
    // 设置线段的属性
    line_list.header.frame_id = "world";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "dashed_lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.05; // Line width

    // 设置点的属性
    points.header = line_list.header;
    points.ns = "end_points";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 1;
    points.type = visualization_msgs::Marker::SPHERE_LIST;
    points.scale.x = 0.08;
    points.scale.y = 0.08;

    setMarkerColor(line_list, color, a);
    setMarkerColor(points, orange, a);

    // 遍历轨迹
    for (int i = 0; i < matrix.rows(); i += 3) {
        // 遍历轨迹中的每个点
        for (int j = 0; j < matrix.cols() - 2; j += 2) {  // 修改间隔以创建虚线效果
            geometry_msgs::Point p_start, p_end;
            p_start.x = matrix(i, j);
            p_start.y = matrix(i + 1, j);
            p_start.z = matrix(i + 2, j);

            p_end.x = matrix(i, j + 1);
            p_end.y = matrix(i + 1, j + 1);
            p_end.z = matrix(i + 2, j + 1);

            line_list.points.push_back(p_start);
            line_list.points.push_back(p_end);
        }

        // 添加末尾点
        geometry_msgs::Point p_end;
        p_end.x = matrix(i, matrix.cols() - 1);
        p_end.y = matrix(i + 1, matrix.cols() - 1);
        p_end.z = matrix(i + 2, matrix.cols() - 1);
        points.points.push_back(p_end);
    }

    publisher_map_[topic].publish(line_list);
    publisher_map_[topic].publish(points);  // 发布末尾点
  }

  template <class BALLS, class TOPIC>
  void visualize_balls(const BALLS& balls,
                       const TOPIC& topic,
                       const Color color = blue,
                       const double a = 0.2) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub =
          nh_.advertise<visualization_msgs::MarkerArray>(topic, 10);
      publisher_map_[topic] = pub;
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    setMarkerColor(marker, color, a);
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.reserve(balls.size() + 1);
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    for (const auto& ball : balls) {
      setMarkerPose(marker, ball[0], ball[1], ball[2]);
      auto d = 2 * ball.r;
      setMarkerScale(marker, d, d, d);
      marker_array.markers.push_back(marker);
      marker.id++;
    }
    publisher_map_[topic].publish(marker_array);
  }

  template <class ELLIPSOID, class TOPIC>
  void visualize_ellipsoids(const ELLIPSOID& ellipsoids,
                            const TOPIC& topic,
                            const Color color = blue,
                            const double a = 0.2) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub =
          nh_.advertise<visualization_msgs::MarkerArray>(topic, 10);
      publisher_map_[topic] = pub;
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    setMarkerColor(marker, color, a);
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.reserve(ellipsoids.size() + 1);
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    for (const auto& e : ellipsoids) {
      setMarkerPose(marker, e.c[0], e.c[1], e.c[2], e.R);
      setMarkerScale(marker, 2 * e.rx, 2 * e.ry, 2 * e.rz);
      marker_array.markers.push_back(marker);
      marker.id++;
    }
    publisher_map_[topic].publish(marker_array);
  }

  template <class PAIRLINE, class TOPIC>
  // eg for PAIRLINE: std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
  void visualize_pairline(const PAIRLINE& pairline, const TOPIC& topic) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10);
      publisher_map_[topic] = pub;
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    setMarkerPose(marker, 0, 0, 0);
    setMarkerColor(marker, greenblue, 1);
    setMarkerScale(marker, 0.02, 0.02, 0.02);
    marker.points.resize(2 * pairline.size());
    for (size_t i = 0; i < pairline.size(); ++i) {
      marker.points[2 * i + 0].x = pairline[i].first[0];
      marker.points[2 * i + 0].y = pairline[i].first[1];
      marker.points[2 * i + 0].z = pairline[i].first[2];
      marker.points[2 * i + 1].x = pairline[i].second[0];
      marker.points[2 * i + 1].y = pairline[i].second[1];
      marker.points[2 * i + 1].z = pairline[i].second[2];
    }
    publisher_map_[topic].publish(marker);
  }

  // v0 -> v1 theta
  template <class TOPIC>
  void visualize_fan_shape_meshes(const std::vector<Eigen::Vector3d>& v0,
                                  const std::vector<Eigen::Vector3d>& v1,
                                  const std::vector<double>& thetas,
                                  const TOPIC& topic) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10);
      publisher_map_[topic] = pub;
    }
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.frame_id = "world";
    marker.id = 0;
    setMarkerPose(marker, 0, 0, 0);
    setMarkerScale(marker, 1, 1, 1);
    setMarkerColor(marker, green, 0.1);
    int M = v0.size();
    // int M = 1;
    for (int i = 0; i < M; ++i) {
      Eigen::Vector3d dp = v1[i] - v0[i];
      double theta0 = atan2(dp.y(), dp.x());
      double r = dp.norm();
      geometry_msgs::Point center;
      center.x = v0[i].x();
      center.y = v0[i].y();
      center.z = v0[i].z();
      geometry_msgs::Point p = center;
      p.x += r * cos(theta0 - thetas[i]);
      p.y += r * sin(theta0 - thetas[i]);
      for (double theta = theta0 - thetas[i] + 0.1; theta < theta0 + thetas[i]; theta += 0.1) {
        marker.points.push_back(center);
        marker.points.push_back(p);
        p = center;
        p.x += r * cos(theta);
        p.y += r * sin(theta);
        marker.points.push_back(p);
      }
    }
    publisher_map_[topic].publish(marker);
  }

};

}  // namespace visualization
