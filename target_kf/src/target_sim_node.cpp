#include <ros/ros.h>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <target_kf/target_kf.h>
#include <fstream>
#include <iostream>
#include <target_kf/kfvis.hpp>
#include <predict_trajectory_msgs/PredictTrajectory.h>
#include <random>
#include <chrono>

ros::Publisher target_predict_odom_pub_, ekf_ctrv_traj_pub_, ekf_cv_traj_pub_, imm_traj_pub_, final_traj_pub_;
shared_ptr<EKF_CTRV> ekf_ctrv_ptr_;
shared_ptr<EKF_CV> ekf_cv_ptr_;
shared_ptr<IMM> imm_ptr_;
ros::Time last_update_stamp_;
Vector3d odom_p_ = Vector3d::Zero();
deque<Vector3d> history_pos_;
deque<vector<Vector3d>> history_ctrv_pretraj_, history_cv_pretraj_, history_imm_pretraj_, history_final_pretraj_;
ofstream file_pre_ade_("/home/chai/catkin_ws_tracker2/log/predict_ade.txt");
ofstream file_pre_fde_("/home/chai/catkin_ws_tracker2/log/predict_fde.txt");
ofstream file_odom_("/home/chai/catkin_ws_tracker2/log/odom.txt");
bool is_lost_ = false;
std::shared_ptr<visualization::Visualization> visPtr_;


predict_trajectory_msgs::PredictTrajectory convertTraj(const vector<Vector3d>& predict_traj){
  predict_trajectory_msgs::PredictTrajectory traj;
  traj.header.frame_id = "world";
  traj.header.stamp = ros::Time::now();
  for (const auto &p : predict_traj) {
    geometry_msgs::Point point;
    point.x = p(0);
    point.y = p(1);
    point.z = p(2);
    traj.positions.push_back(point);
  }
  return traj;
}

double calADE(const vector<Vector3d>& predict_traj, const deque<Vector3d>& actual_traj) {
  double total_error = 0.0;
  int count = 0;

  for (size_t i = 0; i < predict_traj.size(); ++i) {
    // 计算预测点和实际点之间的欧式距离
    if (i < actual_traj.size()) {
      Vector3d error = predict_traj[i] - actual_traj[i];
      total_error += sqrt(error.dot(error));
      count++;
    }
  }

  return count > 0 ? total_error / count : 0.0;
}

double calADE(const vector<Vector3d>& predict_traj, const vector<Vector3d>& actual_traj) {
  double total_error = 0.0;
  int count = 0;

  for (size_t i = 0; i < predict_traj.size(); ++i) {
    // 计算预测点和实际点之间的欧式距离
    if (i < actual_traj.size()) {
      Vector3d error = predict_traj[i] - actual_traj[i];
      total_error += sqrt(error.dot(error));
      count++;
    }
  }

  return count > 0 ? total_error / count : 0.0;
}

double calFDE(const vector<Vector3d>& predict_traj, const deque<Vector3d>& actual_traj) {
  Vector3d error;
  for (size_t i = 0; i < predict_traj.size(); ++i) {
    if (i < actual_traj.size()) {
      error = predict_traj[i] - actual_traj[i];
    }
  }

  return error.norm();
}

double calFDE(const vector<Vector3d>& predict_traj, const vector<Vector3d>& actual_traj) {
  Vector3d error;
  for (size_t i = 0; i < predict_traj.size(); ++i) {
    if (i < actual_traj.size()) {
      error = predict_traj[i] - actual_traj[i];
    }
  }

  return error.norm();
}

void targetOdomCallback(const nav_msgs::OdometryConstPtr& odom_msg) {
  Quaterniond odom_q;
  odom_p_(0) = odom_msg->pose.pose.position.x;
  odom_p_(1) = odom_msg->pose.pose.position.y;
  odom_p_(2) = odom_msg->pose.pose.position.z;

  if (!imm_ptr_->is_initialized_) {
    // init
    imm_ptr_->reset(odom_p_);
    ekf_ctrv_ptr_->reset(odom_p_);
    ekf_cv_ptr_->reset(odom_p_);
    ROS_WARN("KF Initialization finished");  
  } else {
    double update_dt = (ros::Time::now() - last_update_stamp_).toSec();
    if (update_dt > 0.05) {
      is_lost_ = true;
      ROS_WARN("target lost!!");
    } else {
      is_lost_ = false;
    }
    if (update_dt > 3.0) {
      imm_ptr_->reset(odom_p_);
      ekf_ctrv_ptr_->reset(odom_p_);
      ekf_cv_ptr_->reset(odom_p_); 
      ROS_WARN("reset!");
    }
  }
  last_update_stamp_ = ros::Time::now();
  return;
}

void predictStateCallback(const ros::TimerEvent& event) {
  if (imm_ptr_->is_initialized_) {
    double update_dt = (ros::Time::now() - last_update_stamp_).toSec();
    if (update_dt > 2.0) {
      imm_ptr_->reset(odom_p_);
      ekf_ctrv_ptr_->reset(odom_p_);
      ekf_cv_ptr_->reset(odom_p_);
      ROS_WARN("reset!");
    } else {
      unsigned seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
      std::default_random_engine generator(seed);
      std::normal_distribution<double> distribution(0.0, 0.0);
      Vector3d odom_p_noise = odom_p_;
      odom_p_noise(0) += distribution(generator);
      odom_p_noise(1) += distribution(generator);
      odom_p_noise(2) += distribution(generator);
      ros::Time t0 = ros::Time::now();
      ekf_ctrv_ptr_->prediction();
      ekf_cv_ptr_->prediction();
      ekf_ctrv_ptr_->update(odom_p_noise);
      ekf_cv_ptr_->update(odom_p_noise);
      vector<vector<Vector3d>> predict_trajs = imm_ptr_->immFilter(odom_p_noise , is_lost_);
      vector<Vector3d> ctrv_traj = ekf_ctrv_ptr_->predict_traj();
      vector<Vector3d> cv_traj = ekf_cv_ptr_->predict_traj();
      visPtr_->visualize_dashed_path(ctrv_traj, "ctrv_traj", visualization::Color::greenblue, 0.8);
      visPtr_->visualize_dashed_path(cv_traj, "cv_traj", visualization::Color::red, 0.8);
      visPtr_->visualize_dashed_path(predict_trajs[0], "imm_ctrv_traj", visualization::Color::blue, 0.8);
      visPtr_->visualize_dashed_path(predict_trajs[1], "imm_cv_traj", visualization::Color::green, 0.8);
      visPtr_->visualize_dashed_path(predict_trajs[2], "imm_traj", visualization::Color::orange, 0.8);
      // ros::Time t1 = ros::Time::now();
      // cout << "predict time(ms): " << ((t1 - t0).toSec() * 1e3) << endl;
      

      // ---------- getOdom ----------
      if (!file_pre_fde_.is_open() || !file_pre_ade_.is_open()) {
        ROS_ERROR("Can't open file");
      } else {
        file_odom_ << "ctrv_odom:[" << abs(odom_p_.x() - ekf_ctrv_ptr_->getPos().x()) << "," <<  abs(odom_p_.y() - ekf_ctrv_ptr_->getPos().y()) << "," <<  abs(odom_p_.z() - ekf_ctrv_ptr_->getPos().z()) << "] "
                   << "cv_odom:[" <<  abs(odom_p_.x() - ekf_cv_ptr_->getPos().x()) << "," <<  abs(odom_p_.y() - ekf_cv_ptr_->getPos().y()) << "," <<  abs(odom_p_.z() - ekf_cv_ptr_->getPos().z()) << "] "
                   << "imm_odom:[" <<  abs(odom_p_.x() - imm_ptr_->getPos().x()) << "," <<  abs(odom_p_.y() - imm_ptr_->getPos().y()) << "," <<  abs(odom_p_.z() - imm_ptr_->getPos().z()) << "] " << endl;         
      }

      // ---------- calADE && FDE ----------
      history_pos_.push_back(odom_p_);
      history_ctrv_pretraj_.push_back(ctrv_traj);
      history_cv_pretraj_.push_back(cv_traj);
      history_imm_pretraj_.push_back(predict_trajs[2]);
      vector<Vector3d> history_1, history_2, history_3, history_4;
      vector<Vector3d> ctrv_1, ctrv_2, ctrv_3, ctrv_4;
      vector<Vector3d> cv_1, cv_2, cv_3, cv_4;
      vector<Vector3d> imm_1, imm_2, imm_3, imm_4;
      
      if (history_pos_.size() > 40) {
        // predict_ade && predict_fde
        history_1.clear();history_2.clear();history_3.clear();history_4.clear();
        ctrv_1.clear();ctrv_2.clear();ctrv_3.clear();ctrv_4.clear();
        cv_1.clear();cv_2.clear();cv_3.clear();cv_4.clear();
        imm_1.clear();imm_2.clear();imm_3.clear();imm_4.clear();

        for (int i = 0; i < history_pos_.size(); ++i) {
          if (i < 10) {
            history_1.push_back(history_pos_[i]);
            ctrv_1.push_back(history_ctrv_pretraj_[0][i]);
            cv_1.push_back(history_cv_pretraj_[0][i]);
            imm_1.push_back(history_imm_pretraj_[0][i]);
          } else if (i >= 10 && i < 20) {
            history_2.push_back(history_pos_[i]);
            ctrv_2.push_back(history_ctrv_pretraj_[0][i]);
            cv_2.push_back(history_cv_pretraj_[0][i]);
            imm_2.push_back(history_imm_pretraj_[0][i]);
          } else if (i >= 20 && i < 30) {
            history_3.push_back(history_pos_[i]);
            ctrv_3.push_back(history_ctrv_pretraj_[0][i]);
            cv_3.push_back(history_cv_pretraj_[0][i]);
            imm_3.push_back(history_imm_pretraj_[0][i]);
          } else if (i >= 30 && i < 40) {
            history_4.push_back(history_pos_[i]);
            ctrv_4.push_back(history_ctrv_pretraj_[0][i]);
            cv_4.push_back(history_cv_pretraj_[0][i]);
            imm_4.push_back(history_imm_pretraj_[0][i]);
          } else {
            break;
          }
        }

        if (!file_pre_fde_.is_open() || !file_pre_ade_.is_open()) {
          ROS_ERROR("Can't open file");
        } else {
          double ade_ctrv = calADE(ctrv_1, history_1);
          double ade_cv = calADE(cv_1, history_1);
          double ade_imm = calADE(imm_1, history_1);
          double fde_ctrv = calFDE(ctrv_1, history_1);
          double fde_cv = calFDE(cv_1, history_1);
          double fde_imm = calFDE(imm_1, history_1);
          file_pre_ade_ << "Time: 0.5" << " CTRV_ADE: " << ade_ctrv << "  CV_ADE: " << ade_cv << "  IMM_ADE: " << ade_imm << endl;
          file_pre_fde_ << "Time: 0.5" << " CTRV_FDE: " << fde_ctrv << "  CV_FDE: " << fde_cv << "  IMM_FDE: " << fde_imm << endl;
        
          ade_ctrv = calADE(ctrv_2, history_2);
          ade_cv = calADE(cv_2, history_2);
          ade_imm = calADE(imm_2, history_2);
          fde_ctrv = calFDE(ctrv_2, history_2);
          fde_cv = calFDE(cv_2, history_2);
          fde_imm = calFDE(imm_2, history_2);
          file_pre_ade_ << "Time: 1.0" << " CTRV_ADE: " << ade_ctrv << "  CV_ADE: " << ade_cv << "  IMM_ADE: " << ade_imm << endl;
          file_pre_fde_ << "Time: 1.0" << " CTRV_FDE: " << fde_ctrv << "  CV_FDE: " << fde_cv << "  IMM_FDE: " << fde_imm << endl;

          ade_ctrv = calADE(ctrv_3, history_3);
          ade_cv = calADE(cv_3, history_3);
          ade_imm = calADE(imm_3, history_3);
          fde_ctrv = calFDE(ctrv_3, history_3);
          fde_cv = calFDE(cv_3, history_3);
          fde_imm = calFDE(imm_3, history_3);
          file_pre_ade_ << "Time: 1.5" << " CTRV_ADE: " << ade_ctrv << "  CV_ADE: " << ade_cv << "  IMM_ADE: " << ade_imm << endl;
          file_pre_fde_ << "Time: 1.5" << " CTRV_FDE: " << fde_ctrv << "  CV_FDE: " << fde_cv << "  IMM_FDE: " << fde_imm << endl;

          ade_ctrv = calADE(ctrv_4, history_4);
          ade_cv = calADE(cv_4, history_4);
          ade_imm = calADE(imm_4, history_4);
          fde_ctrv = calFDE(ctrv_4, history_4);
          fde_cv = calFDE(cv_4, history_4);
          fde_imm = calFDE(imm_4, history_4);
          file_pre_ade_ << "Time: 2.0" << " CTRV_ADE: " << ade_ctrv << "  CV_ADE: " << ade_cv << "  IMM_ADE: " << ade_imm << endl;
          file_pre_fde_ << "Time: 2.0" << " CTRV_FDE: " << fde_ctrv << "  CV_FDE: " << fde_cv << "  IMM_FDE: " << fde_imm << endl;
        }

        history_pos_.pop_front();
        history_ctrv_pretraj_.pop_front();
        history_cv_pretraj_.pop_front();
        history_imm_pretraj_.pop_front();
      }


      // ---------- publish traj && target odom ----------
      final_traj_pub_.publish(convertTraj(predict_trajs[2]));

      nav_msgs::Odometry target_odom;
      target_odom.header.stamp = ros::Time::now();
      target_odom.header.frame_id = "world";
      target_odom.pose.pose.position.x = imm_ptr_->x_.x();
      target_odom.pose.pose.position.y = imm_ptr_->x_.y();
      target_odom.pose.pose.position.z = imm_ptr_->x_.z();
      target_odom.twist.twist.linear.x = imm_ptr_->x_(3) * cos(imm_ptr_->x_(4));
      target_odom.twist.twist.linear.y = imm_ptr_->x_(3) * sin(imm_ptr_->x_(4));
      target_odom.twist.twist.linear.z = imm_ptr_->x_(6);
      target_predict_odom_pub_.publish(target_odom);
    } 
  }

  return;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "target_ukf");
  ros::NodeHandle nh("~");
  last_update_stamp_ = ros::Time::now();
  visPtr_ = std::make_shared<visualization::Visualization>(nh);

  int kf_rate = 30;
  imm_ptr_ = make_shared<IMM>(1.0 / kf_rate, 2.0);
  ekf_ctrv_ptr_ = make_shared<EKF_CTRV>(1.0 / kf_rate, 2.0);
  ekf_cv_ptr_ = make_shared<EKF_CV>(1.0 / kf_rate, 2.0);

  // Timer
  ros::Timer target_predict_timer_ = nh.createTimer(ros::Duration(1.0 / kf_rate), &predictStateCallback);

  // Sub
  ros::Subscriber target_odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 10, &targetOdomCallback, ros::TransportHints().tcpNoDelay());

  // Pub
  target_predict_odom_pub_ = nh.advertise<nav_msgs::Odometry>("target_odom", 1);
  final_traj_pub_ = nh.advertise<predict_trajectory_msgs::PredictTrajectory>("final_traj", 10);
  
  ros::spin();
  
  // file_ade_.close();
  // file_fde_.close();

  return 0;
}
