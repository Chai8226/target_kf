#ifndef TARGET_KF_H
#define TARGET_KF_H

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "iostream"
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <deque>

using namespace Eigen;
using namespace std;


Quaterniond euler2quaternion(const Vector3d& euler) {
  double cr = cos(euler(0) / 2);
  double sr = sin(euler(0) / 2);
  double cp = cos(euler(1) / 2);
  double sp = sin(euler(1) / 2);
  double cy = cos(euler(2) / 2);
  double sy = sin(euler(2) / 2);
  Quaterniond q;
  q.w() = cr * cp * cy + sr * sp * sy;
  q.x() = sr * cp * cy - cr * sp * sy;
  q.y() = cr * sp * cy + sr * cp * sy;
  q.z() = cr * cp * sy - sr * sp * cy;
  return q;
}
Vector3d quaternion2euler(const Quaterniond& q) {
  Matrix3d m = q.toRotationMatrix();
  Vector3d rpy;
  rpy.x() = atan2(m(2, 1), m(2, 2));
  rpy.y() = asin(-m(2, 0));
  rpy.z() = atan2(m(1, 0), m(0, 0));
  return rpy;
}

// x=[px, py, pz, v, \theta, \omega, vz]
struct EKF_CTRV {
    double dt_, predict_time_;
    MatrixXd JA_, H_;
    MatrixXd G_, Q_, Qc_, R_;
    MatrixXd P_, K_, S_;
    VectorXd x_, z_diff_, x_pre_;
    double std_x_, std_y_, std_z_;
    double std_a_, std_w_, std_az_;

    int num_steps_;

    EKF_CTRV(){};
    EKF_CTRV(double _dt, double _predict_time) : dt_(_dt), predict_time_(_predict_time) {
        num_steps_ = predict_time_ / dt_;

        x_.setZero(7);
        z_diff_.setZero(3);

        // noise variance
        // std_x_ = 0.1;
        // std_y_ = 0.1;
        // std_z_ = 0.1;
        // std_az_ = 0.2;
        // std_a_ = 0.5;
        // std_w_ = 0.1;
        std_x_ = 0.05;
        std_y_ = 0.05;
        std_z_ = 0.05;
        std_az_ = 1.0;
        std_a_ = 1.0;
        std_w_ = 0.2;
        
        // state covariance matrix
        P_.setIdentity(7, 7);
        
        // H_: state space -> measurement space
        H_.setZero(3, 7);
        H_(0, 0) = 1.0;
        H_(1, 1) = 1.0;
        H_(2, 2) = 1.0;

        // measurement noise covariance matrix
        S_.setZero(3, 3);
        R_.setZero(3, 3);
        Qc_.setZero(3, 3);
        R_(0, 0) = std_x_ * std_x_;
        R_(1, 1) = std_y_ * std_y_;
        R_(2, 2) = std_z_ * std_z_;
        Qc_(0, 0) = std_a_ * std_a_;
        Qc_(1, 1) = std_az_ * std_az_;
        Qc_(2, 2) = std_w_ * std_w_;

        // CV noise covariance matrix
        Q_.setZero(7, 7);
        G_.setZero(7, 3);

        // Jacobi
        JA_.setIdentity(7, 7);
        
        // Kalman gain
        K_.setZero(7, 3);
    }
    
    // State transition function
    inline VectorXd ctrvTransition(const VectorXd& x) {
        VectorXd x_pre = x;
        if (abs(x(5)) > 0.001) {
            x_pre(0) = x(0) + x(3) * (sin(x(5) * dt_ + x(4)) - sin(x(4))) / x(5);
            x_pre(1) = x(1) + x(3) * (-cos(x(5) * dt_ + x(4)) + cos(x(4))) / x(5);
            x_pre(2) = x(2) + x(6) * dt_;
            x_pre(3) = x(3);
            x_pre(4) = x(4) + x(5) * dt_;
            x_pre(5) = x(5);
            x_pre(6) = x(6);
        } else {
            x_pre(0) = x(0) + x(3) * dt_ * cos(x(4));
            x_pre(1) = x(1) + x(3) * dt_ * sin(x(4));
            x_pre(2) = x(2) + x(6) * dt_;
            x_pre(3) = x(3);
            x_pre(4) = x(4);
            x_pre(5) = x(5);
            x_pre(6) = x(6);
        }
        controlAngle(x_pre(4));

        return x_pre;
    }

    inline VectorXd ctrvTransitionWithNoise(const VectorXd& x, const double& a, const double& dw) {
        VectorXd x_pre = x;
        double na = a;
        double ndw = dw;
        
        if (abs(x(3)) < 0.2 && abs(x(3)) < abs(a)) {
            na = ndw = x(3);
        }

        if (abs(x(5)) > 0.001) {
            x_pre(0) = x(0) + x(3) * (sin(x(5) * dt_ + x(4)) - sin(x(4))) / x(5) + 0.5 * na * dt_ * dt_ * cos(x(4));
            x_pre(1) = x(1) + x(3) * (-cos(x(5) * dt_ + x(4)) + cos(x(4))) / x(5) + 0.5 * na * dt_ * dt_ * sin(x(4));
            x_pre(2) = x(2) + x(6) * dt_;
            x_pre(3) = (x(3) + na * dt_) > 2.0 ? 2.0 : ((x(3) + na * dt_) < -2.0 ? -2.0 : (x(3) + na * dt_));
            x_pre(4) = x(4) + x(5) * dt_ + 0.5 * ndw * dt_ * dt_;
            x_pre(5) = x(5) + ndw * dt_;
            x_pre(6) = x(6);
        } else {
            x_pre(0) = x(0) + x(3) * dt_ * cos(x(4)) + 0.5 * na * dt_ * dt_ * cos(x(4));
            x_pre(1) = x(1) + x(3) * dt_ * sin(x(4)) + 0.5 * na * dt_ * dt_ * sin(x(4));
            x_pre(2) = x(2) + x(6) * dt_;
            x_pre(3) = (x(3) + na * dt_) > 2.0 ? 2.0 : ((x(3) + na * dt_) < -2.0 ? -2.0 : (x(3) + na * dt_));
            x_pre(4) = x(4) + x(5) * dt_ + 0.5 * ndw * dt_ * dt_;
            x_pre(5) = x(5) + ndw * dt_;
            x_pre(6) = x(6);
        }
        controlAngle(x_pre(4));

        return x_pre;
    }

    inline void controlAngle(double& psi) {
        while (psi > M_PI) psi -= 2 * M_PI;
        while (psi < -M_PI) psi += 2 * M_PI;

        return;
    }

    inline void calJacobi() {
        if (abs(x_(5)) > 0.001) {
            double temp = x_(5) * dt_ + x_(4);  // wdt + \theta
            JA_(0, 3) = (sin(temp) - sin(x_(4))) / x_(5);
            JA_(0, 4) = x_(3) / x_(5) * (cos(temp) - cos(x_(4)));
            JA_(0, 5) = x_(3) * dt_ * cos(temp) / x_(5) - x_(3) * (sin(temp) - sin(x_(4))) / (x_(5) * x_(5));
            JA_(1, 3) = (-cos(temp) + cos(x_(4))) / x_(5);
            JA_(1, 4) = x_(3) * (sin(temp) - sin(x_(4))) / x_(5);
            JA_(1, 5) = x_(3) * dt_ * sin(temp)  / x_(5) - x_(3) * (-cos(temp) + cos(x_(4))) / (x_(5) * x_(5));
            JA_(2, 6) = dt_;
            JA_(4, 5) = dt_;
        } else {
            JA_(0, 3) = dt_ * cos(x_(4));
            JA_(0, 4) = -1 * x_(3) * dt_ * sin(x_(4));
            JA_(1, 3) = dt_ * sin(x_(4));
            JA_(1, 4) = x_(3) * dt_ * cos(x_(4));
            JA_(2, 6) = dt_;
            JA_(4, 5) = dt_;
        }
        
        return;
    }

    inline void calQ() {
        G_(0, 0) = 0.5 * dt_ * dt_ * cos(x_(4));
        G_(1, 0) = 0.5 * dt_ * dt_ * sin(x_(4));
        G_(2, 1) = 0.5 * dt_ * dt_;
        G_(3, 0) = dt_;
        G_(4, 2) = 0.5 * dt_ * dt_;
        G_(5, 2) = dt_;
        G_(6, 1) = dt_;

        Q_ = G_ * Qc_ * G_.transpose();
    }

    inline void prediction() {
        x_pre_ = ctrvTransition(x_);
        controlAngle(x_(4));
        calJacobi();
        calQ();
        P_ = JA_ * P_ * JA_.transpose() + Q_;
        return;
    }

    inline bool update(const Vector3d& z) {
        S_ = H_ * P_ * H_.transpose() + R_;
        K_ = P_ * H_.transpose() * S_.inverse();
        VectorXd x_tmp = x_pre_ + K_ * (z - H_ * x_pre_);
        
        // NOTE check valid
        static double vmax = 3.0;
        if (x_tmp(2) > vmax) {
            return false;
        }

        z_diff_ = z - H_ * x_pre_;
        x_ = x_pre_ + K_ * z_diff_;
        controlAngle(x_(4));
        P_ = P_ - K_ * H_ * P_;

        return true;
    }
    
    inline vector<Vector3d> predict_traj () {
        vector<Vector3d> traj;
        VectorXd x_temp = x_;
        traj.push_back(Vector3d(x_temp(0), x_temp(1), x_temp(2)));

        for (int i = 1; i < num_steps_ + 1; ++i) {
            x_temp = ctrvTransition(x_temp);
            traj.push_back(Vector3d(x_temp(0), x_temp(1), x_temp(2)));
        }

        return traj;
    }

    inline void reset(const Vector3d& z) {
        x_.setZero();
        x_.head(3) = z;
        P_ << std_x_ * std_x_, 0, 0, 0, 0, 0, 0,
              0, std_y_ * std_y_, 0, 0, 0, 0, 0,
              0, 0, std_z_ * std_z_, 0, 0, 0, 0,
              0, 0, 0, 1, 0, 0, 0,
              0, 0, 0, 0, 1, 0, 0,
              0, 0, 0, 0, 0, 1, 0,
              0, 0, 0, 0, 0, 0, 1;
    }
    
    inline Vector3d getPos() {
        return x_.head(3);
    }
};

// x=[px, py, pz, v, \theta, \omega=0, vz]
struct EKF_CV {
    double dt_, predict_time_;
    MatrixXd JA_, H_;
    MatrixXd G_, Q_, Qc_, R_;
    MatrixXd P_, K_, S_;
    VectorXd x_, z_diff_, x_pre_;
    double std_x_, std_y_, std_z_;
    double std_a_, std_w_, std_az_;

    deque<MatrixXd> gammas_;
    MatrixXd C_, Qc_hat_;
    double w_;
    int num_steps_;

    EKF_CV(){};
    EKF_CV(double _dt, double _predict_time) : dt_(_dt), predict_time_(_predict_time) {
        num_steps_ = predict_time_ / dt_;
        x_.setZero(7);
        z_diff_.setZero(3);

        // noise variance
        std_x_ = 0.05;
        std_y_ = 0.05;
        std_z_ = 0.05;
        std_az_ = 1.0;
        std_a_ = 1.0;
        std_w_ = 0.2;
        
        // state covariance matrix
        P_.setIdentity(7, 7);
        
        // H_: state space -> measurement space
        H_.setZero(3, 7);
        H_(0, 0) = 1.0;
        H_(1, 1) = 1.0;
        H_(2, 2) = 1.0;

        // measurement noise covariance matrix
        S_.setZero(3, 3);
        R_.setZero(3, 3);
        Qc_.setZero(3, 3);
        R_(0, 0) = std_x_ * std_x_;
        R_(1, 1) = std_y_ * std_y_;
        R_(2, 2) = std_z_ * std_z_;
        Qc_(0, 0) = std_a_ * std_a_;
        Qc_(1, 1) = std_az_ * std_az_;
        Qc_(2, 2) = std_w_ * std_w_;

        // CV noise covariance matrix
        Q_.setZero(7, 7);
        G_.setZero(7, 3);

        // Jacobi
        JA_.setIdentity(7, 7);
        
        // Kalman gain
        K_.setZero(7, 3);
    }
    
    // State transition function
    inline VectorXd ctrvTransition(const VectorXd& x) {
        VectorXd x_pre = x;
        x_pre(0) = x(0) + x(3) * dt_ * cos(x(4));
        x_pre(1) = x(1) + x(3) * dt_ * sin(x(4));
        x_pre(2) = x(2) + x(6) * dt_;
        x_pre(3) = x(3);
        x_pre(4) = x(4);
        x_pre(5) = x(5);
        x_pre(6) = x(6);

        return x_pre;
    }

    inline VectorXd ctrvTransitionWithNoise(const VectorXd& x, const double& a, const double& dw, const int& i) {
        VectorXd x_pre = x;
        double na = a;
        double ndw = dw;
        
        if (abs(na) > 2.0) {
            na = (na > 0) ? 2.0 : -2.0;
        }
        if (abs(x(3)) < 0.2 && abs(x(3)) < abs(a)) {
            na = ndw = (x(3) > 0) ? std::min(x(3), 2.0) : std::max(x(3), -2.0);
        }

        x_pre(0) = x(0) + x(3) * dt_ * cos(x(4)) + 0.5 * na * dt_ * dt_ * cos(x(4));
        x_pre(1) = x(1) + x(3) * dt_ * sin(x(4)) + 0.5 * na * dt_ * dt_ * sin(x(4));
        x_pre(2) = x(2) + x(6) * dt_;
        
        // v < 2
        double updated_v = x(3) + na * dt_;
        if (updated_v > 2.0) updated_v = 2.0;
        else if (updated_v < -2.0) updated_v = -2.0;
        x_pre(3) = updated_v;
        
        x_pre(4) = (i > 1) ? x(4) : x(4) + dw;
        x_pre(5) = x(5);
        x_pre(6) = x(6);

        return x_pre;
    }

    inline void controlAngle(double& psi) {
        while (psi > M_PI) psi -= 2 * M_PI;
        while (psi < -M_PI) psi += 2 * M_PI;

        return;
    }

    inline void calJacobi() {
        JA_(0, 3) = dt_ * cos(x_(4));
        JA_(0, 4) = -1 * x_(3) * dt_ * sin(x_(4));
        JA_(1, 3) = dt_ * sin(x_(4));
        JA_(1, 4) = x_(3) * dt_ * cos(x_(4));
        JA_(2, 6) = dt_;
        JA_(4, 5) = dt_;

        return;
    }

    inline void calQ() {
        G_(0, 0) = 0.5 * dt_ * dt_ * cos(x_(4));
        G_(1, 0) = 0.5 * dt_ * dt_ * sin(x_(4));
        G_(2, 1) = 0.5 * dt_ * dt_;
        G_(3, 0) = dt_;
        G_(4, 2) = 0.5 * dt_ * dt_;
        G_(5, 2) = dt_;
        G_(6, 1) = dt_;

        Q_ = G_ * Qc_ * G_.transpose();
    }

    inline void prediction() {
        x_pre_ = ctrvTransition(x_); 
        controlAngle(x_(4));
        calJacobi();
        calQ();

        P_ = JA_ * P_ * JA_.transpose() + Q_;
        return;
    }

    inline vector<Vector3d> predict_traj () {
        vector<Vector3d> traj;
        VectorXd x_temp = x_;
        traj.push_back(Vector3d(x_temp(0), x_temp(1), x_temp(2)));

        for (int i = 1; i < num_steps_ + 1; ++i) {
            x_temp = ctrvTransition(x_temp);
            traj.push_back(Vector3d(x_temp(0), x_temp(1), x_temp(2)));
        }

        return traj;
    }

    inline bool update(const Vector3d& z) {
        S_ = H_ * P_ * H_.transpose() + R_;
        K_ = P_ * H_.transpose() * S_.inverse();
        VectorXd x_tmp = x_pre_ + K_ * (z - H_ * x_pre_);

        // NOTE check valid
        static double vmax = 3.0;
        if (x_tmp(2) > vmax) {
            return false;
        }

        z_diff_ = z - H_ * x_pre_;
        x_ = x_pre_ + K_ * z_diff_;
        controlAngle(x_(4));
        P_ = P_ - K_ * H_ * P_;

        return true;
    }
  
    inline void reset(const Vector3d& z) {
        x_.setZero();
        x_.head(3) = z;
        P_ << std_x_ * std_x_, 0, 0, 0, 0, 0, 0,
              0, std_y_ * std_y_, 0, 0, 0, 0, 0,
              0, 0, std_z_ * std_z_, 0, 0, 0, 0,
              0, 0, 0, 1, 0, 0, 0,
              0, 0, 0, 0, 1, 0, 0,
              0, 0, 0, 0, 0, 1, 0,
              0, 0, 0, 0, 0, 0, 1;
    }

    inline Vector3d getPos() {
        return x_.head(3);
    }
};

// \tau=[p1, p2, ..., pn]
struct KF_TRAJ {
    double dt_;
    int n_;
    MatrixXd A_, C_;
    MatrixXd Q_, R_;
    MatrixXd P_, K_;
    MatrixXd tau_;

    KF_TRAJ(){};
    KF_TRAJ(int _num, double _dt) : dt_(_dt), n_(_num) {
        A_.setIdentity(n_, n_);
        P_.setZero(n_, n_);
        C_.setIdentity(n_, n_);
        K_.setZero(n_, n_);

        Q_.setIdentity(n_, n_);
        R_.setIdentity(n_, n_);

        Q_ = Q_ * 0.5;
        R_ = R_ * 0.5;

        tau_.setZero(n_, 3);
    }

    inline MatrixXd vectorToMatrix(const vector<Vector3d>& v_traj) {
        MatrixXd m_traj(v_traj.size(), 3);

        for (size_t i = 0; i < v_traj.size(); ++i) {
            m_traj.row(i) = v_traj[i].transpose();
        }

        return m_traj;
    }

    inline void predict() {
        tau_ = A_ * tau_;
        P_ = A_ * P_ * A_.transpose() + Q_;
    }

    inline void update(const vector<Vector3d>& v_traj) {
        MatrixXd z = vectorToMatrix(v_traj);

        K_ = P_ * C_.transpose() * (C_ * P_ * C_.transpose() + R_).inverse();
        tau_ = tau_ + K_ * (z - C_ * tau_);
        P_ = P_ - K_ * C_ * P_;
    }
    
    inline void reset(const Vector3d& z) {
        tau_.setZero(n_, 3);
        P_.setZero(n_, n_);
    }

};

// x=[px, py, pz, v, \theta, \omega=0, vz]
struct IMM {
    // model
    EKF_CTRV ctrv_;
    EKF_CV cv_;
    KF_TRAJ traj_;

    // predict state
    VectorXd x_, x_hat_ctrv_, x_hat_cv_;
    // state covariance matrix / state transition matrix
    MatrixXd P_, PT_, P_hat_ctrv_, P_hat_cv_;
    // fusion correlation coefficient
    MatrixXd lambda_;
    // model credibility
    VectorXd mu_;
    // likelihood value
    VectorXd Hat_;

    vector<Vector3d> last_traj_;

    double dt_, predict_time_;
    int num_steps_, num_lost_;
    bool is_initialized_, is_updated_, init_traj_;

    IMM(double _dt, double _predict_time) : dt_(_dt), predict_time_(_predict_time) {
        num_steps_ = predict_time_ / dt_;
        num_lost_ = 0;

        ctrv_ = EKF_CTRV(dt_, predict_time_);
        cv_ = EKF_CV(dt_, predict_time_);
        traj_ = KF_TRAJ(num_steps_+1, dt_);

        x_.setZero(7);
        P_.setIdentity(7, 7);
        Hat_.setZero(2);

        PT_.setZero(2, 2);
        PT_ << 0.9, 0.1,
               0.1, 0.9;

        // lambda_.setZero(2, 2);
        mu_.setZero(2);
        mu_ << 0.5, 0.5;
        
        is_initialized_ = false;
        is_updated_ = false;
        init_traj_ = false;
    };

    inline vector<Vector3d> matrixToVector3d(const MatrixXd& matrix) {
        vector<Vector3d> traj;

        if (matrix.cols() != 3) {
            ROS_ERROR("matrix's cols must be 3!");
        }

        traj.reserve(matrix.rows());

        for (int i = 0; i < matrix.rows(); ++i) {
            traj.push_back(Vector3d(matrix(i, 0), matrix(i, 1), matrix(i, 2)));
        }

        return traj;
    }

    inline vector<vector<Vector3d>> immFilter(const Vector3d& target_odom, const bool& is_lost) {
        // avoid occlusion
        Vector3d target_p;
        if (is_lost) {
            num_lost_++;
            target_p = last_traj_[num_lost_];
        } else {
            num_lost_ = 0;
            target_p = target_odom;
        }
        
        // mix
        Vector2d c_bar(0, 0);
        c_bar = PT_ * mu_;
        
        Matrix2d mu;
        mu.setZero();
        for (int i = 0; i < 2; ++i) {
            mu.row(i) = (PT_.row(i).array() * mu_(i)) / c_bar.transpose().array();
        }

        Matrix2d mu_t = mu.transpose();
        mu = mu_t;
        VectorXd ctrv_x_pre = ctrv_.x_;
        MatrixXd ctrv_P_pre = ctrv_.P_;
        VectorXd cv_x_pre = cv_.x_;
        MatrixXd cv_P_pre = cv_.P_;

        // ctrv_.x_ = mu(0, 0) * ctrv_x_pre + mu(0, 1) * cv_x_pre;
        // cv_.x_ = mu(1, 0) * ctrv_x_pre + mu(1, 1) * cv_x_pre;
        // ctrv_.P_ = mu(0, 0) * (ctrv_P_pre + (ctrv_.x_ - ctrv_x_pre) * (ctrv_.x_ - ctrv_x_pre).transpose()) + 
        //            mu(0, 1) * (cv_P_pre + (ctrv_.x_ - cv_x_pre) * (ctrv_.x_ - cv_x_pre).transpose());
        // cv_.P_ = mu(1, 0) * (ctrv_P_pre + (cv_.x_ - ctrv_x_pre) * (cv_.x_ - ctrv_x_pre).transpose()) +
        //          mu(1, 1) * (cv_P_pre + (cv_.x_ - cv_x_pre) * (cv_.x_ - cv_x_pre).transpose());
        
        // update
        ctrv_.prediction();
        cv_.prediction();
        ctrv_.update(target_p);
        cv_.update(target_p);
        VectorXd ctrv_diff = target_odom - ctrv_.H_ * ctrv_.x_pre_;
        VectorXd cv_diff = target_odom - cv_.H_ * cv_.x_pre_;

        // cal likelihood value
        Hat_(0) = exp(-0.5 * ctrv_diff.transpose() * ctrv_.S_.inverse() * ctrv_diff) / sqrt(2 * M_PI * 2 * M_PI * ctrv_.S_.determinant());
        Hat_(1) = exp(-0.5 * cv_diff.transpose() * cv_.S_.inverse() * cv_diff) / sqrt(2 * M_PI * 2 * M_PI * cv_.S_.determinant());

        // model credibility update
        double c = c_bar(0) * Hat_(0) + c_bar(1) * Hat_(1);
        mu_(0) = c_bar(0) * Hat_(0) / c;
        mu_(1) = c_bar(1) * Hat_(1) / c;
        cout << "mu_: " << mu_.transpose() << endl;

        // model fusion
        x_ = mu_(0) * ctrv_.x_ + mu_(1) * cv_.x_;
        P_ = mu_(0) * (ctrv_.P_ + (x_ - ctrv_.x_) * (x_ - ctrv_.x_).transpose()) + 
             mu_(1) * (cv_,P_ + (x_ - cv_.x_) * (x_ - cv_.x_).transpose());

        // predict target future trajectory
        vector<vector<Vector3d>> predict_trajs;
        vector<Vector3d> ctrv_traj, cv_traj, imm_traj, final_traj;
        ctrv_traj.resize(num_steps_+1);
        cv_traj.resize(num_steps_+1);
        imm_traj.resize(num_steps_+1);
        ctrv_traj[0] = Vector3d(x_(0), x_(1), x_(2));
        cv_traj[0] = Vector3d(x_(0), x_(1), x_(2));
        imm_traj[0] = Vector3d(x_(0), x_(1), x_(2));
        
        VectorXd x_ctrv = ctrv_.x_;
        VectorXd x_cv = cv_.x_;
        VectorXd x_imm = x_;
        for (int i = 1; i < num_steps_ + 1; ++i) {
            x_ctrv = ctrv_.ctrvTransition(x_ctrv);
            ctrv_traj[i] = Vector3d(x_ctrv(0), x_ctrv(1), x_ctrv(2));

            x_cv = cv_.ctrvTransition(x_cv);
            cv_traj[i] = Vector3d(x_cv(0), x_cv(1), x_cv(2));

            x_imm = mu_(0) * x_ctrv + mu_(1) * x_cv;
            imm_traj[i] = Vector3d(x_imm(0), x_imm(1), x_imm(2));
        }

        if (!init_traj_) {
            traj_.tau_ = traj_.vectorToMatrix(imm_traj);
            final_traj = imm_traj;
            init_traj_ = true;
        } else {
            traj_.predict();
            traj_.update(imm_traj);
            final_traj = matrixToVector3d(traj_.tau_);
        }

        predict_trajs.push_back(ctrv_traj);
        predict_trajs.push_back(cv_traj);
        predict_trajs.push_back(imm_traj);
        predict_trajs.push_back(final_traj);

        last_traj_ = imm_traj;

        return predict_trajs;
    }

    inline void reset(const Vector3d& z) {
        ctrv_.reset(z);
        cv_.reset(z);

        x_.setZero();
        x_.head(3) = z;

        is_initialized_ = true;
    }

    inline Vector3d getPos() {
        return x_.head(3);
    }
};



// ===================== no use ====================//
// x=[px, py, pz, vx, vy, vz]
struct KF_CV {
    double dt;
    MatrixXd A, B, C;
    MatrixXd Qt, Rt;
    MatrixXd Sigma, K;
    VectorXd x;

    // states: x, y, z, vx, vy, vz
    KF_CV(double _dt) : dt(_dt) {
        A.setIdentity(6, 6);
        A(0, 3) = dt;
        A(1, 4) = dt;
        A(2, 5) = dt;

        Sigma.setZero(6, 6);

        B.setZero(6, 3);
        double t2 = dt * dt / 2;
        B(0, 0) = t2;
        B(1, 1) = t2;
        B(2, 2) = t2;
        B(3, 0) = dt;
        B(4, 1) = dt;
        B(5, 2) = dt;

        C.setZero(3, 6);
        C(0, 0) = 1;
        C(1, 1) = 1;
        C(2, 2) = 1;
        
        K.setZero(6, 3);

        Qt.setIdentity(3, 3);
        Rt.setIdentity(3, 3);
        Qt(0, 0) = 4;    // x
        Qt(1, 1) = 4;    // y
        Qt(2, 2) = 1;    // z
        Rt(0, 0) = 0.1;
        Rt(1, 1) = 0.1;
        Rt(2, 2) = 0.1;

        x.setZero(6);
    }
    
    inline vector<Vector3d> predict() {
        x = A * x;
        Sigma = A * Sigma * A.transpose() + B * Qt * B.transpose();

        // predict target future trajectory
        double predict_time = 2.0;
        int num_steps = predict_time / dt;
        VectorXd x_temp = x;
        vector<Vector3d> predict_points;
        predict_points.push_back(Vector3d(x_temp(0), x_temp(1), x_temp(2)));

        for (int i = 0; i < num_steps; ++i) {
            x_temp = A * x_temp;
            predict_points.push_back(Vector3d(x_temp(0), x_temp(1), x_temp(2)));
        }

        return predict_points;
    }

    inline bool update(const Vector3d& z) {
        K = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
        VectorXd x_tmp = x + K * (z - C * x);
        
        // NOTE check valid
        static double vmax = 4;
        if (x_tmp.tail(3).norm() > vmax) {
            return false;
        }
        
        x = x + K * (z - C * x);
        Sigma = Sigma - K * C * Sigma;
        return true;
    }
    
    inline void reset(const Vector3d& z) {
        x.setZero();
        x.head(3) = z;
        Sigma.setZero();
    }

    inline const Vector3d pos() const {
        return x.head(3);
    }

    inline const Vector3d vel() const {
        return x.tail(3);
    }

};

// x=[px, py, v, \theta, \omega=0]
struct UKF_CTRV {
    // initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    double dt_;
    // State/Augmented state/measurement dimension
    int n_x_, n_z_, n_aug_;
    // Sigma point spreading parameter
    double lambda_;

    // Process noise 
    double std_a_;
    double std_yawdd_;
    // Laser measurement noise
    double std_py_, std_px_;
    // normalized innovation squared(NIS)
    double nis_lidar_;
    
    // state vector , measurement state
    VectorXd x_, z_, z_diff_;
    // state covariance matrix
    MatrixXd P_;
    // Kalman gain
    MatrixXd K_;
    // measurement covariance matrix S
    MatrixXd S_;
    // predicted sigma points matrix
    MatrixXd Xsig_pred_;
    // measurement noise covariance matrix
    MatrixXd R_;
    // Weights of sigma points
    VectorXd weights_;

    UKF_CTRV(){};
    UKF_CTRV(double _dt) : dt_(_dt) {
        // initial state vector (x,y,v,\theta,\omega)
        x_ = VectorXd(5);
        // initial covariance matrix
        P_ = MatrixXd(5, 5);
        // Process noise standard deviation longitudinal acceleration in m/s^2
        std_a_ = 1.0;
        // Process noise standard deviation yaw acceleration in rad/s^2
        std_yawdd_ = 0.5;

        /**
         * These are provided by the sensor manufacturer.
        */
        std_px_ = 0.10;
        std_py_ = 0.10;

        // init param
        n_x_ = 5;  // 原状态维度
        n_z_ = 2;  // 测量状态维度
        n_aug_ = 7;  // 增广状态维度（原维度增加噪声-加速度和角加速度）
        Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);  // 预测sigma点集
        
        lambda_ = 3 - n_aug_;  // lambda
        weights_ = VectorXd(2 * n_aug_ + 1);  // sigma点的权重
        weights_.fill(0.5 / (lambda_ + n_aug_));  // w_i = (lambda+n)/2
        weights_(0) = lambda_ / (lambda_ + n_aug_);  // w_1 = lambda/(lambda+n)

        // 添加测量噪声协方差矩阵
        R_ = MatrixXd(2, 2);
        R_.fill(0.0);
        R_(0, 0) = std_px_ * std_px_;
        R_(1, 1) = std_py_ * std_py_;
        
        is_initialized_ = false;
    };

    // State transition function
    inline VectorXd ctrvTransition(const VectorXd& x) {       
        VectorXd x_pre = x;
        if (fabs(x(4)) > 0.001) {
            x_pre(0) += x(2) / x(4) * (sin(x(3) + x(4) * dt_) - sin(x(3)));
            x_pre(1) += x(2) / x(4) * (cos(x(3)) - cos(x(3) + x(4) * dt_));
        } else {
            x_pre(0) += x(2) * dt_ * cos(x(3));
            x_pre(1) += x(2) * dt_ * sin(x(3));
        }
        x_pre(2) = x(2);
        x_pre(3) += x(4) * dt_;
        x_pre(4) = x(4);

        return x_pre;
    }

    inline void reset(const Vector3d& z) {
        x_.setZero();
        x_.head(2) = z.head(2);
        P_ << std_px_ * std_px_, 0, 0, 0, 0,
              0, std_py_ * std_py_, 0, 0, 0,
              0, 0, 1, 0, 0,
              0, 0, 0, 1, 0,
              0, 0, 0, 0, 1;
    }

    inline void prediction() {
        // create augmented mean vector
        VectorXd x_aug = VectorXd(n_aug_);
        // create augmented sigma point matrix
        MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
        Xsig_aug.fill(0.0);
        // create augmented state covariance
        MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
        P_aug.fill(0.0);

        // create augmented mean state
        x_aug.head(n_x_) = x_;
        x_aug(5) = 0;
        x_aug(6) = 0;

        // create augmented covariance matrix and square root matrix
        P_aug.fill(0.0);
        P_aug.topLeftCorner(n_x_, n_x_) = P_;
        P_aug(5, 5) = std_a_ * std_a_;
        P_aug(6, 6) = std_yawdd_ * std_yawdd_;
        MatrixXd L = P_aug.llt().matrixL();

        // generate sigma points
        Xsig_aug.col(0) = x_aug;
        for (int i = 0; i < n_aug_; ++i) {
            Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
            Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
        }
        
        // predict sigma points
        for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
            // extract values for better readability
            double p_x = Xsig_aug(0, i);
            double p_y = Xsig_aug(1, i);
            double v = Xsig_aug(2, i);
            double yaw = Xsig_aug(3, i);
            double yawd = Xsig_aug(4, i);
            double nu_a = Xsig_aug(5, i);
            double nu_yawdd = Xsig_aug(6, i);

            // predicted state values
            double px_p, py_p;

            // avoid division by zero
            if (fabs(yawd) > 0.001) {
                px_p = p_x + v / yawd * (sin(yaw + yawd * dt_) - sin(yaw));
                py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * dt_));
            } else {
                px_p = p_x + v * dt_ * cos(yaw);
                py_p = p_y + v * dt_ * sin(yaw);
            }

            double v_p = v;
            double yaw_p = yaw + yawd * dt_;
            double yawd_p = yawd;

            // add noise
            px_p = px_p + 0.5 * nu_a * dt_ * dt_ * cos(yaw);
            py_p = py_p + 0.5 * nu_a * dt_ * dt_ * sin(yaw);
            v_p = v_p + nu_a * dt_;

            yaw_p = yaw_p + 0.5 * nu_yawdd * dt_ * dt_;
            yawd_p = yawd_p + nu_yawdd * dt_;

            // write predicted sigma point into right column
            Xsig_pred_(0, i) = px_p;
            Xsig_pred_(1, i) = py_p;
            Xsig_pred_(2, i) = v_p;
            Xsig_pred_(3, i) = yaw_p;
            Xsig_pred_(4, i) = yawd_p;
        }

        // 预测均值和协方差矩阵
        x_.fill(0.0);
        for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
            x_ = x_ + weights_(i) * Xsig_pred_.col(i);
        }
        P_.fill(0.0);
        for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
            VectorXd x_diff = Xsig_pred_.col(i) - x_;

            while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
            while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

            P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
        }
        return;
    }

    inline bool update(const Vector3d& z) {
        z_ = Vector2d(z.x(), z.y());
        // create matrix for sigma points in measurement space
        MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);

        // mean predicted measurement
        VectorXd z_pred = VectorXd(n_z_);
        z_pred.fill(0.0);

        // measurement covariance matrix S
        S_ = MatrixXd(n_z_, n_z_);
        S_.fill(0.0);

        // transform sigma points into measurement space
        for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
            // 2n+1 simga points
            // extract values for better readability
            double p_x = Xsig_pred_(0, i);
            double p_y = Xsig_pred_(1, i);

            // measurement model
            Zsig(0, i) = p_x;
            Zsig(1, i) = p_y;
            // mean predicted measurement
            z_pred += weights_(i) * Zsig.col(i);
        }

        // innovation covariance matrix S
        for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
            // 2n+1 simga points
            // residual
            VectorXd z_diff = Zsig.col(i) - z_pred;

            S_ += weights_(i) * z_diff * z_diff.transpose();
        }

        S_ += R_;

        // create matrix for cross correlation Tc
        MatrixXd Tc = MatrixXd(n_x_, n_z_);
        Tc.fill(0.0);
        for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
            // residual
            VectorXd z_diff = Zsig.col(i) - z_pred;
            // state difference
            VectorXd x_diff = Xsig_pred_.col(i) - x_;

            // normalize angles
            while (x_diff(3) > M_PI) x_diff(3) -= 2 * M_PI;
            while (x_diff(3) < -M_PI) x_diff(3) += 2 * M_PI;

            Tc += weights_(i) * x_diff * z_diff.transpose();
        }
        
        //  Kalman gain K_;
        K_ = Tc * S_.inverse();
        
        // residual
        z_diff_ = z_ - z_pred;
        // compute normalized innovation squared(NIS)
        nis_lidar_ = z_diff_.transpose() * S_.inverse() * z_diff_;
        
        VectorXd x_tmp = x_ + K_ * z_diff_;
        static double vmax = 3.0;
        if (x_tmp(2) > vmax) {
            return false;
        }

        // predict state mean and covariance matrix
        x_ += K_ * z_diff_;
        P_ -= K_ * S_ * K_.transpose();

        return true;
    }  
};


#endif  //TARGET_KF_H