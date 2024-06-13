#ifndef _TARGET_DATA_H_
#define _TARGET_DATA_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

using namespace std;
using std::vector;
using Eigen::Vector3d;

namespace fast_planner {
  // Spatial-Temporal Point Data
  struct STPData {
    Vector3d position;
    double time;
  };

  // Spatial-Temporal Traj Data
  struct STTData {
    vector<STPData> stTraj;
    bool isStatic;
  };

  // Target data
  class TargetData {
    public:
      vector<STTData> saved_trajs_;
      STTData temp_traj_;
      double predict_time_;

      // test
      STTData test_traj_;
      STTData vis_traj_;
      
      
      void targrtTrajProcess(const STTData& temp_traj);
      void compareSpaceTimeSimilarity(const STTData& st_traj);
      double calSpaceTimeSimilarity(const STTData& st_traj1, const STTData& st_traj2);
      double calTimeSimilarity(const STTData& st_traj1, const STTData& st_traj2);
      double calSpaceSimilarity(const STTData& st_traj1, const STTData& st_traj2);
      double calSimilarity(const Vector3d& p1, const Vector3d& p2);
      double trapezoidalIntegration(const vector<double>& times, const vector<double>& similarities);
      Vector3d interpolatePoint(const STPData& start, const STPData& end, double time);
      void getEvenlySpacedPoints(vector<Vector3d>& astar_path, int n);
      
  };

}  // namespace fast_planner
#endif