#include <ros/ros.h>
#include <target_kf/target_data.h>


namespace fast_planner {
    
  void TargetData::targrtTrajProcess(const STTData& temp_traj) {
    STTData traj;

    // 1. path plan method 
    if (!temp_traj.isStatic) {
      // (1) Use path planning at collision
      // traj = pathSearch1(temp_traj);
      // (2) If a collision occurs, path planning is directly used for the starting point and end point.
      // traj = pathSearch2(temp_traj);
    } else {
      traj = temp_traj;
    }

    // 2. Spatial-Temporal Probability Estimation
    if (saved_trajs_.empty()) {
      saved_trajs_.push_back(traj);
      ROS_INFO_STREAM_THROTTLE(1.0, "Spatial-Temporal Traj'size is: " << saved_trajs_.size());
    } else {
      compareSpaceTimeSimilarity(traj);
      ROS_INFO_STREAM_THROTTLE(1.0, "Spatial-Temporal Traj'size is: " << saved_trajs_.size());
    }
  }

  void TargetData::compareSpaceTimeSimilarity(const STTData& st_traj) {
    for (auto &saved_st_traj : saved_trajs_) {
      double stscore = calSpaceTimeSimilarity(st_traj, saved_st_traj);
      cout << "SpaceTime Similarity: " << stscore << endl;
      if (stscore < 0.2) {
        saved_trajs_.push_back(st_traj);
        return;
      } else {
        saved_st_traj = st_traj;
      }
    }
  }

  double TargetData::calSpaceTimeSimilarity(const STTData& st_traj1, const STTData& st_traj2) {
    return (calTimeSimilarity(st_traj1, st_traj2) + calSpaceSimilarity(st_traj1, st_traj2)) / 2;
  }

  double TargetData::calTimeSimilarity(const STTData& st_traj1, const STTData& st_traj2) {
    if (st_traj1.isStatic || st_traj2.isStatic) {
      // Handle static trajectory case
    }

    // Find overlapping time interval
    double start_time = max(st_traj1.stTraj.front().time, st_traj2.stTraj.front().time);
    double end_time = min(st_traj1.stTraj.back().time, st_traj2.stTraj.back().time);

    if (start_time >= end_time) return 0.0; // No overlap
    
    vector<double> times;
    vector<double> similarities;

    // Iterate through the overlapping time interval
    for (size_t i = 0, j = 0; (i < st_traj1.stTraj.size() - 1) && (j < st_traj2.stTraj.size() - 1);) {
      // Find the next time interval
      double nextTime = std::min(st_traj1.stTraj[i + 1].time, st_traj2.stTraj[j + 1].time);

      // Only proceed if the next time is within the overlapping interval
      if (nextTime <= start_time || nextTime > end_time) {
        if (st_traj1.stTraj[i + 1].time < st_traj2.stTraj[j + 1].time) ++i;
        else ++j;
        continue;
      }

      // Interpolate points at the next time
      Vector3d interpPoint1 = interpolatePoint(st_traj1.stTraj[i], st_traj1.stTraj[i + 1], nextTime);
      Vector3d interpPoint2 = interpolatePoint(st_traj2.stTraj[j], st_traj2.stTraj[j + 1], nextTime);

      // Calculate similarity
      double sim = calSimilarity(interpPoint1, interpPoint2);

      // Save the times and similarities
      times.push_back(nextTime);
      similarities.push_back(sim);

      // Move to the next interval
      if (st_traj1.stTraj[i + 1].time == nextTime) ++i;
      if (st_traj2.stTraj[j + 1].time == nextTime) ++j;

      // Update the start time
      start_time = nextTime;
    }

    // Calculate the integral using trapezoidal rule
    double integral = trapezoidalIntegration(times, similarities);
    
    // Normalize the integral by the overlapping time duration
    double duration = end_time - times.front();
    return duration > 0 ? integral / duration : 0.0;
  }

  double TargetData::calSpaceSimilarity(const STTData& st_traj1, const STTData& st_traj2) {
    if (st_traj1.isStatic || st_traj2.isStatic) {
      // Handle static trajectory case
    }
    
    // Find the common time interval between two trajectories
    double start_time = max(st_traj1.stTraj.front().time, st_traj2.stTraj.front().time);
    double end_time = min(st_traj1.stTraj.back().time, st_traj2.stTraj.back().time);

    if (start_time >= end_time) return 0.0; // No overlap
        
    vector<double> times;
    vector<double> distances;
    vector<double> similarities;

    // Calculate distances and similarities
    double accumulated_distance = 0.0;
    for (size_t i = 0, j = 0; (i < st_traj1.stTraj.size() - 1) && (j < st_traj2.stTraj.size() - 1);) {
      // Find the next time interval
      double nextTime = std::min(st_traj1.stTraj[i + 1].time, st_traj2.stTraj[j + 1].time);

      // Only proceed if the next time is within the overlapping interval
      if (nextTime <= start_time || nextTime > end_time) {
        if (st_traj1.stTraj[i + 1].time < st_traj2.stTraj[j + 1].time) ++i;
        else ++j;
        continue;
      }
      
      // Interpolate points at the next time
      Vector3d interpPoint1 = interpolatePoint(st_traj1.stTraj[i], st_traj1.stTraj[i + 1], nextTime);
      Vector3d interpPoint2 = interpolatePoint(st_traj2.stTraj[j], st_traj2.stTraj[j + 1], nextTime);
      
      // Calculate similarity
      double sim = calSimilarity(interpPoint1, interpPoint2);

      // Calculate the distance moved on trajectory 1 in this interval
      double distance_moved = (interpPoint1 - st_traj1.stTraj[i].position).norm();
      accumulated_distance += distance_moved;

      // Save the accumulated distance and similarity
      distances.push_back(accumulated_distance);
      similarities.push_back(sim);

      // Move to the next interval
      if (st_traj1.stTraj[i + 1].time == nextTime) ++i;
      if (st_traj2.stTraj[j + 1].time == nextTime) ++j;

      // Update the start time
      start_time = nextTime;
    }

    // Calculate the integral using trapezoidal rule for space
    double integral = trapezoidalIntegration(distances, similarities);

    // Normalize the integral by the total distance covered
    double total_distance = distances.back();
    return total_distance > 0 ? integral / total_distance : 0.0;
  } 

  double TargetData::calSimilarity(const Vector3d& p1, const Vector3d& p2) {
    return exp(-2*pow((p1 - p2).norm(), 4));
  }

  double TargetData::trapezoidalIntegration(const vector<double>& times, const vector<double>& similarities) {
    double integral = 0.0;
    for (size_t i = 0; i < times.size() - 1; ++i) {
      double h = times[i + 1] - times[i];
      integral += 0.5 * h * (similarities[i] + similarities[i + 1]);
    }
    return integral;
  }

  Vector3d TargetData::interpolatePoint(const STPData& start, const STPData& end, double time) {
    if (start.time == end.time) {
      return start.position; // Avoid division by zero
    }
    double t = (time - start.time) / (end.time - start.time);
    return start.position + t * (end.position - start.position);
  }
  
  void TargetData::getEvenlySpacedPoints(vector<Vector3d>& astar_path, int n) {
    if (astar_path.size() <= 1 || n <= 1) {
        return; 
    }

    vector<Vector3d> path = astar_path;
    astar_path.clear();

    double totalLength = 0.0;
    vector<double> segmentLengths;

    // Compute the total length and the length of each segment
    for (size_t i = 0; i < path.size() - 1; ++i) {
        double dist = (path[i + 1] - path[i]).norm();
        totalLength += dist;
        segmentLengths.push_back(dist);
    }

    // The interval of distance between each point
    double interval = totalLength / n;

    // Variables to keep track of the progress along the path
    double accumulatedLength = 0.0;
    size_t segmentIndex = 0;

    // Add the first point
    astar_path.push_back(path.front());

    // Generate the evenly spaced points
    for (int i = 1; i < n; ++i) { // -1 because we add the first and last points by default
        double targetLength = interval * i;
        
        // Advance to the correct segment
        while (segmentIndex < segmentLengths.size() - 1 &&
               accumulatedLength + segmentLengths[segmentIndex] < targetLength) {
            accumulatedLength += segmentLengths[segmentIndex];
            ++segmentIndex;
        }

        // Linearly interpolate within the current segment
        double lerpFactor = (targetLength - accumulatedLength) / segmentLengths[segmentIndex];
        Vector3d interpolatedPoint = path[segmentIndex] + 
                                     lerpFactor * (path[segmentIndex + 1] - path[segmentIndex]);
        astar_path.push_back(interpolatedPoint);
    }

    // Add the last point
    astar_path.push_back(path.back());
  }
  
}
