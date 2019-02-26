// Cost functions for trajectory generation.

#include <vector>

using namespace std;

class CostFnInterface {
 public:
  CostFnInterface() {}
  virtual ~CostFnInterface() {}

  // Compute the cost for the given trajectory, sensor fusion data, and timing.
  // NOTE: Costs must be in the range [0, 1].
  //
  // coeffs_s: coefficients of polynomial describing s trajectory.
  // coeffs_d: coefficients of polynomial describing d trajectory.
  // actual_time: time it takes to complete trajectory.
  // sensor_fusion: sensor fusion data of nearby vehicles.
  // previous_path_timesteps: number of timesteps left in previous path.
  // target_time: requested execution time of trajectory.
  // target_{s, d}: requested final position in Frenet coordinates.
  // target_{s, d}_dot: requested final velocities in Frenet coordinates.
  virtual double ComputeCost(const vector<double>& coeffs_s,
                             const vector<double>& coeffs_d,
                             double actual_time,
                             const vector<vector<double>>& sensor_fusion,
                             int previous_path_timesteps, double target_time,
                             double target_s, double target_d,
                             double target_s_dot, double target_d_dot) = 0;
};

// Penalizes trajectories that miss the target duration.
class TimeDiffCostFn : public CostFnInterface {
 public:
  TimeDiffCostFn() {}
  ~TimeDiffCostFn() override {}

  double ComputeCost(const vector<double>& coeffs_s,
                     const vector<double>& coeffs_d,
                     double actual_time,
                     const vector<vector<double>>& sensor_fusion,
                     int previous_path_timesteps, double target_time,
                     double target_s, double target_d,
                     double target_s_dot, double target_d_dot) override {
    return logistic(abs(actual_time - target_time) / target_time);
  }
};

// Penalizes trajectories whose end position differs from the s target state
// (including derivatives).
class SDiffCostFn : public CostFnInterface {
 public:
  SDiffCostFn() {}
  ~SDiffCostFn() override {}

  double ComputeCost(const vector<double>& coeffs_s,
                     const vector<double>& coeffs_d,
                     double actual_time,
                     const vector<vector<double>>& sensor_fusion,
                     int previous_path_timesteps, double target_time,
                     double target_s, double target_d,
                     double target_s_dot, double target_d_dot) override {
    const vector<double> coeffs_s_dot = differentiate(coeffs_s);
    const vector<double> coeffs_s_double_dot = differentiate(coeffs_s_dot);

    double cost = 0.0;
    cost += logistic(
        abs(target_s - evaluate_at(coeffs_s, target_time)) / SIGMA_S);
    cost += logistic(
        abs(target_s_dot - evaluate_at(
            coeffs_s_dot, target_time)) / SIGMA_S_DOT);
    // Target acceleration is always 0.0.
    cost += logistic(abs(0.0 - evaluate_at(
        coeffs_s_double_dot, target_time)) / SIGMA_S_DOUBLE_DOT);
    return cost / 3.0;
  }
};

// Penalizes trajectories whose end position differs from the d target state.
class DDiffCostFn : public CostFnInterface {
 public:
  DDiffCostFn() {}
  ~DDiffCostFn() override {}

  double ComputeCost(const vector<double>& coeffs_s,
                     const vector<double>& coeffs_d,
                     double actual_time,
                     const vector<vector<double>>& sensor_fusion,
                     int previous_path_timesteps, double target_time,
                     double target_s, double target_d,
                     double target_s_dot, double target_d_dot) override {
    const vector<double> coeffs_d_dot = differentiate(coeffs_d);
    const vector<double> coeffs_d_double_dot = differentiate(coeffs_d_dot);

    double cost = 0.0;
    cost += logistic(
        abs(target_d - evaluate_at(coeffs_d, target_time)) / SIGMA_D);
    cost += logistic(
        abs(target_d_dot - evaluate_at(
            coeffs_d_dot, target_time)) / SIGMA_D_DOT);
    // Target acceleration is always 0.0.
    cost += logistic(abs(0.0 - evaluate_at(
        coeffs_d_double_dot, target_time)) / SIGMA_D_DOUBLE_DOT);
    return cost / 3.0;
  }
};

// Penalizes trajectories that take the car outside of its lane.
class SplitLaningCostFn : public CostFnInterface {
 public:
  SplitLaningCostFn(const vector<double>& maps_x, const vector<double>& maps_y,
                   const vector<double>& maps_s)
      : maps_x_(maps_x), maps_y_(maps_y), maps_s_(maps_s) {}
  ~SplitLaningCostFn() override {}

  double ComputeCost(const vector<double>& coeffs_s,
                     const vector<double>& coeffs_d,
                     double actual_time,
                     const vector<vector<double>>& sensor_fusion,
                     int previous_path_timesteps, double target_time,
                     double target_s, double target_d,
                     double target_s_dot, double target_d_dot) override {
    const double dt = target_time / float(RESOLUTION);
    double t = dt;
    for (int i = 1; i < RESOLUTION; ++i) {
      const double pos_d = evaluate_at(coeffs_d, t);
      const int lane = GetLane(pos_d);
      if (GetCarLeftEdge(pos_d) < GetLeftEdgeOfLane(lane) ||
          GetCarRightEdge(pos_d) > GetRightEdgeOfLane(lane)) {
        return 1.0;
      }
      t += dt;
    }
    return 0.0;
  }

 private:
   // Returns the Frenet d coordinate of the left edge of the given lane.
  double GetLeftEdgeOfLane(int lane) {
    return lane * LANE_WIDTH;
  }

  // Returns the Frenet d coordinate of the right edge of the given lane.
  double GetRightEdgeOfLane(int lane) {
    return (lane + 1) * LANE_WIDTH;
  }

  // Returns the left edge of the car at Frenet coordinate d.
  double GetCarLeftEdge(double d) {
    return d - CAR_WIDTH / 2.0;
  }

  // Returns the right edge of the car at Frenet coordinate d.
  double GetCarRightEdge(double d) {
    return d + CAR_WIDTH / 2.0;
  }

  const vector<double>& maps_x_;
  const vector<double>& maps_y_;
  const vector<double>& maps_s_;
};

// Penalizes trajectories that exceed the speed limit.
class SpeedLimitCostFn : public CostFnInterface {
 public:
  SpeedLimitCostFn() {}
  ~SpeedLimitCostFn() override {}

  double ComputeCost(const vector<double>& coeffs_s,
                     const vector<double>& coeffs_d,
                     double actual_time,
                     const vector<vector<double>>& sensor_fusion,
                     int previous_path_timesteps, double target_time,
                     double target_s, double target_d,
                     double target_s_dot, double target_d_dot) override {
    const vector<double> coeffs_s_dot = differentiate(coeffs_s);

    const double dt = target_time / float(RESOLUTION);
    double t = dt;
    for (int i = 1; i < RESOLUTION; ++i) {
      const double velocity = evaluate_at(coeffs_s_dot, t);
      if (velocity > SPEED_LIMIT) {
        return 1.0;
      }
      t += dt;
    }
    return 0.0;
  }
};

// Rewards trajectories with higher average speeds.
class FasterBetterCostFn : public CostFnInterface {
 public:
  FasterBetterCostFn() {}
  ~FasterBetterCostFn() override {}

  double ComputeCost(const vector<double>& coeffs_s,
                     const vector<double>& coeffs_d,
                     double actual_time,
                     const vector<vector<double>>& sensor_fusion,
                     int previous_path_timesteps, double target_time,
                     double target_s, double target_d,
                     double target_s_dot, double target_d_dot) override {
    const double avg_v = evaluate_at(coeffs_s, target_time) / target_time;
    const double target_avg_v = target_s / target_time;
    return logistic(2 * (target_avg_v - avg_v) / avg_v);
  }
};

// Penalizes trajectories whose acceleration differs from the expectation.
class TotalAccelerationCostFn : public CostFnInterface {
 public:
  TotalAccelerationCostFn() {}
  ~TotalAccelerationCostFn() override {}

  double ComputeCost(const vector<double>& coeffs_s,
                     const vector<double>& coeffs_d,
                     double actual_time,
                     const vector<vector<double>>& sensor_fusion,
                     int previous_path_timesteps, double target_time,
                     double target_s, double target_d,
                     double target_s_dot, double target_d_dot) override {
    const vector<double> coeffs_s_dot = differentiate(coeffs_s);
    const vector<double> coeffs_s_double_dot = differentiate(coeffs_s_dot);
    const vector<double> coeffs_s_triple_dot =
        differentiate(coeffs_s_double_dot);

    const vector<double> coeffs_d_dot = differentiate(coeffs_d);
    const vector<double> coeffs_d_double_dot = differentiate(coeffs_d_dot);
    const vector<double> coeffs_d_triple_dot =
        differentiate(coeffs_d_double_dot);

    const double dt = target_time / float(RESOLUTION);
    double t = dt;
    double total_accel = 0.0;
    for (int i = 1; i < RESOLUTION; ++i) {
      // Integrate to get accumulated acceleration.
      const double accel_s = evaluate_at(coeffs_s_triple_dot, t) * dt;
      const double accel_d = evaluate_at(coeffs_d_triple_dot, t) * dt;
      total_accel += sqrt(pow(accel_s, 2) + pow(accel_d, 2));
    }
    const double accel_per_sec = total_accel / target_time;
    return logistic(accel_per_sec / EXPECTED_ACCELERATION_IN_ONE_SECOND);
  }
};

// Penalizes trajectories whose accelerations that exceed the maximum allowed.
class MaxAccelerationCostFn : public CostFnInterface {
 public:
  MaxAccelerationCostFn() {}
  ~MaxAccelerationCostFn() override {}

  double ComputeCost(const vector<double>& coeffs_s,
                     const vector<double>& coeffs_d,
                     double actual_time,
                     const vector<vector<double>>& sensor_fusion,
                     int previous_path_timesteps, double target_time,
                     double target_s, double target_d,
                     double target_s_dot, double target_d_dot) override {
    const vector<double> coeffs_s_dot = differentiate(coeffs_s);
    const vector<double> coeffs_s_double_dot = differentiate(coeffs_s_dot);
    const vector<double> coeffs_s_triple_dot =
        differentiate(coeffs_s_double_dot);

    const vector<double> coeffs_d_dot = differentiate(coeffs_d);
    const vector<double> coeffs_d_double_dot = differentiate(coeffs_d_dot);
    const vector<double> coeffs_d_triple_dot =
        differentiate(coeffs_d_double_dot);

    const double dt = target_time / float(RESOLUTION);
    double t = dt;
    for (int i = 1; i < RESOLUTION; ++i) {
      // Integrate to get accumulated acceleration.
      const double accel_s = evaluate_at(coeffs_s_triple_dot, t) * dt;
      const double accel_d = evaluate_at(coeffs_d_triple_dot, t) * dt;
      const double accel = sqrt(pow(accel_s, 2) + pow(accel_d, 2));
      if (accel > MAX_ACCELERATION) {
        return 1.0;
      }
    }
    return 0.0; 
  }
};

// Penalizes trajectories whose jerks differ from expectation.
class TotalJerkCostFn : public CostFnInterface {
 public:
  TotalJerkCostFn() {}
  ~TotalJerkCostFn() override {}

  double ComputeCost(const vector<double>& coeffs_s,
                     const vector<double>& coeffs_d,
                     double actual_time,
                     const vector<vector<double>>& sensor_fusion,
                     int previous_path_timesteps, double target_time,
                     double target_s, double target_d,
                     double target_s_dot, double target_d_dot) override {
    const vector<double> coeffs_s_dot = differentiate(coeffs_s);
    const vector<double> coeffs_s_double_dot = differentiate(coeffs_s_dot);
    const vector<double> coeffs_s_triple_dot =
        differentiate(coeffs_s_double_dot);
    const vector<double> coeffs_s_quad_dot =
        differentiate(coeffs_s_triple_dot);

    const vector<double> coeffs_d_dot = differentiate(coeffs_d);
    const vector<double> coeffs_d_double_dot = differentiate(coeffs_d_dot);
    const vector<double> coeffs_d_triple_dot =
        differentiate(coeffs_d_double_dot);
    const vector<double> coeffs_d_quad_dot =
        differentiate(coeffs_d_triple_dot);

    const double dt = target_time / float(RESOLUTION);
    double t = dt;
    double total_jerk = 0.0;
    for (int i = 1; i < RESOLUTION; ++i) {
      // Integrate to get accumulated jerk.
      const double jerk_s = evaluate_at(coeffs_s_quad_dot, t) * dt;
      const double jerk_d = evaluate_at(coeffs_d_quad_dot, t) * dt;
      total_jerk += sqrt(pow(jerk_s, 2) + pow(jerk_d, 2));
    }
    const double jerk_per_sec = total_jerk / target_time;
    return logistic(jerk_per_sec / EXPECTED_JERK_IN_ONE_SECOND);
  }
};

// Penalizes trajectories whose jerks exceed the maximum allowed.
class MaxJerkCostFn : public CostFnInterface {
 public:
  MaxJerkCostFn() {}
  ~MaxJerkCostFn() override {}

  double ComputeCost(const vector<double>& coeffs_s,
                     const vector<double>& coeffs_d,
                     double actual_time,
                     const vector<vector<double>>& sensor_fusion,
                     int previous_path_timesteps, double target_time,
                     double target_s, double target_d,
                     double target_s_dot, double target_d_dot) override {
    const vector<double> coeffs_s_dot = differentiate(coeffs_s);
    const vector<double> coeffs_s_double_dot = differentiate(coeffs_s_dot);
    const vector<double> coeffs_s_triple_dot =
        differentiate(coeffs_s_double_dot);
    const vector<double> coeffs_s_quad_dot =
        differentiate(coeffs_s_triple_dot);

    const vector<double> coeffs_d_dot = differentiate(coeffs_d);
    const vector<double> coeffs_d_double_dot = differentiate(coeffs_d_dot);
    const vector<double> coeffs_d_triple_dot =
        differentiate(coeffs_d_double_dot);
    const vector<double> coeffs_d_quad_dot =
        differentiate(coeffs_d_triple_dot);

    const double dt = target_time / float(RESOLUTION);
    double t = dt;
    for (int i = 1; i < RESOLUTION; ++i) {
      // Integrate to get accumulated jerk.
      const double jerk_s = evaluate_at(coeffs_s_quad_dot, t) * dt;
      const double jerk_d = evaluate_at(coeffs_d_quad_dot, t) * dt;
      const double jerk = sqrt(pow(jerk_s, 2) + pow(jerk_d, 2));
      if (jerk > MAX_JERK) {
        return 1.0;
      }
    }
    return 0.0; 
  }
};

// Penalizes colliding with other cars.
// Assumes cars stay in their lanes and travel at constant speed.
class CollisionCostFn : public CostFnInterface {
 public:
  CollisionCostFn() {}
  ~CollisionCostFn() override {}

  double ComputeCost(const vector<double>& coeffs_s,
                     const vector<double>& coeffs_d,
                     double actual_time,
                     const vector<vector<double>>& sensor_fusion,
                     int previous_path_timesteps, double target_time,
                     double target_s, double target_d,
                     double target_s_dot, double target_d_dot) override {
    // Check each other car for a collision one car at a time.
    for (int i = 0; i < sensor_fusion.size(); ++i) {
      const vector<double> car_data = sensor_fusion[i];
      const double vx = car_data[3];
      const double vy = car_data[4];
      const double speed = sqrt(pow(vx, 2) + pow(vy, 2));
      const double s = car_data[5];
      const double d = car_data[6];

      double dt = target_time / RESOLUTION;
      double t = dt;
      double sensor_fusion_t = (previous_path_timesteps + 1) * dt;
      for (int j = 1; j < RESOLUTION; ++j) {
        if (DistanceToCar(evaluate_at(coeffs_s, t),
                          evaluate_at(coeffs_d, t),
                          CarAtTime(s, speed, sensor_fusion_t), d) < 0.0) {
          return 1.0;
        }
        t += RESOLUTION;
        sensor_fusion_t += RESOLUTION;
      }
    }
    return 0.0;
  }
};

// Penalizes getting close to other cars.
// Assumes cars stay in their lanes and travel at constant speed.
class GiveSpaceCostFn : public CostFnInterface {
 public:
  GiveSpaceCostFn() {}
  ~GiveSpaceCostFn() override {}

  double ComputeCost(const vector<double>& coeffs_s,
                     const vector<double>& coeffs_d,
                     double actual_time,
                     const vector<vector<double>>& sensor_fusion,
                     int previous_path_timesteps, double target_time,
                     double target_s, double target_d,
                     double target_s_dot, double target_d_dot) override {
    double nearest = DBL_MAX;
    for (int i = 0; i < sensor_fusion.size(); ++i) {
      const vector<double> car_data = sensor_fusion[i];
      const double vx = car_data[3];
      const double vy = car_data[4];
      const double speed = sqrt(pow(vx, 2) + pow(vy, 2));
      const double s = car_data[5];
      const double d = car_data[6];

      double dt = target_time / RESOLUTION;
      double t = dt;
      double sensor_fusion_t = (previous_path_timesteps + 1) * dt;
      for (int j = 1; j < RESOLUTION; ++j) {
        const double gap_size = DistanceToCar(
            evaluate_at(coeffs_s, t), evaluate_at(coeffs_d, t),
            CarAtTime(s, speed, sensor_fusion_t), d);
        if (gap_size < nearest) {
          nearest = gap_size;
        }
        t += RESOLUTION;
        sensor_fusion_t += RESOLUTION;
      }
    }
    return logistic((2 * GetCarDiagonal()) / nearest);
  }
};
