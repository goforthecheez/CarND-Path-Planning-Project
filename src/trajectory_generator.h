// Trajectory generator.

#include <algorithm>
#include <cfloat>
#include <random>
#include <unordered_map>

#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"

#include "cost_fns.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Computes the best trajectory following map waypoints.
class TrajectoryGenerator {
 public:
  TrajectoryGenerator(const vector<double>& maps_x,
                      const vector<double>& maps_y,
                      const vector<double>& maps_s,
                      const vector<double>& maps_dx,
                      const vector<double>& maps_dy)
      : maps_x_(maps_x), maps_y_(maps_y), maps_s_(maps_s), maps_dx_(maps_dx),
        maps_dy_(maps_dy) {
    // Create cost functions and tunable weights.
    cost_fn_weights_.emplace(
        "time_diff", make_pair(new TimeDiffCostFn(), 0.0));
    cost_fn_weights_.emplace(
        "s_diff", make_pair(new SDiffCostFn(), 0.0));
    cost_fn_weights_.emplace(
        "d_diff", make_pair(new DDiffCostFn(), 3.0));
    cost_fn_weights_.emplace(
        "off_roading",
        make_pair(new SplitLaningCostFn(maps_x_, maps_y_, maps_s_), 1.0));
    cost_fn_weights_.emplace(
        "speed_limit", make_pair(new SpeedLimitCostFn(), 5.0));
    cost_fn_weights_.emplace(
        "faster_better", make_pair(new FasterBetterCostFn(), 0.7));
    cost_fn_weights_.emplace(
        "total_acceleration", make_pair(new TotalAccelerationCostFn(), 2.0));
    cost_fn_weights_.emplace(
        "max_acceleration", make_pair(new MaxAccelerationCostFn(), 7.0));
    cost_fn_weights_.emplace(
        "total_jerk", make_pair(new TotalJerkCostFn(), 1.0));
    cost_fn_weights_.emplace(
        "max_jerk", make_pair(new MaxJerkCostFn(), 3.0));
    cost_fn_weights_.emplace(
        "collision", make_pair(new CollisionCostFn(), 10.0));
    cost_fn_weights_.emplace(
        "give_space", make_pair(new GiveSpaceCostFn(), 0.5));
  }

  ~TrajectoryGenerator() {
    for (auto cost_fn : cost_fn_weights_) {
      delete cost_fn.second.first;
    }
  }

  // Computes the best trajectory and returns it as next_{x, y}_vals.
  //
  // sensor_fusion: data on nearby cars.
  // previous_path_timesteps: timesteps remaining in the previous path.
  // next_{x, y}_vals: the trajectory to execute, already partially populated.
  // car_{x, y}: SDC's current position in Cartesian coordinates.
  // car_{s, d}: SDC's current position in Frenet coordinates.
  // car_yaw: SDC's current heading in radians.
  // car_speed: SDC's current speed.
  // target_lane: lane to end the trajectory in.
  // target_v: speed to aim for.
  void GenerateTrajectory(const vector<vector<double>>& sensor_fusion,
                          int previous_path_timesteps,
                          vector<double> *next_x_vals,
                          vector<double> *next_y_vals, double init_d, double init_yaw, 
                          double car_x, double car_y, double car_s,
                          double car_d, double car_yaw, double car_speed,
                          int target_lane, double target_v);

 private:
  // Generates target states approximately time_horizon seconds in the future.
  //
  // candidates: To be populated with candidate target states. Each target
  //   state is a length 7 vector of the form {s, s_dot, s_double_dot,
  //   d, d_dot, d_double_dot, elapsed_time}
  // car_{s, d}: SDC's current position in Frenet coordinates.
  // car_speed: SDC's current speed.
  // time_horizon: generate target states approximately this far in the future.
  // target_{s, d}: the target trajectory end position in Frenet coordinates.
  // target_speed: the target end of trajectory speed.
  void GenerateCandidateTargets(vector<vector<double>> *candidates,
                                double car_s, double car_d, double car_speed,
                                double time_horizon, double target_s,
                                double target_d, double target_speed);

  // Computes the jerk-minimizing trajectory that goes from start state to
  // a desired end state in time t.
  //
  // start: length three array of initial coordinates [s, s_dot, s_double_dot]
  // end: length three array of final coordinates
  // t: time in seconds
  //
  // Returns the coefficients of the 5th-order jerk-minimizing polynomial.
  vector<double> JerkMinimizingTrajectory(
      const vector<double>& start, const vector<double>& end, double t);

  // Returns index of best trajectory from a set of candidate trajectories,
  // sensor fusion data of nearby cars, and the requested execution time,
  // position, and speed.
  int BestTrajectory(const vector<vector<double>>& cand_trajs_s,
                     const vector<vector<double>>& cand_trajs_d,
                     const vector<double>& actual_times,
                     const vector<vector<double>>& sensor_fusion,
                     int previous_path_timesteps, double target_time,
                     double target_s, double target_d, double target_s_dot,
                     double target_d_dot);

  // Computes weighted sum of all the cost functions for a single trajectory
  // from sensor fusion data of nearby cars and the requested execution time,
  // position, and speed.
  double ComputeTotalCost(const vector<double>& traj_s_coeffs,
                          const vector<double>& traj_d_coeffs,
                          double actual_time,
                          const vector<vector<double>>& sensor_fusion,
                          int previous_path_timesteps, double target_time,
                          double target_s, double target_d, double target_s_dot,
                          double target_d_dot);

  // Convert Frenet frame polynomial trajectories into discretized Cartesian
  // coordinate trajectory.
  //
  // traj_s_coeffs: length 6 vector of coefficients of s polynomial.
  // traj_d_coeffs: length 6 vector of coefficients of d polynomial.
  // elapsed_t: discretized trajectory to be generated from 0 to this time.
  // car_{x, y}: car's current position in Cartesian coordinates.
  // car_{x, d}: car's current position in Frenet coordinates.
  // car_yaw: car's curent heading.
  // traj_{x, y}: discretized trajectory in Cartesian coordinates.
  void DiscretizeTrajectory(const vector<double>& traj_s_coeffs,
                            const vector<double>& traj_d_coeffs,
                            double elapsed_t, double car_x, double car_y,
                            double car_s, double car_d, double car_yaw,
                            vector<double> *traj_x, vector<double> *traj_y,
			    double init_d, double init_yaw);

  // Returns the Frenet d coordinate of the given lane.
  double GetCenterOfLane(int lane) {
    return (lane + 0.5) * LANE_WIDTH;
  }

  // Map waypoints.
  const vector<double>& maps_x_;
  const vector<double>& maps_y_;
  const vector<double>& maps_s_;
  const vector<double>& maps_dx_;
  const vector<double>& maps_dy_;
  // Map from cost function name to cost functions and weights.
  unordered_map<string, pair<CostFnInterface *, float>> cost_fn_weights_;
};

void TrajectoryGenerator::GenerateTrajectory(
    const vector<vector<double>>& sensor_fusion,
    int previous_path_timesteps,
    vector<double> *next_x_vals, vector<double> *next_y_vals,
    double init_d, double init_yaw, 
    double car_x, double car_y, double car_s, double car_d, double car_yaw,
    double car_speed, int target_lane, double target_v) {
  // Don't compute too much future path.
  if (next_x_vals->size() > PATH_RECALCULATION_SIZE) return;

  // Start state.
  // Assumptions:
  //   (1) The car is traveling in positive s direction.
  //   (2) Acceleration is 0.0.
  vector<double> start_s;
  start_s.push_back(car_s);
  start_s.push_back(car_speed);
  start_s.push_back(0.0);
  vector<double> start_d;
  start_d.push_back(car_d);
  start_d.push_back(0.0);
  start_d.push_back(0.0);

  // Move forward as far as possible in the alotted time.
  const double time_horizon = TIMESTEP * NUM_POINTS_IN_PATH;
  double target_speed;
  if (target_v < car_speed) {
    target_speed = max(target_v, 0.0);
  } else {
    target_speed = min(mph2mps(SPEED_LIMIT), target_v);
  }
  const double avg_accel = (target_speed - car_speed) / time_horizon;
  const double max_forward_s = car_speed * time_horizon;
                               + 0.5 * avg_accel * pow(time_horizon, 2);
  const double target_s = car_s + max_forward_s;
  // Aim for the center of the target lane.
  const double target_d = GetCenterOfLane(target_lane);

  // Generate candidate targets.
  vector<vector<double>> candidates;
  GenerateCandidateTargets(&candidates, car_s, car_d, car_speed, time_horizon,
                           target_s, target_d, target_speed);

  // Compute candidate jerk-minimizing trajectories.
  vector<vector<double>> cand_trajs_s;
  vector<vector<double>> cand_trajs_d;
  vector<double> actual_times;
  for (const vector<double>& c : candidates) {
    vector<double> end_s;
    end_s.push_back(c[0]);
    end_s.push_back(c[1]);
    end_s.push_back(c[2]);
    vector<double> end_d;
    end_d.push_back(c[3]);
    end_d.push_back(c[4]);
    end_d.push_back(c[5]);
    double t = c[6];
    vector<double> coeffs_s = JerkMinimizingTrajectory(start_s, end_s, t);
    vector<double> coeffs_d = JerkMinimizingTrajectory(start_d, end_d, t);

    cand_trajs_s.push_back(coeffs_s);
    cand_trajs_d.push_back(coeffs_d);
    actual_times.push_back(t);
  }

  // Select best trajectory and convert it into x, y point path.
  int best = BestTrajectory(
      cand_trajs_s, cand_trajs_d, actual_times, sensor_fusion,
      previous_path_timesteps, time_horizon, target_s, target_d,
      target_speed, 0.0);
  // TODO(adelinew): Update docstring & function params. (best_x, best_y)
  DiscretizeTrajectory(cand_trajs_s[best], cand_trajs_d[best],
                       actual_times[best], car_x, car_y, car_s, car_d, car_yaw,
                       next_x_vals, next_y_vals, init_d, init_yaw);
}

void TrajectoryGenerator::GenerateCandidateTargets(
    vector<vector<double>> *candidates, double car_s, double car_d,
    double car_speed, double time_horizon, double target_s, double target_d,
    double target_speed) {
  random_device rd;
  default_random_engine gen(rd());
  normal_distribution<double> dist_s(0, SIGMA_S);
  normal_distribution<double> dist_s_dot(0, SIGMA_S_DOT);
  normal_distribution<double> dist_s_double_dot(0, SIGMA_S_DOUBLE_DOT);
  normal_distribution<double> dist_d(0, SIGMA_D);
  normal_distribution<double> dist_d_dot(0, SIGMA_D_DOT);
  normal_distribution<double> dist_d_double_dot(0, SIGMA_D_DOUBLE_DOT);

  double t = time_horizon - TIMESTEP_RANGE_TO_GEN_TARGETS * TIMESTEP;
  while (t <= time_horizon + TIMESTEP_RANGE_TO_GEN_TARGETS * TIMESTEP) {
    for (int i = 0; i < NUM_TARGETS; ++i) {
      vector<double> cand;
      cand.push_back(target_s + dist_s(gen));
      cand.push_back(target_speed + dist_s_dot(gen));
      // Although the car may accelerate, at the end of the trajectory, it
      // should be traveling at constant speed.
      cand.push_back(0.0 + dist_s_double_dot(gen));
      cand.push_back(target_d + dist_d(gen));
      // Similarly, the car may change lanes, but it should be stable at the
      // end of its trajectory.
      cand.push_back(0.0 + dist_d_dot(gen));
      cand.push_back(0.0 + dist_d_double_dot(gen));
      cand.push_back(t);
      candidates->push_back(cand);
    }
    t += TIMESTEP;
  }
}

vector<double> TrajectoryGenerator::JerkMinimizingTrajectory(
    const vector<double>& start, const vector<double>& end, double t) {
  MatrixXd lhs(3, 3);
  lhs << pow(t, 3), pow(t, 4), pow(t, 5),
         3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4),
         6 * t, 12 * pow(t, 2), 20 * pow(t, 3);

  MatrixXd rhs(3, 1);
  rhs << end[0] - (start[0] + start[1] * t + 0.5 * start[2] * pow(t, 2)),
         end[1] - (start[1] + start[2] * t),
         end[2] - start[2];

  auto Q = lhs.householderQr();
  const MatrixXd solution = Q.solve(rhs);

  vector<double> out = {start[0], start[1], 0.5 * start[2]};
  for (int i = 0; i < solution.size(); ++i) {
    out.push_back(solution.data()[i]);
  }
  return out;
}

int TrajectoryGenerator::BestTrajectory(
    const vector<vector<double>>& cand_trajs_s,
    const vector<vector<double>>& cand_trajs_d,
    const vector<double>& actual_times,
    const vector<vector<double>>& sensor_fusion, int previous_path_timesteps,
    double target_time, double target_s, double target_d, double target_s_dot,
    double target_d_dot) {
  double min_cost = DBL_MAX;
  double best = -1;
  for (int i = 0; i < actual_times.size(); ++i) {
    double cost = ComputeTotalCost(
        cand_trajs_s[i], cand_trajs_d[i], actual_times[i], sensor_fusion,
        previous_path_timesteps, target_time, target_s, target_d, target_s_dot,
        target_d_dot);
    if (cost < min_cost) {
      min_cost = cost;
      best = i;
    }
  }
  return best;
}

double TrajectoryGenerator::ComputeTotalCost(
    const vector<double>& traj_s_coeffs, const vector<double>& traj_d_coeffs,
    double actual_time, const vector<vector<double>>& sensor_fusion,
    int previous_path_timesteps, double target_time, double target_s,
    double target_d, double target_s_dot, double target_d_dot) {
  double total_cost = 0.0;
  for (auto it : cost_fn_weights_) {
    CostFnInterface *cost_fn = it.second.first;
    const float weight = it.second.second;
    total_cost += weight * cost_fn->ComputeCost(
        traj_d_coeffs, traj_s_coeffs, actual_time, sensor_fusion, previous_path_timesteps,
        target_time, target_s, target_d, target_s_dot, target_d_dot);
  }
  return total_cost;
}

void TrajectoryGenerator::DiscretizeTrajectory(
    const vector<double>& traj_s_coeffs,
    const vector<double>& traj_d_coeffs,
    double elapsed_t, double car_x, double car_y, double car_s, double car_d,
    double car_yaw, vector<double> *next_x_vals, vector<double> *next_y_vals,
    double init_d, double init_yaw) {
  //  for (int i = 0; i < traj_s_coeffs.size(); ++i) {
  //    cout << traj_s_coeffs[i] << ",";
  //  }
  //  cout << " [s]" << endl;
  //  for (int i = 0; i < traj_d_coeffs.size(); ++i) {
  //    cout << traj_d_coeffs[i] << ",";
  //  }
  //  cout << " [d]" << endl;

  int prev_wp = -1;
  // Advance to the next waypoint.
  while (car_s > maps_s_[prev_wp + 1] && (prev_wp < (int)(maps_s_.size() - 1) )) {
    ++prev_wp;
  }
  const int wp2 = (prev_wp + 1) % maps_s_.size();
  const int wp3 = (prev_wp + 2) % maps_s_.size();
  const int wp4 = (prev_wp + 3) % maps_s_.size();
  const int prev_prev_wp = (prev_wp + 179) % maps_s_.size();

  // Transform to vehicle coordinates.
  const double perp_yaw = car_yaw - (pi() / 2);
  const double car_proj_x = car_x - car_d * cos(perp_yaw);
  const double car_proj_y = car_y - car_d * sin(perp_yaw);
  double rot;
  if (next_x_vals->size() > 1) {
    rot = atan2(car_y - (*next_y_vals)[next_x_vals->size() - 2],
                car_x - (*next_x_vals)[next_x_vals->size() - 2]);
  } else {
    rot = atan2(maps_y_[wp2] - maps_y_[prev_wp],
			   maps_x_[wp2] - maps_x_[prev_wp]);
  }
  MatrixXd to_vehicle(2, 2);
  to_vehicle << cos(rot), sin(rot),
                -sin(rot), cos(rot);

  VectorXd car_proj_prev(2);
  double car_proj_x_prev;
  double car_proj_y_prev;
  if (next_x_vals->size() > 1) {
    car_proj_x_prev = (*next_x_vals)[next_x_vals->size() - 2] - car_d * cos(perp_yaw);
    car_proj_y_prev = (*next_y_vals)[next_y_vals->size() - 2] - car_d * sin(perp_yaw);
  }
  car_proj_prev << car_proj_x_prev - car_proj_x, car_proj_y_prev - car_proj_y;

  VectorXd prev_prev_wp_shifted(2);
  prev_prev_wp_shifted << maps_x_[prev_prev_wp] - car_proj_x,
                          maps_y_[prev_prev_wp] - car_proj_y;
  VectorXd prev_wp_shifted(2);
  prev_wp_shifted << maps_x_[prev_wp] - car_proj_x, maps_y_[prev_wp] - car_proj_y;
  VectorXd wp2_shifted(2);
  wp2_shifted << maps_x_[wp2] - car_proj_x, maps_y_[wp2] - car_proj_y;
  VectorXd wp3_shifted(2);
  wp3_shifted << maps_x_[wp3] - car_proj_x, maps_y_[wp3] - car_proj_y;
  VectorXd wp4_shifted(2);
  wp3_shifted << maps_x_[wp4] - car_proj_x, maps_y_[wp4] - car_proj_y;

  VectorXd car_proj_prev_veh;
  if (next_x_vals->size() > 1) {
    car_proj_prev_veh = to_vehicle * car_proj_prev;
  }
  const VectorXd prev_prev_wp_veh = to_vehicle * prev_prev_wp_shifted;
  const VectorXd prev_wp_veh = to_vehicle * prev_wp_shifted;
  const VectorXd wp2_veh = to_vehicle * wp2_shifted;
  const VectorXd wp3_veh = to_vehicle * wp3_shifted;
  const VectorXd wp4_veh = to_vehicle * wp4_shifted;
  cout << maps_x_[prev_wp] << ", " << maps_x_[wp2] << endl;

  // Fit a spline.
  vector<double> pts_x;
  pts_x.push_back(prev_prev_wp_veh[0]);
  if (next_x_vals->size() > 1 && car_proj_prev_veh[0] < prev_wp_veh[0]) {
    pts_x.push_back(car_proj_prev_veh[0]);
    pts_x.push_back(prev_wp_veh[0]);
  } else if (next_x_vals->size() > 1) {
    pts_x.push_back(prev_wp_veh[0]);
    pts_x.push_back(car_proj_prev_veh[0]);
  } else {
    pts_x.push_back(prev_wp_veh[0]);
  }
  bool add_zero = 0.0 > pts_x.back();
  if (add_zero) {
    pts_x.push_back(0.0);
  }
  pts_x.push_back(wp2_veh[0]);
  pts_x.push_back(wp3_veh[0]);
  //pts_x.push_back(wp4_veh[0]);

  vector<double> pts_y;
  pts_y.push_back(prev_prev_wp_veh[1]);
  if (next_x_vals->size() > 1 && car_proj_prev_veh[0] < prev_wp_veh[0]) {
    pts_y.push_back(car_proj_prev_veh[1]);
    pts_y.push_back(prev_wp_veh[1]);
  } else if (next_x_vals->size() > 1) {
    pts_y.push_back(prev_wp_veh[1]);
    pts_y.push_back(car_proj_prev_veh[1]);
  } else {
    pts_y.push_back(prev_wp_veh[1]);
  }
  if (add_zero) {
    pts_y.push_back(0.0);
  }
  pts_y.push_back(wp2_veh[1]);
  pts_y.push_back(wp3_veh[1]);
  //pts_y.push_back(wp4_veh[1]);

  tk::spline spline;
  spline.set_points(pts_x, pts_y);

  MatrixXd to_global(2, 2);
  to_global << cos(rot), -sin(rot),
               sin(rot), cos(rot);

  double prev_x_proj = 0.0;
  double prev_y_proj = 0.0;
  double t = TIMESTEP;
  while (t < elapsed_t) {
    //cout << evaluate_at(traj_s_coeffs, t) << " [t = " << t << ", s]" << endl;
    //cout << evaluate_at(traj_d_coeffs, t) << " [t = " << t << ", d]" << endl;
    const double s = evaluate_at(traj_s_coeffs, t);
    // Approximate car movement along x as being in a straight line.
    const double x_proj = s - car_s;
    const double y_proj = spline(x_proj);

    const double heading = atan2(y_proj - prev_y_proj, x_proj - prev_x_proj);
    const double perp_heading = heading - (pi() / 2);

    const double d = evaluate_at(traj_d_coeffs, t);
    VectorXd xy(2);
    xy << x_proj + d * cos(perp_heading), y_proj + d * sin(perp_heading);

    // Transform back to global coordinates.
    const VectorXd global_xy = to_global * xy;

    next_x_vals->push_back(global_xy[0] + car_proj_x);
    //cout << "x: " << x << endl;
    next_y_vals->push_back(global_xy[1] + car_proj_y);
    //cout << "y: " << y << endl;

    t += TIMESTEP;
    prev_x_proj = x_proj;
    prev_y_proj = y_proj;
  }
}
