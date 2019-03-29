// Behavior planner.
//
// This is a very simple behavior planner.
// First of all, the car always tries to change into the leftmost lane.
// If the SDC is getting close to the car in front of it, then:
//   (1) first see it is possible to change one lane to the left
//       (a faster lane),
//   (2) if not, see if it is possible to change one lane to the right
//       (a slower lane),
//   (3) otherwise, keep its lane but slow down.
// If the car is not blocked in front, then it will try to speed up.
// If none of the above are possible, the car will maintain its speed.

#include <vector>

using namespace std;

// Returns the front edge of the car at Frenet coordinate s.
double GetCarFrontEdge(double s) {
  return s + CAR_LENGTH / 2.0;
}

// Returns the back edge of the car at Frenet coordinate s.
double GetCarBackEdge(double s) {
  return s - CAR_LENGTH / 2.0;
}

// Returns the lane number to the left of the given Frenet d-coordinate.
int OneLaneLeft(double d) {
  return GetLane(d) - 1;
}

// Returns the lane number to the right of the given Frenet d-coordinate.
int OneLaneRight(double d) {
  return GetLane(d) + 1;
}

// Given a lane number, returns whether it's valid.
bool LaneIsOnRoad(int lane) {
  return lane >= 0 && lane < NUM_LANES;
}

// Given a Frenet d-coordinate, returns whether it's valid.
bool LocIsOnRoad(double d) {
  return LaneIsOnRoad(GetLane(d));
}

// Returns the index in sensor_fusion of the rearmost car ahead of the SDC
// in the given lane and the size of the s-coordinate gap with the SDC.
//
// car_s: current Frenet s-coordinate of SDC.
// sensor_fusion: data on nearby cars.
// previous_path_timesteps: indicates how much delay to apply to sensor fusion data.
// lane: the lane to inspect.
vector<double> LastCarAheadInLane(
    double car_s, const vector<vector<double>>& sensor_fusion,
    int previous_path_timesteps, int lane) {
  double closest_gap = DBL_MAX;
  // Index into sensor_fusion of the car of interest.
  double sf_index = -1;
  for (int i = 0; i < sensor_fusion.size(); ++i) {
    const vector<double>& car_data = sensor_fusion[i];
    const double d = car_data[6];
    if (GetLane(d) != lane) {
      continue;
    }

    const double s = car_data[5];
    const double vx = car_data[3];
    const double vy = car_data[4];
    const double speed = sqrt(pow(vx, 2) + pow(vy, 2));

    const double other_s = s + speed * TIMESTEP * previous_path_timesteps;
    if (other_s < car_s) {
      continue;
    }
    const double gap = GetCarBackEdge(other_s) - GetCarFrontEdge(car_s);
    if (gap < closest_gap) {
      closest_gap = gap;
      sf_index = i;
    }
  }
  return {(double) sf_index, closest_gap};
}

// Returns the index in sensor_fusion of the first car behind the SDC in the
// given lane and the size of the gap with the SDC.
//
// car_s: current Frenet s-coordinate of SDC.
// sensor_fusion: data on nearby cars.
// previous_path_timesteps: indicates how much delay to apply to sensor fusion data.
// lane: the lane to inspect.
vector<double> FirstCarBehindInLane(
    double car_s, const vector<vector<double>>& sensor_fusion,
    int previous_path_timesteps, int lane) {
  double closest_gap = DBL_MAX;
  // Index into sensor_fusion of the car of interest.
  double sf_index = -1;
  for (int i = 0; i < sensor_fusion.size(); ++i) {
    const vector<double>& car_data = sensor_fusion[i];
    const double d = car_data[6];
    if (GetLane(d) != lane) {
      continue;
    }

    const double s = car_data[5];
    const double vx = car_data[3];
    const double vy = car_data[4];
    const double speed = sqrt(pow(vx, 2) + pow(vy, 2));

    const double other_s = s + speed * TIMESTEP * previous_path_timesteps;
    if (other_s > car_s) {
      continue;
    }
    const double gap = GetCarBackEdge(car_s) - GetCarFrontEdge(other_s);
    if (gap < closest_gap) {
      closest_gap = gap;
      sf_index = i;
    }
  }
  return {(double) sf_index, closest_gap};
}

// Determines whether it is safe for the SDC to speed up.
// It is safe for the SDC to speed up if the closest car ahead of it in its
// lane is more than MIN_BUFFER_SPEED_UP ahead of it, or there are no cars ahead.
//
// car_{s, d}: current position of SDC in Frenet coordinates.
// car_speed: current speed of SDC.
// sensor_fusion: data on nearby cars.
// previous_path_timesteps: indicates how much delay to apply to sensor fusion data.
bool OkayToSpeedUp(
    double car_s, double car_d, double car_speed,
    const vector<vector<double>>& sensor_fusion, int previous_path_timesteps) {
  int lane = GetLane(car_d);
  const vector<double> last_ahead = LastCarAheadInLane(
      car_s, sensor_fusion, previous_path_timesteps, lane);
  const int sf_index = (int) last_ahead[0];
    if (sf_index == -1) {
    // No cars in front.
      return true;
  }

  const vector<double>& other_car_data = sensor_fusion[sf_index];
  const double other_s = other_car_data[5];
  return (other_s - car_s) > MIN_BUFFER_SPEED_UP;
}

// The SDC should try to change lanes if it is either blocked in front or not
// traveling at the speed limit.
//
// NOTE: Assumes all cars, including the SDC, move at constant speed.
//
// car_{s, d}: current position of SDC in Frenet coordinates.
// car_speed: current speed of SDC.
// sensor_fusion: data on nearby cars.
// previous_path_timesteps: indicates how much delay to apply to sensor fusion data.
bool ShouldTryToChangeLanes(
    double car_s, double car_d, double car_speed,
    const vector<vector<double>>& sensor_fusion, int previous_path_timesteps) {
  int lane = GetLane(car_d);
  const vector<double> last_ahead = LastCarAheadInLane(
      car_s, sensor_fusion, previous_path_timesteps, lane);
  const int sf_index = (int) last_ahead[0];
  if (sf_index == -1) {
    // No cars in front.
    return false;
  }

  // Estimate gap at end of next trajectory, assuming no lane change.
  const double car_time = TIMESTEP * NUM_POINTS_IN_PATH;
  const double car_future_s = car_s + car_speed * car_time;

  const vector<double>& other_car_data = sensor_fusion[sf_index];
  const double other_vx = other_car_data[3];
  const double other_vy = other_car_data[4];
  const double other_speed = sqrt(pow(other_vx, 2) + pow(other_vy, 2));
  const double other_s = other_car_data[5];
  const double other_time =
      TIMESTEP * (NUM_POINTS_IN_PATH + previous_path_timesteps);
  const double other_future_s = other_s + other_speed * other_time;

  return (other_future_s - car_future_s) < MIN_BUFFER_WITH_CAR_FRONT;
}

// Determines whether a lane change would result in collision.
//
// NOTE: Assumes all cars, including the SDC, move at constant speed.
//
// car_{s, d}: current position of SDC in Frenet coordinates.
// car_speed: current speed of SDC.
// sensor_fusion: data on nearby cars.
// previous_path_timesteps: indicates how much delay to apply to sensor fusion data.
// lane: the target lane of the lane change.
bool LaneChangeIsSafe(double car_s, double car_d, double car_speed,
                      const vector<vector<double>>& sensor_fusion,
                      int previous_path_timesteps, int lane) {
  if (!LaneIsOnRoad(lane)) {
    return false;
  }

  // Check that the front is OK.
  const vector<double> last_ahead = LastCarAheadInLane(
      car_s, sensor_fusion, previous_path_timesteps, lane);
  const int last_ahead_idx = (int) last_ahead[0];
  const double last_ahead_gap = last_ahead[1];
  const bool front_ok = (last_ahead_idx == -1) ||
                        (last_ahead_gap > MIN_BUFFER_WITH_CAR_FRONT);

  // Check that the back is OK.
  const vector<double> first_behind = FirstCarBehindInLane(
      car_s, sensor_fusion, previous_path_timesteps, lane);
  const int first_behind_idx = (int) first_behind[0];
  const double first_behind_gap = first_behind[1];
  const bool back_ok = (first_behind_idx == -1) ||
                       (first_behind_gap > MIN_BUFFER_WITH_CAR_BACK);

  return front_ok && back_ok;
}

// The behavior planner returns which lane the car should aim for in its next
// trajectory generation phase and whether it should slow down (1 = yes).
//
// car_{s, d}: current position of SDC in Frenet coordinates.
// car_speed: current speed of SDC.
// sensor_fusion: data on nearby cars.
// previous_path_timesteps: indicates how much delay to apply to sensor fusion data.
vector<double> PlanBehavior(double curr_s, double car_s, double car_d, double car_speed,
                            const vector<vector<double>>& sensor_fusion,
                            int previous_path_timesteps) {
  // Provided the car is moving fast enough, always try to be in the fast lane.
  if ((car_speed > mph2mps(POSTED_SPEED_LIMIT * 0.75)
       && GetLane(car_d) == 1) ||
      (car_speed > mph2mps(POSTED_SPEED_LIMIT * 0.5) && GetLane(car_d) == 2)) {
    if (LaneChangeIsSafe(car_s, car_d, car_speed, sensor_fusion,
                         previous_path_timesteps, OneLaneLeft(car_d))) {
      // Include small correction factor to reduce tangential acceleration.
      return {(double) OneLaneLeft(car_d), car_speed - mph2mps(2.0)};
    }
  }

  if (ShouldTryToChangeLanes(car_s, car_d, car_speed, sensor_fusion,
                             previous_path_timesteps)) {
    // Try a left lane change.
    // Include small correction to reduce tangential acceleration.
    if (LaneChangeIsSafe(car_s, car_d, car_speed, sensor_fusion,
                         previous_path_timesteps, OneLaneLeft(car_d))) {
      return {(double) OneLaneLeft(car_d), car_speed - mph2mps(2.0)};
    }
    // Try a right lane change.
    // Include small correction to reduce tangential acceleration.
    if (LaneChangeIsSafe(car_s, car_d, car_speed, sensor_fusion,
                         previous_path_timesteps, OneLaneRight(car_d))) {
      return {(double) OneLaneRight(car_d), car_speed - mph2mps(2.0)};
    }

    // Can't change lanes, so slow down.
    return {(double) GetLane(car_d), car_speed - SPEED_INCREMENT_PER_PATH};
  }

  // Nothing directly ahead; try to speed up.
  if (OkayToSpeedUp(car_s, car_d, car_speed, sensor_fusion,
                    previous_path_timesteps)) {
    // HACK: Due to greater warp in the Frenet-to-Cartesian conversion in the
    // right lanes, use a slower speed limit and smaller speed increment.
    double actual_speed_limit = mph2mps(SPEED_LIMIT) - 1.0 * GetLane(car_d);
    return {(double) GetLane(car_d),
            min(actual_speed_limit,
		car_speed + SPEED_INCREMENT_PER_PATH - 0.5 * GetLane(car_d))};
  }

  // The default driving behavior is to maintain speed.
  return {(double) GetLane(car_d), car_speed};
}
