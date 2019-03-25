#include <cfloat>
#include <math.h>
#include <vector>

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Convert miles per hour to meters per second.
double mph2mps(double x) {
  // 5280 ft/mile.
  // 0.3048 m/ft
  // 3600 s/hr.
  return x * 5280.0 * 0.3048 / 3600.0;
}

// Computes logistic function at x.
double logistic(double x) {
  return 1.0 / (1.0 + exp(-x));
}

// Evaluate a polynomial with coefficients coeffs at eval_at.
double evaluate_at(const vector<double>& coeffs, double eval_at) {
  double power = 1.0;
  double sum = 0.0;
  for (double coeff : coeffs) {
    sum += coeff * power;
    power *= eval_at;
  }
  if (eval_at == 0.0) {
    cout << "IT IS: " << sum;
  }
  return sum;
}

// Differentiates a polynomial with coefficients coeffs.
vector<double> differentiate(const vector<double>& coeffs) {
  vector<double> derivative;
  for (int i = 1; i < coeffs.size(); i++) {
    derivative.push_back(coeffs[i] * i);
  }
  return derivative;
}

// Computes the distance between (x1, y1) and (x2, y2).
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Returns row number of closest waypoint to given (x, y) global coordinate.
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y) {
  double closest_len = DBL_MAX;
  int closest_waypoint = 0;

  for(int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if(dist < closest_len) {
      closest_len = dist;
      closest_waypoint = i;
    }
  }

  return closest_waypoint;
}

// Returns the row number of the next waypoint to aim for given a car's (x, y)
// global coordinates and heading theta.
int NextWaypoint(double x, double y, double theta,
                 const vector<double> &maps_x,
                 const vector<double> &maps_y) {
  int closest_waypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  const double map_x = maps_x[closest_waypoint];
  const double map_y = maps_y[closest_waypoint];
  const double heading = atan2((map_y - y),(map_x - x));

  double angle = fabs(theta - heading);
  angle = min(2 * pi() - angle, angle);

  // If angle > pi/2, the closest waypoint is behind us.
  if(angle > pi() / 2) {
    ++closest_waypoint;
    if (closest_waypoint == maps_x.size()) {
      closest_waypoint = 0;
    }
  }

  return closest_waypoint;
}

// Transform from Cartesian (x, y) coordinates to Frenet (s, d) coordinates.
vector<double> GetFrenet(double x, double y, double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y) {
  const int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);
  int prev_wp = next_wp - 1;
  if (next_wp == 0) {
  prev_wp  = maps_x.size() - 1;
  }

  // Compute the normal vector n components between waypoints.
  const double n_x = maps_x[next_wp] - maps_x[prev_wp];
  const double n_y = maps_y[next_wp] - maps_y[prev_wp];
  // Compute the Cartesian coordinate x components between waypoints.
  const double x_x = x - maps_x[prev_wp];
  const double x_y = y - maps_y[prev_wp];

  // Find the projection of x onto n.
  const double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  const double proj_x = proj_norm * n_x;
  const double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  // See if d value is positive or negative by comparing it to a center point.
  const double center_x = 1000 - maps_x[prev_wp];
  const double center_y = 2000 - maps_y[prev_wp];
  const double center_to_pos = distance(center_x, center_y, x_x, x_y);
  const double center_to_ref = distance(center_x, center_y, proj_x, proj_y);

  if(center_to_pos <= center_to_ref) {
    frenet_d *= -1;
  }

  // Calculate s value.
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }
  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet (s, d) coordinates to Cartesian (x, y).
vector<double> GetXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y) {
  int prev_wp = -1;
  // Advance to the next waypoint.
  while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1) )) {
    ++prev_wp;
  }
  const int wp2 = (prev_wp + 1) % maps_x.size();

  const double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
                               (maps_x[wp2] - maps_x[prev_wp]));
  // The x,y,s along the segment
  const double seg_s = (s - maps_s[prev_wp]);
  const double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  const double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  // Add offset from centerline.
  const double perp_heading = heading - (pi() / 2);
  const double x = seg_x + d * cos(perp_heading);
  const double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

// Returns the lane of a car at the given Frenet d coordinate.
int GetLane(double d) {
  return (int) d / LANE_WIDTH;
}

// Returns the Frenet s coordinate that is traveling in its lane at constant
// speed for time seconds.
double CarAtTime(double s, double speed, double time) {
  return s + speed * time;
}

// Computes the length of the diagonal of the car.
double GetCarDiagonal() {
  return sqrt(pow(CAR_LENGTH, 2) + pow(CAR_WIDTH, 2));
}

// Returns the distance of the gap between the SDC and another car, with
// positions given in Frenet coordinates. If the cars are in collision,
// return -1.0.
double DistanceToCar(double car_s, double car_d, double other_s,
                     double other_d) {
  // Locally, Frenet coordinates approximate Cartesian coordinates.
  const double pos_distance = distance(car_s, car_d, other_s, other_d);
  // Give a conservative estimate using the longest car measurement;
  // i.e. the diagonal.
  const double gap_size = pos_distance - GetCarDiagonal();
  return gap_size > 0.0 ? gap_size : -1.0;
}
