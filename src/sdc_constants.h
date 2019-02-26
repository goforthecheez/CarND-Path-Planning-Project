const double TIMESTEP = 0.02;  // seconds

// Road-related constants.
const double SPEED_LIMIT = 45.0;  // mph
const int NUM_LANES = 3;
const int LANE_WIDTH = 4;  // meters
const double CAR_LENGTH = 5.0;  // meters
const double CAR_WIDTH = 3.0;  // meters

// Trajectory generation constants.
// Lengths of trajectories to send to the simulation.
const int NUM_POINTS_IN_PATH = 80;
// Only when the number of points in the previous path falls below this
// number, compute the next trajectory.
const int PATH_RECALCULATION_SIZE = 30;
// The maximum speed increase allowed in a trajectory.
const double SPEED_INCREMENT_PER_PATH = 2.5;  // m/s
// Standard deviations of s, d, and t for candidate target generation.
const double SIGMA_S = 1.5;
const double SIGMA_S_DOT = 0.2;
const double SIGMA_S_DOUBLE_DOT = 0.05;
const double SIGMA_D = 0.04;
const double SIGMA_D_DOT = 0.02;
const double SIGMA_D_DOUBLE_DOT = 0.01;
// Generate candidate targets this many timesteps before the target time to
// this many timesteps after the target time.
const int TIMESTEP_RANGE_TO_GEN_TARGETS = 3;
// At each timestep, generate this many candidate targets.
const int NUM_TARGETS = 100;

// Cost function constants.
// Number of points to use when evaluating trajectories.
const int RESOLUTION = 40;
const double MAX_ACCELERATION = 10.0;  // m/s^2
const double EXPECTED_ACCELERATION_IN_ONE_SECOND = 2.0;  // m/s^2
const double MAX_JERK = 10.0;  // m/s^3
const double EXPECTED_JERK_IN_ONE_SECOND = 1.0;  // m/s^3

// Behavior planner constants.
// The size of the gap to leave with other cars.
const double MIN_BUFFER_WITH_CAR = 30.0;  // meters
