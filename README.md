# Kalman Filter

## Resources

- [Great First Kalman Filter Tutorial](https://www.kalmanfilter.net/default.aspx)
- [Kalman Filter Wikipedia](https://en.wikipedia.org/wiki/Kalman_filter)
- [LU Decomposition](https://en.wikipedia.org/wiki/LU_decomposition)

## Kalman Filter Usage

The Kalman Filter library can use N dimensions and either a constant velocity or constant acceleration model. This is configured using the template parameters. A short example is show below, and a more complete example can be found in the [example](example) directory.

```c++

// Create a Kalman Filter with 3 dimensions (x, y, z)
// and 3 states (position, velocity, acceleration)
Filter::KalmanFilterParameters parameters;
parameters.processNoise = 0.1;

Filter::KalmanFilter<3, 3> kf(parameters);

// Create initial measurement
std::chrono::time_point<std::chrono::system_clock> time = std::chrono::system_clock::now();

double position[3] = {0.0, 0.0, 0.0};
double velocity[3] = {0.0, 0.0, 0.0};
double acceleration[3] = {0.0, 0.0, 0.0};
double positionVar[3] = {0.0, 0.0, 0.0};
double velocityVar[3] = {0.0, 0.0, 0.0};
double accelarationVar[3] = {0.0, 0.0, 0.0};

Filter::PointState<3, 3> initialMeasurement(position, velocity, acceleration,
                                            positionVar, velocityVar, accelarationVar, time);

// Initialize the Kalman Filter
kf.initialize(initialMeasurement);

// Create a measurement update
double updatedPosition[3] = {0.0, 0.0, 0.0};
double updatedVelocity[3] = {0.0, 0.0, 0.0};
double updatedAcceleration[3] = {0.0, 0.0, 0.0};
std::chrono::time_point<std::chrono::system_clock> newTime = std::chrono::system_clock::now();

Filter::PointState<3, 3> newMeasurement(position, velocity, acceleration,
                                        positionVar, velocityVar, accelarationVar, newTime);

// Update the Kalman Filter
kf.update(newMeasurement);

// Get state
Filter::PointState<3, 3> state = kf.getState();

// Get residuals
Matrix<9, 1> residuals = kf.getResiduals();

```
