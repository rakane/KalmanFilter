#include "PointState.h"
#include "KalmanFilter.h"

#include <iostream>
#include <random>
#include <chrono>
#include <sstream>
#include <string>

// Run python scripts to plot data
template <unsigned int NUM_DIMENSIONS, unsigned int NUM_STATES>
void execPlotScript(
    const std::vector<Filter::PointState<NUM_DIMENSIONS, NUM_STATES>>& truth, 
    const std::vector<Filter::PointState<NUM_DIMENSIONS, NUM_STATES>>& states, 
    const std::vector<Filter::PointState<NUM_DIMENSIONS, NUM_STATES>>& measurements,
    const std::vector<double>& residualSum)
{
    // Plot x,y,z position data vs truth
    std::ostringstream oss;
    oss << "python3 plot.py ";

    // Num dimensions
    oss << NUM_DIMENSIONS << " ";

    for(unsigned int dimensionIdx = 0; dimensionIdx < NUM_DIMENSIONS; dimensionIdx++)
    {
        for(unsigned int i = 0; i < truth.size(); i++)
        {
            oss << truth[i].state[dimensionIdx][0];
            i != truth.size() - 1 ? oss << "," : oss << " ";
        }
        oss << " ";

        for(unsigned int i = 0; i < states.size(); i++)
        {
            oss << states[i].state[dimensionIdx][0];
            i != states.size() - 1 ? oss << "," : oss << " ";
        }
        oss << " ";

        for(unsigned int i = 0; i < measurements.size(); i++)
        {
            oss << measurements[i].state[dimensionIdx][0];
            i != measurements.size() - 1 ? oss << "," : oss << " ";
        }
        oss << " ";   
    }

    system(oss.str().c_str());

    // Plot residuals
    std::ostringstream oss2;
    oss2 << "python3 plotResiduals.py ";

    for(unsigned int i = 0; i < residualSum.size(); i++)
    {
        oss2 << residualSum[i];
        i != residualSum.size() - 1 ? oss2 << "," : oss2 << " ";
    }

    system(oss2.str().c_str());
}

// 3D Example with 3 state variables
void run3DExample()
{
    // Random number generator
    std::random_device rd;
    std::mt19937 gen(rd());

    // INPUTS
    const unsigned int NUM_ITERATIONS = 20;
    const double TRUTH_INITIAL_POS_X = 0.0;
    const double TRUTH_INITIAL_POS_Y = 500.0;
    const double TRUTH_INITIAL_POS_Z = 1000.0;
    double TRUTH_INITIAL_POS[3] = {TRUTH_INITIAL_POS_X, TRUTH_INITIAL_POS_Y, TRUTH_INITIAL_POS_Z};

    const double TRUTH_INITIAL_VEL_X = 0.1;
    const double TRUTH_INITIAL_VEL_Y = 0.2;
    const double TRUTH_INITIAL_VEL_Z = 0.02;
    double TRUTH_INITIAL_VEL[3] = {TRUTH_INITIAL_VEL_X, TRUTH_INITIAL_VEL_Y, TRUTH_INITIAL_VEL_Z};

    const double TRUTH_INITIAL_ACCEL_X = 0.0;
    const double TRUTH_INITIAL_ACCEL_Y = 0.0;
    const double TRUTH_INITIAL_ACCEL_Z = 0.0;
    double TRUTH_INITIAL_ACCEL[3] = {TRUTH_INITIAL_ACCEL_X, TRUTH_INITIAL_ACCEL_Y, TRUTH_INITIAL_ACCEL_Z};

    const double MEAS_POS_STD = 2.0;
    const double MEAS_VEL_STD = 0.5;
    const double MEAS_ACCEL_STD = 0.5;
    const double MEAS_POS_VAR = std::pow(MEAS_POS_STD, 2);
    const double MEAS_VEL_VAR = std::pow(MEAS_VEL_STD, 2);
    const double MEAS_ACCEL_VAR = std::pow(MEAS_ACCEL_STD, 2);

    // Run the filter for X iterations (1 second apart)
    std::vector<Filter::PointState<3, 3>> truth;
    std::vector<Filter::PointState<3, 3>> measurements;
    std::vector<Filter::PointState<3, 3>> states;
    std::vector<double> residualSums;

    Filter::KalmanFilterParameters parameters;
    parameters.processNoise = 0.1;

    Filter::KalmanFilter<3, 3> kf(parameters);

    // Initialize the filter state
    std::chrono::time_point<std::chrono::system_clock> startTime = std::chrono::system_clock::now();
    std::normal_distribution<double> positionXDistribution(TRUTH_INITIAL_POS_X, MEAS_POS_STD);
    std::normal_distribution<double> positionYDistribution(TRUTH_INITIAL_POS_Y, MEAS_POS_STD);
    std::normal_distribution<double> positionZDistribution(TRUTH_INITIAL_POS_Z, MEAS_POS_STD);
    std::normal_distribution<double> velocityXDistribution(TRUTH_INITIAL_VEL_X, MEAS_VEL_STD);
    std::normal_distribution<double> velocityYDistribution(TRUTH_INITIAL_VEL_Y, MEAS_VEL_STD);
    std::normal_distribution<double> velocityZDistribution(TRUTH_INITIAL_VEL_Z, MEAS_VEL_STD);
    std::normal_distribution<double> accelerationXDistribution(TRUTH_INITIAL_ACCEL_X, MEAS_ACCEL_STD);
    std::normal_distribution<double> accelerationYDistribution(TRUTH_INITIAL_ACCEL_Y, MEAS_ACCEL_STD);
    std::normal_distribution<double> accelerationZDistribution(TRUTH_INITIAL_ACCEL_Z, MEAS_ACCEL_STD);

    double position[3] = {positionXDistribution(gen), positionYDistribution(gen), positionZDistribution(gen)};
    double velocity[3] = {velocityXDistribution(gen), velocityYDistribution(gen), velocityZDistribution(gen)};
    double acceleration[3] = {accelerationXDistribution(gen), accelerationYDistribution(gen), accelerationZDistribution(gen)};
    double positionVar[3] = {MEAS_POS_VAR, MEAS_POS_VAR, MEAS_POS_VAR};
    double velocityVar[3] = {MEAS_VEL_VAR, MEAS_VEL_VAR, MEAS_VEL_VAR};
    double accelarationVar[3] = {MEAS_ACCEL_VAR, MEAS_ACCEL_VAR, MEAS_ACCEL_VAR};

    Filter::PointState<3, 3> state(position, velocity, acceleration,
                                        positionVar, velocityVar, accelarationVar, startTime);

    kf.initializeState(state);

    // Log initial data
    truth.emplace_back(TRUTH_INITIAL_POS, TRUTH_INITIAL_VEL, TRUTH_INITIAL_ACCEL,
                        positionVar, velocityVar, accelarationVar, startTime);

    states.emplace_back(position, velocity, acceleration,
                        positionVar, velocityVar, accelarationVar, startTime);

    measurements.emplace_back(position, velocity, acceleration,
                        positionVar, velocityVar, accelarationVar, startTime);

    // Run the filter
    for(unsigned int i = 1; i <= NUM_ITERATIONS; i++)
    {
        // New time
        std::chrono::time_point<std::chrono::system_clock> time = startTime + std::chrono::seconds(i);

        // Calculate true target state
        double deltaT = std::chrono::duration_cast<std::chrono::nanoseconds>(
            time - startTime).count() / 1.0E9;

        double trueAccelerationX = TRUTH_INITIAL_ACCEL_X;
        double trueAccelerationY = TRUTH_INITIAL_ACCEL_Y;
        double trueAccelerationZ = TRUTH_INITIAL_ACCEL_Z;

        double trueVelocityX = TRUTH_INITIAL_VEL_X + TRUTH_INITIAL_ACCEL_X * deltaT;
        double trueVelocityY = TRUTH_INITIAL_VEL_Y + TRUTH_INITIAL_ACCEL_Y * deltaT;
        double trueVelocityZ = TRUTH_INITIAL_VEL_Z + TRUTH_INITIAL_ACCEL_Z * deltaT;

        double truePositionX = TRUTH_INITIAL_POS_X + TRUTH_INITIAL_VEL_X * deltaT 
                                + 0.5 * TRUTH_INITIAL_ACCEL_X * std::pow(deltaT, 2);
        double truePositionY = TRUTH_INITIAL_POS_Y + TRUTH_INITIAL_VEL_Y * deltaT
                                + 0.5 * TRUTH_INITIAL_ACCEL_Y * std::pow(deltaT, 2);
        double truePositionZ = TRUTH_INITIAL_POS_Z + TRUTH_INITIAL_VEL_Z * deltaT
                                + 0.5 * TRUTH_INITIAL_ACCEL_Z * std::pow(deltaT, 2);

        double truePosition[3] = {truePositionX, truePositionY, truePositionZ};
        double trueVelocity[3] = {trueVelocityX, trueVelocityY, trueVelocityZ};
        double trueAcceleration[3] = {trueAccelerationX, trueAccelerationY, trueAccelerationZ};

        truth.emplace_back(truePosition, trueVelocity, trueAcceleration, 
                            positionVar, velocityVar, accelarationVar, 
                            time);

        // Generate a measurement with variance
        std::normal_distribution<double> positionDistributionX(truePositionX, MEAS_POS_STD);
        std::normal_distribution<double> positionDistributionY(truePositionY, MEAS_POS_STD);
        std::normal_distribution<double> positionDistributionZ(truePositionZ, MEAS_POS_STD);
        std::normal_distribution<double> velocityDistributionX(trueVelocityX, MEAS_VEL_STD);
        std::normal_distribution<double> velocityDistributionY(trueVelocityY, MEAS_VEL_STD);
        std::normal_distribution<double> velocityDistributionZ(trueVelocityZ, MEAS_VEL_STD);
        std::normal_distribution<double> accelerationDistributionX(trueAccelerationX, MEAS_ACCEL_STD);
        std::normal_distribution<double> accelerationDistributionY(trueAccelerationY, MEAS_ACCEL_STD);
        std::normal_distribution<double> accelerationDistributionZ(trueAccelerationZ, MEAS_ACCEL_STD);

        double measurementPosition[3] = {positionDistributionX(gen), positionDistributionY(gen), positionDistributionZ(gen)};
        double measurementVelocity[3] = {velocityDistributionX(gen), velocityDistributionY(gen), velocityDistributionZ(gen)};
        double measurementAcceleration[3] = {0.0, 0.0, 0.0};

        // Update the filter with measurement
        Filter::PointState<3, 3> measurement(
            measurementPosition, measurementVelocity, measurementAcceleration,
            positionVar, velocityVar, accelarationVar, time);

        kf.update(measurement);

        // Get the state
        Filter::PointState<3, 3> kfState = kf.getState();

        // Get residuals
        Matrix<9, 1> residuals = kf.getResiduals();

        // Calculate residual sum
        double residualSum = 0.0;
        for(unsigned int i = 0; i < 3; i++)
        {
            residualSum += residuals[i][0];
        }

        residualSums.push_back(residualSum);
        measurements.emplace_back(measurementPosition, measurementVelocity, measurementAcceleration,
                        positionVar, velocityVar, accelarationVar, time);

        double statePosition[3] = {kfState.state[0][0], kfState.state[1][0], kfState.state[2][0]};
        double stateVelocity[3] = {kfState.state[3][0], kfState.state[4][0], kfState.state[5][0]};
        double stateAcceleration[3] = {kfState.state[6][0], kfState.state[7][0], kfState.state[8][0]};
        double statePositionVar[3] = {kfState.covariance[0][0], kfState.covariance[1][1], kfState.covariance[2][2]};
        double stateVelocityVar[3] = {kfState.covariance[3][3], kfState.covariance[4][4], kfState.covariance[5][5]};
        double stateAccelerationVar[3] = {kfState.covariance[6][6], kfState.covariance[7][7], kfState.covariance[8][8]};
        std::chrono::time_point<std::chrono::system_clock> stateTime = kfState.getTime();

        states.emplace_back(statePosition, stateVelocity, stateAcceleration,
                        statePositionVar, stateVelocityVar, stateAccelerationVar, stateTime);


        if(i == NUM_ITERATIONS - 1)
        {
            std::cout << "Final Truth State: " << std::endl;
            std::cout << "  Position: " << truePosition[0] << ", " << truePosition[1] << ", " << truePosition[2] << std::endl;
            std::cout << "  Velocity: " << trueVelocity[0] << ", " << trueVelocity[1] << ", " << trueVelocity[2] << std::endl;
            std::cout << "  Acceleration: " << trueAcceleration[0] << ", " << trueAcceleration[1] << ", " << trueAcceleration[2] << std::endl;
        }
    }

    Filter::PointState<3, 3> finalState = kf.getState();
    double finalPosition[3];
    double finalVelocity[3];
    double finalAcceleration[3];
    finalState.getPosition(finalPosition);
    finalState.getVelocity(finalVelocity);
    finalState.getAcceleration(finalAcceleration);

    

    std::cout << "\nFinal Filter State: " << std::endl;
    std::cout << "  Position: " << finalPosition[0] << ", " << finalPosition[1] << ", " << finalPosition[2] << std::endl;
    std::cout << "  Velocity: " << finalVelocity[0] << ", " << finalVelocity[1] << ", " << finalVelocity[2] << std::endl;
    std::cout << "  Acceleration: " << finalAcceleration[0] << ", " << finalAcceleration[1] << ", " << finalAcceleration[2] << std::endl;
    std::cout << std::endl;

    execPlotScript<3, 3>(truth, states, measurements, residualSums);
}

// 2D Example with 3 state variables
void run2DExample()
{
    // Random number generator
    std::random_device rd;
    std::mt19937 gen(rd());

    // INPUTS
    const unsigned int NUM_ITERATIONS = 10000;
    const double TRUTH_INITIAL_POS_X = 0.0;
    const double TRUTH_INITIAL_POS_Y = 500.0;
    double TRUTH_INITIAL_POS[2] = {TRUTH_INITIAL_POS_X, TRUTH_INITIAL_POS_Y};

    const double TRUTH_INITIAL_VEL_X = 0.1;
    const double TRUTH_INITIAL_VEL_Y = 0.2;
    double TRUTH_INITIAL_VEL[2] = {TRUTH_INITIAL_VEL_X, TRUTH_INITIAL_VEL_Y};

    const double TRUTH_INITIAL_ACCEL_X = 0.0;
    const double TRUTH_INITIAL_ACCEL_Y = 0.0;
    double TRUTH_INITIAL_ACCEL[2] = {TRUTH_INITIAL_ACCEL_X, TRUTH_INITIAL_ACCEL_Y};

    const double MEAS_POS_STD = 2.0;
    const double MEAS_VEL_STD = 0.5;
    const double MEAS_ACCEL_STD = 0.5;
    const double MEAS_POS_VAR = std::pow(MEAS_POS_STD, 2);
    const double MEAS_VEL_VAR = std::pow(MEAS_VEL_STD, 2);
    const double MEAS_ACCEL_VAR = std::pow(MEAS_ACCEL_STD, 2);

    // Run the filter for X iterations (1 second apart)
    std::vector<Filter::PointState<2, 3>> truth;
    std::vector<Filter::PointState<2, 3>> measurements;
    std::vector<Filter::PointState<2, 3>> states;
    std::vector<double> residualSums;

    Filter::KalmanFilterParameters parameters;
    parameters.processNoise = 0.1;

    Filter::KalmanFilter<2, 3> kf(parameters);

    // Initialize the filter state
    std::chrono::time_point<std::chrono::system_clock> startTime = std::chrono::system_clock::now();
    std::normal_distribution<double> positionXDistribution(TRUTH_INITIAL_POS_X, MEAS_POS_STD);
    std::normal_distribution<double> positionYDistribution(TRUTH_INITIAL_POS_Y, MEAS_POS_STD);
    std::normal_distribution<double> velocityXDistribution(TRUTH_INITIAL_VEL_X, MEAS_VEL_STD);
    std::normal_distribution<double> velocityYDistribution(TRUTH_INITIAL_VEL_Y, MEAS_VEL_STD);
    std::normal_distribution<double> accelerationXDistribution(TRUTH_INITIAL_ACCEL_X, MEAS_ACCEL_STD);
    std::normal_distribution<double> accelerationYDistribution(TRUTH_INITIAL_ACCEL_Y, MEAS_ACCEL_STD);

    double position[2] = {positionXDistribution(gen), positionYDistribution(gen)};
    double velocity[2] = {velocityXDistribution(gen), velocityYDistribution(gen)};
    double acceleration[2] = {accelerationXDistribution(gen), accelerationYDistribution(gen)};
    double positionVar[2] = {MEAS_POS_VAR, MEAS_POS_VAR};
    double velocityVar[2] = {MEAS_VEL_VAR, MEAS_VEL_VAR};
    double accelerationVar[2] = {MEAS_ACCEL_VAR, MEAS_ACCEL_VAR};

    Filter::PointState<2, 3> state(position, velocity, acceleration,
                                        positionVar, velocityVar, accelerationVar, startTime);

    kf.initializeState(state);

    // Log data
    truth.emplace_back(TRUTH_INITIAL_POS, TRUTH_INITIAL_VEL, TRUTH_INITIAL_ACCEL,
                        positionVar, velocityVar, accelerationVar, startTime);

    states.emplace_back(position, velocity, acceleration,
                        positionVar, velocityVar, accelerationVar, startTime);

    measurements.emplace_back(position, velocity, acceleration,
                        positionVar, velocityVar, accelerationVar, startTime);

    for(unsigned int i = 1; i <= NUM_ITERATIONS; i++)
    {
        // New time
        std::chrono::time_point<std::chrono::system_clock> time = startTime + std::chrono::seconds(i);

        // Calculate true target state
        double deltaT = std::chrono::duration_cast<std::chrono::nanoseconds>(
            time - startTime).count() / 1.0E9;

        double trueVelocityX = TRUTH_INITIAL_VEL_X;
        double trueVelocityY = TRUTH_INITIAL_VEL_Y;

        double truePositionX = TRUTH_INITIAL_POS_X + TRUTH_INITIAL_VEL_X * deltaT 
                                + 0.5 * TRUTH_INITIAL_ACCEL_X * std::pow(deltaT, 2);
        double truePositionY = TRUTH_INITIAL_POS_Y + TRUTH_INITIAL_VEL_Y * deltaT
                                + 0.5 * TRUTH_INITIAL_ACCEL_Y * std::pow(deltaT, 2);

        double truePosition[2] = {truePositionX, truePositionY};
        double trueVelocity[2] = {trueVelocityX, trueVelocityY};

        truth.emplace_back(truePosition, trueVelocity, 
                            positionVar, velocityVar, 
                            time);

        // Generate a measurement with variance
        std::normal_distribution<double> positionDistributionX(truePositionX, MEAS_POS_STD);
        std::normal_distribution<double> positionDistributionY(truePositionY, MEAS_POS_STD);
        std::normal_distribution<double> velocityDistributionX(trueVelocityX, MEAS_VEL_STD);
        std::normal_distribution<double> velocityDistributionY(trueVelocityY, MEAS_VEL_STD);

        double measurementPosition[2] = {positionDistributionX(gen), positionDistributionY(gen)};
        double measurementVelocity[2] = {velocityDistributionX(gen), velocityDistributionY(gen)};
        double measurementAccelaration[2] = {0.0, 0.0};


        // Update the filter with measurement
        Filter::PointState<2, 3> measurement(measurementPosition, measurementVelocity, measurementAccelaration,
                                                positionVar, velocityVar, accelerationVar, time);

        kf.update(measurement);

        // Get the state
        Filter::PointState<2, 3> kfState = kf.getState();

        // Get residuals
        Matrix<6, 1> residuals = kf.getResiduals();

        // Calculate residual sum
        double residualSum = 0.0;
        for(unsigned int i = 0; i < 2; i++)
        {
            residualSum += residuals[i][0];
        }

        residualSums.push_back(residualSum);
        measurements.emplace_back(measurementPosition, measurementVelocity,
                        positionVar, velocityVar, time);

        double statePosition[2] = {kfState.state[0][0], kfState.state[1][0]};
        double stateVelocity[2] = {kfState.state[3][0], kfState.state[4][0]};
        double statePositionVar[2] = {kfState.covariance[0][0], kfState.covariance[1][1]};
        double stateVelocityVar[2] = {kfState.covariance[3][3], kfState.covariance[4][4]};
        std::chrono::time_point<std::chrono::system_clock> stateTime = kfState.getTime();

        states.emplace_back(statePosition, stateVelocity,
                        statePositionVar, stateVelocityVar, stateTime);


        if(i == NUM_ITERATIONS - 1)
        {
            std::cout << "\nFinal Truth State: " << std::endl;
            std::cout << "  Position: " << truePosition[0] << ", " << truePosition[1] << std::endl;
            std::cout << "  Velocity: " << trueVelocity[0] << ", " << trueVelocity[1] << std::endl;
        }
    }

    Filter::PointState<2, 3> finalState = kf.getState();
    double finalPosition[2];
    double finalVelocity[2];
    finalState.getPosition(finalPosition);
    finalState.getVelocity(finalVelocity);

    std::cout << "Final Filter State: " << std::endl;
    std::cout << "  Position: " << finalPosition[0] << ", " << finalPosition[1] << std::endl;
    std::cout << "  Velocity: " << finalVelocity[0] << ", " << finalVelocity[1] << std::endl;

    execPlotScript<2, 3>(truth, states, measurements, residualSums);
}

// 3D Example with 2 state variables
void run3DExample2()
{
    // Random number generator
    std::random_device rd;
    std::mt19937 gen(rd());

    // INPUTS
    const unsigned int NUM_ITERATIONS = 10000;
    const double TRUTH_INITIAL_POS_X = 0.0;
    const double TRUTH_INITIAL_POS_Y = 500.0;
    const double TRUTH_INITIAL_POS_Z = 1000.0;
    double TRUTH_INITIAL_POS[3] = {TRUTH_INITIAL_POS_X, TRUTH_INITIAL_POS_Y, TRUTH_INITIAL_POS_Z};

    const double TRUTH_INITIAL_VEL_X = 0.1;
    const double TRUTH_INITIAL_VEL_Y = 0.2;
    const double TRUTH_INITIAL_VEL_Z = 0.02;
    double TRUTH_INITIAL_VEL[3] = {TRUTH_INITIAL_VEL_X, TRUTH_INITIAL_VEL_Y, TRUTH_INITIAL_VEL_Z};

    const double MEAS_POS_STD = 2.0;
    const double MEAS_VEL_STD = 0.5;
    const double MEAS_POS_VAR = std::pow(MEAS_POS_STD, 2);
    const double MEAS_VEL_VAR = std::pow(MEAS_VEL_STD, 2);

    // Run the filter for X iterations (1 second apart)
    std::vector<Filter::PointState<3, 2>> truth;
    std::vector<Filter::PointState<3, 2>> measurements;
    std::vector<Filter::PointState<3, 2>> states;
    std::vector<double> residualSums;

    Filter::KalmanFilterParameters parameters;
    parameters.processNoise = 0.1;

    Filter::KalmanFilter<3, 2> kf(parameters);

    // Initialize the filter state
    std::chrono::time_point<std::chrono::system_clock> startTime = std::chrono::system_clock::now();
    std::normal_distribution<double> positionXDistribution(TRUTH_INITIAL_POS_X, MEAS_POS_STD);
    std::normal_distribution<double> positionYDistribution(TRUTH_INITIAL_POS_Y, MEAS_POS_STD);
    std::normal_distribution<double> positionZDistribution(TRUTH_INITIAL_POS_Z, MEAS_POS_STD);
    std::normal_distribution<double> velocityXDistribution(TRUTH_INITIAL_VEL_X, MEAS_VEL_STD);
    std::normal_distribution<double> velocityYDistribution(TRUTH_INITIAL_VEL_Y, MEAS_VEL_STD);
    std::normal_distribution<double> velocityZDistribution(TRUTH_INITIAL_VEL_Z, MEAS_VEL_STD);

    double position[3] = {positionXDistribution(gen), positionYDistribution(gen), positionZDistribution(gen)};
    double velocity[3] = {velocityXDistribution(gen), velocityYDistribution(gen), velocityZDistribution(gen)};
    double positionVar[3] = {MEAS_POS_VAR, MEAS_POS_VAR, MEAS_POS_VAR};
    double velocityVar[3] = {MEAS_VEL_VAR, MEAS_VEL_VAR, MEAS_VEL_VAR};

    Filter::PointState<3, 2> state(position, velocity,
                                        positionVar, velocityVar, startTime);

    kf.initializeState(state);

    // Log initial data
    truth.emplace_back(TRUTH_INITIAL_POS, TRUTH_INITIAL_VEL,
                        positionVar, velocityVar, startTime);

    states.emplace_back(position, velocity,
                        positionVar, velocityVar, startTime);

    measurements.emplace_back(position, velocity,
                        positionVar, velocityVar, startTime);

    // Run the filter
    for(unsigned int i = 1; i <= NUM_ITERATIONS; i++)
    {
        // New time
        std::chrono::time_point<std::chrono::system_clock> time = startTime + std::chrono::seconds(i);

        // Calculate true target state
        double deltaT = std::chrono::duration_cast<std::chrono::nanoseconds>(
            time - startTime).count() / 1.0E9;

        double trueVelocityX = TRUTH_INITIAL_VEL_X;
        double trueVelocityY = TRUTH_INITIAL_VEL_Y;
        double trueVelocityZ = TRUTH_INITIAL_VEL_Z;

        double truePositionX = TRUTH_INITIAL_POS_X + TRUTH_INITIAL_VEL_X * deltaT;
        double truePositionY = TRUTH_INITIAL_POS_Y + TRUTH_INITIAL_VEL_Y * deltaT;
        double truePositionZ = TRUTH_INITIAL_POS_Z + TRUTH_INITIAL_VEL_Z * deltaT;

        double truePosition[3] = {truePositionX, truePositionY, truePositionZ};
        double trueVelocity[3] = {trueVelocityX, trueVelocityY, trueVelocityZ};

        truth.emplace_back(truePosition, trueVelocity, 
                            positionVar, velocityVar, time);

        // Generate a measurement with variance
        std::normal_distribution<double> positionDistributionX(truePositionX, MEAS_POS_STD);
        std::normal_distribution<double> positionDistributionY(truePositionY, MEAS_POS_STD);
        std::normal_distribution<double> positionDistributionZ(truePositionZ, MEAS_POS_STD);
        std::normal_distribution<double> velocityDistributionX(trueVelocityX, MEAS_VEL_STD);
        std::normal_distribution<double> velocityDistributionY(trueVelocityY, MEAS_VEL_STD);
        std::normal_distribution<double> velocityDistributionZ(trueVelocityZ, MEAS_VEL_STD);

        double measurementPosition[3] = {positionDistributionX(gen), positionDistributionY(gen), positionDistributionZ(gen)};
        double measurementVelocity[3] = {velocityDistributionX(gen), velocityDistributionY(gen), velocityDistributionZ(gen)};

        // Update the filter with measurement
        Filter::PointState<3, 2> measurement(measurementPosition, measurementVelocity, positionVar, velocityVar, time);

        kf.update(measurement);

        // Get the state
        Filter::PointState<3, 2> kfState = kf.getState();

        // Get residuals
        Matrix<6, 1> residuals = kf.getResiduals();

        // Calculate residual sum
        double residualSum = 0.0;
        for(unsigned int i = 0; i < 3; i++)
        {
            residualSum += residuals[i][0];
        }

        residualSums.push_back(residualSum);
        measurements.emplace_back(measurementPosition, measurementVelocity, positionVar, velocityVar, time);

        double statePosition[3] = {kfState.state[0][0], kfState.state[1][0], kfState.state[2][0]};
        double stateVelocity[3] = {kfState.state[3][0], kfState.state[4][0], kfState.state[5][0]};
        double statePositionVar[3] = {kfState.covariance[0][0], kfState.covariance[1][1], kfState.covariance[2][2]};
        double stateVelocityVar[3] = {kfState.covariance[3][3], kfState.covariance[4][4], kfState.covariance[5][5]};
        std::chrono::time_point<std::chrono::system_clock> stateTime = kfState.getTime();

        states.emplace_back(statePosition, stateVelocity, statePositionVar, stateVelocityVar, stateTime);

        if(i == NUM_ITERATIONS - 1)
        {
            std::cout << "Final Truth State: " << std::endl;
            std::cout << "  Position: " << truePosition[0] << ", " << truePosition[1] << ", " << truePosition[2] << std::endl;
            std::cout << "  Velocity: " << trueVelocity[0] << ", " << trueVelocity[1] << ", " << trueVelocity[2] << std::endl;
        }
    }

    Filter::PointState<3, 2> finalState = kf.getState();
    double finalPosition[3];
    double finalVelocity[3];
    finalState.getPosition(finalPosition);
    finalState.getVelocity(finalVelocity);    

    std::cout << "\nFinal Filter State: " << std::endl;
    std::cout << "  Position: " << finalPosition[0] << ", " << finalPosition[1] << ", " << finalPosition[2] << std::endl;
    std::cout << "  Velocity: " << finalVelocity[0] << ", " << finalVelocity[1] << ", " << finalVelocity[2] << std::endl;
    std::cout << std::endl;

    execPlotScript<3, 2>(truth, states, measurements, residualSums);
}

int main(int argc, char** argv)
{
    unsigned int numDimensions = 3;
    unsigned int numStates = 3;

    if(numDimensions == 3)
    {
        if(numStates == 3)
        {
            run3DExample();
        }
        else
        {
            std::cout << "Running 3D example 2" << std::endl;
            run3DExample2();
        }
    }
    else
    {
        run2DExample();
    }

    return 0;
}