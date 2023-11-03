#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <iostream>
#include <chrono>

#include "PointState.h"
#include "Matrix.h"

namespace Filter {
    struct KalmanFilterParameters {
        double processNoise;
    };

    template <unsigned int NUM_DIMENSIONS, unsigned int NUM_STATES>
    class KalmanFilter {
    public:
        // Constructor
        KalmanFilter(KalmanFilterParameters parameters);

        // Destructor
        ~KalmanFilter();

        // Initialize the state
        void initializeState(const PointState<NUM_DIMENSIONS, NUM_STATES>& state);

        // Update the state
        void update(const PointState<NUM_DIMENSIONS, NUM_STATES>& measurement);

        // Get the state
        PointState<NUM_DIMENSIONS, NUM_STATES> getState() const;

        // Get the post-update residuals
        Matrix<NUM_DIMENSIONS * NUM_STATES, 1> getResiduals() const;

    private:

        // Predict the state
        PointState<NUM_DIMENSIONS, NUM_STATES> predict(std::chrono::time_point<std::chrono::system_clock> time);

        // Get observation matrix
        Matrix<NUM_DIMENSIONS * NUM_STATES, NUM_DIMENSIONS * NUM_STATES> getObservationMatrix(const Filter::PointState<NUM_DIMENSIONS, NUM_STATES>& measurement) const;

        // Get predicted state matrix
        Matrix<NUM_DIMENSIONS * NUM_STATES, 1> getPredictedStateMatrix(const PointState<NUM_DIMENSIONS, NUM_STATES>& predictedState) const;

        // Get measurement state matrix
        Matrix<NUM_DIMENSIONS * NUM_STATES, 1> getMeasurementStateMatrix(const PointState<NUM_DIMENSIONS, NUM_STATES>& measurement) const;
        
        // Compute residuals
        void computeResiduals(const PointState<NUM_DIMENSIONS, NUM_STATES>& measurement);

        //
        // Data members
        //

        // Filter parameters
        KalmanFilterParameters parameters_;

        // State
        PointState<NUM_DIMENSIONS, NUM_STATES> state_;

        // Updated residuals
        double residuals_[NUM_DIMENSIONS * NUM_STATES];

        // Initialized
        bool initialized_;

    };
}

template <unsigned int NUM_DIMENSIONS, unsigned int NUM_STATES>
Filter::KalmanFilter<NUM_DIMENSIONS, NUM_STATES>::KalmanFilter(KalmanFilterParameters parameters)
    : parameters_(parameters), state_(), initialized_(false)
{
    // Initialize residuals to zero
    for(unsigned int i = 0; i < NUM_STATES * NUM_DIMENSIONS; i++)
    {
        residuals_[i] = 0.0;
    }
}

template <unsigned int NUM_DIMENSIONS, unsigned int NUM_STATES>
Filter::KalmanFilter<NUM_DIMENSIONS, NUM_STATES>::~KalmanFilter()
{
    // Do nothing
}

template <unsigned int NUM_DIMENSIONS, unsigned int NUM_STATES>
void Filter::KalmanFilter<NUM_DIMENSIONS, NUM_STATES>::initializeState(const PointState<NUM_DIMENSIONS, NUM_STATES>& state)
{
    state_ = state;
    initialized_ = true;
}

template <unsigned int NUM_DIMENSIONS, unsigned int NUM_STATES>
Filter::PointState<NUM_DIMENSIONS, NUM_STATES> Filter::KalmanFilter<NUM_DIMENSIONS, NUM_STATES>::getState() const
{
    return state_;
}

template <unsigned int NUM_DIMENSIONS, unsigned int NUM_STATES>
Matrix<NUM_DIMENSIONS * NUM_STATES, 1> Filter::KalmanFilter<NUM_DIMENSIONS, NUM_STATES>::getResiduals() const
{
    Matrix<NUM_DIMENSIONS * NUM_STATES, 1> residuals;

    for (unsigned int i = 0; i < NUM_STATES * NUM_DIMENSIONS; i++)
    {
        residuals[i][0] = residuals_[i];
    }

    return residuals;
}

template <unsigned int NUM_DIMENSIONS, unsigned int NUM_STATES>
void Filter::KalmanFilter<NUM_DIMENSIONS, NUM_STATES>::update(const PointState<NUM_DIMENSIONS, NUM_STATES>& measurement)
{
    // If the filter has not been initialized, set the initial 
    // state to the measurement and return
    if(!initialized_)
    {
        state_ = measurement;
        initialized_ = true;
        return;
    }

    // Predict current state to measurement time
    PointState<NUM_DIMENSIONS, NUM_STATES> predictedState = predict(measurement.time);

    // Update state time
    state_.time = measurement.time;

    // Create identity matrix (I) for use later
    Matrix<NUM_STATES * NUM_DIMENSIONS, NUM_STATES * NUM_DIMENSIONS> identityMatrix;
    for(unsigned int i = 0; i < NUM_STATES * NUM_DIMENSIONS; i++)
    {
        for(unsigned int j = 0; j < NUM_STATES * NUM_DIMENSIONS; j++)
        {
            identityMatrix[i][j] = 0.0;

            if(i == j)
            {
                identityMatrix[i][j] = 1.0;
            }
        }
    }

    // Get observation matrix (Hk)
    Matrix<NUM_STATES * NUM_DIMENSIONS, NUM_STATES * NUM_DIMENSIONS> observationMatrix = getObservationMatrix(measurement);

    // Get predicted state matrix (Xk|k-1)
    Matrix <NUM_STATES * NUM_DIMENSIONS, 1> predictedStateMatrix = getPredictedStateMatrix(predictedState);

    // Get predicted covariance matrix (Pk|k-1)
    Matrix<NUM_STATES * NUM_DIMENSIONS, NUM_STATES * NUM_DIMENSIONS> predictedCovarianceMatrix(predictedState.covariance);

    // Get measurement state matrix (Zk)
    Matrix <NUM_STATES * NUM_DIMENSIONS, 1> measurementStateMatrix = getMeasurementStateMatrix(measurement);

    // Get measurement covariance matrix (Rk)
    Matrix<NUM_STATES * NUM_DIMENSIONS, NUM_STATES * NUM_DIMENSIONS> measurementCovarianceMatrix(measurement.covariance);

    // Compute state residual (Yk = Zk - Hk * Xk|k-1)
    Matrix<NUM_STATES * NUM_DIMENSIONS, 1> stateResidual = measurementStateMatrix - (observationMatrix * predictedStateMatrix);

    // Compute covariance residual (Sk = Hk * Pk|k-1 * HkT + Rk)
    Matrix<NUM_STATES * NUM_DIMENSIONS, NUM_STATES * NUM_DIMENSIONS> covarianceResidual = observationMatrix * predictedCovarianceMatrix * 
                                        observationMatrix.transpose() + measurementCovarianceMatrix;

    // Calculate Kalman gain (Kk = Pk|k-1 * HkT * Sk^-1)
    Matrix<NUM_STATES * NUM_DIMENSIONS, NUM_STATES * NUM_DIMENSIONS> kalmanGain = predictedCovarianceMatrix * observationMatrix.transpose() 
                                * covarianceResidual.inverse();

    // Update state (Xk = Xk|k-1 + Kk * Yk), this could also be represented 
    // as: Xk = (I - KkHk) * Xk|k-1 + KkZk
    Matrix<NUM_STATES * NUM_DIMENSIONS, 1> updatedState = predictedStateMatrix + kalmanGain * stateResidual;

    for(unsigned int i = 0; i < NUM_STATES * NUM_DIMENSIONS; i++)
    {
        state_.state[i][0] = updatedState[i][0];
    }

    // Update covariance (Pk = (I - Kk * Hk) * Pk|k-1)
    Matrix<NUM_STATES * NUM_DIMENSIONS, NUM_STATES * NUM_DIMENSIONS> updatedCovariance 
        = (identityMatrix - kalmanGain * observationMatrix) * predictedCovarianceMatrix;

    for(unsigned int i = 0; i < NUM_STATES * NUM_DIMENSIONS; i++)
    {
        for(unsigned int j = 0; j < NUM_STATES * NUM_DIMENSIONS; j++)
        {
            state_.covariance[i][j] = updatedCovariance[i][j];
        }
    }

    // Compute post-fit residual (Yk|k = Zk - Hk * Xk|k)
    computeResiduals(measurement);
}

template <unsigned int NUM_DIMENSIONS, unsigned int NUM_STATES>
Filter::PointState<NUM_DIMENSIONS, NUM_STATES> Filter::KalmanFilter<NUM_DIMENSIONS, NUM_STATES>::predict(
    std::chrono::time_point<std::chrono::system_clock> time)
{

    PointState<NUM_DIMENSIONS, NUM_STATES> predictedState = state_;
    predictedState.time = time;

    double deltaT = std::chrono::duration_cast<std::chrono::nanoseconds>(
        predictedState.time - state_.time).count() / 1.0e9;

    // Constant position equations:
    //  Xn = Xn-1
    //  Vn = 0
    //  An = 0
    //
    // pPn = pPn-1
    // pVn = 0
    // pAn = 0

    // Constant velocity equations:
    //  Xn = Xn-1 + Vn-1 * deltaT
    //  Vn = Vn-1
    //  An = 0
    //
    // pPn = pPn-1 + pVn-1 * deltaT^2
    // pVn = pVn-1
    // pAn = 0

    // Constant accelaration equations:
    //  Xn = Xn-1 + Vn-1 * deltaT + 0.5 * An-1 * deltaT^2
    //  Vn = Vn-1 + An-1 * deltaT
    //  An = An-1
    //
    // pPn = pPn-1 + pVn-1 * deltaT + 0.5 * pAn-1 * deltaT^2
    // pVn = pVn-1 + pAn-1 * deltaT
    // pAn = pAn-1

    // State
    for(unsigned int dimensionIdx = 0; dimensionIdx < NUM_DIMENSIONS; dimensionIdx++)
    {
        const unsigned int positionIdx = dimensionIdx;
        const unsigned int velocityIdx = NUM_DIMENSIONS + dimensionIdx;
        const unsigned int accelerationIdx = 2 * NUM_DIMENSIONS + dimensionIdx;

        //
        // State
        //

        // Position
        if(NUM_STATES == 1)
        {
            predictedState.state[positionIdx][0] = state_.state[positionIdx][0];
        } 
        else if(NUM_STATES == 2)
        {
            predictedState.state[positionIdx][0] = state_.state[positionIdx][0] + state_.state[velocityIdx][0] * deltaT;
        }
        else if(NUM_STATES == 3)
        {
            predictedState.state[positionIdx][0] = state_.state[positionIdx][0] + state_.state[velocityIdx][0] * deltaT 
                                                    + 0.5 * state_.state[accelerationIdx][0] * std::pow(deltaT, 2);
        }

        // Velocity
        if(NUM_STATES == 1)
        {
            predictedState.state[velocityIdx][0] = 0.0;
        }
        else if(NUM_STATES == 2)
        {
            predictedState.state[velocityIdx][0] = state_.state[velocityIdx][0];
        }
        else if(NUM_STATES == 3)
        {
            predictedState.state[velocityIdx][0] = state_.state[velocityIdx][0] + state_.state[accelerationIdx][0] * deltaT;
        
        }

        // Acceleration
        if(NUM_STATES == 3)
        {
            predictedState.state[accelerationIdx][0] = state_.state[accelerationIdx][0];
        }

        //
        // Covariance
        //

        // Position
        if(NUM_STATES == 1)
        {
            predictedState.covariance[positionIdx][positionIdx] = state_.covariance[positionIdx][positionIdx] + parameters_.processNoise;
        }
        else if(NUM_STATES == 2)
        {
            predictedState.covariance[positionIdx][positionIdx] = state_.covariance[positionIdx][positionIdx] 
                                                                    + state_.covariance[velocityIdx][velocityIdx] * deltaT; // std::pow(deltaT, 2);
        }
        else if(NUM_STATES == 3)
        {
            predictedState.covariance[positionIdx][positionIdx] = state_.covariance[positionIdx][positionIdx] 
                                                                    + state_.covariance[velocityIdx][velocityIdx] * std::pow(deltaT, 2) 
                                                                    + 0.5 * state_.covariance[accelerationIdx][accelerationIdx] * std::pow(deltaT, 2);
        }

        // Velocity
        if(NUM_STATES == 1)
        {
            predictedState.covariance[velocityIdx][velocityIdx] = 0.0;
        }
        else if(NUM_STATES == 2)
        {
            predictedState.covariance[velocityIdx][velocityIdx] = state_.covariance[velocityIdx][velocityIdx] + parameters_.processNoise;
        }
        else if(NUM_STATES == 3)
        {
            predictedState.covariance[velocityIdx][velocityIdx] = state_.covariance[velocityIdx][velocityIdx] 
                                                                    + state_.covariance[accelerationIdx][accelerationIdx] * std::pow(deltaT, 2);
        }

        // Acceleration
        if(NUM_STATES == 3)
        {
            predictedState.covariance[accelerationIdx][accelerationIdx] = state_.covariance[accelerationIdx][accelerationIdx] + parameters_.processNoise;
        }
    }

    return predictedState;
}

template <unsigned int NUM_DIMENSIONS, unsigned int NUM_STATES>
Matrix<NUM_DIMENSIONS * NUM_STATES, NUM_DIMENSIONS * NUM_STATES> Filter::KalmanFilter<NUM_DIMENSIONS, NUM_STATES>::getObservationMatrix(
    const Filter::PointState<NUM_DIMENSIONS, NUM_STATES>& measurement) const
{
    unsigned int MEASUREMENT_STATES = 1;

    // If all accelaration states are measured, then set MEASUREMENT_STATES to 3
    if(NUM_STATES == 3)
    {
        MEASUREMENT_STATES = 2;

        for(unsigned int i = 0; i < NUM_DIMENSIONS; i++)
        {
            const unsigned int accelarationIdx = 2 * NUM_DIMENSIONS + i;
            if(std::abs(measurement.state[accelarationIdx][0] - 0.0) > std::numeric_limits<double>::epsilon())
            {
                MEASUREMENT_STATES = 3;
                break;
            }
        }
    } else if(NUM_STATES == 2)
    {
        MEASUREMENT_STATES = 1;

        for(unsigned int i = 0; i < NUM_DIMENSIONS; i++)
        {
            const unsigned int velocityIdx = NUM_DIMENSIONS + i;
            if(std::abs(measurement.state[velocityIdx][0] - 0.0) > std::numeric_limits<double>::epsilon())
            {
                MEASUREMENT_STATES = 2;
                break;
            }
        }
    }    

    // Create observation matrix (Hk), which is a 9x9 matrix with 1's on the diagonal
    // for the position and velocity states
    Matrix<NUM_STATES * NUM_DIMENSIONS, NUM_STATES * NUM_DIMENSIONS> observationMatrix;
   
    for(unsigned int i = 0; i < NUM_STATES * NUM_DIMENSIONS; i++)
    {
        for(unsigned int j = 0; j < NUM_STATES * NUM_DIMENSIONS; j++)
        {
            observationMatrix[i][j] = 0.0;

            // Only measured position and velocity
            if(i == j && i < NUM_DIMENSIONS * MEASUREMENT_STATES)
            {
                observationMatrix[i][j] = 1.0;
            }
        }
    }

    return observationMatrix;
}

template <unsigned int NUM_DIMENSIONS, unsigned int NUM_STATES>
Matrix<NUM_DIMENSIONS * NUM_STATES, 1> Filter::KalmanFilter<NUM_DIMENSIONS, NUM_STATES>::getPredictedStateMatrix(
    const PointState<NUM_DIMENSIONS, NUM_STATES>& predictedState) const
{
    // Create predicted state matrix (Xk|k-1)
    Matrix<NUM_DIMENSIONS * NUM_STATES, 1> predictedStateMatrix = predictedState.state;

    return predictedStateMatrix;
}

template <unsigned int NUM_DIMENSIONS, unsigned int NUM_STATES>
Matrix<NUM_DIMENSIONS * NUM_STATES, 1> Filter::KalmanFilter<NUM_DIMENSIONS, NUM_STATES>::getMeasurementStateMatrix(
    const PointState<NUM_DIMENSIONS, NUM_STATES>& measurement) const
{
    // Create measurement state matrix (Zk)
    Matrix<NUM_DIMENSIONS * NUM_STATES, 1> measurementStateMatrix = measurement.state;

    return measurementStateMatrix;
}

template <unsigned int NUM_DIMENSIONS, unsigned int NUM_STATES>
void Filter::KalmanFilter<NUM_DIMENSIONS, NUM_STATES>::computeResiduals(const PointState<NUM_DIMENSIONS, NUM_STATES>& measurement)
{
    for(unsigned int i = 0; i < NUM_DIMENSIONS * NUM_STATES; i++)
    {
        residuals_[i] = measurement.state[i][0] - state_.state[i][0];
    }
}

#endif // MULTI_KALMAN_FILTER_H