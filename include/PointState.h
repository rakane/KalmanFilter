#ifndef POINT_STATE_H
#define POINT_STATE_H

#include <iostream>
#include <chrono>
#include "Matrix.h"

namespace Filter
{
    template<unsigned int NUM_DIMENSIONS, unsigned int NUM_STATES>
    class PointState
    {
    public:
        // Default constructor
        PointState(): state(), covariance(), time()
        {
            // Initialize position to 0
            for(unsigned int i = 0; i < NUM_DIMENSIONS * NUM_STATES; i++)
            {
                state[i][0] = 0.0;
            }

            // Initialize covariance to 0
            for(unsigned int i = 0; i < NUM_DIMENSIONS * NUM_STATES; i++)
            {
                for(unsigned int j = 0; j < NUM_DIMENSIONS * NUM_STATES; j++)
                {
                    covariance[i][j] = 0.0;
                }
            }
        }

        // Position only constructor
        PointState(double position[NUM_DIMENSIONS], double positionVar[NUM_DIMENSIONS], 
                        std::chrono::time_point<std::chrono::system_clock> time)
        {
            // Initialize position to 0
            for(unsigned int i = 0; i < NUM_DIMENSIONS * NUM_STATES; i++)
            {
                state[i][0] = 0.0;
            }

            // Initialize covariance to 0
            for(unsigned int i = 0; i < NUM_DIMENSIONS * NUM_STATES; i++)
            {
                for(unsigned int j = 0; j < NUM_DIMENSIONS * NUM_STATES; j++)
                {
                    covariance[i][j] = 0.0;
                }
            }

            // Set position
            for(unsigned int i = 0; i < NUM_DIMENSIONS; i++)
            {
                state[i][0] = position[i];
                covariance[i][i] = positionVar[i];
            }

            this->time = time;       
        }     

        // Position/Velocity only constructor
        PointState(double position[NUM_DIMENSIONS], double velocity[NUM_DIMENSIONS],
                      double positionVar[NUM_DIMENSIONS], double velocityVar[NUM_DIMENSIONS],
                        std::chrono::time_point<std::chrono::system_clock> time)
        {
            // Initialize position to 0
            for(unsigned int i = 0; i < NUM_DIMENSIONS * NUM_STATES; i++)
            {
                state[i][0] = 0.0;
            }

            // Initialize covariance to 0
            for(unsigned int i = 0; i < NUM_DIMENSIONS * NUM_STATES; i++)
            {
                for(unsigned int j = 0; j < NUM_DIMENSIONS * NUM_STATES; j++)
                {
                    covariance[i][j] = 0.0;
                }
            }

            // Set position
            for(unsigned int i = 0; i < NUM_DIMENSIONS; i++)
            {
                state[i][0] = position[i];
                covariance[i][i] = positionVar[i];
            }

            // Set velocity
            if(NUM_STATES > 1)
            {
                for(unsigned int i = 0; i < NUM_DIMENSIONS; i++)
                {
                    state[NUM_DIMENSIONS + i][0] = velocity[i];
                    covariance[NUM_DIMENSIONS + i][NUM_DIMENSIONS + i] = velocityVar[i];
                }
            }

            this->time = time;
        }  

        // Position/Velocity/Accelaration constructor
        PointState(double position[NUM_DIMENSIONS], double velocity[NUM_DIMENSIONS], double acceleration[NUM_DIMENSIONS],
                      double positionVar[NUM_DIMENSIONS], double velocityVar[NUM_DIMENSIONS], double accelerationVar[NUM_DIMENSIONS],
                      std::chrono::time_point<std::chrono::system_clock> time)
        {
            // Initialize position to 0
            for(unsigned int i = 0; i < NUM_DIMENSIONS * NUM_STATES; i++)
            {
                state[i][0] = 0.0;
            }

            // Initialize covariance to 0
            for(unsigned int i = 0; i < NUM_DIMENSIONS * NUM_STATES; i++)
            {
                for(unsigned int j = 0; j < NUM_DIMENSIONS * NUM_STATES; j++)
                {
                    covariance[i][j] = 0.0;
                }
            }

            // Set position
            for(unsigned int i = 0; i < NUM_DIMENSIONS; i++)
            {
                state[i][0] = position[i];
                covariance[i][i] = positionVar[i];
            }

            // Set velocity
            if(NUM_STATES > 1)
            {
                for(unsigned int i = 0; i < NUM_DIMENSIONS; i++)
                {
                    state[NUM_DIMENSIONS + i][0] = velocity[i];
                    covariance[NUM_DIMENSIONS + i][NUM_DIMENSIONS + i] = velocityVar[i];
                }
            }

            // Set acceleration
            if(NUM_STATES > 2)
            {
                for(unsigned int i = 0; i < NUM_DIMENSIONS; i++)
                {
                    state[2 * NUM_DIMENSIONS + i][0] = acceleration[i];
                    covariance[2 * NUM_DIMENSIONS + i][2 * NUM_DIMENSIONS + i] = accelerationVar[i];
                }
            }

            this->time = time;
        }
        
        bool getPosition(double position[NUM_DIMENSIONS])
        {
            for(unsigned int i = 0; i < NUM_DIMENSIONS; i++)
            {
                position[i] = state[i][0];
            }

            return true;
        }

        bool getVelocity(double velocity[NUM_DIMENSIONS])
        {
            bool success = false;

            if(NUM_STATES > 1)
            {
                success = true;

                for(unsigned int i = 0; i < NUM_DIMENSIONS; i++)
                {
                    velocity[i] = state[NUM_DIMENSIONS + i][0];
                }
            }

            return success;
        }

        bool getAcceleration(double acceleration[NUM_DIMENSIONS])
        {
            bool success = false;

            if(NUM_STATES > 2)
            {
                success = true;

                for(unsigned int i = 0; i < NUM_DIMENSIONS; i++)
                {
                    acceleration[i] = state[2 * NUM_DIMENSIONS + i][0];
                }
            }

            return success;
        }

        bool getPositionVariance(double positionVariances[NUM_DIMENSIONS])
        {
            for(unsigned int i = 0; i < NUM_DIMENSIONS; i++)
            {
                positionVariances[i] = covariance[i][i];
            }

            return true;
        }

        bool getVelocityVariance(double velocityVariances[NUM_DIMENSIONS])
        {
            bool success = false;

            if(NUM_STATES > 1)
            {
                success = true;

                for(unsigned int i = 0; i < NUM_DIMENSIONS; i++)
                {
                    velocityVariances[i] = covariance[NUM_DIMENSIONS + i][NUM_DIMENSIONS + i];
                }
            }

            return success;
        }

        bool getAccelerationVariance(double accelerationVariances[NUM_DIMENSIONS])
        {
            bool success = false;

            if(NUM_STATES > 2)
            {
                success = true;

                for(unsigned int i = 0; i < NUM_DIMENSIONS; i++)
                {
                    accelerationVariances[i] = covariance[2 * NUM_DIMENSIONS + i][2 * NUM_DIMENSIONS + i];
                }
            }

            return success;
        }

        bool getCovariance(double covariance[NUM_DIMENSIONS * NUM_STATES][NUM_DIMENSIONS * NUM_STATES])
        {
            for(unsigned int i = 0; i < NUM_DIMENSIONS * NUM_STATES; i++)
            {
                for(unsigned int j = 0; j < NUM_DIMENSIONS * NUM_STATES; j++)
                {
                    covariance[i][j] = this->covariance[i][j];
                }
            }

            return true;
        }

        std::chrono::time_point<std::chrono::system_clock> getTime()
        {
            return time;
        }

        // Data
        Matrix<NUM_DIMENSIONS * NUM_STATES, 1> state;
        Matrix<NUM_DIMENSIONS * NUM_STATES, NUM_DIMENSIONS * NUM_STATES> covariance;
        std::chrono::time_point<std::chrono::system_clock> time;
    };


}


#endif // POINT_STATE_H