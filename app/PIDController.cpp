
/**
 * @file    PIDController.cpp
 * @author  Senthil Hariharan Arul
 * @copyright 
 * @brief Methods for the class PIDController.
 */

#include "PIDController.hpp"
#include <iostream>
#include <cmath>

/**
 * @brief Constructor PIDController
 * @param kp The proportional gain value double
 * @param ki The integral gain value double
 * @param kd The derivative gain value double
 * @param aVel The actual velocity double
 * The following constructor initializes the values of kProportional,
 * kIntegral, kDerivative and actual velocity.
*/
PIDController::PIDController(double kp, double ki, double kd, double aVel) {
    kProportional = kp;
    kIntegral = ki;
    kDerivative = kd;
    actualVelocity = aVel;
    error = 0;
    newVelocity = 0;
}

/**
 * @brief function getProportional
 * @return kProportional of type double
 * The following function getProportional return the value of proportional
 * gain.
 */
auto PIDController::getProportional() -> double {
    // Return K proportional value
    return kProportional;
}

/** 
 * @brief function getIntegral
 * @return kIntegral of type double
 * The following function getIntegral return the value of Integral
 * gain.
 */
auto PIDController::getIntegral() -> double {
    // Return K integral value
    return kIntegral;
}

/**
 * @brief function getDerivative
 * @return kDerivative of type double
 * The following function getDerivative return the value of derivative
 * gain.
 */
auto PIDController::getDerivative() -> double {
    // Return K derivative value
    return kDerivative;
}

/**
 * @brief function tuneController
 * @return newVelocity of type double
 * The following function getProportional return the value of proportional
 * gain.
 */
auto PIDController::tuneController() -> double {
    // Get the values of Kp,Ki and Kd from respective functions

    // Write while loop to calculate new velocity using PID until the new

    // velocity converges to setpoint within the error threshold limit.

    // Limit the PID output to max and min values if it exceed them
    return 12.7;
}

/**
 * @brief function calculateError
 * @return error of type double
 * Calculate the error between setpoint and actual velocity.
 */
auto PIDController::calculateError() -> double {
    // Calculate error between setpoint and actual velocity
    // Return error
    return 10;
}

/**
 * @brief destructor for class PIDController
 */
PIDController::~PIDController() {
}

