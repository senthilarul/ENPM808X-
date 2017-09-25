/**
 * @file    PIDController.cpp
 * @author  Senthil Hariharan Arul
 * @copyright 
 * @brief Methods for the class PIDController.
 */

#include "PIDController.hpp"
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

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
    actualVelocity  = aVel;
}

/**
 * @brief function getProportional
 * @return kProportional of type double
 * The following function getProportional return the value of proportional 
 * gain.
 */
auto PIDController::getProportional() -> double {
    return kProportional; 
}

/** 
 * @brief function getIntegral
 * @return kIntegral of type double
 * The following function getIntegral return the value of Integral 
 * gain.
 */
auto PIDController::getIntegral() -> double {
    return kIntegral; 
}

/**
 * @brief function getDerivative
 * @return kDerivative of type double
 * The following function getDerivative return the value of derivative 
 * gain.
 */
auto PIDController::getDerivative() -> double {
    return kDerivative; 
}

/** 
 * @brief function tuneController
 * @return newVelocity of type double
 * The following function getProportional return the value of proportional 
 * gain.
 */
auto PIDController::tuneController() -> double {
    double kp = getProportional();
    double ki = getIntegral();
    double kd = getDerivative();
    std::cout << kp <<" "<< ki<<" "<<kd<<"\t";
    double loopCounter=0;
    while ( std::abs(calculateError()) > errorThreshold ) {
    loopCounter++;
    error = calculateError();
    std::cout << "error: "<< error <<std::endl;
    totalError += error;
    std::cout << kp * error << " "<< ki * totalError *dTime << " "<< ((error - previousError)/dTime) * kd << std::endl;
    double newVel = kp * error + ki * totalError *dTime + ((error - previousError)/dTime) * kd;
    previousError = error;
    newVelocity = (newVel > max) ? max : newVel < min ? min : newVel;
    actualVelocity = newVelocity;
    //std::this_thread::sleep_for (std::chrono::seconds(1));
    }
    std::cout << "new velocity now is" << newVelocity <<"error"<<std::abs(calculateError());
return newVelocity;
}

/** 
 * @brief function calculateError
 * @return error of type double
 * Calculate the error between setpoint and actual velocity.
 */
auto PIDController::calculateError() -> double {
    error = setPoint - actualVelocity;
    return error;
}
/** 
 * @brief destructor for class PIDController
 */
PIDController::~PIDController() {
}
