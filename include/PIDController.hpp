/**
 * @file    PIDController.hpp
 * @author  Senthil Hariharan Arul
 * @copyright GNU Public License
 * @brief Declaration of class PIDController.
 */

#pragma once

#include<iostream>

/**
 * @brief Class PIDController
 * The following class PIDController aids in calculation of the newVelocity
 * using setpoint and current velocity. It implements functions to tune 
 * controller
*/

class PIDController {
 private:
        double kProportional;  /*!< Kp value*/
        double kIntegral;  /*!< Ki value*/
        double kDerivative;  /*!< Kd value*/
        double error;
        double max = 10;
        double min = -10;
        double previousError = 0;
        double totalError = 0;
        double errorThreshold = 0.00001;  /*!< can differ from setpoint by 0.01*/
        double dTime = 0.1;
        double setPoint = 1;
        double newVelocity;
        double actualVelocity;

 public:
        //! Constructer to set proportional, derivative and integral constant
        PIDController(double kp, double ki, double kd, double aVel);
        //! Returns Kp value
        auto getProportional()->double;
        //! Returns Ki value
        auto getIntegral()->double;
        //! Return Kd value
        auto getDerivative()->double;
        //! Function to tune controller
        auto tuneController()->double;
        //! Function returns error value
        auto calculateError()->double;
        //! Destructor for the class
        ~PIDController();
};
