/**
 * @file    main.cpp
 * @author  Senthil Hariharan Arul
 * @copyright GNU Public License
 * @brief Calls the function to tune controller
 */

#include <iostream>
#include <memory>
#include "PIDController.hpp"

int main() {
    std::shared_ptr<PIDController>PID;

    double ref {1.0};
    double currentVel {0.1};

    std::cout << "Reference velocity: " << ref << std::endl;
    std::cout << "The current velocity is: " << currentVel << std::endl;
    PID = std::make_shared<PIDController>(0.01, 0.01, 0.01, 0.1);
    std::cout << "Velocity after added PID controller: " << \
    			PID->tuneController() << std::endl;
}
