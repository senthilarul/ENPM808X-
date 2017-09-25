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
    PID = std::make_shared<PIDController>(0.01, 0.01, 0.01, 0.1);
    PID->tuneController();
}
