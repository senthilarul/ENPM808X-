/**
 * @file    test.cpp
 * @author  Senthil Hariharan Arul
 * @copyright GNU Public License
 * @brief Defines test to check mothods of class PIDController.
 */

#include <gtest/gtest.h>
#include <PIDController.hpp>
#include <memory>

/**
 *@brief Test if the gain constants are stored correctly. 
 */
TEST(test, checkconstantvalues) {
    PIDController PID(0.01, 0.01, 0.01, 1);
    ASSERT_FLOAT_EQ(PID.getProportional(), 0.01);
    ASSERT_FLOAT_EQ(PID.getIntegral(), 0.01);
    ASSERT_FLOAT_EQ(PID.getDerivative(), 0.01);
}

/**
 *@brief Test if the newVelocity after PID converges to the setpoint within
  given threshold limit. 
 */
TEST(test1, checkconvergance) {
    PIDController PID(0.01, 0.01, 0.01, 0.1);
    EXPECT_NEAR(PID.tuneController(), 1, 0.001);
}




