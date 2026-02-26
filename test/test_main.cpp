/**
 * @file test_main.cpp
 * @brief GoogleTest entry point for the BCE test suite.
 */

#include <gtest/gtest.h>

int main(int argc, char** argv) {
    // Hand control to GoogleTest after framework initialization.
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
