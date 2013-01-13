//testAppTest.cpp
#include "../../src/Scan3dApp.h"
#include <gtest/gtest.h>
 
TEST(testAppTest, testAppCanBeInstantiated) {
	Scan3dApp app;
}
 
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}