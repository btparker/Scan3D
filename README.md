3dScan
======

Collaborators:
Greg,

To run tests
============
cd test
cmake CMakeLists.txt
make
./runTests

To add tests
============
add tests to test/src/blahblah.cpp
add testname to line: add_executable(runTests test/src/blahblah.cpp)
........
profit