# ROStest for c++ Project Report

## What is Unit Testing?

UNIT TESTING is a type of software testing where individual units or components of the software are tested. The purpose is to validate that each unit of the software code performs as expected. Unit Testing is done during the development (coding phase) of an application by the developers. Unit Tests isolate a section of code and verify its correctness. A unit may be an individual function, method, procedure, module, or object.


![image](Unit-Testing.png "Unit Testing")


Here, are the key reasons to perform unit testing in software engineering:
Unit tests help to fix bugs early in the development cycle and save costs.
It helps the developers to understand the testing code base and enables them to make changes quickly
Good unit tests serve as project documentation
Unit tests help with code re-use. Migrate both your code and your tests to your new project. Tweak the code until the tests run again.
Conditions for a test to be “Good”:
Tests should be independent and repeatable.
Tests should be well organized and reflect the structure of the tested code.
Tests should be portable and reusable.
When tests fail, they should provide as much information about the problem as possible.
The testing framework should liberate test writers from housekeeping chores and let them focus on the test content.
Tests should be fast.

## What is gtest?

googletest is a testing framework and helps you write better C++ tests.
Notable features:
Can repeat tests like 100 times.
Built-in assertions.
Easy to call tests.
Generating an Extensible Markup Language (XML) report is easy.

Learn more about gtest [here](https://github.com/google/googletest).

Gtest uses the term “Test Case” for grouping related “Tests”
International Software Testing Qualifications Board([ISTQB](http://www.istqb.org/)) uses “Test Suite” for grouping related “Test Cases”.

        ISTQB	             googletest
         Test Suite 	⇔	Test Case
            Test Case 	⇔ 	Test

A test program can contain multiple test suites. A test suite contains one or many tests. You should group your tests into test suites that reflect the structure of the tested code. When multiple tests in a test suite need to share common objects and subroutines, you can put them into a test fixture class.

## What is ROStest?
rostest is an extension to roslaunch that enables roslaunch files to be used as test fixtures.

Learn more about ROStest [here](http://wiki.ros.org/rostest).

## How?
[Example Package on GitHub](https://github.com/carpit680/testing.git)

### Installation
Gtest is now a rosdep and hence no installation is required.
### Directory structure
By convention, rostest files and gtest(.cpp) files for a package go in a test subdirectory.
### Writing Tests
The basic structure of a test looks like this:   
``` 
// include header file for the node that is to be tested.
#include "<package-name>/<header-file.h>"
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(TestSuite, testCase1)
{
<test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
}

// Declare another test
TEST(TestSuite, testCase2)
{
<test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
} 
```

Initialize ROS if your tests are using ROS.

### Naming Convention:
Test suites are CamelCased, like C++ types.   
Test cases are camelCased, like C++ functions.

### Rostest file
The file is similar to a roslaunch file but with a .test extension instead of a .launch extension and is roslaunch compatible. You can start nodes and set params just like a normal launch file.
The <test> tag is syntactically similar to the <node> tag. The <test> tag indicates that the node is actually a test node to run.

An example <test> tag:

```
<launch>

  <test test-name="<gtest-node-name>" pkg="<name-of-the-test-package>" type="<name-of-test-file-.cpp>" time-limit="<optional-time-limit>" args="<optional-pass-arguments-to-test-nodes>" retry="<optional-number-of-times-to-retry-test>" />

</launch>
```
You can use the following XML tags inside of a <test> tag:
        
``` <env> ```
Set an environment variable for the node.

``` <remap> ```
Set a remapping argument for this node.

``` <rosparam> ```
Load a rosparam file into this node's ~/local namespace.

``` <param> ```
Set a parameter in the node's ~/local namespace.

Do not use any respawn, output, or machine attributes.   

### CMakeLists.txt
Create a gtest executable with the target name <gtest-node-name> which is not built by make all but only by make tests. Add one/multiple source files to the executable target and register the rostest launch file you made above.
        
```
add_rostest_gtest(<gtest-node-name> test/<rostest-launch-file> test/<gtest-node-file.cpp>)
```
Link libraries if necessary
```
target_link_libraries(<gtest-node-name> [libraries-to-depend-on,-e.g. ${catkin_LIBRARIES}])
```
Wrap all testing stuff in a conditional block:
```
if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)
  add_rostest_gtest(<gtest-node-name> test/<rostest-launch-file> test/<gtest-node-file.cpp>)
  target_link_libraries(<gtest-node-name> [libraries-to-depend-on,-e.g. ${catkin_LIBRARIES}])

endif()
```
### Package.xml
Declare rostest as a test dependency, along with any other test-only dependencies:
```
<test_depend>rostest</test_depend>
```
### Running rostest
Manually from cmd: 
```
rostest test_rospy rospy.test
```
Or while building:
```
catkin_make run_tests
```
Testing just low-level functionality without rostest:   
### CMakelists.txt
```
if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)
  catkin_add_gtest(<gtest-name> test/<gtest-file.cpp>)
  target_link_libraries(<gtest-name> [libraries-to-depend-on,-e.g. ${catkin_LIBRARIES}])

endif()
```

If you want to build an executable against gtest, but not declare it to be a test on its own (e.g., when you intend the executable to be run by rostest), use ``` catkin_add_executable_with_gtest() ```.

### Package.xml
Add:
``` 
<test_depend>rosunit</test_depend>
```
Do not add rostest as a test dependency. The ``` catkin_add_gtest() ``` macro will pull in the necessary flags.
### Running low-level Tests
Manually by running the following on cmd:   
```
./bin/test/<gtest-name>
```

OR   

while building:    
```
make test
```

OR

```
roscore
catkin_make run_tests
```

OR

to run a specific package simply tab complete to find the specific package tests you'd like to run.
```
roscore
catkin_make run_tests<TAB><TAB>
```
## Call catkin_add_gtest() and add_rostest_gtest() separately
### CMakeLists.txt
Here I refer to unit tests as the low-level tests (ie: no ROS environment) and integration tests as tests done with rostest that use ros nodes

Add in two options at the top of your CMake (or your root CMake)
```
option(BUILD_UNIT_TESTS "Build the low-level tests" OFF)
option(BUILD_INTEGRATION_TESTS "Build the ros node integration tests" OFF)
```
Write your tests block similar to this example:
```
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  if(BUILD_UNIT_TESTS)
    catkin_add_gtest(<gtest-name> test/<gtest-file.cpp>)

    if(TARGET <gtest-name>)
      target_link_libraries(<gtest-name> [libraries-to-depend-on,-e.g. ${catkin_LIBRARIES}])

    endif()
  endif(BUILD_UNIT_TESTS)

  if(BUILD_INTEGRATION_TESTS)
    add_rostest_gtest(<gtest-node-name> test/<rostest-launch-file> test/<gtest-node-file.cpp>)

    if(TARGET primary_controller_integration_tests)
      target_link_libraries(<gtest-node-name> [libraries-to-depend-on,-e.g. ${catkin_LIBRARIES}]))

    endif()
  endif(BUILD_INTEGRATION_TESTS)
endif(CATKIN_ENABLE_TESTING)
```

Then when invoking catkin, use
``` catkin_make run_tests --cmake-args -DBUILD_UNIT_TESTS=OFF -DBUILD_INTEGRATION_TESTS=ON ```

Using a custom CMake option like ``` BUILD_INTEGRATION_TESTS ``` which defaults to ``` OFF ``` will prevent you from running these tests on e.g. the ROS buildfarm (since it doesn't know about the option and won't enable it).


## Citations
[rostest - ROS Wiki](http://wiki.ros.org/rostest)

[gtest - ROS Wiki](http://wiki.ros.org/gtest)

[rostest - Minimum Working Example - ROS Answers: Open Source Q&A Forum](https://answers.ros.org/question/181708/rostest-minimum-working-example/?answer=181934?answer=181934#post-id-181934)

[How to best call catkin_add_gtest and add_rostest_gtest separately? - ROS Answers: Open Source Q&A Forum](https://answers.ros.org/question/271167/how-to-best-call-catkin_add_gtest-and-add_rostest_gtest-separately/)

[googletest/primer.md at master · google/googletest (github.com)](https://github.com/google/googletest/blob/master/docs/primer.md)
