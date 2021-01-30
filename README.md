# ROStest for c++ Project Report

## What is Unit Testing?

UNIT TESTING is a type of software testing where individual units or components of the software are tested. The purpose is to validate that each unit of the software code performs as expected. Unit Testing is done during the development (coding phase) of an application by the developers. Unit Tests isolate a section of code and verify its correctness. A unit may be an individual function, method, procedure, module, or object.


![image](testing/Unit-Testing.png "Unit Testing")


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
Learn about using gtest for ROS using [gtest - ROS Wiki](http://wiki.ros.org/gtest) 

Examine the simple sample package provided here as an example. 


