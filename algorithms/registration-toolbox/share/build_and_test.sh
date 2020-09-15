#!/usr/bin/env bash

catkin build registration_toolbox --no-deps && catkin run_tests registration_toolbox --no-deps && catkin_test_results ../../../../../build/registration_toolbox
