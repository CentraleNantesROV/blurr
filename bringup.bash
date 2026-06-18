#!/usr/bin/env bash

source $(realpath $(dirname $0))/rmt.bash
ros2 launch blurr bringup.launch
