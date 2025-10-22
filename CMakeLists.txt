cmake_minimum_required(VERSION 3.0.2)
project(beginner_tutorials)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/rec_node.py
  scripts/center.py
  # (เดี๋ยวเราจะมาเพิ่ม arduino_connect.py ทีหลัง)
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
