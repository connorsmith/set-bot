cmake_minimum_required(VERSION 2.8.3)
project(connor_set_bot)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir config launch src)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)