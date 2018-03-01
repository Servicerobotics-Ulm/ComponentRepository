cmake_minimum_required(VERSION 2.8.3)
project(hey5_description)

find_package(catkin REQUIRED)
catkin_package()

foreach(dir config gazebo launch meshes robots urdf)
  install(DIRECTORY ${dir}/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


install(FILES LICENSE.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
