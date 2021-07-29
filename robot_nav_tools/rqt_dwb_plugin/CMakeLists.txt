cmake_minimum_required(VERSION 3.0.2)
project(rqt_dwb_plugin)

find_package(catkin REQUIRED)
catkin_python_setup()

catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(roslint)
  roslint_python()
  roslint_add_test()
endif()

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES scripts/live_panel
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
