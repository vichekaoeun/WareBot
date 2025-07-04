cmake_minimum_required(VERSION 3.8)
project(warebot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Task.msg"
  "srv/TaskStatus.srv"
  DEPENDENCIES std_msgs
)

install(PROGRAMS
  warebot/task.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME task
)

install(PROGRAMS
  warebot/task_status_service.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME service
)

install(PROGRAMS
  warebot/broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME broadcaster
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()