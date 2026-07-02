cmake_minimum_required(VERSION 3.5)
project(blurr)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_BUILD_TYPE Release)

ament_auto_add_executable(pwm src/pwm.cpp)
    
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    )
endif()

install(PROGRAMS src/teleop.py src/monitor.py
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})
  
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
