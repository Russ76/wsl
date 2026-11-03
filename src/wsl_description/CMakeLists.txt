cmake_minimum_required(VERSION 3.16)
project(wsl_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf test/test_urdf.py
    ENV WSL_DESCRIPTION_URDF_DIR=${CMAKE_CURRENT_SOURCE_DIR}/urdf
    TIMEOUT 60)
endif()

ament_package()
