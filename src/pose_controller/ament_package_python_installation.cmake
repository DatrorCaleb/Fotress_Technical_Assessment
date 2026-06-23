macro(ament_package_python_installation)
  find_package(ament_cmake_python REQUIRED)
  ament_python_install_package(${PROJECT_NAME})
endmacro()
